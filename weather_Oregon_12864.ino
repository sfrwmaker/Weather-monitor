// The Weather Station@arduinoMEGA2560
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <SdFat.h>
#include <Time.h>
#include <TimeLib.h>
#include <DS3232RTC.h>
#include <EEPROM.h>
#include <math.h>
#include <sunMoon.h>
#include <WlessOregonV2.h>

#define DHT_PIN        4
#define DHTTYPE    DHT22

#define TINY_FONT      u8g2_font_profont10_tf
#define RUS_FONT_S     u8g2_font_crox1h_tf
#define RUS_FONT_L     u8g2_font_crox3hb_tf
#define DIGIT_FONT     u8g2_font_osb18_tn

// lcd 128x64 screen in software mode
#define EN_SCK_PIN      48
#define RW_MOSI_PIN     47
#define RS_CS_PIN       49
#define RST_PIN          8

#define LIGHT_SENSOR    A0                      // A0, Photoresistor
#define LCD_BLGHT_PIN   46

// Rotary encoder interface
const byte R_MAIN_PIN =  2;                     // Rotary Encoder main pin (right)
const byte R_SECD_PIN =  5;                     // Rotary Encoder second pin (left)
const byte R_BUTN_PIN =  3;                     // Rotary Encoder push button pin

#define SD_PIN          53                      // SD card reader select pin
#define HB_PIN          A2                      // The pin for heartBeat reset

#define OUR_latitude    55.751244
#define OUR_longtitude  37.618423
#define OUR_timezone    180                     // localtime with UTC difference in minutes

const uint16_t high_pressure = 767-8;           // High pressure depends on altitude
const uint16_t low_pressure  = 757-8;           // Low pressure  depends on altitude

const byte max_sensors = 5;                     // The maximum number of the sensors (remote + one internal) connected
const byte w433_INTR   = 5;                     // The wireless 433 MHz sensor @ pin 18, interrupt #5 

static const char* months = "\xDF\xed\xe2\xD4\xe5\xe2\xCC\xe0\xf0\xC0\xef\xf0\xCC\xe0\xe9\xC8\xfe\xed\xC8\xfe\xeb\xC0\xe2\xe3\xD1\xe5\xed\xCE\xea\xf2\xCD\xee\xff\xC4\xe5\xea";
static const char* wday   = "\xC2\xf1\xCF\xed\xC2\xf2\xD1\xf0\xD7\xf2\xCF\xf2\xD1\xe1";

void printDate(time_t date) {
  char buff[20];
  sprintf(buff, "%2d-%02d-%4d %02d:%02d:%02d",
  day(date), month(date), year(date), hour(date), minute(date), second(date));
  Serial.print(buff);
}

//------------------------------------------ Configuration data ------------------------------------------------
/* Config record in the EEPROM has the following format:
  uint32_t ID                           each time increment by 1
  struct cfg                            config data
  byte CRC                              the checksum
*/
struct cfg {
  byte ext_sensor_ID;                               // external sensor ID (0 - random)
  byte backlight_morning;                           // morning time in 10-minute intervals
  byte backlight_evening;                           // evening time in 10-minute intervals
};

class CONFIG {
  public:
    CONFIG() {
      can_write = is_valid = false;
      buffRecords   = 0;
      rAddr = wAddr = 0;
      eLength       = 0;
      nextRecID     = 0;
      byte rs = sizeof(struct cfg) + 5;             // The total config record size
      // Select appropriate record size; The record size should be power of 2, i.e. 8, 16, 32, 64, ... bytes
      for (record_size = 8; record_size < rs; record_size <<= 1);
    }
    void init();
    bool load(void);
    bool isValid(void)                              { return is_valid; }
    void getConfig(struct cfg &Cfg);                // Copy config structure from this class
    void updateConfig(struct cfg &Cfg);             // Copy updated config into this class
    bool saveConfig(struct cfg &Cfg);               // write updated config into the EEPROM
    
  private:
    void defaultConfig(void);
    struct cfg Config;
    bool readRecord(uint16_t addr, uint32_t &recID);
    bool save(void);
    bool can_write;                                 // The flag indicates that data can be saved
    bool is_valid;                                  // Whether tha data was loaded
    byte buffRecords;                               // Number of the records in the outpt buffer
    uint16_t rAddr;                                 // Address of thecorrect record in EEPROM to be read
    uint16_t wAddr;                                 // Address in the EEPROM to start write new record
    uint16_t eLength;                               // Length of the EEPROM, depends on arduino model
    uint32_t nextRecID;                             // next record ID
    byte     record_size;                           // The size of one record in bytes
};

 // Read the records until the last one, point wAddr (write address) after the last record
void CONFIG::init(void) {
  defaultConfig();
  eLength = EEPROM.length();
  byte t, p ,h;
  uint32_t recID;
  uint32_t minRecID = 0xffffffff;
  uint16_t minRecAddr = 0;
  uint32_t maxRecID = 0;
  uint16_t maxRecAddr = 0;
  byte records = 0;

  nextRecID = 0;

  // read all the records in the EEPROM find min and max record ID
  for (uint16_t addr = 0; addr < eLength; addr += record_size) {
    if (readRecord(addr, recID)) {
      ++records;
      if (minRecID > recID) {
        minRecID = recID;
        minRecAddr = addr;
      }
      if (maxRecID < recID) {
        maxRecID = recID;
        maxRecAddr = addr;
      }
    } else {
      break;
    }
  }

  if (records == 0) {
    wAddr = rAddr = 0;
    can_write = true;
    return;
  }

  rAddr = maxRecAddr;
  if (records < (eLength / record_size)) {          // The EEPROM is not full
    wAddr = rAddr + record_size;
    if (wAddr > eLength) wAddr = 0;
  } else {
    wAddr = minRecAddr;
  }
  can_write = true;
}

void CONFIG::getConfig(struct cfg &Cfg) {
  memcpy(&Cfg, &Config, sizeof(struct cfg));
}

void CONFIG::updateConfig(struct cfg &Cfg) {
  memcpy(&Config, &Cfg, sizeof(struct cfg));
}

bool CONFIG::saveConfig(struct cfg &Cfg) {
  updateConfig(Cfg);
  return save();                                     // Save new data into the EEPROM
}

bool CONFIG::save(void) {
  if (!can_write) return can_write;
  if (nextRecID == 0) nextRecID = 1;

  uint16_t startWrite = wAddr;
  uint32_t nxt = nextRecID;
  byte summ = 0;
  for (byte i = 0; i < 4; ++i) {
    EEPROM.write(startWrite++, nxt & 0xff);
    summ <<=2; summ += nxt;
    nxt >>= 8;
  }
  byte* p = (byte *)&Config;
  for (byte i = 0; i < sizeof(struct cfg); ++i) {
    summ <<= 2; summ += p[i];
    EEPROM.write(startWrite++, p[i]);
  }
  summ ++;                                            // To avoid empty records
  EEPROM.write(wAddr+record_size-1, summ);

  rAddr = wAddr;
  wAddr += record_size;
  if (wAddr > EEPROM.length()) wAddr = 0;
  return true;
}

bool CONFIG::load(void) {
  is_valid = readRecord(rAddr, nextRecID);
  nextRecID ++;
  if (!is_valid) defaultConfig();
  return is_valid;
}

bool CONFIG::readRecord(uint16_t addr, uint32_t &recID) {
  byte Buff[record_size];

  for (byte i = 0; i < record_size; ++i) 
    Buff[i] = EEPROM.read(addr+i);
  
  byte summ = 0;
  for (byte i = 0; i < sizeof(struct cfg) + 4; ++i) {

    summ <<= 2; summ += Buff[i];
  }
  summ ++;                                              // To avoid empty fields
  if (summ == Buff[record_size-1]) {                    // Checksumm is correct
    uint32_t ts = 0;
    for (char i = 3; i >= 0; --i) {
      ts <<= 8;
      ts |= Buff[i];
    }
    recID = ts;
    byte i = 4;
    memcpy(&Config, &Buff[4], sizeof(struct cfg));
    return true;
  }
  return false;
}

void CONFIG::defaultConfig(void) {
  Config.ext_sensor_ID     =   0;                       // 0 means - any sensor
  Config.backlight_morning =  48;                       // 8:00
  Config.backlight_evening = 138;                       // 23:00
}

//------------------------------------------ Heart beat class (Watch dog timer based on ne555 IC) --------------
class HB {
  public:
    HB(byte hb) {
      hb_pin = hb;
      pinMode(hb_pin, INPUT);
      period = 0;                                   // auto reset is disabled
      next_reset = 0;
    }
    void reset(void) {
      pinMode(hb_pin, OUTPUT);
      digitalWrite(hb_pin, LOW);                    // reset ne555 timer
      delay(200);
      pinMode(hb_pin, INPUT);
    }
    void setTimeout(uint16_t to = 0)                { period = to; }
    void autoReset(void) {
      if (period == 0) {                            // Automatic reset is disabled, reset now
        reset();
        return;
      }
      if (millis() > next_reset) {
        reset();
        next_reset = millis() + (long)period * 1000; 
      }
    }
  private:
    byte     hb_pin;                                // ne555 reset pin (heart beat)
    uint16_t period;                                // The period to reset the ne555 in seconds
    uint32_t next_reset;                            // time in ms to automatically reset the ne555
};

//------------------------------------------ backlight of the LCD display (lite version) ----------------------
class BL {
  public:
    BL(byte sensorPIN, byte lightPIN, byte start_brightness = 128) {
      sensor_pin          = sensorPIN;
      led_pin             = lightPIN;
      default_brightness  = start_brightness;
      b_night             =  50;
      daily_brightness    = 150;
      b_day               = 500;
    }
    void init(void);                                // Initialize the data
    void adjust(void);                              // Automatically adjust the brightness
    int  getSensorValue(void)                       { return analogRead(sensor_pin); }
    void setBrightness(byte b);
    void turnAuto(bool a);
    void setLimits(uint16_t dark, uint16_t daylight, byte br_nightly, byte br_daily);
    void setNightPeriod(byte Evening, byte Morning);
    bool isDark(void);                              // Whether it is night time or it is dark here
  private:
    int      empAverage(int v);                     // Exponential average value
    byte     sensor_pin;                            // Light sensor pin
    byte     led_pin;                               // Led PWM pin
    uint32_t checkMS;                               // Time in ms when the sensor was checked
    uint32_t ch_step;                               // The time in ms when the brighthess can be adjusted
    bool     automatic;                             // Whether the backlight should be adjusted automatically
    bool     use_local_time;                        // Whether to use local time to switch off the light nightly
    byte     brightness;                            // The backlight brightness
    byte     new_brightness;                        // The baclight brightness to set up
    byte     evening, morning;                      // The time of evening and morning (in 10-minutes interval)
    long     emp;                                   // Exponential average value
    byte     default_brightness;                    // Default brightness of backlight
    uint16_t b_night;                               // light sensor value of the night
    uint16_t b_day;                                 // light sensor value of the day light
    byte     daily_brightness;                      // The maximum brightness of backlight when light between b_night and b_day
    byte     nightly_brightness;                    // The brightness to use nightly
    const byte     emp_k = 8;                       // The exponential average coefficient
    const uint16_t period  = 200;                   // The period in ms to check the photeregister
    const uint16_t ch_period = 5;                   // The period to adjust brightness
};

void BL::init(void) {

  pinMode(led_pin, OUTPUT);
  pinMode(sensor_pin, INPUT);
  int light = analogRead(sensor_pin);
  emp = 0;
  brightness = new_brightness = default_brightness;
  checkMS = ch_step = 0;
  use_local_time = false;
  automatic = true;
  nightly_brightness = 50;
  evening = morning = 0;                            // This value will be overwritten by config
  adjust();
}

int BL::empAverage(int v) {
  long nv = v *emp_k;
  int round_v = emp_k >> 1;
  emp += (nv - emp + round_v) / emp_k;
  int r = (emp + round_v) / emp_k;
  return r;
}
  
void BL::adjust(void) {

  if (!automatic) return;
  
  uint32_t ms = millis();
  if ((ms > ch_step) && (new_brightness != brightness)) {
    if (new_brightness > brightness) ++brightness; else --brightness;
    analogWrite(led_pin, brightness);
    ch_step = ms + ch_period;
  }

  if (ms < checkMS) return;
  checkMS = ms + period;

  // Turn off the backlight at night
  if (isDark()) {
    new_brightness = nightly_brightness;
    return;
  }
  
  int light = analogRead(sensor_pin);

  light = empAverage(light);
  if (light < b_night) {
    new_brightness = nightly_brightness;
    return;
  }

  if (light > b_day) {
    new_brightness = 0;
    return;
  }

  new_brightness = map(light, b_night, b_day, nightly_brightness, daily_brightness);
}

void BL::setBrightness(byte b) {
  brightness = b;
  automatic = false;
  analogWrite(led_pin, brightness);
}

void BL::turnAuto(bool a) {
  automatic = a;
  checkMS = 0;
  if (a) adjust();    
}

void BL::setLimits(uint16_t dark, uint16_t daylight, byte br_nightly, byte br_daily) {
  b_night             = dark;
  b_day               = daylight;
  daily_brightness    = br_daily;
  nightly_brightness  = br_nightly;
}

void BL::setNightPeriod(byte Evening, byte Morning) { // Time in 10-minute intervals from midnight
  if (Evening <= Morning) return;
  if (Evening > 144)    return;
  morning = Morning;
  evening = Evening;
  use_local_time = true;
}

bool BL::isDark(void) {

  if (use_local_time) {
    byte now_t = hour() * 6 + (minute() + 5) /10; // Time in 10-minute intervals from midnight
    return ((now_t <= morning) || (now_t >= evening));
  }

  long light = 0;
  for (byte i = 0; i < 4; ++i) {
    light += analogRead(sensor_pin);
    delay(20);
  }
  light >>= 2;
  return (light < b_night);
}

//------------------------------------------ class BUTTON ------------------------------------------------------
class BUTTON {
  public:
    BUTTON(byte ButtonPIN, unsigned int timeout_ms = 3000) {
      pt = tickTime = 0;
      buttonPIN = ButtonPIN;
      overPress = timeout_ms;
    }
    void init(void) { pinMode(buttonPIN, INPUT_PULLUP); }
    void setTimeout(uint16_t timeout_ms = 3000)    { overPress = timeout_ms; }
    byte intButtonStatus(void)                     { byte m = mode; mode = 0; return m; }
    void cnangeINTR(void);
    byte buttonCheck(void);
    bool buttonTick(void);
  private:
    volatile byte     mode;                         // The button mode: 0 - not pressed, 1 - pressed, 2 - long pressed
    uint16_t          overPress;                    // Maxumum time in ms the button can be pressed
    volatile uint32_t pt;                           // Time in ms when the button was pressed (press time)
    uint32_t          tickTime;                     // The time in ms when the button Tick was set
    byte              buttonPIN;                    // The pin number connected to the button
    const uint16_t    tickTimeout = 200;            // Period of button tick, while tha button is pressed 
    const uint16_t    shortPress = 900;             // If the button was pressed less that this timeout, we assume the short button press
};

void BUTTON::cnangeINTR(void) {                     // Interrupt function, called when the button status changed
  
  bool keyUp = digitalRead(buttonPIN);
  unsigned long now_t = millis();
  if (!keyUp) {                                     // The button has been pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t; 
  } else {
    if (pt > 0) {
      if ((now_t - pt) < shortPress) mode = 1;      // short press
        else mode = 2;                              // long press
      pt = 0;
    }
  }
}

byte BUTTON::buttonCheck(void) {                    // Check the button state, called each time in the main loop

  mode = 0;
  bool keyUp = digitalRead(buttonPIN);              // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp) {                                     // The button is pressed
    if ((pt == 0) || (now_t - pt > overPress)) pt = now_t;
  } else {
    if (pt == 0) return 0;
    if ((now_t - pt) > shortPress)                  // Long press
      mode = 2;
    else
      mode = 1;
    pt = 0;
  } 
  return mode;
}

bool BUTTON::buttonTick(void) {                     // When the button pressed for a while, generate periodical ticks

  bool keyUp = digitalRead(buttonPIN);              // Read the current state of the button
  uint32_t now_t = millis();
  if (!keyUp && (now_t - pt > shortPress)) {        // The button have been pressed for a while
    if (now_t - tickTime > tickTimeout) {
       tickTime = now_t;
       return (pt != 0);
    }
  } else {
    if (pt == 0) return false;
    tickTime = 0;
  } 
  return false;
}

//------------------------------------------ class ENCODER ------------------------------------------------------
class ENCODER {
  public:
    ENCODER(byte aPIN, byte bPIN, int16_t initPos = 0) {
      pt = 0; mPIN = aPIN; sPIN = bPIN; pos = initPos;
      min_pos = -32767; max_pos = 32766; channelB = false; increment = 1;
      changed = 0;
      is_looped = false;
    }
    void init(void) {
      pinMode(mPIN, INPUT_PULLUP);
      pinMode(sPIN, INPUT_PULLUP);
    }
    void    set_increment(byte inc)             { increment = inc; }
    byte    get_increment(void)                 { return increment; }
    int16_t read(void)                          { return pos; }
    void    reset(int16_t initPos, int16_t low, int16_t upp, byte inc = 1, byte fast_inc = 0, bool looped = false);
    bool    write(int16_t initPos);
    void    cnangeINTR(void);
  private:
    int32_t           min_pos, max_pos;
    volatile uint32_t pt;                           // Time in ms when the encoder was rotaded
    volatile uint32_t changed;                      // Time in ms when the value was changed
    volatile bool     channelB;
    volatile int16_t  pos;                          // Encoder current position
    byte              mPIN, sPIN;                   // The pin numbers connected to the main channel and to the socondary channel
    bool              is_looped;                    // Whether the encoder is looped
    byte              increment;                    // The value to add or substract for each encoder tick
    byte              fast_increment;               // The value to change encoder when in runs quickly
    const uint16_t    fast_timeout = 300;           // Time in ms to change encodeq quickly
    const uint16_t    overPress = 1000;
};

bool ENCODER::write(int16_t initPos) {
  if ((initPos >= min_pos) && (initPos <= max_pos)) {
    pos = initPos;
    return true;
  }
  return false;
}

void ENCODER::reset(int16_t initPos, int16_t low, int16_t upp, byte inc, byte fast_inc, bool looped) {
  min_pos = low; max_pos = upp;
  if (!write(initPos)) initPos = min_pos;
  increment = fast_increment = inc;
  if (fast_inc > increment) fast_increment = fast_inc;
  is_looped = looped;
}

void ENCODER::cnangeINTR(void) {                    // Interrupt function, called when the channel A of encoder changed
  
  bool rUp = digitalRead(mPIN);
  unsigned long now_t = millis();
  if (!rUp) {                                       // The channel A has been "pressed"
    if ((pt == 0) || (now_t - pt > overPress)) {
      pt = now_t;
      channelB = digitalRead(sPIN);
    }
  } else {
    if (pt > 0) {
      byte inc = increment;
      if ((now_t - pt) < overPress) {
        if ((now_t - changed) < fast_timeout) inc = fast_increment;
        changed = now_t;
        if (channelB) pos -= inc; else pos += inc;
        if (pos > max_pos) { 
          if (is_looped)
            pos = min_pos;
          else 
            pos = max_pos;
        }
        if (pos < min_pos) {
          if (is_looped)
            pos = max_pos;
          else
            pos = min_pos;
        }
      }
      pt = 0; 
    }
  }
}

//------------------------------------------ class clockArm ------------------------------------------------------
class clockArm {
  public:
    clockArm(void) {}
    void  setHour(byte Hour, byte Minute);
    void  setMinute(byte Minute);
    char  vect(char X, char ti);
    byte  armX(byte cx, char thickIndex, bool HourArm) {
      char dy = vect(dY, thickIndex);
      if (HourArm) {
        char dx = dX;
        dx -= dx>>2;
        return (char)cx + dx - dy; 
      } else {
        return (char)cx + dX - dy;
      }
    }
    byte armY(byte cy, char thickIndex, bool HourArm) {
      char dx = vect(dX, thickIndex);
      if (HourArm) {
        char dy = dY;
        dy -= dy>>2;
        return (char)cy + dy + dx;
      } else {
        return (char)cy + dY + dx;
      }
    }
    byte tailX(byte cx, char thickIndex) {
      char dy = vect(dY, thickIndex);
      char dx = dX;
      dx /= 5;
      return (char)cx - dx - dy; 
    }
    byte tailY(byte cy, char thickIndex) {
      char dx = vect(dX, thickIndex);
      char dy = dY;
      dy /= 5;
      return (char)cy - dy + dx; 
    }
    byte deltaX(void) { return dX; }
    byte deltaY(void) { return dY; }
  private:
    byte quadrant;
    char dX, dY;                                    // Coordinates of Minute or Hour arms
    const byte cArm[16][2] = {
      {0, 20}, {2, 20}, {4, 19}, {6, 19}, {8, 18}, {10, 17}, {12, 16},
      {13, 15}, {15, 14}, {16, 12}, {17, 10}, {18, 8}, {19, 6}, {19, 4}, {20, 2}, {20, 0}
    };
};

char clockArm::vect(char X, char ti) {
  char x = 0;
  if (X < -14) x = -1;
  if (X > 14)  x =  1;
  return x*ti;
}

void clockArm::setHour(byte Hour, byte Minute) {
  Hour %= 12;
  setMinute(Hour * 5 + Minute / 12);
}

void clockArm::setMinute(byte Minute) {
  byte angle;
  
  quadrant = Minute / 15;
  angle    = Minute % 15;

  switch (quadrant) {
    case 0:
      dX = cArm[angle][0];
      dY = cArm[angle][1];
      dY *= -1;
      break;

    case 1:
      dX = cArm[15 - angle][0];
      dY = cArm[15 - angle][1];
      break;

    case 2:
      dX = cArm[angle][0];
      dX *= -1;
      dY = cArm[angle][1];
      break;

    case 3:
      dX = cArm[15 - angle][0];
      dX *= -1;
      dY = cArm[15 - angle][1];
      dY *= -1;
      break;
  }
}

//------------------------------------------ class HISTORY ----------------------------------------------------
#define H_LENGTH 30
class HISTORY {
  public:
    HISTORY(void)                                   { len = 0; }
    void  init(void)                                { len = 0; }
    void  put(int item);
    bool  isFull(void)                              { return len == H_LENGTH; }
    int   last(void)                                { return queue[len-1]; }
    int   top(void)                                 { return queue[0]; }
    int   average(void);
    float dispersion(void);
    float gradient(byte last_n = H_LENGTH);
  private:
    int queue[H_LENGTH];
    byte len;
};

void HISTORY::put(int item) {
  if (len < H_LENGTH) {
    queue[len++] = item;
  } else {
    for (byte i = 0; i < len-1; ++i) queue[i] = queue[i+1];
    queue[H_LENGTH-1] = item;
  }
}

int HISTORY::average(void) {
  long sum = 0;
  if (len == 0) return 0;
  if (len == 1) return queue[0];
  for (byte i = 0; i < len; ++i) sum += queue[i];
  sum += len >> 1;                                  // round the average
  sum /= len;
  return (int)sum;
}

float HISTORY::dispersion(void) {
  if (len < 3) return 1000;
  long sum = 0;
  long avg = average();
  for (byte i = 0; i < len; ++i) {
    long q = queue[i];
    q -= avg;
    q *= q;
    sum += q;
  }
  sum += len << 1;
  float d = (float)sum / (float)len;
  return d;
}

/* approfimating the last elements of the history with the line (y = Ax+B) using method of minimum squares.
 * The gradient is parameter A
*/
float HISTORY::gradient(byte last_n) {
  if (last_n > H_LENGTH) last_n = H_LENGTH;
  if (len < 2) return 0;
  long sx, sx_sq, sxy, sy;
  sx = sx_sq = sxy = sy = 0;
  byte si = 1;
  byte a_len = len;
  if (len > last_n) {
    si = len - last_n + 1;
    a_len = last_n;
  }
  for (byte i = si; i <= len; ++i) {
    sx    += i;
    sx_sq += i*i;
    sxy   += i*queue[i-1];
    sy    += queue[i-1];
  }

  long numerator   = a_len * sxy - sx * sy;
  long denominator = a_len * sx_sq - sx * sx;
  float a = (float)numerator / (float)denominator;
  return a;
}

//------------------------------------------ class lcd DSPLay for weayther station ------------------------------------
class DSPL : public U8G2_ST7920_128X64_F_SW_SPI {
  public:
    DSPL(byte sck, byte rw, byte rs, byte rst) : U8G2_ST7920_128X64_F_SW_SPI(U8G2_R2, sck, rw, rs, rst) {
    }
    void showForecast(byte f);                      // Show the bitmap for the weather forecast
    void showPressure(int pressure);                // Show the symbol pressure + pressure value
    void showTempSign(byte x, byte y);              // Show the temperature symbol
    void showHummSign(byte x, byte y);              // Show the humidity symbol
	  void showPressureSign(byte x, byte y);          // Show the humidity symbol
    void showCentigrade(byte x, byte y);            // Show the sentigrade sign in the position (x; y)
    void showPercent(byte x, byte y);               // Show the percentage sign in the position (x; y)
    void showBattery(byte x, byte y);               // Show the empty battery sign in the position (x; y)
	  void showVarrow(byte x, byte y);                // Show the vertical arrow
    void showClockSetup(byte x, byte y);            // Show the icon indicating the clock setup process
    void showMenuMark(byte x, byte y);              // Show the menu mark
    enum fcast {SUNNY = 0, CLOUDLY = 1, RAIN = 2, MOONY = 3, SNOW = 4};
  private:
    int  myLrint(float a)                          { return int(a+0.5); }
    const uint8_t bmPressure[16]= {
      0b00011000,
      0b00100100,
      0b00100100,
      0b01100100,
      0b00100100,
      0b00100100,
      0b00100100,
      0b01100100,
      0b00100100,
      0b00100100,
      0b00100100,
      0b11111111,
      0b11111111,
      0b11111111,
      0b11111111,
      0b01111110  
    };
    const uint8_t bmCentigrade[8] = {
      0b01100000,
      0b10010000,
      0b10010000,
      0b01100000,
      0b00000000,
      0b00000000,
      0b00000000,
      0b00000000
    };
    const uint8_t bmPercent[8] = {
      0b01100001,
      0b10010010,
      0b10010100,
      0b01101000,
      0b00010110,
      0b00101001,
      0b01001001,
      0b10000110
    };
    const uint8_t bmTemperature[16] = {
      0b00010000,
      0b00101000,
      0b01101000,
      0b00101000,
      0b01101000,
      0b00111000,
      0b01111000,
      0b00111000,
      0b01111000,
      0b00111000,
      0b01111100,
      0b11111110,
      0b11111110,
      0b11111110,
      0b01111100,
      0b00111000  
    };
    const uint8_t bmHumidity[16] = {
      0b00000000,
      0b00100000,
      0b01100000,
      0b11110000,
      0b11110000,
      0b01100010,
      0b00000110,
      0b00001111,
      0b00001111,
      0b00000110,
      0b00000000,
      0b00010000,
      0b00110000,
      0b01111000,
      0b01111000,
      0b00110000  
    };
	  const uint8_t bmVarrow[4] = {
      0b01110000,
      0b01110000,
      0b11111000,
      0b10101000
    };
    const uint8_t bmClock[8] = {
      0b00111100,
      0b01000110,
      0b10001001,
      0b10010001,
      0b10011101,
      0b10000001,
      0b01000010,
      0b00111100
    };
    const uint8_t bmMenuMark[5] = {
      0b00010000,
      0b00011000,
      0b00111100,
      0b00111100,
      0b00011000
    };
    const uint8_t bmBattery[14] = {
      0b01111111, 0b11111000,
      0b10000000, 0b00000100,
      0b10001000, 0b01000111,
      0b10001000, 0b01000101,
      0b10001000, 0b01000111,
      0b10000000, 0b00000100,
      0b01111111, 0b11111000,
    };
    const uint8_t bmSunny[64] = {
      0b10000000, 0b01100001, 0b00000001, 0b00000100,
      0b11000000, 0b00100001, 0b00000010, 0b00001000,
      0b00110000, 0b00010001, 0b00000010, 0b00011000,
      0b00001100, 0b00001000, 0b10000010, 0b00010000,
      0b00000110, 0b00001100, 0b10110100, 0b00100000,
      0b00000001, 0b10000100, 0b11111110, 0b01000000,
      0b00000000, 0b01110011, 0b11111111, 0b10000000,
      0b10000000, 0b00001111, 0b11111111, 0b11000011,
      0b01111000, 0b00001111, 0b11111111, 0b11001100,
      0b00000111, 0b11111111, 0b11111111, 0b11110000,
      0b00000000, 0b00111111, 0b11111111, 0b11100000,
      0b00000000, 0b00011111, 0b11111111, 0b11100000,
      0b00000000, 0b01111111, 0b11111111, 0b11111000,
      0b00000011, 0b11001111, 0b11111111, 0b11100111,
      0b00111100, 0b00001111, 0b11111111, 0b11100000,
      0b11000000, 0b00000111, 0b11111111, 0b11000000
    };
    const uint8_t bmCloudly[64] = {
      0b00001000, 0b00000001, 0b00000000, 0b00001000,
      0b00000100, 0b00000001, 0b10000000, 0b00010000,
      0b00000011, 0b00000000, 0b10000000, 0b00100000,
      0b00000001, 0b10000000, 0b10000000, 0b01000000,
      0b00000000, 0b11000000, 0b10111000, 0b10000000,
      0b00000000, 0b00100011, 0b11111111, 0b00000000,
      0b11100000, 0b00011111, 0b11111110, 0b00000000,
      0b00111000, 0b00011111, 0b10011110, 0b11000000,
      0b00000110, 0b00111111, 0b01100001, 0b11011100,
      0b00000001, 0b11111110, 0b01111111, 0b11101110,
      0b00111100, 0b01111000, 0b11111111, 0b11101110,
      0b01111110, 0b01101011, 0b11111111, 0b01110111,
      0b11111111, 0b10000110, 0b11111111, 0b10101011,
      0b11111111, 0b11111111, 0b11111111, 0b11011101,
      0b11111111, 0b11001111, 0b11111111, 0b11101110,
      0b11111111, 0b11111111, 0b11111111, 0b11101111
    };
    const uint8_t bmRain[64] = {
      0b00000000, 0b00000000, 0b00010000, 0b00000000,
      0b00000000, 0b00000000, 0b00110000, 0b00000000,
      0b00000000, 0b00000000, 0b01111000, 0b01000000,
      0b00000000, 0b00001000, 0b01111000, 0b11000000,
      0b00010000, 0b00011000, 0b00110001, 0b11100000,
      0b00110000, 0b00111100, 0b00000001, 0b11100000,
      0b01111000, 0b00111100, 0b00000000, 0b11000000,
      0b01111000, 0b00011000, 0b00000000, 0b00000000,
      0b00110000, 0b00000000, 0b00000000, 0b00010000,
      0b00000000, 0b00000000, 0b00000010, 0b00110000,
      0b00000000, 0b00100000, 0b00000110, 0b01111000,
      0b00100000, 0b01100000, 0b00001111, 0b01111000,
      0b01100000, 0b11110000, 0b00001111, 0b00110000,
      0b11110000, 0b11110000, 0b00000110, 0b00000000,
      0b11110000, 0b01100000, 0b00000000, 0b00000000,
      0b01100000, 0b00000000, 0b00000000, 0b00000000
    };
    const uint8_t bmSnow[64] = {
      0b00000000, 0b00100100, 0b10000101, 0b00000000,
      0b00001000, 0b00110101, 0b10011010, 0b11000000,
      0b01001001, 0b00001010, 0b00010010, 0b01000000,
      0b01101011, 0b00110101, 0b10000010, 0b00000000,
      0b00010100, 0b00100100, 0b10000000, 0b00000000,
      0b01101011, 0b00000100, 0b00000000, 0b01000000,
      0b01001001, 0b00000000, 0b00000010, 0b01001000,
      0b00001000, 0b00000000, 0b10000011, 0b01011000,
      0b00000000, 0b00000100, 0b10010000, 0b10100000,
      0b00000001, 0b00000110, 0b10110011, 0b01011000,
      0b00001001, 0b00100001, 0b01000010, 0b01001000,
      0b00001101, 0b01100110, 0b10110000, 0b01000000,
      0b00000010, 0b10000100, 0b10010000, 0b00000000,
      0b00001101, 0b01100000, 0b10000100, 0b00000000,
      0b00001001, 0b00100000, 0b00100100, 0b10000000,
      0b00000001, 0b00000000, 0b00110101, 0b10000000
    };
    const uint8_t bmMoony[64] = {
      0b00000000, 0b10000000, 0b00000000, 0b00000000,
      0b00000000, 0b10000000, 0b11111000, 0b00000000,
      0b00000000, 0b10000011, 0b11110000, 0b00000000,
      0b00000111, 0b01100011, 0b11100000, 0b00010000,
      0b00000000, 0b10000111, 0b11100000, 0b00010000,
      0b00000000, 0b10001111, 0b11000000, 0b00010000,
      0b00000100, 0b00001111, 0b11000000, 0b11101100,
      0b00000100, 0b00001111, 0b11000000, 0b00010000,
      0b00000100, 0b00001111, 0b11000000, 0b00010000,
      0b00111011, 0b00001111, 0b11000000, 0b00000000,
      0b00000100, 0b00001111, 0b11100000, 0b00000100,
      0b00000100, 0b00000111, 0b11100000, 0b00000100,
      0b00000000, 0b00000011, 0b11110000, 0b00000100,
      0b00000000, 0b00000000, 0b11111000, 0b00011011,
      0b00000000, 0b00000000, 0b00000000, 0b00000100,
      0b00000000, 0b00000000, 0b00000000, 0b00000100,
    };
};

void DSPL::showForecast(byte f) {
  const byte x = 95;
  const byte y = 0;
  const byte* bmp = bmSunny;
  switch (f) {
    case CLOUDLY:
      bmp = bmCloudly;
      break;
    case RAIN:
      bmp = bmRain;
      break;
    case MOONY:
      bmp = bmMoony;
      break;
    case SNOW:
      bmp = bmSnow;
      break;
  }
  drawBitmap(x, y, 4, 16, bmp);
}

void DSPL::showPressure(int pressure) {
  const byte x = 0;
  const byte y = 0;
  showPressureSign(x, y);
  byte pr = 6;
  if (pressure > high_pressure)
    pr = 10;
  else
    if (pressure < low_pressure) pr = 3;
  drawBox(x+3, y+13-pr, x+2, y+pr);
  char buff[4];
  sprintf(buff, "%3d", pressure);
  setFont(RUS_FONT_S);
  drawStr(x+10, y+16-2, buff);
}

void DSPL::showTempSign(byte x, byte y) {
  drawBitmap(x, y, 1, 16, bmTemperature);
}

void DSPL::showHummSign(byte x, byte y) {
  drawBitmap(x, y, 1, 16, bmHumidity);
}

void DSPL::showPressureSign(byte x, byte y) {
  drawBitmap(x, y, 1, 16, bmPressure);
}

void DSPL::showCentigrade(byte x, byte y) {
  drawBitmap(x, y, 1, 8, bmCentigrade);
}

void DSPL::showPercent(byte x, byte y) {
  drawBitmap(x, y, 1, 8, bmPercent);
}

void DSPL::showBattery(byte x, byte y) {
  drawBitmap(x, y, 2, 7, bmBattery);
}

void DSPL::showVarrow(byte x, byte y) {
  drawBitmap(x, y, 1, 4, bmVarrow);
}

void DSPL::showClockSetup(byte x, byte y) {
  drawBitmap(x, y, 1, 8, bmClock);
}

void DSPL::showMenuMark(byte x, byte y) {
  drawBitmap(x, y, 1, 5, bmMenuMark);
}

//------------------------------------------ class weatherLogger ----------------------------------------------------
class weatherLogger {
  public:
    weatherLogger(void) { canWriteSD = false; }
    void   init(byte sd_pin);
    char*  logName(time_t date=0);                  // Generate log file name based on date
    bool   writeData(byte sensorID, int t, int p, byte h);
    bool   openLog(time_t date=0);
    void   closeLog(void);
    bool   readData(byte &sensorID, int &t, int &p, byte &h, time_t &timestamp);
    void   rmOldLog(time_t date);                   // Remove the log files created before the date
	time_t calculateWriteLogTime(time_t start_time);// Write log at 0, 15, 30 and 45 minutes every hour
  private:
    SdFat sd;
    void long2byte(byte* buff, long data, byte dsize = 4);
	  long byte2long(byte* buff, byte dsize = 4);
	  long checksum(byte *buff, byte bsize = 12);
    bool canWriteSD;
    char log_file_name[13];
    SdFile readLog;
};

void weatherLogger::init(byte sd_pin) {
   canWriteSD = sd.begin(sd_pin, SPI_HALF_SPEED);
}

char* weatherLogger::logName(time_t date) {
 
  if (date == 0) date = now();
  sprintf(log_file_name, "%04d%02d%02d.dat", year(date), month(date), day(date));
  return log_file_name;
}

bool weatherLogger::writeData(byte sensorID, int t, int p, byte h) {

  if (!canWriteSD) return false;
  logName();
  byte record[16];
  time_t now_t = now();
  long2byte(record, now_t);                         // the record begins with timestamp, 4 bytes
  long2byte(&record[4], t, 2);                      // the temperature, 2 bytes
  long2byte(&record[6], p, 2);                      // the pressure, 2 bytes
  record[8] = h;                                    // the humidity, a single byte
  record[9] = sensorID;
  record[10] = record[11] = 0;                      // Skip 10-th and 11-th bytes
  long crc = checksum(record);
  long2byte(&record[12], crc);                      // the checksum, 4 bytes

  SdFile lf;
  if (lf.open(log_file_name, O_RDWR | O_CREAT | O_AT_END )) {
    byte written = lf.write(record, 16);
    lf.close();
  }
}

bool weatherLogger::readData(byte &sensorID, int &t, int &p, byte &h, time_t &timestamp) {

  byte record[16];
  int readb = readLog.read(record, 16);
  if (readb != 16) {
    closeLog();
    return false;
  }
  long crc = checksum(record);
  if (crc == byte2long(&record[12])) {              // Checksumm is correct
    timestamp = byte2long(record);
    t = byte2long(&record[4], 2);
	  p = byte2long(&record[6], 2);
	  h = record[8];
	  sensorID = record[9];
    return true;
  }
  return false;
}

bool weatherLogger::openLog(time_t date) {
  
  logName(date);
  if (!readLog.open(log_file_name, O_READ))
    return false;
  return true;
}

void weatherLogger::closeLog(void) {
  readLog.close();
}

void weatherLogger::long2byte(byte* buff, long data, byte dsize) {
  for (byte i = 0; i < dsize; ++i) {
    buff[i] = data & 0xff;
	  data >>= 8;
  }
}

long weatherLogger::byte2long(byte* buff, byte dsize) {
  long data = 0;
  for (char i = dsize-1; i >= 0; --i) {
    data <<= 8;
	  data |= buff[i];
  }
  return data;
}

long weatherLogger::checksum(byte *buff, byte bsize) {
  long crc = 0;
  for (byte i = 0; i < bsize; ++i) {
    crc <<= 2;
	  crc += buff[i];
  }
  return crc;
}

void weatherLogger::rmOldLog(time_t date) {
  tmElements_t tm;
  SdFile file;
  char f_name[13];

  sd.vwd()->rewind();
  while (file.openNext(sd.vwd(), O_READ)) {
    if (file.isDir()) {
      file.close();
      continue;
    }
    dir_t dir;
    file.dirEntry(&dir);
    memcpy(f_name, dir.name, 8); f_name[8] = '.';
    memcpy(&f_name[9], &dir.name[8], 3); f_name[12] = '\0';
    // Calculate last modification time of the file in date format
    uint16_t f_time = dir.lastWriteTime;
    tm.Second = FAT_SECOND(f_time);
    tm.Minute = FAT_MINUTE(f_time);
    tm.Hour   = FAT_HOUR(f_time);
    f_time    = dir.lastWriteDate;
    tm.Day    = FAT_DAY(f_time);
    tm.Month  = FAT_MONTH(f_time);
    tm.Year   = FAT_YEAR(f_time) - 1970;
    file.close();
    time_t f_date = makeTime(tm);
    if (f_date < date) {
      sd.remove(f_name);
    }
  }
}

time_t weatherLogger::calculateWriteLogTime(time_t start_time) {
  const byte period = 15;

  byte Sec = second(start_time);
  start_time -= Sec;
  byte Min = minute(start_time);
  Min %= period;
  Min = period - Min;
  return start_time + ((uint32_t)Min * 60);
}

//------------------------------------------ class pressureSensor -----------------------------------------------------
class pressureSensor {
  public:
    pressureSensor(Adafruit_BMP085_Unified* Bmp) {
      pBmp     = Bmp;
    }
    void update(void);
    byte forecast(bool is_cold = false, bool is_dark = false);
    int  press(void)                                { return hPress.last(); }
    int  average(void)                              { return hPress.average(); }
    enum fcast { SUNNY = 0, CLOUDLY = 1, RAIN = 2, MOONY = 3, SNOW = 4 }; 
  private:
    Adafruit_BMP085_Unified* pBmp;                  // Pointer to the BMP sensor instance
    HISTORY hPress;                                 // History data for temperature, pressure and humidity
};

void pressureSensor::update(void) {
  sensors_event_t event;
  pBmp->getEvent(&event);
  if (event.pressure) {
    int pressure = int(event.pressure * 0.750064);  // The pressure in mmHg
    hPress.put(pressure);
  }
}

byte pressureSensor::forecast(bool is_cold , bool is_dark) {
  int grad_short = int(hPress.gradient(10) * 100.0 + 0.5);
  int grad_full  = int(hPress.gradient()   * 100.0 + 0.5);
  int pressure = hPress.last();
  if (pressure > high_pressure) {                   // HIGH pressure
    if (grad_short < -8)
      return CLOUDLY;
    else {
      if (grad_full < -8)
        return CLOUDLY;
      if (is_dark)
        return MOONY;
      else
        return SUNNY;
    }
  }
  if (pressure < low_pressure) {                    // LOW pressure
    if (grad_full <= 0) {
      if (is_cold)
        return SNOW;
      else
        return RAIN;
    } else {
      if (grad_short <= 8) {
        return CLOUDLY;
      } else {
        if (is_dark)
          return MOONY;
        else
          return SUNNY;
      }
    }
  }
  if ((grad_short >= -5) && (grad_full >= 0)) {     // NORMAL pressure
    if (is_dark)
      return MOONY;
    else
      return SUNNY;
  } else {
    if (grad_short <= -8) {
      if (is_cold)
        return SNOW;
      else
        return RAIN;
    } else
      return CLOUDLY;
  }
}

//------------------------------------------ class weatherSensor ------------------------------------------------------
class weatherSensor {
  public:
    weatherSensor(void)                             {}
    void   init(void);
    void   update(int temp, byte humm, bool batt);  // Temperature in 10*cencegrees [-500; +500]
    int    temp(void)                               { return hTemp.last(); }
    byte   humm(void)                               { return hHumm.last(); }
    int    aTemp(void)                              { return hTemp.average(); }
    byte   aHumm(void)                              { return hHumm.average(); }
    bool   isBattOK(void)                           { return battOK; }
    time_t lastUpdated(void)                        { return updated; }

  private:
    bool     battOK;                                // Whether the battery of the sensor is OK
    time_t   updated;                               // The time when the sensor data was updated
    HISTORY hTemp, hHumm;                           // History data for temperature and humidity
};

void weatherSensor::init(void) {
  battOK   = false;
  updated  = 0;
  hTemp.init();
  hHumm.init();
}

void weatherSensor::update(int temp, byte humm, bool batt) {
  if (temp < -500) temp = -500;                     // lower temperature limit -50.0 cencegrees
  if (temp >  500) temp =  500;                     // upper temperature limit +50.0 cencegrees
  hTemp.put(temp);
  hHumm.put(humm);
  battOK  = batt;
  updated = now();
}

//------------------------------------------ class sensorPool ---------------------------------------------------------
class sensorPool {
  public:
    sensorPool(void)                               { }
    void  init(void);
    void  update(byte ID, int temp, byte humm, bool batt);
    byte  next(byte ID);                            // Next Sensor ID from the pool
	  byte  previous(byte ID);                        // Previous Sensor ID from the pool
    bool  exists(byte indx);                        // Whether the sensor with ID exists
    void  freeStaled(void);                         // Free staled sensor (the one not updated for a long time)
    byte  numSensors(void);                         /// Number of the sensors, registered in the pool
    int   temp(byte ID);                            // The temperature of the sensor ID
    byte  humm(byte ID);                            // The humidity of the sensor ID
    int   aTemp(byte ID);                           // The average temperature of the sensor ID
    byte  aHumm(byte ID);                           // The average humidity of the sensor ID
    bool  isBattOK(byte ID);                        // Batery status of the sensor ID
  private:
    byte  registerSensor(byte ID);                  // Return sensor index of new registered sensor
    byte  index(byte ID);
    weatherSensor   s[max_sensors];                 // The sensors array
    byte            s_id[max_sensors];              // The sensor ID array, The internal sensor has ID = 0
    bool            in_use[max_sensors];            // Whether this sensor is currently in use
    const time_t    stale_time = 1800;              // timeout for the staled sensor data
};

void sensorPool::init(void) {
  for (byte i = 0; i < max_sensors; ++i) {
    s[i].init();
    in_use[i] = false;
  }
  // Register internal sensor with ID = 0
  s_id[0]   = 0;
  in_use[0] = true; 
}

void sensorPool::update(byte ID, int temp, byte humm, bool batt) {
  byte indx = index(ID);
  if (indx >= max_sensors) {                        // The sensor is not in the list
    indx = registerSensor(ID);
    if (indx >= max_sensors) return;                // Failed to register new sensor, pool is full
  }
  s[indx].update(temp, humm, batt);
}

byte sensorPool::next(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return s_id[0];          // The sensor not found or this sensor is the last i the pool, return the 0-th sensor ID (internal one)
  ++indx;                                           // Start from the next sensor in the pool
  for (; indx < max_sensors; ++indx) {              // Check whether this sensor is in use
    if (in_use[indx]) return s_id[indx];
  }
  return s_id[0];
}

byte sensorPool::previous(byte ID) {
  char indx = index(ID);
  if (indx >= max_sensors) return s_id[0];          // The sensor not found or this sensor is the last i the pool, return the 0-th sensor ID (internal one)
  --indx;                                           // Start from the previous sensor in the pool
  if (indx < 0) indx = max_sensors-1;
  for (; indx >= 0; --indx) {                       // Check whether this sensor is in use
    if (in_use[indx]) return s_id[indx];
  }
  return s_id[0];
}

bool sensorPool::exists(byte ID) {
  byte indx = index(ID);
  return (indx < max_sensors);
}

void sensorPool::freeStaled(void) {
  time_t n = now();
  for (byte i = 0; i < max_sensors; ++i) {
    if (in_use[i]) {
      time_t lu = s[i].lastUpdated();
      if ((n - lu) >= stale_time) {                 // Mark the sensor free and clear the sensor data
        in_use[i] = false;
        s[i].init();
      }
    }
  }
}

byte sensorPool::numSensors(void) {
  byte n = 0;
  for (byte i = 0; i < max_sensors; ++i)
    if (in_use[i]) ++n;
  return n;
}

int sensorPool::temp(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].temp();
}

byte sensorPool::humm(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].humm();
}

int sensorPool::aTemp(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].aTemp();
}

byte sensorPool::aHumm(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return 0;                // No sensor registered
  return s[indx].aHumm();
}

bool sensorPool::isBattOK(byte ID) {
  byte indx = index(ID);
  if (indx >= max_sensors) return false;            // No sensor registered
  return s[indx].isBattOK();
}

byte sensorPool::registerSensor(byte ID) {          // Return sensor index
  for (byte i = 0; i < max_sensors; ++i) {
    if (!in_use[i]) {
      s_id[i]   = ID;
      in_use[i] = true; 
      return i;
    }
  }
  return max_sensors;
}

byte sensorPool::index(byte ID) {                   // return the index of the sensor [0; max_sensors -1] or max_sensors if not found
  byte i = 0;
  for (; i < max_sensors; ++i)
    if (in_use[i] && s_id[i] == ID) break;
  return i;
}

//------------------------------------------ class SCREEN ------------------------------------------------------
class SCREEN {
  public:
    SCREEN* next;                                   // Pointer to the next screen
    SCREEN* nextL;                                  // Pointer to the next Level screen, usually, setup
    SCREEN* main;                                   // Pointer to the main screen

    SCREEN() {
      next = nextL = main = 0;
      scr_timeout = 0;
      time_to_return = 0;
    }
    virtual void    init(void)                      { }
    virtual void    show(void)                      { }
    virtual SCREEN* menu(void)                      { if (this->next != 0) return this->next; else return this; }
    virtual SCREEN* menu_long(void)                 { if (this->nextL != 0) return this->nextL; else return this; }
    virtual void    rotaryValue(int16_t value)      { }
    bool            isSetup(void)                   { return (scr_timeout != 0); }
    void            forceRedraw(void)               { update_screen = 0; }
    SCREEN*         returnToMain(void);
    void resetTimeout(void) {
      if (scr_timeout > 0)
        time_to_return = millis() + scr_timeout*1000;
    }
    void setSCRtimeout(uint16_t t) {
      scr_timeout = t;
      resetTimeout(); 
    }
  protected:
    uint16_t scr_timeout;                             // Timeout is sec. to return to the main screen, canceling all changes
    uint32_t time_to_return;                          // Time in ms to return to main screen
    uint32_t update_screen;                           // Time in ms when the screen should be updated
};

SCREEN* SCREEN::returnToMain(void) {
  if (main && (scr_timeout != 0) && (millis() >= time_to_return)) {
    scr_timeout = 0;
    return main;
  }
  return this;
}

//------------------------------------------ class mainSCREEN ---------------------------------------------------------
class mainSCREEN : public SCREEN {
  public:
    mainSCREEN(DSPL* U8g, ENCODER* pROT, sensorPool* pSP, pressureSensor* pPS, sunMoon* pSM) {
      pD   = U8g;
	    pEn  = pROT;
      pSp  = pSP;
      pPs  = pPS;
      pSm  = pSM;
      main_sensor = 0;
      main_sensor_manual = false;
    }
    virtual void init(void);
    virtual void show(void);
    virtual void rotaryValue(int16_t value);
    void  setMainSensorID(byte id)                    { main_sensor = id; main_sensor_manual = true; }

  private:
    void  showClock(void);                            // Show analog or digital clock
    void  showExtSensor(void);                        // Show external sensor data (right side of the screen)
    bool  getMainSensorData(int& temp, byte& humidity);
    void  showInternalSensorData(void);
    bool  isDark(void);
    DSPL*            pD;                              // Pointer to the screen instance
	  ENCODER*         pEn;                             // Pointer to the encoder instance
    sensorPool*      pSp;                             // Pointer to the sensor pool instance
    sunMoon*         pSm;                             // Pointer to the sunMoon class instance
    pressureSensor*  pPs;                             // Pointer to the Pressure sensor based on BMP180
    byte             main_sensor;                     // The ID of the main external sensor
    bool             main_sensor_manual;              // Whether the main sensor ID was setup manually
    byte             ext_sensor;                      // external sensor ID to be displayed
    clockArm         cArm;
    const byte cx = 100;                              // The analog clock parameters: center X, Y coordinates and the radius
    const byte cy = 41;
    const byte cr = 22;
    const uint16_t  period = 15000;                   // The screen update period
};

void mainSCREEN::init(void) {
  update_screen = 0;
  ext_sensor = 0;
  pEn->reset(0, 0, 1, 1, 1, true);
}

void mainSCREEN::show(void) {
  
  uint32_t nowMS = millis();
  if (nowMS < update_screen) return;
  update_screen = nowMS + period;

  pD->clearBuffer();

  // Show the main sensor data
  int  t = 0;
  byte h = 0;
  if (getMainSensorData(t, h)) {
    char hbuff[6];
    pD->showHummSign(0, 63-16);
    pD->setFont(DIGIT_FONT);
    sprintf(hbuff, "%2d", h);
    byte hw = pD->getStrWidth(hbuff);

    char tbuff1[6];
    tbuff1[0] = ' ';
    if (t < 0) {
      t *= -1;
      tbuff1[0] = '-';
    }
    sprintf(&tbuff1[1], "%d", t / 10);
    byte tw1 = pD->getStrWidth(tbuff1);
    pD->setFont(RUS_FONT_L);
    char tbuff2[4];
    sprintf(tbuff2, ".%1d", t % 10);
    byte tw2 = pD->getStrWidth(tbuff2);

    byte pos = hw;
    if ((tw1 + tw2) > hw) pos = tw1 + tw2;
    pos += 10;
    if (pos > 60) pos = 60;                           // Maximum X position of the right side of digits
    
    pD->setFont(DIGIT_FONT);
    pD->drawStr(pos-hw, 63, hbuff);                   // Show the humidity value
    pD->drawStr(pos-tw1-tw2, 40, tbuff1);             // Show the integer part of the temperature
    pD->setFont(RUS_FONT_L);
    pD->drawStr(pos-tw2, 40, tbuff2);                 // Show the fractional part of the temperature
    pD->showCentigrade(pos+2, 40-14);
    pD->showPercent(pos+2, 63-14);
    if ((pos-tw1-tw2) > 8)
      pD->showTempSign(0, 40-16);
  }
  
  // Show the current pressure
  pD->showPressure(pPs->press());
  showInternalSensorData();

  // Show the Wheather forecast
  bool is_cold  = (pSp->temp(main_sensor) < 0);
  byte fcast = pPs->forecast(is_cold, isDark()); 
  pD->showForecast(fcast);

  if (ext_sensor == 0) {
    showClock();
  } else {
    while ((ext_sensor > 0) && !pSp->exists(ext_sensor))
      ext_sensor = pSp->next(ext_sensor);
    if (ext_sensor > 0)
      showExtSensor();
    else
      update_screen = 0;                              // Show nothing, refresh the screen
  }
  ext_sensor = pSp->next(ext_sensor);

  pD->sendBuffer();
}

void mainSCREEN::rotaryValue(int16_t value) {
  ext_sensor    = pSp->next(ext_sensor);
  update_screen = 0;
}

void mainSCREEN::showClock(void) {
    // Draw clock labels (at 12, 3, 6 and 9 o'clock)
    pD->drawLine(cx+cr-3, cy, cx+cr, cy);             // 3
    pD->drawLine(cx-cr, cy, cx-cr+3, cy);             // 9
    pD->drawLine(cx, cy+cr-3, cx, cy+cr);             // 6
    pD->drawLine(cx, cy-cr, cx, cy-cr+3);             // 12

    time_t now_t = RTC.get();                         // Synchronize the system clock
    setTime(now_t);
    byte Minute = minute();
    // Draw Hour Arm
    cArm.setHour(hour(), Minute);
    for (char i = -1; i < 2; ++i)
      pD->drawLine(cArm.tailX(cx, i), cArm.tailY(cy, i), cArm.armX(cx, i, true), cArm.armY(cy, i, true));
    // Draw Minute Arm Arm
    cArm.setMinute(Minute);
    for (char i = -1; i < 2; ++i)
      pD->drawLine(cArm.tailX(cx, i), cArm.tailY(cy, i), cArm.armX(cx, i, false), cArm.armY(cy, i, false));
}

void mainSCREEN::showExtSensor(void) {
  char buff[8];

  pD->setFont(TINY_FONT);
  sprintf(buff, "%X", ext_sensor);
  byte w_id = pD->getStrWidth(buff);
  pD->drawStr(125-w_id, 26, buff);

  pD->setFont(RUS_FONT_L);
  int  temp     = pSp->temp(ext_sensor);
  byte humidity = pSp->humm(ext_sensor);
  if (!pSp->isBattOK(ext_sensor)) pD->showBattery(80, 20);

  char *p = buff;
  if (temp < 0) {
    *p++ = '-';
    temp *= -1;
  }
  sprintf(p, "%d.%d", temp/10, temp%10);
  byte w = pD->getStrWidth(buff);
  pD->drawStr(124-8-w, 43, buff);
  pD->showCentigrade(126-8, 43-14);

  sprintf(buff, "%d", humidity);
  w = pD->getStrWidth(buff);
  pD->drawStr(124-10-w, 61, buff);
  pD->showPercent(124-8, 61-12);
  pD->drawRFrame(73, 18, 128-73, 64-18, 3);
}

bool mainSCREEN::getMainSensorData(int& temp, byte& humidity) {
  if (main_sensor == 0) {                             // Main sensor ID is not set, select the sensor from the pool
    byte n = pSp->numSensors();
    if (n < 2) return false;                          // Sensor with ID = 0 is the local one
    for (byte i = 0; i < n; ++i) {
      main_sensor = pSp->next(main_sensor);
      if (main_sensor > 0) break;
    }
  }
  if (pSp->exists(main_sensor)) {
    temp     = pSp->temp(main_sensor);
    humidity = pSp->humm(main_sensor);
    return true;
  } else {
    if (!main_sensor_manual)
      main_sensor = 0;
  }
  return false;
}

void mainSCREEN::showInternalSensorData(void) {
  if (pSp->exists(0)) {
    char buff[8];
    int  temp = pSp->temp(0);
    byte humm = pSp->humm(0);
    char *p = buff;
    if (temp < 0) {
      *p++ = '-';
      temp *= -1;
    }
    sprintf(p, "%d.%d", temp/10, temp%10);
    pD->setFont(RUS_FONT_S);
    byte w = pD->getStrWidth(buff);
    pD->drawStr(34, 16-2, buff);
    pD->showCentigrade(36+w, 2);
    sprintf(buff, "%d%c", humm, '%');
    pD->drawStr(64, 16-2, buff);
  }
}

bool mainSCREEN::isDark(void) {
  static time_t last_day = 0;
  static time_t sr = 0;
  static time_t ss = 0;

  time_t t = now();
  time_t ld = t / 86400;                              // 24 hours in seconds
  if (ld != last_day) {
    last_day = ld;
    sr = pSm->sunRise();
    ss = pSm->sunSet();
  }
  return ((t < sr) || (t > ss));
}

//------------------------------------------ class histSCREEN, show the data history ----------------------------------
class histSCREEN : public SCREEN {
  public:
    histSCREEN(DSPL* U8g, ENCODER* pROT, weatherLogger* WL, sensorPool* pSP) {
      update_screen = 0;
      pD     = U8g;
	    pEn    = pROT;
      pWl    = WL;
	    pSp    = pSP;
      period = 15*60;                                 // the history for one day long, show history every 15 minutes
      ID     = 0;
	    start_time  = 0;
    }
    virtual void show(void);
    virtual void init(void);                          // Read the data from the EEPROM log and fill the history Data
    virtual void rotaryValue(int16_t value);          // Select the sensor to be displayed
	  virtual SCREEN* menu(void);                       // Switch between history period (day/week)
  private:
    bool loadHistoryData(byte &sensorID, byte &t, byte &p, byte &h, time_t &timestamp);
    void rebuildHistory(void);
    DSPL*          pD;                                // Pointer to the screen instance
	  ENCODER*       pEn;                               // Pointer to the rotary encoder instance
    weatherLogger* pWl;                               // Pointer to the weatherLogger instance
	  sensorPool*    pSp;                               // Pointer to the sensor pool instance
    byte           hData[4][96];                      // 3 sensor values saved every 15 minutes, 96 samples per day for each sensor
    byte           hdn[96];                           // The number of samples per graph point
    byte           min[3];                            // minimal values for temperature, pressure and humidity
    byte           max[3];                            // maximal values for temperature, pressure and humidity
    uint16_t       period;                            // the interval bettwen history data (in seconds)
    byte           ID;                                // the sensor ID to display history
    time_t         start_time;                        // the time of the first history sample
    const byte     gh = 20;                           // Maximun height of the each graph
	  const uint32_t refresh = 30000;                   // The period in ms to refresh the screen
};

void histSCREEN::init(void) {
  ID = 0;                                             // Show the internal sensor first
  period = 15*60;                                     // Show the daily statistic
  rebuildHistory();
  byte n = pSp->numSensors();
  pEn->reset(0, 0, n, 1, 1, false);
  setSCRtimeout(30);
  update_screen = 0;
}

void histSCREEN::show(void) {
  static byte Y[3][2] = { {9, 20}, {31, 42}, {53, 63} };
  char buff[10]; int t;

  uint32_t nowMS = millis();
  if (nowMS < update_screen) return;
  update_screen = nowMS + refresh;

  pD->clearBuffer();
  pD->setFont(TINY_FONT);
  // Draw min and max temperature. Data in the history is the temperature * 5
  t = (max[0] << 2) - 500;                            // Now t is the maximum temperature * 10
  char *p = buff;
  if (t < 0) {
    *p++ = '-';
    t *= -1;
  }
  sprintf(p, "%d.%d", t / 10, t % 10);
  byte width = pD->getStrWidth(buff);
  pD->drawStr(30-width, Y[0][0], buff);
  t = (min[0] << 2) - 500;                            // t is the minimum temperature * 10
  p = buff;
  if (t < 0) {
    *p++ = '-';
    t *= -1;
  }
  sprintf(p, "%d.%d", t / 10, t % 10);
  width = pD->getStrWidth(buff);
  pD->drawStr(30-width, Y[0][1], buff);

  pD->drawBox(3, Y[1][1]-10, 2, 6);
  // Draw min and max pressure. Data in the history is the pressure - 600
  t = max[1] + 600;                                   // t is the maximum pressure
  sprintf(buff,  "%3d", t);
  width = pD->getStrWidth(buff);
  pD->drawStr(30-width, Y[1][0], buff);
  t = min[1] + 600;                                   // t is the minimum pressure
  sprintf(buff, "%3d", t);
  width = pD->getStrWidth(buff);
  pD->drawStr(30-width, Y[1][1], buff);
    
  // Draw min and max humidity. Data in the history is saved as is
  t = max[2];                                         // t is the maximum humidity
  sprintf(buff, "%2d", t);
  width = pD->getStrWidth(buff);
  pD->drawStr(30-width, Y[2][0], buff);
  t = min[2];                                         // t is the minimum humidity
  sprintf(buff, "%2d", t);
  width = pD->getStrWidth(buff);
  pD->drawStr(30-width, Y[2][1], buff);

  pD->showTempSign(0, Y[0][1]-16);                    // Draw temperature symbol
  pD->showPressureSign(0, Y[1][1]-16);                // Draw pressure symbol
  pD->showHummSign(0, Y[2][1]-16);                    // Draw humidity symbol
  pD->showVarrow(30, 2);                              // Draw the axis
  pD->drawVLine(32, 0 , 63);
  pD->drawHLine(32, 63, 127-32);

  // Draw the label at 0, 6, 12, 18 o'clock
  uint16_t graphStart = start_time % (6*3600);
  if (graphStart != 0)
  graphStart = (3600*6 - graphStart)/period;
  uint16_t graphStep = 3600*6/period;
  for (byte x = graphStart; x < 96; x+= graphStep) {
    byte height = 1;
    if (((start_time + (uint32_t)x*period) % 86400) == 0) height = 3;
     pD->drawVLine(32+x, 63-height, height);
  }

  // Draw the external sensor ID
  if (ID > 0) {
    sprintf(buff, "%X", ID);
    pD->drawStr(50, 10, buff);
  }

  // Draw the graphs
  for (byte s = 0; s < 3; ++s) {
    byte x = 0; byte y = 0;
    for (byte i = 0; i < 96; ++i) {
      if (hData[s][i] > gh) continue;   // Skip wrong data
      byte nx = 32 + i; byte ny = Y[s][1] - hData[s][i];
      if (x == 0) {                     // Save the first point of the graph
        x = nx; y = ny;
        continue;
      }
      pD->drawLine(x, y, nx, ny);
      x = nx; y = ny;
    }
  }
  pD->sendBuffer();   
}

void histSCREEN::rotaryValue(int16_t value) {
  static int old_v = 0;

  if (old_v < value) {                                // Turn encoder to the right
    ID = pSp->next(ID);
  } else {                                            // Turn encoder to the left
    ID = pSp->previous(ID);
  }
  old_v = value;
  period = 15*60;                                     // One day history
  rebuildHistory();
  update_screen = 0;                                  // redraw the screen
}

SCREEN* histSCREEN::menu(void) {
  // encoder button switches history period between 15 min for one day statistic and 2 hours for week statistic
  if (period == 15*60) {                              // 15 minutes
    period = 7200;                                    // 2 hours
  } else {                                            // choose the next sensor
    period = 15*60;
  }
  rebuildHistory();
  update_screen = 0;                                  // redraw the screen
  return this;
}

void histSCREEN::rebuildHistory(void) {
  time_t ts;

  // Clear the history data
  for (byte s = 0; s < 3; ++s) {
    min[s] = 255;
    max[s] = 0;
    for (byte i = 0; i < 96; ++i)
      hData[s][i] = 0;
  }
  for (byte i = 0; i < 96; ++i) hdn[i] = 0;

  // Calculate the history start time
  start_time = now() - (uint32_t)period * 96;
  uint16_t next = start_time % period;
  start_time -= next;

  byte rdata[3];                                      // temporary buffer to read the data: temp, pressure and humidity
  byte sensorID = 0;
  // New log file created at midninghts; Several log files can be used to display the history data 
  for (time_t curr_log = start_time; curr_log < now(); curr_log += 86400) {
    // Skip the day history period, because no data file opened for this day
    if (!pWl->openLog(curr_log)) continue;
    while (loadHistoryData(sensorID, rdata[0], rdata[1], rdata[2], ts) ) {
	  if (sensorID != ID) continue;
      ts -= second(ts);                               // Round the history data to the minute
      // Calculate position of the data in the graph; Data must fit to the time grid
      if (ts < start_time) continue;                  // This data time stamp is before start time

      byte indx = (ts - start_time) / period;
      if (indx >= 96) continue;                       // Only 96 values fit the graph

      for (byte s = 0; s < 3; ++s) {                  // If several data fits timestamp of the grid, use avarage value (on week graph)
        uint16_t d = uint16_t(hData[s][indx]) * hdn[indx] + uint16_t(rdata[s]);
        d /= (hdn[indx] + 1);
        hData[s][indx] = byte(d);
        if (d < min[s]) min[s] = d;
        if (d > max[s]) max[s] = d;
      }
      ++hdn[indx];
    }
  }                                                   // The end of the 'day loop'

  if (min[2] > max[2]) {                              // There was no history data read
    for (byte s = 0; s < 3; ++s) min[s] = max[s] = 0;
    return;
  }

  // Fill empty data with the average of left and right data
  for (byte i = 0; i < 96; ++i) {
    if (hdn[i] == 0) {                                // No data for this grid value loaded
      for (byte s = 0; s < 3; ++s) {
        if (i == 0) {
          if (hdn[1] == 0)
            hData[s][i] = ((uint16_t)min[s] + (uint16_t)max[s]) >> 1;
          else
            hData[s][i] = hData[s][i+i];
        } else {
          uint16_t average = hData[s][i-1];
          if (hdn[i+1] > 0) {
            average += hData[s][i+1];
            average >>= 1;                            // average value of left and right elements
          }
          hData[s][i] = average;
        }
      }
    }
  }

  // Normalize the data [min; max] -> [0; gh]
  for (byte s = 0; s < 3; ++s) {
    if (max[s] > min[s]) {
      float k = (float)gh / ((float)max[s] - (float)min[s]);
      for (byte i = 0; i < 96; ++i) {
        float nd = (float)hData[s][i] - (float)min[s];
        nd *= k;
        nd += 0.5;
        int ndi = (int) nd;
        if (ndi < 0) ndi = 0; else if (ndi > gh) ndi = gh;
        hData[s][i] = ndi;
      }
    } else {                                          // Constant data
      for (byte i = 0; i < 96; ++i) {
        hData[s][i] = gh>>1;
      }
    }
  }
}

bool histSCREEN::loadHistoryData(byte &sensorID, byte &t, byte &p, byte &h, time_t &timestamp) {
  int t1, p1;
  if (pWl->readData(sensorID, t1, p1, h, timestamp)) {
    t1 += 500;                                        // t1 was: -500 <= t1 <= 500 (in centegrees * 10)
    t = t1>>2;                                        // t is in centegrees * 2.5
    p = p1 - 600;
    return true;
  }
  return false;
}

//------------------------------------------ class setupScr, configure the weather station ----------------------------
class setupSCR: public SCREEN {
  public:
    setupSCR(DSPL* U8g, ENCODER* pROT, sensorPool* pSP, CONFIG* pCFG) : SCREEN() {
      pD    = U8g;
      pEn   = pROT;
      pSp   = pSP;
      pCfg  = pCFG;
      index = 0;
    };
    virtual void    init(void);
    virtual void    show(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void    rotaryValue(int16_t value);
  private:
    void          showTime(byte x, byte y, byte t);   // Show morning or evening time
    DSPL*         pD;                                 // Pointer to the Screen instance
    ENCODER*      pEn;                                // Pointer to the encoder instance
    sensorPool*   pSp;                                // Pointer to the sensorPoool instance
    CONFIG*       pCfg;                               // Pointer to the config instance
    byte          ext_sensor[max_sensors+1];          // The available sensor list
    byte          ext_sensor_num;                     // The number of the external sensors in the list
    byte          ext_sensor_index;                   // Index of the external sensor in the list
    byte          index;                              // 0 - sensor, 1 - morning, 2 - evening, 3 - clock
    bool          modifying;                          // Whether the some part of the time is modifying now
    bool          save_cfg;                           // Whether the config was changed
    struct cfg    sCfg;                               // The configuration data
    const uint32_t period = 10000;
};

void setupSCR::init(void) {
  index = 0;
  modifying = false;
  save_cfg  = false; 
  pEn->reset(index, 0, 3, 1, 1, true);
  pCfg->getConfig(sCfg);
  ext_sensor[0]  = 0;
  ext_sensor_num = 1;
  byte s = 0;
  while (s = pSp->next(s) ) {
    ext_sensor[ext_sensor_num++] = s;
  }
  for (s = 0; s < ext_sensor_num; ++s) {
    if (ext_sensor[s] == sCfg.ext_sensor_ID)
    break;
  }
  if (s >= ext_sensor_num) {
    ext_sensor[s] = sCfg.ext_sensor_ID;
    ++ext_sensor_num;
  }
  ext_sensor_index = s;
  setSCRtimeout(30);
  update_screen = 0;
}

void setupSCR::show(void) {
  char buff[6];

  uint32_t nowMS = millis();
  if (nowMS < update_screen) return;
  update_screen = nowMS + period;

  pD->clearBuffer();
  pD->setFont(RUS_FONT_S);
  pD->drawStr(30, 10,  "\xCD\xE0\xF1\xF2\xF0\xEE\xE9\xEA\xE8");                 // Настройки
  pD->drawHLine(28, 12, 60);
  pD->drawStr(10, 24, "\xC2\xED\xE5\xF8\xED\xE8\xE9 \xF1\xE5\xED\xF1\xEE\xF0"); // Внешний сенсор
  pD->drawStr(10, 36, "\xD3\xF2\xF0\xEE");                                      // Утро
  pD->drawStr(10, 48, "\xC2\xE5\xF7\xE5\xF0");                                  // Вечер
  pD->drawStr(10, 60, "\xD7\xEe0\xF1\xFB");                                     // Часы

  // Show the menu marker
  byte p = index;
  if (!modifying) p = pEn->read();
  pD->showMenuMark(0, p*12+18);

  // Show external sensor ID
  if (sCfg.ext_sensor_ID != 0) {
    sprintf(buff, "%X", sCfg.ext_sensor_ID);
  } else {
    sprintf(buff, "%s", "??");
  }
  pD->drawStr(100, 24, buff);

  showTime(50, 36, sCfg.backlight_morning);
  showTime(50, 48, sCfg.backlight_evening);
  
  pD->sendBuffer();
}


void setupSCR::rotaryValue(int16_t value) {
  if (modifying) {
    switch (index) {
      case 0:                                         // External sensor
        ext_sensor_index = value;
        sCfg.ext_sensor_ID = ext_sensor[value];
        break;
      case 1:                                         // morning time
        sCfg.backlight_morning = value;
        break;
      case 2:                                         // evening time
        sCfg.backlight_evening = value;
        break;
      default:
        break;
    }
  }
  update_screen = 0;                                  // redraw the screen
}

SCREEN* setupSCR::menu(void) {
  if (modifying) {                                    // return to selection mode
    pEn->reset(index, 0, 3, 1, 1, true);
    modifying = false;
    save_cfg  = true;
    if (sCfg.backlight_morning > sCfg.backlight_evening)
      sCfg.backlight_morning = sCfg.backlight_evening - 1;
  } else {
    index = pEn->read();
    modifying = true;
    switch (index) {
      case 0:                                         // Prepare to modify external sensor ID
        if (ext_sensor_num > 0)
          pEn->reset(ext_sensor_index, 0, ext_sensor_num-1, 1, 1, true);
        else
          modifying = false;
        break;
      case 1:                                         // Prepare to modify morning time
        pEn->reset(sCfg.backlight_morning, 9, 143, 1, 1, false);
        break;
      case 2:                                         // Prepare to modify evening time
        pEn->reset(sCfg.backlight_evening, 9, 143, 1, 1, false);
        break;
      case 3:
        if (save_cfg) pCfg->saveConfig(sCfg);         // Save modification before setup the clock
        if (next) return next;                        // Switch to clock setup screen
      default:
        break;
    }
  }
  update_screen = 0;                                  // redraw the screen
  return this;  
}

SCREEN* setupSCR::menu_long(void) {
  
  pCfg->saveConfig(sCfg);
  if (main) return main; else return this;
}

void setupSCR::showTime(byte x, byte y, byte t) {
  char buff[8];
  if (t >= 144) t = 143;
  byte H = t / 6;
  byte M = (t % 6) * 10;
  sprintf(buff, "%2d:%02d", H, M);
  pD->drawStr(x, y, buff);
}

//------------------------------------------ class timeSerSCR, setup clock --------------------------------------------
class timeSetSCR: public SCREEN {
  public:
    timeSetSCR(DSPL* U8g, ENCODER* pROT) : SCREEN() {
      pD  = U8g;
	    pEn = pROT;
      index = 0;
    };
    virtual void    init(void);
    virtual void    show(void);
    virtual SCREEN* menu(void);
    virtual SCREEN* menu_long(void);
    virtual void    rotaryValue(int16_t value);
  private:
    DSPL*         pD;                                 // Pointer to the Screen instance
	  ENCODER*      pEn;                                // Pointer to the encoder instance
    byte          index;                              // 0 - Hour, 1 - Minute, 2 - Day, 3 - Month, 4 - Year
	  bool          modifying;                          // Whether the some part of the time is modifying now
    tmElements_t  tm;                                 // time to be set-up
    const uint32_t period = 10000;
    const byte cursor_pos[5][2] = {
      {48, 32},                                       // Hour
      {63, 32},                                       // Minute
      {15, 52},                                       // Day
      {40, 52},                                       // Month
      {70, 52}                                        // Year
    };
};

void timeSetSCR::init(void) {
  if (scr_timeout != 0) return;                       // If the time is editing right now, do not init again     
  RTC.read(tm);                                       // Initialize the time to be set up with the current time
  index = 0;
  modifying = false;
  update_screen = 0;
  pEn->reset(index, 0, 4, 1, 0, true);                // Prepare to select the part of the time to be modified
  setSCRtimeout(20);
}

void timeSetSCR::show(void) {

  uint32_t nowMS = millis();
  if (nowMS < update_screen) return;
  update_screen = nowMS + period;

  char datebuf[25];
  char outbuf[25];

  sprintf(datebuf, "%2d - ", tm.Day);
  for (byte i = 0; i < 3; ++i) datebuf[5+i] = months[(tm.Month-1)*3+i];
  sprintf(&datebuf[8], " - %4d, ", tmYearToCalendar(tm.Year));
  for (byte i = 0; i < 2; ++i) datebuf[17+i] = wday[(tm.Wday-1)*2+i];
  datebuf[19] = '\0';

  pD->clearBuffer();   
  pD->setFont(RUS_FONT_S);
  pD->drawStr(25, 10, "\xD3\xf1\xf2. \xe2\xf0\xe5\xec\xe5\xed\xe8");
  sprintf(outbuf, "%02d:%02d", tm.Hour, tm.Minute);
  pD->drawStr(48, 30, outbuf);
  pD->drawStr(15, 50, datebuf);
  // Highlight the element to be modified
  byte edit_entity = index;
  if (!modifying) edit_entity = pEn->read();
  pD->drawLine(cursor_pos[edit_entity][0], cursor_pos[edit_entity][1], cursor_pos[edit_entity][0] + 12, cursor_pos[edit_entity][1]);
  if (modifying) pD->showClockSetup(110, 2);
  pD->sendBuffer();
}

void timeSetSCR::rotaryValue(int16_t value) {
  if (modifying) {                                    // The user is modifying the part of the time index
    time_t nt = 0;
	  byte* p = 0;
    switch (index) {
	    case 0:                                         // Modifying the Hours
	      p = &tm.Hour;
		    break;
	    case 1:                                         // Modifying the Minutes
	      p = &tm.Minute;
		    break;
	    case 2:                                         // Modufying the day
	      p = &tm.Day;
		    break;
	    case 3:                                         // Modifying the Month
	      p = &tm.Month;
		    break;
	    case 4:                                         // Modifying the Year
	      p = &tm.Year;
		    break;
	    default:
	      break;
	  }
	  if (p) {
	    *p = value;
	    time_t nt = makeTime(tm);                       // Make sure that all the values in the tm structure are correct
	    breakTime(nt, tm);
	    if (value != *p) pEn->write(*p);
	  }
  } else {                                            // Select the item to modify
    index = value;
  }
  update_screen = 0;
}

SCREEN* timeSetSCR::menu(void) {
  if (modifying) {
    pEn->reset(index, 0, 4, 1, 0, true);              // Finish the item modification, return to the selection mode
  } else {
    index = pEn->read();
	  switch (index) {
	    case 0:                                         // Prepare to modify Hours
	      pEn->reset(tm.Hour, 0, 23, 1, 1, true);
		    break;
	    case 1:                                         // Prepare to modify Minutes
	      pEn->reset(tm.Minute, 0, 59, 1, 5, true);
		    break;
	    case 2:                                         // Prepare to modify Day
	      pEn->reset(tm.Day, 1, 31, 1, 1, false);
		    break;
	    case 3:                                         // Prepare to modify Month
	      pEn->reset(tm.Month, 1, 12, 1, 1, true);
		    break;
	    case 4:                                         // Prepare to modify Year
	      pEn->reset(tm.Year, 0, 100, 1, 1, false);
		    break;
	    default:
	      return this;
    }
  }
  modifying = !modifying;
  return this;
}

SCREEN* timeSetSCR::menu_long(void) {
  tm.Second = 0;
  RTC.write(tm);
  scr_timeout = 0;
  if (this->nextL != 0) return this->nextL; else return this; 
}

//------------------------------------------ class sunSCREEN, show sunrise and sunset times ---------------------------
class sunSCREEN : public SCREEN {
  public:
    sunSCREEN(DSPL* U8g, sunMoon* SM) {
      update_screen = 0;
      pD  = U8g;
      pSm = SM;
    }
    virtual void show(void);
    virtual void init(void);
  private:
    DSPL*    pD;                                      // Pointer to the Screen instance
    sunMoon* pSm;                                     // Pointer to the sunMoon instance
    const uint32_t period = 60000;
};

void sunSCREEN::init(void) {
  setSCRtimeout(30);
}

void sunSCREEN::show(void) {

  uint32_t nowMS = millis();
  if (nowMS < update_screen) return;
  update_screen = nowMS + period;

  tmElements_t tm;
  char datebuf[25];
  char hourbuf[6];
  char sunrise[8];
  char sunset[8];

  sprintf(datebuf, "%2d - ", day());
  for (byte i = 0; i < 3; ++i) datebuf[5+i] = months[(month()-1)*3+i];
  sprintf(&datebuf[8], " - %4d, ", year());
  for (byte i = 0; i < 2; ++i) datebuf[17+i] = wday[(weekday()-1)*2+i];
  datebuf[19] = '\0';
  sprintf(hourbuf, "%02d:%02d", hour(), minute());
  time_t rise = pSm->sunRise(); rise += 30;
  sprintf(sunrise, "%02d:%02d", hour(rise), minute(rise));
  rise = pSm->sunSet(); rise += 30;
  sprintf(sunset, "%02d:%02d", hour(rise), minute(rise));

  pD->clearBuffer();  
  pD->setFont(RUS_FONT_L);
  pD->drawStr(44, 18, hourbuf);
  pD->setFont(RUS_FONT_S);
  pD->drawStr(15, 34, datebuf);
  pD->drawStr(0,  50, "\xC2\xEE\xF1\xF5\xEE\xE4");    // Восход
  pD->drawStr(0, 63, sunrise);
  pD->drawStr(96, 50, "\xC7\xE0\xEA\xE0\xF2");        // Закат
  pD->drawStr(98, 63, sunset);
  pD->sendBuffer();
}

// ================================ End of all class definitions ======================================
DSPL                    u8g(EN_SCK_PIN, RW_MOSI_PIN, RS_CS_PIN, RST_PIN);
BL                      bckLight(LIGHT_SENSOR, LCD_BLGHT_PIN);
HB                      hb(HB_PIN);
CONFIG                  stationCfg;
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);
DHT                     dht(DHT_PIN, DHTTYPE);
OregonDecoderV2         orscV2;
sunMoon                 sm;
pressureSensor          pSensor(&bmp);
weatherLogger           wLogger;
sensorPool              sPool;

ENCODER         rotEncoder(R_MAIN_PIN, R_SECD_PIN);
BUTTON          rotButton(R_BUTN_PIN);

mainSCREEN      mainScr(&u8g, &rotEncoder, &sPool, &pSensor, &sm);
histSCREEN      histScr(&u8g, &rotEncoder, &wLogger, &sPool);
setupSCR        setpScr(&u8g, &rotEncoder, &sPool, &stationCfg);
timeSetSCR      tSetScr(&u8g, &rotEncoder);
sunSCREEN       sunScrn(&u8g, &sm);

SCREEN* pScreen = &mainScr;


// call back iroutine for file timestamps
void SDfileDate(uint16_t* date, uint16_t* time) {
 *date = FAT_DATE(year(), month(), day());
 *time = FAT_TIME(hour(), minute(), second());
}

void reloadConfig(void) {
  struct cfg Cfg;
  stationCfg.getConfig(Cfg);
  bckLight.setNightPeriod(Cfg.backlight_evening, Cfg.backlight_morning);
  if (Cfg.ext_sensor_ID > 0)
    mainScr.setMainSensorID(Cfg.ext_sensor_ID);
}

void setup() {
//  Serial.begin(115200);
  time_t now_t = RTC.get();                           // RTC (DS3231 clock) declared as an external variable
  setTime(now_t);
  randomSeed(now_t);

  u8g.begin();
  bmp.begin();
  dht.begin();
  sPool.init();
  wLogger.init(SD_PIN);
  SdFile::dateTimeCallback(SDfileDate);
  delay(1500);

  // Initialize rotary encoder
  rotEncoder.init();
  rotButton.init();
  attachInterrupt(digitalPinToInterrupt(R_MAIN_PIN), rotEncChange,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(R_BUTN_PIN), rotPushChange,  CHANGE);

  mainScr.next  = &sunScrn;
  mainScr.nextL = &setpScr;
  sunScrn.next  = &histScr;
  sunScrn.main  = &mainScr;
  histScr.next  = &mainScr;
  histScr.main  = &mainScr;
  setpScr.main  = &mainScr;
  setpScr.next  = &tSetScr;
  setpScr.nextL = &mainScr;
  tSetScr.nextL = &mainScr;
  tSetScr.main  = &mainScr;

  bckLight.init();
  bckLight.setLimits(50, 500, 1, 150);
  hb.setTimeout(60);                                  // Reset ne555 timer every minute
  
  // Load configuration data
  stationCfg.init();
  stationCfg.load();
  reloadConfig();
  sm.init(OUR_timezone, OUR_latitude, OUR_longtitude);
  orscV2.begin(w433_INTR);
}

void rotEncChange(void) {
  rotEncoder.cnangeINTR();
}

void rotPushChange(void) {
  rotButton.cnangeINTR();
}

void w433Update(void) {
  int  temp = 0;
  byte humm = 0;
  byte ch   = 0;
  byte id   = 0;
  bool battOK = false;
  if (orscV2.receiveData(ch, id, temp, humm, battOK)) {
    if ((id > 0) && (humm > 0)) {
      sPool.update(id, temp, humm, battOK);           // Save the data from remote sensor to the pool
    }
  }
  sPool.freeStaled();
}

void loop() {
 static uint32_t update_data = 0;                       // Time in seconds when we should update the internal sensors data
 static int16_t  old_pos = rotEncoder.read();
 static time_t   write_log = 0;

  bckLight.adjust();
  hb.autoReset();

  SCREEN* nxt = pScreen->returnToMain();
  if (nxt != pScreen) {                               // return to the main screen by timeout
    pScreen = nxt;
    pScreen->init();
  }

  byte bStatus = rotButton.intButtonStatus();
  switch (bStatus) {
    case 2:                                           // long press;
      nxt = pScreen->menu_long();
      if (nxt != pScreen) {
        reloadConfig();
        pScreen = nxt;
        pScreen->init();
      } else {
        if (pScreen->isSetup())
         pScreen->resetTimeout();
      }
      break;
    case 1:                                           // short press
      nxt = pScreen->menu();
      if (nxt != pScreen) {
        pScreen = nxt;
        pScreen->init();
      } else {
        if (pScreen->isSetup())
         pScreen->resetTimeout();
      }
      break;
    case 0:                                           // Not pressed
    default:
      break;
  }

  int16_t pos = rotEncoder.read();
  if (old_pos != pos) {
    pScreen->rotaryValue(pos);
    old_pos = pos;
    if (pScreen->isSetup())
     pScreen->resetTimeout();
  }

  uint32_t nowMS = millis();
  if (nowMS >= update_data) {
    update_data = nowMS + 30000;
    pSensor.update();
    float fh = dht.readHumidity();
    float ft = dht.readTemperature();
    byte  h  = byte(fh + 0.5);
    int   t  = int(ft * 10);                          // The temperature in 10*centegrees
    sPool.update(0, t, h, true);                      // Save the data from local DHT sensor to the pool with sensor ID=0
    pScreen->forceRedraw();
  }

  w433Update();                                       // Check wireless 433MHz sensors for new data
  
  time_t now_t = now();
  if (write_log == 0)                                 // System was rebooted, set Write log time
    write_log = wLogger.calculateWriteLogTime(now_t);

  // Save average data from all the sensors to the file in SD card
  if (now_t >= write_log) {
    write_log = wLogger.calculateWriteLogTime(now_t + 120);
	int aP = pSensor.average();
	byte id = 0;
	do {
	  int  aT = sPool.aTemp(id);
	  byte aH = sPool.aHumm(id);
      wLogger.writeData(id, aT, aP, aH);              // Put the pressure data to the log of all the sensors
	  id = sPool.next(id);
	} while (id > 0);
  }

  // Remove old logs at 2:00 am
  if ((hour() == 2) && (minute() == 0) && (second() == 0)) { 
    wLogger.rmOldLog(now_t - 604800);                 // 7 days in seconds
    delay(1000);
  }

  pScreen->show();

}

