// The Weather Station@atmega328p-pu
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <U8glib.h>
#include <WlessOregonV2.h>

#define MAIN_SENSOR_ID 0xBB

#define DHT_PIN        4
#define DHTTYPE    DHT22

//#define TINY_FONT       u8g_font_profont11r
#define TINY_FONT       u8g_font_profont10r
#define RUS_FONT_S      u8g_font_cronx1h
#define RUS_FONT_L      u8g_font_cronyx3hb
#define DIGIT_FONT      u8g_font_osb18n

// lcd 128x64 screen in software mode
#define EN_SCK_PIN      13
#define RW_MOSI_PIN     11
#define RS_CS_PIN       10
#define RST_PIN          8

#define LIGHT_SENSOR    A0                      // A0, Photoresistor
#define LCD_BLGHT_PIN    5

const uint16_t high_pressure = 767-8;           // High pressure depends on altitude
const uint16_t low_pressure  = 757-8;           // Low pressure  depends on altitude

const byte max_sensors = 3;                     // The maximum number of the sensors (remote + one internal) connected
const byte w433_INTR   = 0;                     // The wireless 433 MHz sensor @ pin #2, interrupt #0

const byte hbPIN       = 16;                    // A2 analog sensor pin

class BL {
  public:
    BL(byte sensorPIN, byte lightPIN, byte start_brightness = 128) {
      sensor_pin          = sensorPIN;
      led_pin             = lightPIN;
    }
    void init(void);                                // Initialize the data
    void adjust(void);                              // Automatically adjust the brightness
  private:
    int      empAverage(int v);                     // Exponential average value
    byte     sensor_pin;                            // Light sensor pin
    byte     led_pin;                               // Led PWM pin
    uint32_t checkMS;                               // Time in ms when the sensor was checked
    uint32_t ch_step;                               // The time in ms when the brighthess can be adjusted
    byte     brightness;                            // The backlight brightness
    byte     new_brightness;                        // The baclight brightness to set up
    long     emp;                                   // Exponential average value
    const uint16_t b_night            = 200;        // light sensor value of the night
    const uint16_t b_day              = 900;        // light sensor value of the day light
    const byte     daily_brightness   = 150;        // The maximum brightness of backlight when light between b_night and b_day
    const byte     nightly_brightness =   1;        // The brightness to use nightly
    const byte     emp_k              =   8;        // The exponential average coefficient
    const uint16_t period             = 500;        // The period in ms to check the photeregister
    const uint16_t ch_period          =   5;        // The period to adjust brightness
    const uint8_t  default_brightness = 128;
};

void BL::init(void) {

  pinMode(led_pin, OUTPUT);
  pinMode(sensor_pin, INPUT);
  int light = analogRead(sensor_pin);
  emp = 0;
  brightness = new_brightness = default_brightness;
  checkMS = ch_step = 0;
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

  uint32_t ms = millis();
  if ((ms > ch_step) && (new_brightness != brightness)) {
    if (new_brightness > brightness) ++brightness; else --brightness;
    analogWrite(led_pin, brightness);
    ch_step = ms + ch_period;
  }

  if (ms < checkMS) return;
  checkMS = ms + period;

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
class DSPL : public U8GLIB_ST7920_128X64_1X {
  public:
//    DSPL(byte sck, byte rw, byte rs, byte rst) : U8GLIB_ST7920_128X64_1X(sck, rw, rs, rst) {      // the software mode
    DSPL(byte sck, byte rw, byte rs, byte rst) : U8GLIB_ST7920_128X64_1X(rs, rst) {               // the hardware mode
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
    enum fcast {SUNNY = 0, CLOUDLY = 1, RAIN = 2};
  private:
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
    const uint8_t bmPressure[16] = {
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
    default:
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

//------------------------------------------ class pressureSensor -----------------------------------------------------
class pressureSensor {
  public:
    pressureSensor(Adafruit_BMP280* Bmp) {
      pBmp     = Bmp;
    }
    void update(void);
    byte forecast(bool is_cold = false, bool is_dark = false);
    int  press(void)                                { return hPress.last(); }
    int  average(void)                              { return hPress.average(); }
    enum fcast { SUNNY = 0, CLOUDLY = 1, RAIN = 2 }; 
  private:
    Adafruit_BMP280* pBmp;                          // Pointer to the BMP sensor instance
    HISTORY hPress;                                 // History data for temperature, pressure and humidity
};

void pressureSensor::update(void) {
  float t = pBmp->readTemperature();
  // The pressure in mmHg
  int pressure = int(pBmp->readPressure() * 0.00750064);
  hPress.put(pressure);
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
      return SUNNY;
    }
  }
  if (pressure < low_pressure) {                    // LOW pressure
    if (grad_full <= 0) {
        return RAIN;
    } else {
      if (grad_short <= 8) {
        return CLOUDLY;
      } else {
        return SUNNY;
      }
    }
  }
  if ((grad_short >= -5) && (grad_full >= 0)) {     // NORMAL pressure
    return SUNNY;
  } else {
    if (grad_short <= -8) {
      return RAIN;
    } else
      return CLOUDLY;
  }
}

//------------------------------------------ class weatherSensor ------------------------------------------------------
class weatherSensor {
  public:
    weatherSensor(void)                             {}
    void     init(void);
    void     update(int temp, byte humm, bool batt);// Temperature in 10*cencegrees [-500; +500]
    int      temp(void)                             { return Temp; }
    byte     humm(void)                             { return Humm; }
    int      aTemp(void)                            { return Temp; }
    byte     aHumm(void)                            { return Humm; }
    bool     isBattOK(void)                         { return battOK; }
    uint32_t lastUpdated(void)                      { return updated; }

  private:
    bool     battOK;                                // Whether the battery of the sensor is OK
    uint32_t updated;                               // The time when the sensor data was updated in ms
    int      Temp;                                  // temperature
    byte     Humm;                                  // humidity
};

void weatherSensor::init(void) {
  battOK   = false;
  updated  = 0;
  Temp     = 0;
  Humm     = 0;
}

void weatherSensor::update(int temp, byte humm, bool batt) {
  if (temp < -500) temp = -500;                     // lower temperature limit -50.0 cencegrees
  if (temp >  500) temp =  500;                     // upper temperature limit +50.0 cencegrees
  Temp    = temp;
  Humm    = humm;
  battOK  = batt;
  updated = millis();
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
    const uint32_t  stale_time = 1800000;           // timeout for the staled sensor data in ms
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
  uint32_t n = millis();
  for (byte i = 0; i < max_sensors; ++i) {
    if (in_use[i]) {
      uint32_t lu = s[i].lastUpdated();
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

//------------------------------------------ class mainSCREEN ---------------------------------------------------------
class mainSCREEN {
  public:
    mainSCREEN(DSPL* U8g, sensorPool* pSP, pressureSensor* pPS) {
      pD   = U8g;
      pSp  = pSP;
      pPs  = pPS;
      main_sensor = 0;
      main_sensor_manual = false;
    }
    void  init(void);
    void  show(void);
    void  forceRedraw(void)                           { update_screen = 0; }
    void  setMainSensorID(byte id)                    { main_sensor = id; main_sensor_manual = true; }

  private:
    void  showClock(void);
    void  showExtSensor(void);                        // Show external sensor data (right side of the screen)
    bool  getMainSensorData(int& temp, byte& humidity);
    void  showInternalSensorData(void);
    DSPL*            pD;                              // Pointer to the screen instance
    sensorPool*      pSp;                             // Pointer to the sensor pool instance
    pressureSensor*  pPs;                             // Pointer to the Pressure sensor based on BMP180
    byte             main_sensor;                     // The ID of the main external sensor
    bool             main_sensor_manual;              // Whether the main sensor ID was setup manually
    byte             ext_sensor;                      // external sensor ID to be displayed
    uint32_t         update_screen;                   // The time when update the screen in ms
    const byte cx = 100;                              // The analog clock parameters: center X, Y coordinates and the radius
    const byte cy = 41;
    const byte cr = 22;
    const uint16_t  period = 17000;                   // The screen update period
};

void mainSCREEN::init(void) {
  update_screen = 0;
  ext_sensor    = 0;
}

void mainSCREEN::show(void) {
  
  uint32_t nowMS = millis();
  if (nowMS < update_screen) return;
  update_screen = nowMS + period;

  pD->firstPage();  
  do {

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
      if (pos > 60) pos = 60;                         // Maximum X position of the right side of digits
    
      pD->setFont(DIGIT_FONT);
      pD->drawStr(pos-hw, 63, hbuff);                 // Show the humidity value
      pD->drawStr(pos-tw1-tw2, 40, tbuff1);           // Show the integer part of the temperature
      pD->setFont(RUS_FONT_L);
      pD->drawStr(pos-tw2, 40, tbuff2);               // Show the fractional part of the temperature
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
    byte fcast = pPs->forecast(is_cold, false); 
    pD->showForecast(fcast);

    showExtSensor();
  } while(pD->nextPage());

  while (!pSp->exists(ext_sensor))
    ext_sensor = pSp->next(ext_sensor);
  ext_sensor = pSp->next(ext_sensor);
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

// ================================ End of all class definitions ======================================
DSPL              u8g(EN_SCK_PIN, RW_MOSI_PIN, RS_CS_PIN, RST_PIN);
BL                bckLight(LIGHT_SENSOR, LCD_BLGHT_PIN);
HB                hb(hbPIN);
Adafruit_BMP280   bmp;
DHT               dht(DHT_PIN, DHTTYPE);
OregonDecoderV2   orscV2;
pressureSensor    pSensor(&bmp);
sensorPool        sPool;
mainSCREEN        mainScr(&u8g, &sPool, &pSensor);

void setup() {
  //Serial.begin(9600);

  u8g.begin();
  bmp.begin(0x76);
  dht.begin();
  sPool.init();

  bckLight.init();
  hb.setTimeout(60);                                  // Reset ne555 timer every minute

  mainScr.setMainSensorID(MAIN_SENSOR_ID);
  orscV2.begin(w433_INTR);
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
 static uint32_t update_data = 0;                     // Time in seconds when we should update the internal sensors data

  w433Update();                                       // Check wireless 433MHz sensors for new data
  bckLight.adjust();
  hb.autoReset();
  
  uint32_t nowMS = millis();
  if (nowMS >= update_data) {
    update_data = nowMS + 30000;
    pSensor.update();
    float fh = dht.readHumidity();
    float ft = dht.readTemperature();
    byte  h  = byte(fh + 0.5);
    int   t  = int(ft * 10);                          // The temperature in 10*centegrees
    sPool.update(0, t, h, true);                      // Save the data from local DHT sensor to the pool with sensor ID=0
    mainScr.forceRedraw();
  }
  
  mainScr.show();
}

