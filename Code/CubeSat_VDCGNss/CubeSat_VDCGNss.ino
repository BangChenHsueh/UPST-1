#include <Wire.h>
#include <TinyGPS++.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <SparkFun_ISM330DHCX.h>
#include <math.h>
#include <DHT11.h>
#include <SPI.h>

#define UART_SERIAL Serial2
#define DS18B20_PORT 2
#define UART_BAUD 9600

#define COMPUTER_SERIAL Serial
#define COMPUTER_BAUD 115200

#define GPS_SERIAL Serial5
#define GPS_BAUD 115200

#define SERIAL5_RX_BUFFER_SIZE 2048

#define DHT11_PIN 37

extern int __heap_start, *__brkval;

//intantiate classes
TinyGPSPlus gps;
SparkFun_ISM330DHCX imu;
OneWire oneWire(DS18B20_PORT); 

const int LED_PIN = LED_BUILTIN;

// const char *gpsStream =
//   "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
//   "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
//   "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
//   "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
//   "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
//   "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

DallasTemperature DallasTemperatureSensor(&oneWire);

sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

int DHT11_Temperature;
int DHT11_Humidity;

#define PIN_MCP3208_CS  0          // MCP3208 CS (SPI1: SCK=27, MOSI=26, MISO=1)
SPIClass &adcSPI = SPI1;

static int PACKAGE_COUNT = 0;

char MainBuffer[256];

DHT11 dht11(DHT11_PIN);

// ---------- ADC / Scaling（改成與 MCP3208 範例一致） ----------
static const float VREF_ADC = 5.000f;   // MCP3208 參考電壓（若你實際接 3.3V 就改回 3.300f）

// CH0 電池：RTOP=4.7k、RBOT=6.8k -> 8.4V -> 5V
static const float CH0_RTOP = 4700.0f;
static const float CH0_RBOT = 6800.0f;

// CH1 太陽能板分壓
static const float CH1_RTOP = 6800.0f;
static const float CH1_RBOT = 5600.0f;

static const float GAIN_CH0 = (CH0_RTOP + CH0_RBOT) / CH0_RBOT;  // ≈1.691
static const float GAIN_CH1 = (CH1_RTOP + CH1_RBOT) / CH1_RBOT;  // ≈2.214

//-----------Helper-----------------

// // 讀 MCP3208 raw（0..4095）
uint16_t mcp3208_read_raw(uint8_t ch) {
  ch &= 0x07;
  uint8_t b1 = 0x06 | (ch >> 2);  // start + single-ended + D2
  uint8_t b2 = (ch & 0x03) << 6;  // D1 D0 << 6
  adcSPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_MCP3208_CS, LOW);
  (void)adcSPI.transfer(b1);
  uint8_t hi = adcSPI.transfer(b2) & 0x0F;
  uint8_t lo = adcSPI.transfer(0x00);
  digitalWrite(PIN_MCP3208_CS, HIGH);
  adcSPI.endTransaction();
  return ((uint16_t)hi << 8) | lo;
}

// raw → ADC 腳位電壓（未分壓還原）
static inline float raw_to_volts(uint16_t raw) {
  return (raw * VREF_ADC) / 4095.0f;
}

static inline void fmt_float(char* buf, size_t sz, float v, int digits) {
  if (isnan(v) || isinf(v)) { buf[0] = '\0'; return; } // 無效 → 留空
  dtostrf(v, 0, digits, buf);
}

void setup() {
  // put your setup code here, to run once:

  pinMode(LED_PIN, OUTPUT);
  
  pinMode(PIN_MCP3208_CS, OUTPUT);
  digitalWrite(PIN_MCP3208_CS, HIGH);
  adcSPI.begin();

  //Wire(12C) Begins
  Wire.begin();

  //imu setup
  imu.begin(Wire);
  imu.deviceReset();

  imu.setDeviceConfig();
  imu.setBlockDataUpdate();

  imu.setAccelDataRate(ISM_XL_ODR_104Hz);
  imu.setAccelFullScale(ISM_4g); 

	imu.setGyroDataRate(ISM_GY_ODR_104Hz);
	imu.setGyroFullScale(ISM_500dps); 

  imu.setAccelFilterLP2();
	imu.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	imu.setGyroFilterLP1();
	imu.setGyroLP1Bandwidth(ISM_MEDIUM);

  //computer Serial Setup
  COMPUTER_SERIAL.begin(COMPUTER_BAUD);

  //E220 Serial Setup
  UART_SERIAL.begin(UART_BAUD);

  //DS18B20 Setup
  DallasTemperatureSensor.setWaitForConversion(true);
  DallasTemperatureSensor.begin();

  //GPS Serial Setup
  GPS_SERIAL.begin(GPS_BAUD);

  //DHT11 Setup
  pinMode(DHT11_PIN, INPUT_PULLUP);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

}

float readTempWithFilter(){
  DallasTemperatureSensor.requestTemperatures();
  float c = DallasTemperatureSensor.getTempCByIndex(0);
  if(c > -100.0f && fabs(c-85.0f) > 0.1f){
    return c;
  }
  return DEVICE_DISCONNECTED_C;
}

int DHT11ReadTempHumidity(int &temperature, int &humidity, DHT11 &sensor){
  int result = sensor.readTemperatureHumidity(temperature, humidity);
  return result;
}

void ProcessGPS(){
  
  while (GPS_SERIAL.available()>0) {
    char c = GPS_SERIAL.read();
    gps.encode(c);
    COMPUTER_SERIAL.print(c);
  }
}


void loop() {
  // while(*gpsStream){
  //   gps.encode(*gpsStream++);
  // }
  // ALWAYS read all incoming GPS data immediately
  // while (GPS_SERIAL.available() > 0) {
  //   char c = GPS_SERIAL.read();
  //   gps.encode(c);
  // }
  ProcessGPS();

  static uint32_t t0 = 0;
  
  if (millis() - t0 < 1000) {
    return;  // Wait until 1 second has passed
  }
  t0 = millis();

  PACKAGE_COUNT++;

  float TempC;

    // 3) 讀 ADC raw
  uint16_t r0 = mcp3208_read_raw(0); // Battery 分壓
  uint16_t r1 = mcp3208_read_raw(1); // Solar   分壓
  uint16_t r2 = mcp3208_read_raw(2); // TS
  uint16_t r3 = mcp3208_read_raw(3); // 3.3V
  uint16_t r4 = mcp3208_read_raw(4); // 5V
  uint16_t r5 = mcp3208_read_raw(5); // 3.3V efuse
  uint16_t r6 = mcp3208_read_raw(6); // EPS 板地（相對 S_GND）
  // uint16_t r7 = mcp3208_read_raw(7); // 空

    // 4) raw→ADC 腳位電壓
  float v0_adc = raw_to_volts(r0);
  float v1_adc = raw_to_volts(r1);
  float v2     = raw_to_volts(r2);
  float v3     = raw_to_volts(r3);
  float v4     = raw_to_volts(r4);
  float v5     = raw_to_volts(r5);
  float v_epsG = raw_to_volts(r6);

    // 5) 分壓還原
  float vBatt_raw  = v0_adc * GAIN_CH0;
  float vSolar_raw = v1_adc * GAIN_CH1;

    // 6) 扣除 EPS_GND（ground offset 校正）
  float vBatt  = vBatt_raw  - v_epsG;
  float vSolar = vSolar_raw - v_epsG;
  if (fabsf(vBatt)  < 0.0005f) vBatt  = 0.0f;  // 避免 -0.00
  if (fabsf(vSolar) < 0.0005f) vSolar = 0.0f;

  // DallasTemperatureSensor.requestTemperatures();
  // float c = 10.00;
  // float c = DallasTemperatureSensor.getTempCByIndex(0);
  // if(c > -100.0f && fabs(c-85.0f) > 0.1f){
  //   TempC = c;
  // }else{
  //   TempC = DEVICE_DISCONNECTED_C;
  // }

  // char TempBuffer[16];
  // if (TempC == DEVICE_DISCONNECTED_C) {
  //   snprintf(TempBuffer, sizeof(TempBuffer), "T:NO_DATA");
  // } else {
  //   snprintf(TempBuffer, sizeof(TempBuffer), "T:%.2f", TempC);
  // }

  char TimeBuffer[32];
  if (gps.time.isValid()) {
    snprintf(TimeBuffer, sizeof(TimeBuffer), "%d:%d:%d",
             gps.time.hour()+8, gps.time.minute(), gps.time.second());
  } else {
    snprintf(TimeBuffer, sizeof(TimeBuffer), "NO_TIME");
  }

  char GPSBuffer[64];
  if (gps.location.isValid()) {
    snprintf(GPSBuffer, sizeof(GPSBuffer), "Lat=%.6f,Lng=%.6f,Alt=%.2f,",
             gps.location.lat(), gps.location.lng(), gps.altitude.meters());
  } else {
    snprintf(GPSBuffer, sizeof(GPSBuffer), "Lat=N/A,Lng=N/A,Alt=N/A,");
  }

  char SATBuffer[2];
  if(gps.time.isValid()){
    snprintf(SATBuffer, sizeof(SATBuffer), "%lu", gps.satellites.value());
  }else{
    snprintf(SATBuffer, sizeof(SATBuffer), "0");
  }

  char IMUBuffer[64];
  if (imu.checkStatus()) {
    imu.getAccel(&accelData);
    imu.getGyro(&gyroData);
    snprintf(IMUBuffer, sizeof(IMUBuffer), "Ax=%.2f,Ay=%.2f,Az=%.2f,Gx=%.2f,Gy=%.2f,Gz=%.2f,",
             accelData.xData, accelData.yData, accelData.zData,
             gyroData.xData, gyroData.yData, gyroData.zData);
  } else {
    snprintf(IMUBuffer, sizeof(IMUBuffer), "NO_IMU");
  }

  char DHT11Buffer[14];
  int result = dht11.readTemperatureHumidity(DHT11_Temperature, DHT11_Humidity);
  // int result = DHT11ReadTempHumidity(DHT11_Temperature, DHT11_Humidity, dht11);
  if (result == 0){
    snprintf(DHT11Buffer, sizeof(DHT11Buffer), "T=%d,H=%d,", DHT11_Temperature, DHT11_Humidity);
  }else {
    snprintf(DHT11Buffer, sizeof(DHT11Buffer), "T=N/A,H=N/A,");
  }

  // 8) 組封包（空值→空字串）(ADC)
  char vb[12], vs[12], v33[12], v5v[12], vef[12];
  char vts[12], vrg[12];

  fmt_float(vb,     sizeof(vb),     vBatt,  3);
  fmt_float(vs,     sizeof(vs),     vSolar, 3);
  fmt_float(v33,    sizeof(v33),    v3,     3);
  fmt_float(v5v,    sizeof(v5v),    v4,     3);
  fmt_float(vef,    sizeof(vef),    v5,     3);
  fmt_float(vts,    sizeof(vts),    v2,     3);      // 修正：用 vts/sizeof(vts)
  fmt_float(vrg,    sizeof(vrg),    v_epsG, 3);      // 修正：回報 EPS_GND

  snprintf(MainBuffer, sizeof(MainBuffer), "Package=%d,UTC=%s,%s%s%sSAT=%s,VBAT=%s,VSOLAR=%s,V3V3=%s,V5V=%s,V3V3EF=%s,TS=%s,EPS=%s", 
  
  PACKAGE_COUNT, TimeBuffer, GPSBuffer, IMUBuffer, DHT11Buffer, SATBuffer, vb, vs, v33, v5v, vef, vts, vrg);

  UART_SERIAL.println(MainBuffer);
  UART_SERIAL.flush();


  // COMPUTER_SERIAL.print("TX: ");
  // COMPUTER_SERIAL.println(MainBuffer);
}


