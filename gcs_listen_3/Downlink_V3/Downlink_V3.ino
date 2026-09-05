#include <OneWire.h>
#include <DallasTemperature.h>
#include <TinyGPS++.h>
#include <SPI.h>

// ---------- Pins / Interfaces ----------
#define PIN_ONEWIRE     2          // DS18B20 DQ
#define PIN_MCP3208_CS  0          // MCP3208 CS (SPI1: SCK=27, MOSI=26, MISO=1)
SPIClass &adcSPI = SPI1;

#define GPS_SERIAL    Serial5      // GNSS 走 Serial5（依你現況）
#define GPS_BAUD      115200

#define E220_SERIAL   Serial2      // E220 走 Serial2
#define E220_BAUD     9600
const int PIN_E220_M0  = -1;       // 若有接 M0/M1/AUX，再填實際腳位；否則保持 -1
const int PIN_E220_M1  = -1;
const int PIN_E220_AUX = -1;

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

// ---------- Objects ----------
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature ds18b20(&oneWire);
TinyGPSPlus gps;

// ---------- Helpers ----------
static inline void e220_wait_aux(unsigned long timeout_ms=50) {
  if (PIN_E220_AUX < 0) { delay(2); return; }
  unsigned long t0 = millis();
  pinMode(PIN_E220_AUX, INPUT);
  while (millis() - t0 < timeout_ms) {
    if (digitalRead(PIN_E220_AUX) == HIGH) break;
  }
}

// 讀 MCP3208 raw（0..4095）
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

void get_gnss_utc(char* out, size_t outsz) {
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(out, outsz, "%04d-%02d-%02dT%02d:%02d:%02dZ",
             (int)gps.date.year(), (int)gps.date.month(), (int)gps.date.day(),
             (int)gps.time.hour(), (int)gps.time.minute(), (int)gps.time.second());
  } else out[0] = '\0';
}

void pump_gnss(unsigned long ms=40) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (GPS_SERIAL.available()) gps.encode(GPS_SERIAL.read());
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);

  ds18b20.begin();

  pinMode(PIN_MCP3208_CS, OUTPUT);
  digitalWrite(PIN_MCP3208_CS, HIGH);
  adcSPI.begin();

  if (PIN_E220_M0 >= 0) pinMode(PIN_E220_M0, OUTPUT);
  if (PIN_E220_M1 >= 0) pinMode(PIN_E220_M1, OUTPUT);
  if (PIN_E220_M0 >= 0) digitalWrite(PIN_E220_M0, LOW);  // E220 模式 0：透明傳輸
  if (PIN_E220_M1 >= 0) digitalWrite(PIN_E220_M1, LOW);
  E220_SERIAL.begin(E220_BAUD);

  GPS_SERIAL.begin(GPS_BAUD);

  Serial.println("Boot OK");
}

// ---------- Loop ----------
void loop() {
  // 1) GNSS
  pump_gnss(50);

  // 2) 溫度
  ds18b20.requestTemperatures();
  float tC = ds18b20.getTempCByIndex(0);
  if (tC <= -127.0f) tC = NAN; // 異常視為空值

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

  // 7) GNSS 欄位（ALT/LON/LAT；時間用 UTC ISO8601）
  char utc[24];  get_gnss_utc(utc, sizeof(utc));
  float alt_m = gps.altitude.isValid()   ? gps.altitude.meters() : NAN;
  float lon   = gps.location.isValid()   ? gps.location.lng()    : NAN;
  float lat   = gps.location.isValid()   ? gps.location.lat()    : NAN;

  // 8) 組封包（空值→空字串）
  char tbuf[16], abuf[12], lonbuf[16], latbuf[16];
  char vb[12], vs[12], v33[12], v5v[12], vef[12];
  char vts[12], vrg[12];

  fmt_float(tbuf,   sizeof(tbuf),   tC,     2);
  fmt_float(abuf,   sizeof(abuf),   alt_m,  1);
  fmt_float(lonbuf, sizeof(lonbuf), lon,    6);
  fmt_float(latbuf, sizeof(latbuf), lat,    6);
  fmt_float(vb,     sizeof(vb),     vBatt,  3);
  fmt_float(vs,     sizeof(vs),     vSolar, 3);
  fmt_float(v33,    sizeof(v33),    v3,     3);
  fmt_float(v5v,    sizeof(v5v),    v4,     3);
  fmt_float(vef,    sizeof(vef),    v5,     3);
  fmt_float(vts,    sizeof(vts),    v2,     3);      // 修正：用 vts/sizeof(vts)
  fmt_float(vrg,    sizeof(vrg),    v_epsG, 3);      // 修正：回報 EPS_GND

  char pkt[256];
  snprintf(pkt, sizeof(pkt),
    "DATA,%lu,T=%s,UTC=%s,ALT=%s,LON=%s,LAT=%s,VBAT=%s,VSOLAR=%s,V3V3=%s,V5V=%s,V3V3EF=%s,TS=%s,EPS=%s\r\n",
    (unsigned long)millis(),
    tbuf, utc, abuf, lonbuf, latbuf, vb, vs, v33, v5v, vef, vts, vrg
  );

  // 9) 發送
  e220_wait_aux();
  E220_SERIAL.print(pkt);
  E220_SERIAL.flush();

  // Debug
  Serial.print("TX -> "); Serial.print(pkt);

  delay(1000); // 1 Hz
}
