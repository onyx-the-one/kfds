#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 38
#endif

// -------------------- User config --------------------
static const int PIN_I2C_SDA = 8;
static const int PIN_I2C_SCL = 9;
static const int PIN_NEOPIXEL = 38;
static const int PIN_BUZZER = 4;

static const char* AP_SSID = "ESP32-LOGGER";
static const char* AP_PASS = "loggertest";

static const uint32_t I2C_FREQ = 400000;
static const uint16_t SAMPLE_RATE_HZ = 10;       // 5..20 recommended
static const uint32_t FLUSH_INTERVAL_MS = 1000;  // file flush interval
static const bool OVERWRITE_LOG_ON_BOOT = false; // false = append
static const bool ENABLE_HUMIDITY_IF_PRESENT = true;

// Impact detect tuning:
static const float IMPACT_G_THRESHOLD = 6.0f;       // touchdown / crash candidate
static const float IMPACT_GYRO_THRESHOLD_DPS = 250; // angular shock support
static const uint16_t IMPACT_CONFIRM_SAMPLES = 2;   // consecutive samples
static const uint32_t BEEP_ON_MS = 120;
static const uint32_t BEEP_OFF_MS = 1400;           // energy efficient duty cycle
static const uint16_t BEEP_FREQ_HZ = 2800;

// Sea level optional if altitude wanted later
static const float SEA_LEVEL_HPA = 1013.25f;
// -----------------------------------------------------

WebServer server(80);
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
Adafruit_BMP280 bmp;
Adafruit_BME280 bme;

enum EnvType { ENV_NONE, ENV_BMP280, ENV_BME280 };
EnvType envType = ENV_NONE;
uint8_t envAddr = 0;
bool mpuFound = false;
uint8_t mpuAddr = 0x68;

File logFile;
String currentCsvLine;
String latestJson;

volatile bool impactLatched = false;
uint16_t impactCounter = 0;
uint32_t lastSampleMs = 0;
uint32_t lastFlushMs = 0;
uint32_t lastBeepToggleMs = 0;
bool beepState = false;

// Latest values
struct SampleData {
    uint32_t ms;
    float tempC;
    float pressurePa;
    float humidityPct;
    bool humidityValid;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
    float accMag_g;
    bool envOk;
    bool mpuOk;
} s;

// ---------- Low-level I2C helpers ----------
bool i2cDevicePresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return (Wire.endTransmission() == 0);
}

bool readReg8(uint8_t addr, uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, 1) != 1) return false;
    val = Wire.read();
    return true;
}

bool writeReg8(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    size_t n = Wire.requestFrom((int)addr, (int)len);
    if (n != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

int16_t be16(const uint8_t *p) {
    return (int16_t)((p[0] << 8) | p[1]);
}

// ---------- Status LED ----------
void setPixel(uint8_t r, uint8_t g, uint8_t b) {
    pixel.setPixelColor(0, pixel.Color(r, g, b));
    pixel.show();
}

void setStatusBoot()    { setPixel(0, 0, 24); }
void setStatusReady()   { setPixel(0, 18, 0); }
void setStatusWarn()    { setPixel(18, 8, 0); }
void setStatusImpact()  { setPixel(24, 0, 0); }
void setStatusLogging() { setPixel(0, 8, 18); }

// ---------- MPU6500 ----------
bool initMPU6500() {
    if (!i2cDevicePresent(0x68) && !i2cDevicePresent(0x69)) return false;
    mpuAddr = i2cDevicePresent(0x68) ? 0x68 : 0x69;

    uint8_t who = 0;
    if (!readReg8(mpuAddr, 0x75, who)) return false;
    if (who != 0x70) return false; // MPU6500 WHO_AM_I

    // Reset
    writeReg8(mpuAddr, 0x6B, 0x80);
    delay(100);

    // Clock source = auto/best available, wake up
    if (!writeReg8(mpuAddr, 0x6B, 0x01)) return false;
    delay(10);

    // DLPF enabled, gyro bandwidth moderate
    if (!writeReg8(mpuAddr, 0x1A, 0x03)) return false;

    // Sample rate divider: Gyro output rate 1kHz/(1+div)
    // 1kHz / (1+49) = 20Hz internal output pacing
    if (!writeReg8(mpuAddr, 0x19, 49)) return false;

    // Gyro full scale ±1000 dps
    if (!writeReg8(mpuAddr, 0x1B, 0x10)) return false;

    // Accel full scale ±8g
    if (!writeReg8(mpuAddr, 0x1C, 0x10)) return false;

    // Accel DLPF / bandwidth
    if (!writeReg8(mpuAddr, 0x1D, 0x03)) return false;

    mpuFound = true;
    return true;
}

bool readMPU6500(float &ax_g, float &ay_g, float &az_g,
                 float &gx_dps, float &gy_dps, float &gz_dps) {
    uint8_t buf[14];
    if (!readRegs(mpuAddr, 0x3B, buf, 14)) return false;

    int16_t ax = be16(&buf[0]);
    int16_t ay = be16(&buf[2]);
    int16_t az = be16(&buf[4]);
    int16_t gx = be16(&buf[8]);
    int16_t gy = be16(&buf[10]);
    int16_t gz = be16(&buf[12]);

    // ±8g => 4096 LSB/g
    ax_g = (float)ax / 4096.0f;
    ay_g = (float)ay / 4096.0f;
    az_g = (float)az / 4096.0f;

    // ±1000 dps => 32.8 LSB/(deg/s)
    gx_dps = (float)gx / 32.8f;
    gy_dps = (float)gy / 32.8f;
    gz_dps = (float)gz / 32.8f;
    return true;
                 }

                 // ---------- Environmental sensor ----------
                 bool detectEnvSensor() {
                     uint8_t candidates[] = {0x76, 0x77};
                     for (uint8_t i = 0; i < 2; i++) {
                         uint8_t addr = candidates[i];
                         if (!i2cDevicePresent(addr)) continue;

                         uint8_t chip = 0;
                         if (!readReg8(addr, 0xD0, chip)) continue;

                         if (chip == 0x60) {
                             if (bme.begin(addr)) {
                                 envType = ENV_BME280;
                                 envAddr = addr;
                                 bme.setSampling(
                                     Adafruit_BME280::MODE_NORMAL,
                                     Adafruit_BME280::SAMPLING_X2,
                                     Adafruit_BME280::SAMPLING_X16,
                                     Adafruit_BME280::SAMPLING_X1,
                                     Adafruit_BME280::FILTER_X4,
                                     Adafruit_BME280::STANDBY_MS_125
                                 );
                                 return true;
                             }
                         } else if (chip == 0x58) {
                             if (bmp.begin(addr)) {
                                 envType = ENV_BMP280;
                                 envAddr = addr;
                                 bmp.setSampling(
                                     Adafruit_BMP280::MODE_NORMAL,
                                     Adafruit_BMP280::SAMPLING_X2,
                                     Adafruit_BMP280::SAMPLING_X16,
                                     Adafruit_BMP280::FILTER_X4,
                                     Adafruit_BMP280::STANDBY_MS_125
                                 );
                                 return true;
                             }
                         }
                     }
                     envType = ENV_NONE;
                     envAddr = 0;
                     return false;
                 }

                 bool readEnv(float &tempC, float &pressurePa, float &humidityPct, bool &humidityValid) {
                     humidityPct = NAN;
                     humidityValid = false;

                     if (envType == ENV_BME280) {
                         tempC = bme.readTemperature();
                         pressurePa = bme.readPressure();
                         humidityPct = bme.readHumidity();
                         humidityValid = ENABLE_HUMIDITY_IF_PRESENT;
                         return !(isnan(tempC) || isnan(pressurePa));
                     }

                     if (envType == ENV_BMP280) {
                         tempC = bmp.readTemperature();
                         pressurePa = bmp.readPressure();
                         return !(isnan(tempC) || isnan(pressurePa));
                     }

                     return false;
                 }

                 // ---------- Storage ----------
                 bool openLog() {
                     const char *mode = OVERWRITE_LOG_ON_BOOT ? "w" : "a";
                     logFile = LittleFS.open("/log.csv", mode);
                     if (!logFile) return false;

                     if (OVERWRITE_LOG_ON_BOOT || logFile.size() == 0) {
                         logFile.println("ms,temp_c,pressure_pa,humidity_pct,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,acc_mag_g,impact");
                         logFile.flush();
                     }
                     return true;
                 }

                 void appendLogLine(const String &line) {
                     if (!logFile) return;
                     logFile.println(line);
                     if (millis() - lastFlushMs >= FLUSH_INTERVAL_MS) {
                         logFile.flush();
                         lastFlushMs = millis();
                     }
                 }

                 // ---------- Web ----------
                 String htmlPage() {
                     String h;
                     h.reserve(3000);
                     h += F(
                         "<!doctype html><html><head><meta charset='utf-8'>"
                         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
                         "<title>ESP32 Logger</title>"
                         "<style>"
                         "body{font-family:system-ui,Arial,sans-serif;background:#111;color:#eee;margin:0;padding:16px}"
                         "h1{font-size:1.1rem} .g{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}"
                         ".c{background:#1c1c1c;border:1px solid #333;border-radius:10px;padding:12px}"
                         ".v{font-size:1.4rem;font-weight:700}.m{color:#aaa;font-size:.9rem}"
                         "a{color:#7cc7ff} pre{white-space:pre-wrap;word-break:break-word}"
                         "</style></head><body>"
                         "<h1>ESP32 Sensor Logger</h1>"
                         "<div class='g'>"
                         "<div class='c'><div class='m'>Environmental sensor</div><div id='env' class='v'>-</div></div>"
                         "<div class='c'><div class='m'>MPU6500</div><div id='mpu' class='v'>-</div></div>"
                         "<div class='c'><div class='m'>Temperature</div><div id='t' class='v'>-</div></div>"
                         "<div class='c'><div class='m'>Pressure</div><div id='p' class='v'>-</div></div>"
                         "<div class='c'><div class='m'>Humidity</div><div id='h' class='v'>-</div></div>"
                         "<div class='c'><div class='m'>Acceleration magnitude</div><div id='a' class='v'>-</div></div>"
                         "<div class='c'><div class='m'>Impact</div><div id='i' class='v'>-</div></div>"
                         "</div><p><a href='/data'>JSON</a> | <a href='/log.csv'>CSV log</a> | <a href='/info'>Info</a></p>"
                         "<pre id='raw'></pre>"
                         "<script>"
                         "async function u(){"
                         "let r=await fetch('/data'); let j=await r.json();"
                         "document.getElementById('env').textContent=j.env_type+' @ 0x'+j.env_addr_hex;"
                         "document.getElementById('mpu').textContent=j.mpu_ok?('OK @ 0x'+j.mpu_addr_hex):'missing';"
                         "document.getElementById('t').textContent=(j.temp_c===null)?'-':j.temp_c.toFixed(2)+' C';"
                         "document.getElementById('p').textContent=(j.pressure_pa===null)?'-':(j.pressure_pa/100.0).toFixed(2)+' hPa';"
                         "document.getElementById('h').textContent=(j.humidity_pct===null)?'-':j.humidity_pct.toFixed(1)+' %';"
                         "document.getElementById('a').textContent=(j.acc_mag_g===null)?'-':j.acc_mag_g.toFixed(2)+' g';"
                         "document.getElementById('i').textContent=j.impact?'LATCHED':'no';"
                         "document.getElementById('raw').textContent=JSON.stringify(j,null,2);"
                         "} setInterval(u,1000); u();"
                         "</script></body></html>"
                     );
                     return h;
                 }

                 void setupWeb() {
                     server.on("/", HTTP_GET, []() {
                         server.send(200, "text/html", htmlPage());
                     });

                     server.on("/data", HTTP_GET, []() {
                         server.send(200, "application/json", latestJson);
                     });

                     server.on("/info", HTTP_GET, []() {
                         String info;
                         info += "AP: ";
                         info += AP_SSID;
                         info += "\nIP: ";
                         info += WiFi.softAPIP().toString();
                         info += "\nEnv sensor: ";
                         info += (envType == ENV_BME280) ? "BME280" : (envType == ENV_BMP280) ? "BMP280" : "none";
                         info += "\nEnv addr: 0x";
                         info += String(envAddr, HEX);
                         info += "\nMPU6500: ";
                         info += mpuFound ? "yes" : "no";
                         info += "\nMPU addr: 0x";
                         info += String(mpuAddr, HEX);
                         info += "\nSample rate Hz: ";
                         info += String(SAMPLE_RATE_HZ);
                         info += "\nLog file: /log.csv";
                         info += "\nImpact latched: ";
                         info += impactLatched ? "yes" : "no";
                         server.send(200, "text/plain", info);
                     });

                     server.on("/log.csv", HTTP_GET, []() {
                         File f = LittleFS.open("/log.csv", "r");
                         if (!f) {
                             server.send(404, "text/plain", "log missing");
                             return;
                         }
                         server.streamFile(f, "text/csv");
                         f.close();
                     });

                     server.begin();
                 }

                 // ---------- Buzzer ----------
                 void updateBeeper() {
                     if (!impactLatched) {
                         noTone(PIN_BUZZER);
                         beepState = false;
                         return;
                     }

                     uint32_t now = millis();
                     if (!beepState) {
                         if (now - lastBeepToggleMs >= BEEP_OFF_MS) {
                             tone(PIN_BUZZER, BEEP_FREQ_HZ);
                             beepState = true;
                             lastBeepToggleMs = now;
                         }
                     } else {
                         if (now - lastBeepToggleMs >= BEEP_ON_MS) {
                             noTone(PIN_BUZZER);
                             beepState = false;
                             lastBeepToggleMs = now;
                         }
                     }
                 }

                 // ---------- Impact detection ----------
                 void checkImpact(float accMag_g, float gx, float gy, float gz) {
                     float gyroMag = sqrtf(gx * gx + gy * gy + gz * gz);
                     bool hit = (accMag_g >= IMPACT_G_THRESHOLD) || (gyroMag >= IMPACT_GYRO_THRESHOLD_DPS);

                     if (hit) {
                         if (impactCounter < 65535) impactCounter++;
                     } else {
                         impactCounter = 0;
                     }

                     if (!impactLatched && impactCounter >= IMPACT_CONFIRM_SAMPLES) {
                         impactLatched = true;
                         setStatusImpact();
                     }
                 }

                 // ---------- JSON ----------
                 String hex2(uint8_t v) {
                     String s = String(v, HEX);
                     s.toUpperCase();
                     if (s.length() < 2) s = "0" + s;
                     return s;
                 }

                 void updateJson() {
                     latestJson = "{";
                     latestJson += "\"ms\":" + String(s.ms);
                     latestJson += ",\"env_type\":\"" + String(envType == ENV_BME280 ? "BME280" : envType == ENV_BMP280 ? "BMP280" : "NONE") + "\"";
                     latestJson += ",\"env_addr_hex\":\"" + hex2(envAddr) + "\"";
                     latestJson += ",\"mpu_ok\":" + String(s.mpuOk ? "true" : "false");
                     latestJson += ",\"mpu_addr_hex\":\"" + hex2(mpuAddr) + "\"";
                     latestJson += ",\"temp_c\":" + String(s.envOk ? String(s.tempC, 2) : "null");
                     latestJson += ",\"pressure_pa\":" + String(s.envOk ? String(s.pressurePa, 2) : "null");
                     latestJson += ",\"humidity_pct\":";
                     latestJson += (s.humidityValid ? String(s.humidityPct, 2) : "null");
                     latestJson += ",\"ax_g\":" + String(s.mpuOk ? String(s.ax_g, 3) : "null");
                     latestJson += ",\"ay_g\":" + String(s.mpuOk ? String(s.ay_g, 3) : "null");
                     latestJson += ",\"az_g\":" + String(s.mpuOk ? String(s.az_g, 3) : "null");
                     latestJson += ",\"gx_dps\":" + String(s.mpuOk ? String(s.gx_dps, 2) : "null");
                     latestJson += ",\"gy_dps\":" + String(s.mpuOk ? String(s.gy_dps, 2) : "null");
                     latestJson += ",\"gz_dps\":" + String(s.mpuOk ? String(s.gz_dps, 2) : "null");
                     latestJson += ",\"acc_mag_g\":" + String(s.mpuOk ? String(s.accMag_g, 3) : "null");
                     latestJson += ",\"impact\":" + String(impactLatched ? "true" : "false");
                     latestJson += "}";
                 }

                 // ---------- Setup / loop ----------
                 void setup() {
                     Serial.begin(115200);
                     delay(200);

                     pixel.begin();
                     pixel.setBrightness(16);
                     setStatusBoot();

                     pinMode(PIN_BUZZER, OUTPUT);
                     noTone(PIN_BUZZER);

                     Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
                     delay(50);

                     if (!LittleFS.begin(true)) {
                         setStatusWarn();
                     } else {
                         openLog();
                     }

                     bool envOk = detectEnvSensor();
                     bool imuOk = initMPU6500();

                     WiFi.mode(WIFI_AP);
                     WiFi.softAP(AP_SSID, AP_PASS);
                     setupWeb();

                     if (envOk) setStatusReady();
                     else setStatusWarn();

                     Serial.println();
                     Serial.println("Boot complete");
                     Serial.print("AP IP: ");
                     Serial.println(WiFi.softAPIP());
                     Serial.print("Env: ");
                     Serial.println(envType == ENV_BME280 ? "BME280" : envType == ENV_BMP280 ? "BMP280" : "NONE");
                     Serial.print("MPU6500: ");
                     Serial.println(imuOk ? "YES" : "NO");
                 }

                 void loop() {
                     server.handleClient();
                     updateBeeper();

                     const uint32_t intervalMs = 1000UL / SAMPLE_RATE_HZ;
                     uint32_t now = millis();
                     if (now - lastSampleMs < intervalMs) return;
                     lastSampleMs = now;

                     s.ms = now;
                     s.envOk = readEnv(s.tempC, s.pressurePa, s.humidityPct, s.humidityValid);
                     s.mpuOk = false;

                     if (mpuFound) {
                         s.mpuOk = readMPU6500(s.ax_g, s.ay_g, s.az_g, s.gx_dps, s.gy_dps, s.gz_dps);
                         if (s.mpuOk) {
                             s.accMag_g = sqrtf(s.ax_g * s.ax_g + s.ay_g * s.ay_g + s.az_g * s.az_g);
                             checkImpact(s.accMag_g, s.gx_dps, s.gy_dps, s.gz_dps);
                         } else {
                             s.accMag_g = NAN;
                         }
                     }

                     if (!impactLatched) {
                         if (s.envOk) setStatusLogging();
                         else setStatusWarn();
                     }

                     String line;
                     line.reserve(160);
                     line += String(s.ms);
                     line += ",";
                     line += s.envOk ? String(s.tempC, 2) : "";
                     line += ",";
                     line += s.envOk ? String(s.pressurePa, 2) : "";
                     line += ",";
                     line += s.humidityValid ? String(s.humidityPct, 2) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.ax_g, 3) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.ay_g, 3) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.az_g, 3) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.gx_dps, 2) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.gy_dps, 2) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.gz_dps, 2) : "";
                     line += ",";
                     line += s.mpuOk ? String(s.accMag_g, 3) : "";
                     line += ",";
                     line += impactLatched ? "1" : "0";

                     appendLogLine(line);
                     updateJson();
                 }
