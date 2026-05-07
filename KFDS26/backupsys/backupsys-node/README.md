 * ESP32-S3 DevKitC-1 N8R8 flight/impact logger
 * - Autodetects BME280 or BMP280 on I2C
 * - Autodetects MPU6500 on I2C
 * - Logs sensor data to LittleFS CSV
 * - Hosts latest data and log file over WiFi AP
 * - Uses onboard NeoPixel on GPIO38 as status LED
 * - Starts loud, interrupting, energy-aware beeper after major impact
 * - Single-file Arduino IDE sketch
 * 
 * Arduino IDE libraries needed:
 *   - Adafruit BMP280 Library
 *   - Adafruit BME280 Library
 *   - Adafruit Unified Sensor
 *   - Adafruit NeoPixel
 *   - LittleFS (bundled with ESP32 core)
 *   - WiFi / WebServer / Wire (bundled)
 * 
 * Suggested wiring:
 *   I2C SDA -> GPIO8   (change below if needed)
 *   I2C SCL -> GPIO9   (change below if needed)
 *   Buzzer -> GPIO4    (or any free GPIO suitable for output)
 *   NeoPixel -> GPIO38 (ESP32-S3 DevKitC-1 v1.1 onboard RGB)
