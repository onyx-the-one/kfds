#include "bme688.h"
#include "node_config.h"

#include <cstring>
#include <cmath>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BME688";

// -----------------------------------------------------------------------------
// Register addresses
// -----------------------------------------------------------------------------
#define REG_CHIP_ID             0xD0
#define REG_RESET               0xE0
#define REG_CTRL_HUM            0x72
#define REG_CTRL_MEAS           0x74
#define REG_CONFIG              0x75

// Data registers (forced mode, field 0)
#define REG_PRESS_MSB           0x1F
#define REG_TEMP_MSB            0x22
#define REG_HUM_MSB             0x25
#define REG_GAS_R_MSB           0x2A
#define REG_GAS_R_LSB           0x2B

// Measurement status
#define REG_MEAS_STATUS_0       0x1D

// Gas heater config
#define REG_RES_HEAT_0          0x5A
#define REG_GAS_WAIT_0          0x64
#define REG_CTRL_GAS_1          0x71

// Chip ID
#define BME688_CHIP_ID          0x61

// Oversampling settings
#define OSR_1X                  0x01
#define OSR_2X                  0x02
#define OSR_4X                  0x03

// Mode
#define MODE_FORCED             0x01

// -----------------------------------------------------------------------------
// SPI handle
// -----------------------------------------------------------------------------
static spi_device_handle_t s_spi = nullptr;

// -----------------------------------------------------------------------------
// SPI helpers — BME688 SPI protocol:
//   Write: bit 7 = 0 (addr & 0x7F), then data byte(s)
//   Read:  bit 7 = 1 (addr | 0x80), then read byte(s)
// -----------------------------------------------------------------------------
static esp_err_t spi_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), val };
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    return spi_device_transmit(s_spi, &t);
}

static esp_err_t spi_read_reg(uint8_t reg, uint8_t *val)
{
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    uint8_t rx[2] = {};
    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;
    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err == ESP_OK) *val = rx[1];
    return err;
}

static esp_err_t spi_read_buf(uint8_t reg, uint8_t *buf, size_t len)
{
    uint8_t tx[1 + len];
    memset(tx, 0, sizeof(tx));
    tx[0] = static_cast<uint8_t>(reg | 0x80);

    uint8_t rx[1 + len];
    memset(rx, 0, sizeof(rx));

    spi_transaction_t t = {};
    t.length = (1 + len) * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err == ESP_OK) {
        memcpy(buf, rx + 1, len);
    }
    return err;
}

// -----------------------------------------------------------------------------
// Calibration data
// -----------------------------------------------------------------------------
static struct {
    // Temperature
    uint16_t t1;
    int16_t  t2;
    int8_t   t3;

    // Pressure
    uint16_t p1;
    int16_t  p2;
    int8_t   p3;
    int16_t  p4;
    int16_t  p5;
    int8_t   p7;
    int8_t   p6;
    int16_t  p8;
    int16_t  p9;
    uint8_t  p10;

    // Humidity
    uint16_t h1;
    uint16_t h2;
    int8_t   h3;
    int8_t   h4;
    int8_t   h5;
    uint8_t  h6;
    int8_t   h7;

    // Gas
    int8_t   gh1;
    int16_t  gh2;
    int8_t   gh3;

    uint8_t  res_heat_range;
    int8_t   res_heat_val;
    int8_t   range_sw_err;
} s_calib;

static int32_t s_t_fine;

// -----------------------------------------------------------------------------
// Read calibration data from BME688
// -----------------------------------------------------------------------------
static bool read_calibration(void)
{
    uint8_t coeff1[25]; // 0x89 - 0xA1 (25 bytes for T/P)
    uint8_t coeff2[16]; // 0xE1 - 0xF0 (16 bytes for H + gas)
    uint8_t coeff3[5];  // 0x00 - 0x04

    if (spi_read_buf(0x89, coeff1, 25) != ESP_OK) return false;
    if (spi_read_buf(0xE1, coeff2, 16) != ESP_OK) return false;
    if (spi_read_buf(0x00, coeff3, 5) != ESP_OK) return false;

    // Temperature
    s_calib.t1 = (uint16_t)((coeff2[9] << 8) | coeff2[8]);
    s_calib.t2 = (int16_t)((coeff1[2] << 8) | coeff1[1]);
    s_calib.t3 = (int8_t)coeff1[3];

    // Pressure
    s_calib.p1  = (uint16_t)((coeff1[6] << 8) | coeff1[5]);
    s_calib.p2  = (int16_t)((coeff1[8] << 8) | coeff1[7]);
    s_calib.p3  = (int8_t)coeff1[9];
    s_calib.p4  = (int16_t)((coeff1[12] << 8) | coeff1[11]);
    s_calib.p5  = (int16_t)((coeff1[14] << 8) | coeff1[13]);
    s_calib.p6  = (int8_t)coeff1[16];
    s_calib.p7  = (int8_t)coeff1[15];
    s_calib.p8  = (int16_t)((coeff1[20] << 8) | coeff1[19]);
    s_calib.p9  = (int16_t)((coeff1[22] << 8) | coeff1[21]);
    s_calib.p10 = (uint8_t)coeff1[23];

    // Humidity
    s_calib.h1 = (uint16_t)(((uint16_t)coeff2[2] << 4) | (coeff2[1] & 0x0F));
    s_calib.h2 = (uint16_t)(((uint16_t)coeff2[0] << 4) | (coeff2[1] >> 4));
    s_calib.h3 = (int8_t)coeff2[3];
    s_calib.h4 = (int8_t)coeff2[4];
    s_calib.h5 = (int8_t)coeff2[5];
    s_calib.h6 = (uint8_t)coeff2[6];
    s_calib.h7 = (int8_t)coeff2[7];

    // Gas
    s_calib.gh1 = (int8_t)coeff2[12];
    s_calib.gh2 = (int16_t)((coeff2[11] << 8) | coeff2[10]);
    s_calib.gh3 = (int8_t)coeff2[13];

    // Additional gas calibration
    uint8_t tmp;
    spi_read_reg(0x02, &tmp);
    s_calib.res_heat_range = (tmp >> 4) & 0x03;

    spi_read_reg(0x00, &tmp);
    s_calib.res_heat_val = (int8_t)tmp;

    spi_read_reg(0x04, &tmp);
    s_calib.range_sw_err = (int8_t)((tmp & 0xF0) >> 4);

    return true;
}

// -----------------------------------------------------------------------------
// Compensation algorithms (from Bosch datasheet)
// -----------------------------------------------------------------------------
static float compensate_temperature(uint32_t adc_temp)
{
    float var1 = ((((float)adc_temp / 16384.0f) - ((float)s_calib.t1 / 1024.0f))
                  * (float)s_calib.t2);
    float var2 = (((((float)adc_temp / 131072.0f) - ((float)s_calib.t1 / 8192.0f))
                   * (((float)adc_temp / 131072.0f) - ((float)s_calib.t1 / 8192.0f)))
                  * ((float)s_calib.t3 * 16.0f));
    s_t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

static float compensate_pressure(uint32_t adc_press)
{
    float var1 = ((float)s_t_fine / 2.0f) - 64000.0f;
    float var2 = var1 * var1 * ((float)s_calib.p6 / 131072.0f);
    var2 = var2 + (var1 * (float)s_calib.p5 * 2.0f);
    var2 = (var2 / 4.0f) + ((float)s_calib.p4 * 65536.0f);
    var1 = (((float)s_calib.p3 * var1 * var1 / 16384.0f) +
            ((float)s_calib.p2 * var1)) / 524288.0f;
    var1 = (1.0f + var1 / 32768.0f) * (float)s_calib.p1;

    if (var1 == 0.0f) return 0.0f;

    float press = 1048576.0f - (float)adc_press;
    press = ((press - (var2 / 4096.0f)) * 6250.0f) / var1;
    var1 = ((float)s_calib.p9 * press * press) / 2147483648.0f;
    var2 = press * ((float)s_calib.p8 / 32768.0f);
    float var3 = (press / 256.0f) * (press / 256.0f) * (press / 256.0f)
                 * ((float)s_calib.p10 / 131072.0f);
    press = press + (var1 + var2 + var3 + ((float)s_calib.p7 * 128.0f)) / 16.0f;
    return press;
}

static float compensate_humidity(uint16_t adc_hum, float temp_comp)
{
    float var1 = (float)adc_hum -
                 (((float)s_calib.h1 * 16.0f) +
                  (((float)s_calib.h3 / 2.0f) * temp_comp));
    float var2 = var1 *
                 (((float)s_calib.h2 / 262144.0f) *
                  (1.0f + (((float)s_calib.h4 / 16384.0f) * temp_comp) +
                   (((float)s_calib.h5 / 1048576.0f) * temp_comp * temp_comp)));
    float var3 = (float)s_calib.h6 / 16384.0f;
    float var4 = (float)s_calib.h7 / 2097152.0f;
    float hum  = var2 + ((var3 + (var4 * temp_comp)) * var2 * var2);

    if (hum > 100.0f) hum = 100.0f;
    if (hum < 0.0f) hum = 0.0f;
    return hum;
}

static float calc_gas_resistance(uint16_t gas_adc, uint8_t gas_range)
{
    static const float lookup_k1[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 0.0f, -0.8f,
        0.0f, 0.0f, -0.2f, -0.5f, 0.0f, -1.0f, 0.0f, 0.0f
    };
    static const float lookup_k2[16] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.1f, 0.7f, 0.0f, -0.8f,
        -0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
    };

    float var1 = 1340.0f + (5.0f * (float)s_calib.range_sw_err);
    float var2 = var1 * (1.0f + lookup_k1[gas_range] / 100.0f);
    float var3 = 1.0f + (lookup_k2[gas_range] / 100.0f);
    float gas_res = 1.0f / (var3 * 0.000000125f * (float)(1 << gas_range)
                            * (((float)gas_adc - 512.0f) / var2 + 1.0f));
    return gas_res;
}

// -----------------------------------------------------------------------------
// Compute heater resistance register value for a target temperature
// -----------------------------------------------------------------------------
static uint8_t calc_res_heat(uint16_t target_temp_c)
{
    float var1 = ((float)s_calib.gh1 / 16.0f) + 49.0f;
    float var2 = (((float)s_calib.gh2 / 32768.0f) * 0.0005f) + 0.00235f;
    float var3 = (float)s_calib.gh3 / 1024.0f;
    float var4 = var1 * (1.0f + (var2 * (float)target_temp_c));
    float var5 = var4 + (var3 * 25.0f);  // ambient = 25°C approx
    float res_heat = 3.4f * ((var5 * (4.0f / (4.0f + (float)s_calib.res_heat_range))
                               * (1.0f / (1.0f + ((float)s_calib.res_heat_val * 0.002f))))
                              - 25.0f);
    return static_cast<uint8_t>(res_heat);
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

esp_err_t bme688_init(spi_host_device_t host)
{
    // Add BME688 to the shared SPI bus — Mode 0 (CPOL=0, CPHA=0)
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = SPI_CLK_ENV;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = PIN_CS_ENV;
    dev_cfg.queue_size = 4;

    esp_err_t err = spi_bus_add_device(host, &dev_cfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Read and verify chip ID
    uint8_t chip_id = 0;
    err = spi_read_reg(REG_CHIP_ID, &chip_id);
    if (err != ESP_OK || chip_id != BME688_CHIP_ID) {
        ESP_LOGE(TAG, "BME688 not found (chip_id=0x%02X)", chip_id);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BME688 detected (chip_id=0x%02X)", chip_id);

    // Soft reset
    spi_write_reg(REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read calibration
    if (!read_calibration()) {
        ESP_LOGE(TAG, "Failed to read calibration data");
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Configure gas heater: 300°C target, 100ms wait
    uint8_t res_heat = calc_res_heat(300);
    spi_write_reg(REG_RES_HEAT_0, res_heat);
    spi_write_reg(REG_GAS_WAIT_0, 0x59); // 100ms encoded

    // Enable gas measurement, use heater profile 0
    // ctrl_gas_1: run_gas=1 (bit 4), nb_conv=0 (bits 3:0)
    spi_write_reg(REG_CTRL_GAS_1, 0x10);

    // Humidity oversampling x1
    spi_write_reg(REG_CTRL_HUM, OSR_1X);

    // config: IIR filter off
    spi_write_reg(REG_CONFIG, 0x00);

    ESP_LOGI(TAG, "BME688 initialized with gas heater at 300°C");
    return ESP_OK;
}

esp_err_t bme688_read(bme688_data_t *data)
{
    if (!data || !s_spi) return ESP_ERR_INVALID_ARG;

    // Trigger forced measurement: temp x2, pressure x4, forced mode
    // ctrl_meas: osrs_t[7:5]=010(x2), osrs_p[4:2]=011(x4), mode[1:0]=01(forced)
    spi_write_reg(REG_CTRL_MEAS, static_cast<uint8_t>((OSR_2X << 5) | (OSR_4X << 2) | MODE_FORCED));

    // Wait for measurement to complete (T+P+H+gas ≈ 200ms with gas heater)
    vTaskDelay(pdMS_TO_TICKS(200));

    // Check measurement status
    uint8_t status;
    spi_read_reg(REG_MEAS_STATUS_0, &status);
    bool new_data = (status & 0x80) != 0;

    if (!new_data) {
        // Retry once after short delay
        vTaskDelay(pdMS_TO_TICKS(50));
        spi_read_reg(REG_MEAS_STATUS_0, &status);
        new_data = (status & 0x80) != 0;
        if (!new_data) return ESP_ERR_TIMEOUT;
    }

    // Read raw data (pressure: 3 bytes, temp: 3 bytes, humidity: 2 bytes)
    uint8_t buf[8];
    esp_err_t err = spi_read_buf(REG_PRESS_MSB, buf, 8);
    if (err != ESP_OK) return err;

    uint32_t adc_press = ((uint32_t)buf[0] << 12) | ((uint32_t)buf[1] << 4) | (buf[2] >> 4);
    uint32_t adc_temp  = ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (buf[5] >> 4);
    uint16_t adc_hum   = ((uint16_t)buf[6] << 8) | buf[7];

    // Read gas resistance
    uint8_t gas_buf[2];
    spi_read_buf(REG_GAS_R_MSB, gas_buf, 2);
    uint16_t gas_adc  = ((uint16_t)gas_buf[0] << 2) | (gas_buf[1] >> 6);
    uint8_t gas_range  = gas_buf[1] & 0x0F;
    bool gas_valid     = (gas_buf[1] & 0x20) != 0;  // gas_valid_r bit
    bool heat_stab     = (gas_buf[1] & 0x10) != 0;  // heat_stab_r bit

    // Compensate
    data->temperature_c = compensate_temperature(adc_temp);
    data->pressure_pa   = compensate_pressure(adc_press);
    data->humidity_rh   = compensate_humidity(adc_hum, data->temperature_c);

    if (gas_valid && heat_stab) {
        data->gas_resistance = calc_gas_resistance(gas_adc, gas_range);
        data->gas_valid = true;
    } else {
        data->gas_resistance = 0.0f;
        data->gas_valid = false;
    }

    return ESP_OK;
}
