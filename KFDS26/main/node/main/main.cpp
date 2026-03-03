// main.cpp
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "polysense_proto.h"

// -------- Configuration --------

// I2C (same pins as Arduino sketch)
#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   GPIO_NUM_5
#define I2C_MASTER_SCL_IO   GPIO_NUM_4
#define I2C_MASTER_FREQ_HZ  400000

// UART: adjust if your board uses different pins
#define UART_PORT           UART_NUM_0
#define UART_BAUDRATE       115200
#define UART_TX_PIN         GPIO_NUM_43
#define UART_RX_PIN         GPIO_NUM_44

// BME280 & MPU6500 I2C addresses
#define BME280_ADDR_1       0x76
#define BME280_ADDR_2       0x77
#define MPU6500_ADDR        0x68

static const char *TAG = "POLYSENSE";

// -------- I2C init --------

static esp_err_t i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(
        I2C_MASTER_NUM, addr, &reg, 1, data, len, pdMS_TO_TICKS(100)
    );
}

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(
        I2C_MASTER_NUM, addr, buf, 2, pdMS_TO_TICKS(100)
    );
}

// -------- BME280 minimal driver with compensation --------

static uint8_t g_bme280_addr = BME280_ADDR_1;

// Calibration registers
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t  dig_H1;
static int16_t  dig_H2, dig_H3;
static int16_t  dig_H4, dig_H5;
static int8_t   dig_H6;
static int32_t  t_fine;

static bool bme280_read_calib()
{
    uint8_t buf[26];

    // T1..T3, P1..P9
    if (i2c_read_reg(g_bme280_addr, 0x88, buf, 24) != ESP_OK) return false;

    dig_T1 = (uint16_t)(buf[1] << 8 | buf[0]);
    dig_T2 = (int16_t)(buf[3] << 8 | buf[2]);
    dig_T3 = (int16_t)(buf[5] << 8 | buf[4]);

    dig_P1 = (uint16_t)(buf[7] << 8 | buf[6]);
    dig_P2 = (int16_t)(buf[9] << 8 | buf[8]);
    dig_P3 = (int16_t)(buf[11] << 8 | buf[10]);
    dig_P4 = (int16_t)(buf[13] << 8 | buf[12]);
    dig_P5 = (int16_t)(buf[15] << 8 | buf[14]);
    dig_P6 = (int16_t)(buf[17] << 8 | buf[16]);
    dig_P7 = (int16_t)(buf[19] << 8 | buf[18]);
    dig_P8 = (int16_t)(buf[21] << 8 | buf[20]);
    dig_P9 = (int16_t)(buf[23] << 8 | buf[22]);

    // H1
    if (i2c_read_reg(g_bme280_addr, 0xA1, &dig_H1, 1) != ESP_OK) return false;

    // H2..H6
    uint8_t hbuf[7];
    if (i2c_read_reg(g_bme280_addr, 0xE1, hbuf, 7) != ESP_OK) return false;

    dig_H2 = (int16_t)(hbuf[1] << 8 | hbuf[0]);
    dig_H3 = hbuf[2];
    dig_H4 = (int16_t)((hbuf[3] << 4) | (hbuf[4] & 0x0F));
    dig_H5 = (int16_t)((hbuf[5] << 4) | (hbuf[4] >> 4));
    dig_H6 = (int8_t)hbuf[6];

    return true;
}

static int32_t bme280_compensate_T(int32_t adc_T)
{
    int32_t var1  = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
                     ((int32_t)dig_T2)) >> 11;
    int32_t var2  = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                      ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                     ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;
    int32_t T  = (t_fine * 5 + 128) >> 8; // °C * 100
    return T;
}

static uint32_t bme280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
           ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1) *
            ((int64_t)dig_P1)) >> 33;

    if (var1 == 0) return 0; // avoid div by zero

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return (uint32_t)p; // Pa * 256; we will divide later
}

static uint32_t bme280_compensate_H(int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = t_fine - ((int32_t)76800);
    v_x1_u32r = (((((adc_H << 14) -
                    (((int32_t)dig_H4) << 20) -
                    (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                  (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                       (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                     ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r -
                 (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                   ((int32_t)dig_H1)) >> 4));
    if (v_x1_u32r < 0) v_x1_u32r = 0;
    if (v_x1_u32r > 419430400) v_x1_u32r = 419430400;
    return (uint32_t)(v_x1_u32r >> 12); // %RH * 1024
}

static bool bme280_detect_and_init()
{
    uint8_t id = 0;
    uint8_t reg = 0xD0;

    if (i2c_master_write_read_device(
            I2C_MASTER_NUM, BME280_ADDR_1, &reg, 1,
            &id, 1, pdMS_TO_TICKS(100)) == ESP_OK && id == 0x60) {
        g_bme280_addr = BME280_ADDR_1;
    } else if (i2c_master_write_read_device(
                   I2C_MASTER_NUM, BME280_ADDR_2, &reg, 1,
                   &id, 1, pdMS_TO_TICKS(100)) == ESP_OK && id == 0x60) {
        g_bme280_addr = BME280_ADDR_2;
    } else {
        ESP_LOGE(TAG, "BME280 not found");
        return false;
    }

    // Reset
    i2c_write_reg(g_bme280_addr, 0xE0, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    if (!bme280_read_calib()) {
        ESP_LOGE(TAG, "BME280 calib read failed");
        return false;
    }

    // ctrl_hum: oversampling x1
    i2c_write_reg(g_bme280_addr, 0xF2, 0x01);
    // ctrl_meas: temp x1, pressure x1, normal mode (0b00100111)
    i2c_write_reg(g_bme280_addr, 0xF4, 0x27);
    // config: standby 0.5ms, filter off
    i2c_write_reg(g_bme280_addr, 0xF5, 0x00);

    ESP_LOGI(TAG, "BME280 at 0x%02X", g_bme280_addr);
    return true;
}

static bool bme280_read(float *T_c, float *P_pa, float *H_rh)
{
    uint8_t data[8];
    if (i2c_read_reg(g_bme280_addr, 0xF7, data, 8) != ESP_OK) return false;

    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) |
                              ((uint32_t)data[1] << 4) |
                              ((uint32_t)data[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)data[3] << 12) |
                              ((uint32_t)data[4] << 4) |
                              ((uint32_t)data[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)data[6] << 8) | data[7]);

    if (adc_T == 0x800000 || adc_P == 0x800000 || adc_H == 0x8000) return false;

    int32_t T_x100 = bme280_compensate_T(adc_T);
    uint32_t P_raw = bme280_compensate_P(adc_P);  // Pa * 256
    uint32_t H_raw = bme280_compensate_H(adc_H);  // %RH * 1024

    *T_c  = T_x100 / 100.0f;
    *P_pa = (float)P_raw / 256.0f;
    *H_rh = (float)H_raw / 1024.0f;
    return true;
}

// -------- MPU6500 minimal driver --------

static esp_err_t mpu6500_init()
{
    uint8_t buf[2];

    // Wake, PLL with X gyro
    buf[0] = 0x6B; buf[1] = 0x01;
    if (i2c_master_write_to_device(I2C_MASTER_NUM, MPU6500_ADDR,
                                   buf, 2, pdMS_TO_TICKS(100)) != ESP_OK)
        return ESP_FAIL;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Gyro: ±2000 dps
    buf[0] = 0x1B; buf[1] = 0x18;
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6500_ADDR,
                               buf, 2, pdMS_TO_TICKS(100));
    // Accel: ±16 g
    buf[0] = 0x1C; buf[1] = 0x18;
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6500_ADDR,
                               buf, 2, pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "MPU6500 init done");
    return ESP_OK;
}

static bool mpu6500_read(float *ax, float *ay, float *az,
                         float *gx, float *gy, float *gz)
{
    uint8_t buf[14];
    uint8_t reg = 0x3B;
    if (i2c_master_write_read_device(
            I2C_MASTER_NUM, MPU6500_ADDR, &reg, 1,
            buf, 14, pdMS_TO_TICKS(100)) != ESP_OK)
        return false;

    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx_raw = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy_raw = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz_raw = (int16_t)((buf[12] << 8) | buf[13]);

    const float accel_sens = 1.0f / 2048.0f; // 16g
    const float gyro_sens  = 1.0f / 16.4f;   // 2000 dps

    *ax = ax_raw * accel_sens * 9.80665f;
    *ay = ay_raw * accel_sens * 9.80665f;
    *az = az_raw * accel_sens * 9.80665f;

    *gx = gx_raw * gyro_sens;
    *gy = gy_raw * gyro_sens;
    *gz = gz_raw * gyro_sens;

    return true;
}

// -------- UART ------

static void uart_init()
{
    uart_config_t uart_config;
    memset(&uart_config, 0, sizeof(uart_config));  // ensure all fields = 0

    uart_config.baud_rate = UART_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity    = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk= UART_SCLK_DEFAULT;
    // rx_flow_ctrl_thresh, flags, backup_before_sleep all remain 0

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

// -------- Main loop --------

static uint32_t env_sample_id = 0;
static uint32_t imu_sample_id = 0;

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "PolySense backup node starting");

    ESP_ERROR_CHECK(i2c_master_init());
    bme280_detect_and_init();
    mpu6500_init();
    uart_init();

    const uint8_t node_id = 1;
    uint8_t tx_buf[128];

    while (true) {
        uint32_t uptime_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

        // ENV_TELEM
        float T_c = 0.0f, P_pa = 0.0f, H_rh = 0.0f;
        bool env_ok = bme280_read(&T_c, &P_pa, &H_rh);

        ps_env_telem_t env = {};
        env.uptime_ms          = uptime_ms;
        env.sample_id          = env_sample_id++;
        env.temperature_c_x100 = (int16_t)(T_c * 100.0f);
        env.pressure_pa        = (uint32_t)P_pa;
        env.humidity_rh_x100   = (uint16_t)(H_rh * 100.0f);
        env.env_flags          = 0;
        if (env_ok) {
            env.env_flags |= (1 << 0) | (1 << 1) | (1 << 2);
        }

        size_t env_len = ps_build_frame(
            tx_buf, sizeof(tx_buf),
            node_id,
            PS_MSG_ENV_TELEM,
            (const uint8_t *)&env,
            sizeof(env)
        );
        if (env_len > 0) {
            uart_write_bytes(UART_PORT, (const char *)tx_buf, env_len);
        }

        // IMU_TELEM
        float ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
        bool imu_ok = mpu6500_read(&ax, &ay, &az, &gx, &gy, &gz);

        ps_imu_telem_t imu = {};
        imu.uptime_ms       = uptime_ms;
        imu.sample_id       = imu_sample_id++;
        imu.ax_mps2_x1000   = (int16_t)(ax * 1000.0f);
        imu.ay_mps2_x1000   = (int16_t)(ay * 1000.0f);
        imu.az_mps2_x1000   = (int16_t)(az * 1000.0f);
        imu.gx_dps_x1000    = (int16_t)(gx * 1000.0f);
        imu.gy_dps_x1000    = (int16_t)(gy * 1000.0f);
        imu.gz_dps_x1000    = (int16_t)(gz * 1000.0f);
        imu.imu_flags       = imu_ok ? ((1 << 0) | (1 << 1)) : 0;

        size_t imu_len = ps_build_frame(
            tx_buf, sizeof(tx_buf),
            node_id,
            PS_MSG_IMU_TELEM,
            (const uint8_t *)&imu,
            sizeof(imu)
        );
        if (imu_len > 0) {
            uart_write_bytes(UART_PORT, (const char *)tx_buf, imu_len);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // ~5 Hz
    }
}
