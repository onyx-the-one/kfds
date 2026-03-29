#include "icm42688p.h"
#include "node_config.h"

#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ICM42688P";

// -----------------------------------------------------------------------------
// Register map (Bank 0)
// -----------------------------------------------------------------------------
#define REG_DEVICE_CONFIG       0x11
#define REG_PWR_MGMT0          0x4E
#define REG_GYRO_CONFIG0       0x4F
#define REG_ACCEL_CONFIG0      0x50
#define REG_TEMP_DATA1         0x1D
#define REG_ACCEL_DATA_X1      0x1F
#define REG_GYRO_DATA_X1       0x25
#define REG_WHO_AM_I           0x75
#define REG_BANK_SEL           0x76

#define WHO_AM_I_VALUE         0x47

// PWR_MGMT0 bits
#define PWR_GYRO_LN            0x0C    // Gyro low-noise mode
#define PWR_ACCEL_LN           0x03    // Accel low-noise mode

// ACCEL_CONFIG0: ±16g (bits[6:5]=0b11), ODR 1kHz (bits[3:0]=0b0110)
#define ACCEL_FS_16G_1KHZ      0x66

// GYRO_CONFIG0: ±2000dps (bits[6:5]=0b00), ODR 1kHz (bits[3:0]=0b0110)
#define GYRO_FS_2000DPS_1KHZ   0x06

// Sensitivity at ±16g: 2048 LSB/g
#define ACCEL_SENSITIVITY      2048.0f
// Sensitivity at ±2000dps: 16.4 LSB/dps
#define GYRO_SENSITIVITY       16.4f
// Gravity constant
#define G_MPS2                 9.80665f

// SPI read bit
#define SPI_READ               0x80

// -----------------------------------------------------------------------------
// SPI handle
// -----------------------------------------------------------------------------
static spi_device_handle_t s_spi = NULL;

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
    uint8_t tx[2] = { static_cast<uint8_t>(reg | SPI_READ), 0x00 };
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
    tx[0] = reg | SPI_READ;

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

bool icm42688p_init(void)
{
    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = IMU_SPI_MOSI;
    bus_cfg.miso_io_num = IMU_SPI_MISO;
    bus_cfg.sclk_io_num = IMU_SPI_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 64;

    esp_err_t err = spi_bus_initialize(IMU_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Add device — SPI mode 0, clock per config
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = IMU_SPI_CLOCK_HZ;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = IMU_SPI_CS;
    dev_cfg.queue_size = 4;

    err = spi_bus_add_device(IMU_SPI_HOST, &dev_cfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    // Check WHO_AM_I
    uint8_t who = 0;
    if (spi_read_reg(REG_WHO_AM_I, &who) != ESP_OK || who != WHO_AM_I_VALUE) {
        ESP_LOGE(TAG, "WHO_AM_I mismatch: got 0x%02X, expected 0x%02X", who, WHO_AM_I_VALUE);
        return false;
    }
    ESP_LOGI(TAG, "ICM-42688-P detected (WHO_AM_I=0x%02X)", who);

    // Soft reset
    spi_write_reg(REG_DEVICE_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(2));

    // Select Bank 0
    spi_write_reg(REG_BANK_SEL, 0x00);

    // Configure gyro: ±2000 dps, 1 kHz ODR
    spi_write_reg(REG_GYRO_CONFIG0, GYRO_FS_2000DPS_1KHZ);

    // Configure accel: ±16 g, 1 kHz ODR
    spi_write_reg(REG_ACCEL_CONFIG0, ACCEL_FS_16G_1KHZ);

    // Enable accel + gyro in low-noise mode
    spi_write_reg(REG_PWR_MGMT0, PWR_GYRO_LN | PWR_ACCEL_LN);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "ICM-42688-P initialized: ±16g, ±2000dps, 1kHz ODR");
    return true;
}

bool icm42688p_read(icm42688p_data_t *data)
{
    if (!data || !s_spi) return false;

    // Read 12 bytes: accel XYZ (6) starting at 0x1F, then gyro XYZ (6) at 0x25
    // They are contiguous: 0x1F-0x24 (accel), 0x25-0x2A (gyro)
    uint8_t buf[12];
    if (spi_read_buf(REG_ACCEL_DATA_X1, buf, 12) != ESP_OK) {
        return false;
    }

    // Data is big-endian 16-bit signed
    int16_t ax_raw = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay_raw = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az_raw = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx_raw = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gy_raw = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gz_raw = (int16_t)((buf[10] << 8) | buf[11]);

    data->ax = (ax_raw / ACCEL_SENSITIVITY) * G_MPS2;
    data->ay = (ay_raw / ACCEL_SENSITIVITY) * G_MPS2;
    data->az = (az_raw / ACCEL_SENSITIVITY) * G_MPS2;

    data->gx = gx_raw / GYRO_SENSITIVITY;
    data->gy = gy_raw / GYRO_SENSITIVITY;
    data->gz = gz_raw / GYRO_SENSITIVITY;

    return true;
}
