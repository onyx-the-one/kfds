#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ICM-42688-P 6-axis IMU data
typedef struct {
    float ax;   // m/s²
    float ay;
    float az;
    float gx;   // degrees/s
    float gy;
    float gz;
} icm42688p_data_t;

// Initialize the ICM-42688-P on the shared SPI bus.
// Do NOT call spi_bus_initialize() — only spi_bus_add_device().
esp_err_t icm42688p_init(spi_host_device_t host);

// Read accelerometer and gyroscope data.
esp_err_t icm42688p_read(icm42688p_data_t *data);

#ifdef __cplusplus
}
#endif
