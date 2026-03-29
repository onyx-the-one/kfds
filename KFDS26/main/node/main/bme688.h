#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// BME688 environmental + gas sensor data
typedef struct {
    float temperature_c;    // °C
    float pressure_pa;      // Pa
    float humidity_rh;      // %RH
    float gas_resistance;   // Ohm
    bool  gas_valid;        // true if gas measurement completed
} bme688_data_t;

// Initialize BME688 on the shared SPI bus.
// Do NOT call spi_bus_initialize() — only spi_bus_add_device().
esp_err_t bme688_init(spi_host_device_t host);

// Read compensated T, P, H, and gas resistance (forced mode).
esp_err_t bme688_read(bme688_data_t *data);

#ifdef __cplusplus
}
#endif
