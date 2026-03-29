#pragma once

#include <stdbool.h>
#include <stdint.h>

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

// Initialize I2C bus and detect BME688 (scans 0x76 then 0x77).
// Returns true on success.
bool bme688_init(void);

// Read compensated T, P, H, and gas resistance.
// Returns true on success.
bool bme688_read(bme688_data_t *data);

#ifdef __cplusplus
}
#endif
