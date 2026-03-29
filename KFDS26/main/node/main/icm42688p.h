#pragma once

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

// Initialize the ICM-42688-P on the configured SPI bus.
// Returns true on success (WHO_AM_I validated, sensor configured).
bool icm42688p_init(void);

// Read accelerometer and gyroscope data.
// Returns true on success.
bool icm42688p_read(icm42688p_data_t *data);

#ifdef __cplusplus
}
#endif
