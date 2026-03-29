#pragma once

#include "driver/spi_master.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Link quality metrics
typedef struct {
    int16_t rssi_dbm;       // last packet RSSI in dBm
    int8_t  snr_db_x10;     // last packet SNR in dB × 10
    int16_t rssi_inst_dbm;  // instantaneous RSSI
} e22_link_quality_t;

// Initialize the E22-900M22S (SX1262) on the shared SPI bus.
// Do NOT call spi_bus_initialize() — only spi_bus_add_device().
esp_err_t e22_lora_init(spi_host_device_t host);

// Transmit a buffer over LoRa. Blocks until TX complete or timeout.
esp_err_t e22_transmit(const uint8_t *data, size_t len);

// Set the radio to RX mode (continuous).
esp_err_t e22_start_rx(void);

// Check if a packet has been received. If yes, copies data to buf and returns
// the number of bytes received via out_len. Returns ESP_ERR_NOT_FOUND if no packet.
esp_err_t e22_receive(uint8_t *buf, size_t buf_size, size_t *out_len);

// Get link quality from last received packet.
void e22_get_link_quality(e22_link_quality_t *lq);

// Get instantaneous RSSI.
int16_t e22_get_rssi_inst(void);

#ifdef __cplusplus
}
#endif
