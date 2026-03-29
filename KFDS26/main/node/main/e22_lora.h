#pragma once

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

// Initialize the E22-900M22S (SX1262) on the configured SPI bus.
// Configures frequency, SF, BW, CR, TX power from node_config.h defaults.
// Returns true on success.
bool e22_init(void);

// Transmit a buffer over LoRa. Blocks until TX complete or timeout.
// Returns true on success.
bool e22_transmit(const uint8_t *data, size_t len);

// Set the radio to RX mode (continuous). Use e22_receive() to check for data.
bool e22_start_rx(void);

// Check if a packet has been received. If yes, copies data to buf and returns
// the number of bytes received. Returns 0 if no packet available.
// buf must be at least 256 bytes.
size_t e22_receive(uint8_t *buf, size_t buf_size);

// Get link quality from last received packet.
void e22_get_link_quality(e22_link_quality_t *lq);

// Get instantaneous RSSI (useful for channel assessment).
int16_t e22_get_rssi_inst(void);

#ifdef __cplusplus
}
#endif
