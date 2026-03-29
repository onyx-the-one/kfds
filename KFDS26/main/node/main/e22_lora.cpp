#include "e22_lora.h"
#include "node_config.h"

#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "E22_LORA";

// -----------------------------------------------------------------------------
// SX1262 command opcodes
// -----------------------------------------------------------------------------
#define SX_CMD_SET_SLEEP                0x84
#define SX_CMD_SET_STANDBY              0x80
#define SX_CMD_SET_FS                   0xC1
#define SX_CMD_SET_TX                   0x83
#define SX_CMD_SET_RX                   0x82
#define SX_CMD_SET_RF_FREQUENCY         0x86
#define SX_CMD_SET_PACKET_TYPE          0x8A
#define SX_CMD_SET_MODULATION_PARAMS    0x8B
#define SX_CMD_SET_PACKET_PARAMS        0x8C
#define SX_CMD_SET_TX_PARAMS            0x8E
#define SX_CMD_SET_BUFFER_BASE_ADDR     0x8F
#define SX_CMD_SET_DIO_IRQ_PARAMS       0x08
#define SX_CMD_CLEAR_IRQ_STATUS         0x02
#define SX_CMD_GET_IRQ_STATUS           0x12
#define SX_CMD_GET_RX_BUFFER_STATUS     0x13
#define SX_CMD_GET_PACKET_STATUS        0x14
#define SX_CMD_GET_RSSI_INST            0x15
#define SX_CMD_WRITE_BUFFER             0x0E
#define SX_CMD_READ_BUFFER              0x1E
#define SX_CMD_WRITE_REGISTER           0x0D
#define SX_CMD_READ_REGISTER            0x1D
#define SX_CMD_SET_DIO2_AS_RF_SWITCH    0x9D
#define SX_CMD_SET_DIO3_AS_TCXO_CTRL   0x97
#define SX_CMD_CALIBRATE                0x89
#define SX_CMD_SET_PA_CONFIG            0x95
#define SX_CMD_GET_STATUS               0xC0

// Standby modes
#define STDBY_RC                        0x00
#define STDBY_XOSC                      0x01

// Packet type
#define PACKET_TYPE_LORA                0x01

// IRQ masks
#define IRQ_TX_DONE                     0x0001
#define IRQ_RX_DONE                     0x0002
#define IRQ_TIMEOUT                     0x0200
#define IRQ_ALL                         0x03FF

// SX1262 register for sync word
#define REG_LORA_SYNC_WORD_MSB          0x0740

// TX timeout: 10 seconds in 15.625 µs steps
#define TX_TIMEOUT_10S                  640000

// RX continuous
#define RX_CONTINUOUS                   0xFFFFFF

// -----------------------------------------------------------------------------
// SPI handle and state
// -----------------------------------------------------------------------------
static spi_device_handle_t s_spi = NULL;
static e22_link_quality_t  s_last_lq = {};

// -----------------------------------------------------------------------------
// BUSY pin polling
// -----------------------------------------------------------------------------
static void wait_busy(void)
{
    while (gpio_get_level((gpio_num_t)LORA_PIN_BUSY)) {
        vTaskDelay(1);
    }
}

// -----------------------------------------------------------------------------
// Low-level SPI command helpers
// -----------------------------------------------------------------------------
static esp_err_t sx_cmd(const uint8_t *cmd, size_t cmd_len,
                        uint8_t *resp, size_t resp_len)
{
    wait_busy();

    size_t total = cmd_len + resp_len;
    uint8_t tx[total];
    uint8_t rx[total];
    memset(tx, 0x00, total);
    memset(rx, 0x00, total);
    memcpy(tx, cmd, cmd_len);

    spi_transaction_t t = {};
    t.length = total * 8;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    esp_err_t err = spi_device_transmit(s_spi, &t);
    if (err == ESP_OK && resp && resp_len > 0) {
        memcpy(resp, rx + cmd_len, resp_len);
    }
    return err;
}

static esp_err_t sx_cmd_no_resp(const uint8_t *cmd, size_t len)
{
    return sx_cmd(cmd, len, NULL, 0);
}

static void sx_set_standby(uint8_t mode)
{
    uint8_t cmd[] = { SX_CMD_SET_STANDBY, mode };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_write_register(uint16_t addr, const uint8_t *data, size_t len)
{
    uint8_t cmd[3 + len];
    cmd[0] = SX_CMD_WRITE_REGISTER;
    cmd[1] = (addr >> 8) & 0xFF;
    cmd[2] = addr & 0xFF;
    memcpy(cmd + 3, data, len);
    sx_cmd_no_resp(cmd, 3 + len);
}

static void sx_write_buffer(uint8_t offset, const uint8_t *data, size_t len)
{
    uint8_t cmd[2 + len];
    cmd[0] = SX_CMD_WRITE_BUFFER;
    cmd[1] = offset;
    memcpy(cmd + 2, data, len);
    sx_cmd_no_resp(cmd, 2 + len);
}

static void sx_read_buffer(uint8_t offset, uint8_t *data, size_t len)
{
    uint8_t cmd[] = { SX_CMD_READ_BUFFER, offset, 0x00 }; // NOP status byte
    sx_cmd(cmd, sizeof(cmd), data, len);
}

static void sx_set_rf_frequency(uint32_t freq_hz)
{
    // freq_reg = freq_hz * 2^25 / 32e6
    uint32_t freq_reg = (uint32_t)((uint64_t)freq_hz * (1ULL << 25) / 32000000ULL);
    uint8_t cmd[] = {
        SX_CMD_SET_RF_FREQUENCY,
        (uint8_t)((freq_reg >> 24) & 0xFF),
        (uint8_t)((freq_reg >> 16) & 0xFF),
        (uint8_t)((freq_reg >> 8) & 0xFF),
        (uint8_t)(freq_reg & 0xFF)
    };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_packet_type(uint8_t type)
{
    uint8_t cmd[] = { SX_CMD_SET_PACKET_TYPE, type };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

// SF: 5-12, BW: 0=7.8k,1=10.4k,...,4=31.25k,5=62.5k,6=125k,7=250k,8=500k
// CR: 1=4/5, 2=4/6, 3=4/7, 4=4/8
static uint8_t bw_to_sx(uint32_t bw_hz)
{
    if (bw_hz <= 7800)   return 0x00;
    if (bw_hz <= 10400)  return 0x08;
    if (bw_hz <= 15600)  return 0x01;
    if (bw_hz <= 20800)  return 0x09;
    if (bw_hz <= 31250)  return 0x02;
    if (bw_hz <= 41700)  return 0x0A;
    if (bw_hz <= 62500)  return 0x03;
    if (bw_hz <= 125000) return 0x04;
    if (bw_hz <= 250000) return 0x05;
    return 0x06; // 500kHz
}

static void sx_set_modulation_params(uint8_t sf, uint32_t bw_hz, uint8_t cr)
{
    uint8_t bw = bw_to_sx(bw_hz);
    // LowDataRateOptimize: enable if symbol time > 16ms
    uint8_t ldro = 0;
    if (sf >= 11 && bw <= 0x04) ldro = 1;

    uint8_t cmd[] = { SX_CMD_SET_MODULATION_PARAMS, sf, bw, (uint8_t)(cr - 4), ldro };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_packet_params(uint16_t preamble, bool implicit_header,
                                  uint8_t payload_len, bool crc_on, bool invert_iq)
{
    uint8_t cmd[] = {
        SX_CMD_SET_PACKET_PARAMS,
        (uint8_t)((preamble >> 8) & 0xFF),
        (uint8_t)(preamble & 0xFF),
        implicit_header ? (uint8_t)0x01 : (uint8_t)0x00,
        payload_len,
        crc_on ? (uint8_t)0x01 : (uint8_t)0x00,
        invert_iq ? (uint8_t)0x01 : (uint8_t)0x00
    };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_tx_params(int8_t power_dbm, uint8_t ramp_time)
{
    uint8_t cmd[] = { SX_CMD_SET_TX_PARAMS, (uint8_t)power_dbm, ramp_time };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_pa_config(uint8_t pa_duty, uint8_t hp_max, uint8_t dev_sel, uint8_t pa_lut)
{
    uint8_t cmd[] = { SX_CMD_SET_PA_CONFIG, pa_duty, hp_max, dev_sel, pa_lut };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_buffer_base_addr(uint8_t tx_base, uint8_t rx_base)
{
    uint8_t cmd[] = { SX_CMD_SET_BUFFER_BASE_ADDR, tx_base, rx_base };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_dio_irq_params(uint16_t irq_mask, uint16_t dio1_mask,
                                   uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t cmd[] = {
        SX_CMD_SET_DIO_IRQ_PARAMS,
        (uint8_t)(irq_mask >> 8), (uint8_t)(irq_mask),
        (uint8_t)(dio1_mask >> 8), (uint8_t)(dio1_mask),
        (uint8_t)(dio2_mask >> 8), (uint8_t)(dio2_mask),
        (uint8_t)(dio3_mask >> 8), (uint8_t)(dio3_mask)
    };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static uint16_t sx_get_irq_status(void)
{
    uint8_t cmd[] = { SX_CMD_GET_IRQ_STATUS, 0x00 };
    uint8_t resp[2] = {};
    sx_cmd(cmd, sizeof(cmd), resp, 2);
    return ((uint16_t)resp[0] << 8) | resp[1];
}

static void sx_clear_irq(uint16_t mask)
{
    uint8_t cmd[] = { SX_CMD_CLEAR_IRQ_STATUS, (uint8_t)(mask >> 8), (uint8_t)(mask) };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_tx(uint32_t timeout_us)
{
    // Timeout in 15.625µs steps
    uint32_t to = (timeout_us * 64) / 1000;
    uint8_t cmd[] = {
        SX_CMD_SET_TX,
        (uint8_t)((to >> 16) & 0xFF),
        (uint8_t)((to >> 8) & 0xFF),
        (uint8_t)(to & 0xFF)
    };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_rx(uint32_t timeout_code)
{
    uint8_t cmd[] = {
        SX_CMD_SET_RX,
        (uint8_t)((timeout_code >> 16) & 0xFF),
        (uint8_t)((timeout_code >> 8) & 0xFF),
        (uint8_t)(timeout_code & 0xFF)
    };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

static void sx_set_dio2_as_rf_switch(bool enable)
{
    uint8_t cmd[] = { SX_CMD_SET_DIO2_AS_RF_SWITCH, enable ? (uint8_t)0x01 : (uint8_t)0x00 };
    sx_cmd_no_resp(cmd, sizeof(cmd));
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool e22_init(void)
{
    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = LORA_SPI_MOSI;
    bus_cfg.miso_io_num = LORA_SPI_MISO;
    bus_cfg.sclk_io_num = LORA_SPI_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 256 + 16;

    esp_err_t err = spi_bus_initialize(LORA_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(err));
        return false;
    }

    // Add SPI device — SX1262 uses CPOL=0, CPHA=0 (mode 0)
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = 8 * 1000 * 1000; // 8 MHz
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = LORA_SPI_CS;
    dev_cfg.queue_size = 4;

    err = spi_bus_add_device(LORA_SPI_HOST, &dev_cfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        return false;
    }

    // Configure GPIO pins
    gpio_config_t io_conf = {};

    // BUSY pin — input
    io_conf.pin_bit_mask = (1ULL << LORA_PIN_BUSY);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    // DIO1 pin — input
    io_conf.pin_bit_mask = (1ULL << LORA_PIN_DIO1);
    gpio_config(&io_conf);

    // RST pin — output
    io_conf.pin_bit_mask = (1ULL << LORA_PIN_RST);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // Hardware reset
    gpio_set_level((gpio_num_t)LORA_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)LORA_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(20));

    // Wait for BUSY to go low
    wait_busy();

    // Set to standby RC
    sx_set_standby(STDBY_RC);

    // Set DIO2 as RF switch control (E22 module uses this)
    sx_set_dio2_as_rf_switch(true);

    // Set packet type to LoRa
    sx_set_packet_type(PACKET_TYPE_LORA);

    // Set RF frequency
    sx_set_rf_frequency(LORA_FREQUENCY_HZ);

    // PA config for SX1262: paDutyCycle=0x04, hpMax=0x07, deviceSel=0x00(SX1262), paLut=0x01
    sx_set_pa_config(0x04, 0x07, 0x00, 0x01);

    // TX params: power, ramp time 200us
    sx_set_tx_params(LORA_TX_POWER_DBM, 0x04);

    // Modulation params
    sx_set_modulation_params(LORA_SF, LORA_BW, LORA_CR);

    // Packet params: explicit header, CRC on, no IQ inversion
    sx_set_packet_params(LORA_PREAMBLE_LEN, false, 255, true, false);

    // Set sync word (LoRa private network)
    uint8_t sw[2] = {
        (uint8_t)((LORA_SYNC_WORD & 0xF0) | 0x04),
        (uint8_t)(((LORA_SYNC_WORD & 0x0F) << 4) | 0x04)
    };
    sx_write_register(REG_LORA_SYNC_WORD_MSB, sw, 2);

    // Buffer base addresses
    sx_set_buffer_base_addr(0x00, 0x00);

    // IRQ: TX done + RX done + timeout on DIO1
    sx_set_dio_irq_params(IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT,
                          IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT,
                          0x0000, 0x0000);

    // Clear any pending IRQs
    sx_clear_irq(IRQ_ALL);

    ESP_LOGI(TAG, "E22 (SX1262) initialized: %lu Hz, SF%d, BW%lu, CR4/%d, %d dBm",
             (unsigned long)LORA_FREQUENCY_HZ, LORA_SF,
             (unsigned long)LORA_BW, LORA_CR, LORA_TX_POWER_DBM);
    return true;
}

bool e22_transmit(const uint8_t *data, size_t len)
{
    if (!data || len == 0 || len > 255 || !s_spi) return false;

    sx_set_standby(STDBY_RC);

    // Update packet params with actual payload length
    sx_set_packet_params(LORA_PREAMBLE_LEN, false, (uint8_t)len, true, false);

    // Write payload to FIFO
    sx_write_buffer(0x00, data, len);

    // Clear IRQs
    sx_clear_irq(IRQ_ALL);

    // Start TX with 10 second timeout
    sx_set_tx(10000000); // 10s in µs

    // Wait for TX done or timeout (poll DIO1 or IRQ status)
    uint32_t start = (uint32_t)(esp_timer_get_time() / 1000ULL);
    while (true) {
        uint16_t irq = sx_get_irq_status();
        if (irq & IRQ_TX_DONE) {
            sx_clear_irq(IRQ_ALL);
            return true;
        }
        if (irq & IRQ_TIMEOUT) {
            ESP_LOGW(TAG, "TX timeout");
            sx_clear_irq(IRQ_ALL);
            sx_set_standby(STDBY_RC);
            return false;
        }

        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
        if (now - start > 12000) {
            ESP_LOGE(TAG, "TX hard timeout");
            sx_set_standby(STDBY_RC);
            return false;
        }
        vTaskDelay(1);
    }
}

bool e22_start_rx(void)
{
    if (!s_spi) return false;

    sx_set_standby(STDBY_RC);
    sx_clear_irq(IRQ_ALL);

    // Packet params for max-length RX
    sx_set_packet_params(LORA_PREAMBLE_LEN, false, 255, true, false);

    // Continuous RX
    sx_set_rx(RX_CONTINUOUS);
    return true;
}

size_t e22_receive(uint8_t *buf, size_t buf_size)
{
    if (!buf || buf_size == 0 || !s_spi) return 0;

    uint16_t irq = sx_get_irq_status();
    if (!(irq & IRQ_RX_DONE)) return 0;

    sx_clear_irq(IRQ_ALL);

    // Get RX buffer status: payload length and start offset
    uint8_t cmd_status[] = { SX_CMD_GET_RX_BUFFER_STATUS, 0x00 };
    uint8_t status_resp[2] = {};
    sx_cmd(cmd_status, sizeof(cmd_status), status_resp, 2);

    uint8_t payload_len = status_resp[0];
    uint8_t rx_start    = status_resp[1];

    if (payload_len == 0 || payload_len > buf_size) return 0;

    // Read payload
    sx_read_buffer(rx_start, buf, payload_len);

    // Get packet status for link quality
    uint8_t cmd_pkt[] = { SX_CMD_GET_PACKET_STATUS, 0x00 };
    uint8_t pkt_resp[3] = {};
    sx_cmd(cmd_pkt, sizeof(cmd_pkt), pkt_resp, 3);

    s_last_lq.rssi_dbm     = -(int16_t)pkt_resp[0] / 2;
    s_last_lq.snr_db_x10   = ((int8_t)pkt_resp[1]) * 10 / 4;
    s_last_lq.rssi_inst_dbm = -(int16_t)pkt_resp[2] / 2;

    return payload_len;
}

void e22_get_link_quality(e22_link_quality_t *lq)
{
    if (lq) *lq = s_last_lq;
}

int16_t e22_get_rssi_inst(void)
{
    uint8_t cmd[] = { SX_CMD_GET_RSSI_INST, 0x00 };
    uint8_t resp[1] = {};
    sx_cmd(cmd, sizeof(cmd), resp, 1);
    return -(int16_t)resp[0] / 2;
}
