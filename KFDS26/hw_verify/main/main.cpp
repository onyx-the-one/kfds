#include <cstring>
#include <cstdio>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "led_strip.h"

static const char *TAG = "HW_VERIFY";

// ── Board wiring ─────────────────────────────────────────────────────────
static constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_11;
static constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_13;
static constexpr gpio_num_t PIN_SPI_SCLK = GPIO_NUM_12;

static constexpr gpio_num_t PIN_CS_ENV  = GPIO_NUM_9;   // BME688
static constexpr gpio_num_t PIN_CS_IMU  = GPIO_NUM_10;  // ICM-42688-P
static constexpr gpio_num_t PIN_CS_LORA = GPIO_NUM_14;  // SX1262 NSS
static constexpr gpio_num_t PIN_CS_GPS  = GPIO_NUM_15;  // NEO-M9N CS

static constexpr gpio_num_t PIN_LORA_BUSY = GPIO_NUM_7;
static constexpr gpio_num_t PIN_LORA_DIO1 = GPIO_NUM_8;
static constexpr gpio_num_t PIN_LORA_RST  = GPIO_NUM_16;

static constexpr gpio_num_t PIN_NEOPIXEL = GPIO_NUM_38;

static constexpr spi_host_device_t SPI_HOST = SPI2_HOST;
static constexpr int SPI_HZ = 100000;

// ── Shared buffers ───────────────────────────────────────────────────────
static uint8_t s_tx[512] __attribute__((aligned(4)));
static uint8_t s_rx[512] __attribute__((aligned(4)));

static led_strip_handle_t s_led = nullptr;
static bool s_any_fail = false;

// ── Utilities ────────────────────────────────────────────────────────────
static void set_fail()
{
    s_any_fail = true;
}

static size_t count_equal(const uint8_t *buf, size_t len, uint8_t v)
{
    size_t n = 0;
    for (size_t i = 0; i < len; ++i) {
        if (buf[i] == v) {
            ++n;
        }
    }
    return n;
}

static void dump_hex(const char *label, const uint8_t *buf, size_t len)
{
    char line[3 * 32 + 1];
    for (size_t i = 0; i < len; i += 32) {
        size_t chunk = (len - i > 32) ? 32 : (len - i);
        size_t off = 0;
        for (size_t j = 0; j < chunk; ++j) {
            off += snprintf(line + off, sizeof(line) - off, "%02X ", buf[i + j]);
        }
        line[(off < sizeof(line)) ? off : (sizeof(line) - 1)] = '\0';

        ESP_LOGI(TAG, "%s [%03u..%03u]: %s",
                 label,
                 (unsigned)i,
                 (unsigned)(i + chunk - 1),
                 line);
    }
}

static void log_pin_levels(const char *where)
{
    ESP_LOGI(TAG,
             "[%s] MOSI=%d MISO=%d SCLK=%d | CS_ENV=%d CS_IMU=%d CS_LORA=%d CS_GPS=%d | BUSY=%d DIO1=%d RST=%d",
             where,
             gpio_get_level(PIN_SPI_MOSI),
             gpio_get_level(PIN_SPI_MISO),
             gpio_get_level(PIN_SPI_SCLK),
             gpio_get_level(PIN_CS_ENV),
             gpio_get_level(PIN_CS_IMU),
             gpio_get_level(PIN_CS_LORA),
             gpio_get_level(PIN_CS_GPS),
             gpio_get_level(PIN_LORA_BUSY),
             gpio_get_level(PIN_LORA_DIO1),
             gpio_get_level(PIN_LORA_RST));
}

static void miso_bias_test(const char *where)
{
    gpio_set_direction(PIN_SPI_MISO, GPIO_MODE_INPUT);

    gpio_set_pull_mode(PIN_SPI_MISO, GPIO_PULLUP_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    int up = gpio_get_level(PIN_SPI_MISO);

    gpio_set_pull_mode(PIN_SPI_MISO, GPIO_PULLDOWN_ONLY);
    vTaskDelay(pdMS_TO_TICKS(5));
    int down = gpio_get_level(PIN_SPI_MISO);

    gpio_set_pull_mode(PIN_SPI_MISO, GPIO_FLOATING);
    vTaskDelay(pdMS_TO_TICKS(2));
    int floating = gpio_get_level(PIN_SPI_MISO);

    ESP_LOGI(TAG, "[%s] MISO bias test: pullup=%d pulldown=%d floating=%d",
             where, up, down, floating);
}

static void led_init()
{
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = PIN_NEOPIXEL;
    strip_config.max_leds = 1;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.resolution_hz = 10 * 1000 * 1000;
    rmt_config.flags.with_dma = false;

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "led_strip_new_rmt_device failed: %s", esp_err_to_name(err));
        s_led = nullptr;
    }
}

static void led_set(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led) {
        return;
    }
    led_strip_set_pixel(s_led, 0, r, g, b);
    led_strip_refresh(s_led);
}

static void init_gpio()
{
    gpio_reset_pin(PIN_CS_ENV);
    gpio_reset_pin(PIN_CS_IMU);
    gpio_reset_pin(PIN_CS_LORA);
    gpio_reset_pin(PIN_CS_GPS);
    gpio_reset_pin(PIN_LORA_RST);
    gpio_reset_pin(PIN_LORA_BUSY);
    gpio_reset_pin(PIN_LORA_DIO1);
    gpio_reset_pin(PIN_SPI_MISO);

    // OUTPUTs
    gpio_set_direction(PIN_CS_ENV, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_CS_IMU, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_CS_LORA, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_CS_GPS, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_direction(PIN_LORA_RST, GPIO_MODE_INPUT_OUTPUT);

    // INPUTs
    gpio_set_direction(PIN_LORA_BUSY, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_LORA_DIO1, GPIO_MODE_INPUT);
    gpio_set_direction(PIN_SPI_MISO, GPIO_MODE_INPUT);

    // Safe idle state first
    gpio_set_level(PIN_CS_ENV, 1);
    gpio_set_level(PIN_CS_IMU, 1);
    gpio_set_level(PIN_CS_LORA, 1);
    gpio_set_level(PIN_CS_GPS, 1);
    gpio_set_level(PIN_LORA_RST, 1);

    // Temporary substitute for external 10k pullups on CS lines
    gpio_set_pull_mode(PIN_CS_ENV, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS_IMU, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS_LORA, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_CS_GPS, GPIO_PULLUP_ONLY);

    // Radio status pins: weak default high bias, easier to spot active drive low/high transitions
    gpio_set_pull_mode(PIN_LORA_BUSY, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(PIN_LORA_DIO1, GPIO_PULLUP_ONLY);

    // MISO left floating for the bias/open-bus test
    gpio_set_pull_mode(PIN_SPI_MISO, GPIO_FLOATING);

    // Reset is a driven output; no internal pull required
    gpio_set_pull_mode(PIN_LORA_RST, GPIO_FLOATING);
}

static void cs_low(gpio_num_t pin)
{
    gpio_set_level(pin, 0);
    esp_rom_delay_us(3);
}

static void cs_high(gpio_num_t pin)
{
    esp_rom_delay_us(3);
    gpio_set_level(pin, 1);
    esp_rom_delay_us(3);
}

// ── SPI helpers ──────────────────────────────────────────────────────────
static esp_err_t spi_bus_up()
{
    spi_bus_config_t buscfg = {};
    buscfg.mosi_io_num = PIN_SPI_MOSI;
    buscfg.miso_io_num = PIN_SPI_MISO;
    buscfg.sclk_io_num = PIN_SPI_SCLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = sizeof(s_tx);
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER |
                   SPICOMMON_BUSFLAG_MOSI |
                   SPICOMMON_BUSFLAG_MISO |
                   SPICOMMON_BUSFLAG_SCLK |
                   SPICOMMON_BUSFLAG_IOMUX_PINS;

    esp_err_t err = spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_DISABLED);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

static spi_device_handle_t spi_add_dev(int mode, int hz)
{
    spi_device_interface_config_t cfg = {};
    cfg.clock_speed_hz = hz;
    cfg.mode = mode;
    cfg.spics_io_num = -1;
    cfg.queue_size = 1;

    spi_device_handle_t dev = nullptr;
    esp_err_t err = spi_bus_add_device(SPI_HOST, &cfg, &dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device(mode=%d hz=%d) failed: %s",
                 mode, hz, esp_err_to_name(err));
        return nullptr;
    }

    int khz = 0;
    err = spi_device_get_actual_freq(dev, &khz);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SPI device added: mode=%d req=%d Hz actual=%d kHz", mode, hz, khz);
    } else {
        ESP_LOGW(TAG, "spi_device_get_actual_freq failed: %s", esp_err_to_name(err));
    }

    return dev;
}

static void spi_remove_dev(spi_device_handle_t dev)
{
    if (dev) {
        esp_err_t err = spi_bus_remove_device(dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "spi_bus_remove_device failed: %s", esp_err_to_name(err));
        }
    }
}

static esp_err_t raw_xfer(spi_device_handle_t dev,
                          gpio_num_t cs_pin,
                          const uint8_t *tx,
                          uint8_t *rx,
                          size_t len,
                          const char *label)
{
    if (!dev || !tx || len == 0 || len > sizeof(s_tx)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(s_tx, 0, sizeof(s_tx));
    memset(s_rx, 0, sizeof(s_rx));
    memcpy(s_tx, tx, len);

    spi_transaction_t t = {};
    t.length = len * 8;
    t.tx_buffer = s_tx;
    t.rx_buffer = s_rx;

    ESP_LOGI(TAG, "---- %s ----", label);
    dump_hex("TX", s_tx, len);
    log_pin_levels("before");
    miso_bias_test("before");

    esp_err_t err = spi_device_acquire_bus(dev, portMAX_DELAY);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_acquire_bus failed: %s", esp_err_to_name(err));
        return err;
    }

    if (cs_pin != GPIO_NUM_NC) {
        cs_low(cs_pin);
    }

    err = spi_device_polling_transmit(dev, &t);

    if (cs_pin != GPIO_NUM_NC) {
        cs_high(cs_pin);
    }

    spi_device_release_bus(dev);

    if (rx && err == ESP_OK) {
        memcpy(rx, s_rx, len);
    }

    ESP_LOGI(TAG, "%s -> %s", label, esp_err_to_name(err));
    dump_hex("RX", s_rx, len);
    log_pin_levels("after");
    miso_bias_test("after");

    return err;
}

static esp_err_t reg_write(spi_device_handle_t dev, gpio_num_t cs,
                           uint8_t reg, uint8_t val, bool clear_msb)
{
    uint8_t tx[2] = {
        clear_msb ? static_cast<uint8_t>(reg & 0x7F) : reg,
        val
    };
    return raw_xfer(dev, cs, tx, nullptr, sizeof(tx), "reg_write");
}

static esp_err_t reg_read_1(spi_device_handle_t dev, gpio_num_t cs,
                            uint8_t reg, uint8_t *val, bool set_msb)
{
    uint8_t tx[2] = {
        set_msb ? static_cast<uint8_t>(reg | 0x80) : reg,
        0x00
    };
    uint8_t rx[2] = {};
    esp_err_t err = raw_xfer(dev, cs, tx, rx, sizeof(tx), "reg_read_1");
    if (err == ESP_OK && val) {
        *val = rx[1];
    }
    return err;
}

// ── Open-bus diagnostic ──────────────────────────────────────────────────
static void probe_open_bus()
{
    ESP_LOGI(TAG, "================ OPEN BUS SAMPLE ================");
    spi_device_handle_t dev = spi_add_dev(0, SPI_HZ);
    if (!dev) {
        set_fail();
        return;
    }

    uint8_t tx[32];
    uint8_t rx[32];
    memset(tx, 0xFF, sizeof(tx));
    memset(rx, 0, sizeof(rx));

    esp_err_t err = raw_xfer(dev, GPIO_NUM_NC, tx, rx, sizeof(tx), "open_bus_no_cs");
    if (err != ESP_OK) {
        set_fail();
    }

    ESP_LOGI(TAG, "open-bus: zero=%u ff=%u other=%u",
             (unsigned)count_equal(rx, sizeof(rx), 0x00),
             (unsigned)count_equal(rx, sizeof(rx), 0xFF),
             (unsigned)(sizeof(rx)
                        - count_equal(rx, sizeof(rx), 0x00)
                        - count_equal(rx, sizeof(rx), 0xFF)));

    spi_remove_dev(dev);
}

// ── BME688 ───────────────────────────────────────────────────────────────
static bool probe_bme688()
{
    ESP_LOGI(TAG, "================ BME688 ================");
    bool ok = false;
    spi_device_handle_t dev = spi_add_dev(0, SPI_HZ);
    if (!dev) {
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        uint8_t chip = 0;
        uint8_t status = 0;

        reg_write(dev, PIN_CS_ENV, 0x73, 0x00, true);
        vTaskDelay(pdMS_TO_TICKS(2));
        reg_read_1(dev, PIN_CS_ENV, 0xD0, &chip, true);
        reg_read_1(dev, PIN_CS_ENV, 0x1D, &status, true);

        ESP_LOGI(TAG, "BME688 try %d: chip_id=0x%02X status=0x%02X", i + 1, chip, status);
        if (chip == 0x61) {
            ok = true;
        }
    }

    spi_remove_dev(dev);

    if (!ok) {
        set_fail();
    }
    return ok;
}

// ── ICM-42688-P ──────────────────────────────────────────────────────────
static bool probe_icm42688p()
{
    ESP_LOGI(TAG, "================ ICM-42688-P ================");
    bool ok = false;

    for (int mode = 0; mode <= 3; mode += 3) {
        spi_device_handle_t dev = spi_add_dev(mode, SPI_HZ);
        if (!dev) {
            continue;
        }

        for (int i = 0; i < 3; ++i) {
            uint8_t who = 0;
            reg_read_1(dev, PIN_CS_IMU, 0x75, &who, true);
            ESP_LOGI(TAG, "ICM mode=%d try=%d WHO_AM_I=0x%02X", mode, i + 1, who);
            if (who == 0x47) {
                ok = true;
            }
        }

        spi_remove_dev(dev);
    }

    if (!ok) {
        set_fail();
    }
    return ok;
}

// ── NEO-M9N ──────────────────────────────────────────────────────────────
static bool probe_neo_m9n()
{
    ESP_LOGI(TAG, "================ NEO-M9N ================");
    bool ok = false;
    spi_device_handle_t dev = spi_add_dev(0, SPI_HZ);
    if (!dev) {
        return false;
    }

    {
        uint8_t tx[64];
        uint8_t rx[64];
        memset(tx, 0xFF, sizeof(tx));
        memset(rx, 0, sizeof(rx));
        raw_xfer(dev, PIN_CS_GPS, tx, rx, sizeof(tx), "gps_idle_poll");

        size_t zeros = count_equal(rx, sizeof(rx), 0x00);
        size_t ffs   = count_equal(rx, sizeof(rx), 0xFF);
        ESP_LOGI(TAG, "GPS idle poll: zero=%u ff=%u other=%u",
                 (unsigned)zeros,
                 (unsigned)ffs,
                 (unsigned)(sizeof(rx) - zeros - ffs));

        for (size_t i = 0; i + 1 < sizeof(rx); ++i) {
            if (rx[i] == 0xB5 && rx[i + 1] == 0x62) {
                ok = true;
            }
        }

        if (zeros != sizeof(rx) && ffs != sizeof(rx)) {
            ok = true;
        }
    }

    {
        uint8_t req[8] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};
        raw_xfer(dev, PIN_CS_GPS, req, nullptr, sizeof(req), "gps_mon_ver_req");
        vTaskDelay(pdMS_TO_TICKS(50));

        uint8_t tx[128];
        uint8_t rx[128];
        memset(tx, 0xFF, sizeof(tx));
        memset(rx, 0, sizeof(rx));
        raw_xfer(dev, PIN_CS_GPS, tx, rx, sizeof(tx), "gps_mon_ver_readback");

        for (size_t i = 0; i + 1 < sizeof(rx); ++i) {
            if (rx[i] == 0xB5 && rx[i + 1] == 0x62) {
                ok = true;
            }
        }
    }

    spi_remove_dev(dev);

    if (!ok) {
        set_fail();
    }
    return ok;
}

// ── SX1262 / E22 ─────────────────────────────────────────────────────────
static bool probe_sx1262()
{
    ESP_LOGI(TAG, "================ SX1262 / E22 ================");
    bool ok = false;
    spi_device_handle_t dev = spi_add_dev(0, SPI_HZ);
    if (!dev) {
        return false;
    }

    ESP_LOGI(TAG, "LoRa pins before reset: BUSY=%d DIO1=%d RST=%d",
             gpio_get_level(PIN_LORA_BUSY),
             gpio_get_level(PIN_LORA_DIO1),
             gpio_get_level(PIN_LORA_RST));

    gpio_set_level(PIN_LORA_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(PIN_LORA_RST, 1);

    int busy_wait = 500;
    while (gpio_get_level(PIN_LORA_BUSY) == 1 && busy_wait-- > 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    ESP_LOGI(TAG, "SX1262 BUSY after reset: %s (%d ms left)",
             busy_wait > 0 ? "WENT LOW" : "TIMEOUT",
             busy_wait);

    if (busy_wait > 0) {
        uint8_t tcxo_cmd[] = {0x97, 0x00, 0x00, 0x01, 0x40};
        raw_xfer(dev, PIN_CS_LORA, tcxo_cmd, nullptr, sizeof(tcxo_cmd), "sx1262_set_tcxo");

        int tcxo_wait = 200;
        while (gpio_get_level(PIN_LORA_BUSY) == 1 && tcxo_wait-- > 0) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        ESP_LOGI(TAG, "SX1262 BUSY after TCXO cmd: %s (%d ms left)",
                 tcxo_wait > 0 ? "WENT LOW" : "TIMEOUT",
                 tcxo_wait);
    } else {
        ESP_LOGE(TAG, "SX1262 did not come out of reset — BUSY stuck high, skipping TCXO cmd");
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    ESP_LOGI(TAG, "LoRa pins after reset+TCXO: BUSY=%d DIO1=%d RST=%d",
             gpio_get_level(PIN_LORA_BUSY),
             gpio_get_level(PIN_LORA_DIO1),
             gpio_get_level(PIN_LORA_RST));

    for (int i = 0; i < 5; ++i) {
        uint8_t tx[2] = {0xC0, 0x00};
        uint8_t rx[2] = {};
        raw_xfer(dev, PIN_CS_LORA, tx, rx, sizeof(tx), "sx1262_get_status");
        uint8_t st = rx[1];
        ESP_LOGI(TAG, "SX1262 GetStatus try=%d -> 0x%02X (mode=%u cmd=%u)",
                 i + 1, st, (st >> 4) & 0x07, (st >> 1) & 0x07);
        if (st != 0x00 && st != 0xFF) {
            ok = true;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    {
        uint8_t tx[5] = {0x1D, 0x07, 0x40, 0x00, 0x00};
        uint8_t rx[5] = {};
        raw_xfer(dev, PIN_CS_LORA, tx, rx, sizeof(tx), "sx1262_read_syncword");
        ESP_LOGI(TAG, "SX1262 syncword reg byte = 0x%02X", rx[4]);
        if (rx[4] != 0x00 && rx[4] != 0xFF) {
            ok = true;
        }
    }

    spi_remove_dev(dev);

    if (!ok) {
        set_fail();
    }
    return ok;
}

// ── app_main ─────────────────────────────────────────────────────────────
extern "C" void app_main(void)
{
    led_init();
    led_set(0, 0, 40);

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "KFDS26 HW_VERIFY");
    ESP_LOGI(TAG, "SPI host=%d hz=%d", SPI_HOST, SPI_HZ);
    ESP_LOGI(TAG,
             "Pins: MOSI=%d MISO=%d SCLK=%d ENV=%d IMU=%d LORA=%d GPS=%d BUSY=%d DIO1=%d RST=%d LED=%d",
             PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCLK,
             PIN_CS_ENV, PIN_CS_IMU, PIN_CS_LORA, PIN_CS_GPS,
             PIN_LORA_BUSY, PIN_LORA_DIO1, PIN_LORA_RST, PIN_NEOPIXEL);
    ESP_LOGI(TAG, "========================================");

    init_gpio();
    log_pin_levels("startup");
    miso_bias_test("startup");

    if (spi_bus_up() != ESP_OK) {
        set_fail();
        led_set(40, 0, 0);
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    log_pin_levels("post_bus_init");
    miso_bias_test("post_bus_init");

    probe_open_bus();

    bool bme_ok  = probe_bme688();
    bool imu_ok  = probe_icm42688p();
    bool gps_ok  = probe_neo_m9n();
    bool lora_ok = probe_sx1262();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "SUMMARY: BME=%s IMU=%s GPS=%s LORA=%s",
             bme_ok ? "PASS" : "FAIL",
             imu_ok ? "PASS" : "FAIL",
             gps_ok ? "PASS" : "FAIL",
             lora_ok ? "PASS" : "FAIL");
    ESP_LOGI(TAG, "OVERALL: %s", s_any_fail ? "FAIL" : "PASS");
    ESP_LOGI(TAG, "========================================");

    led_set(s_any_fail ? 40 : 0, s_any_fail ? 0 : 40, 0);

    while (1) {
        log_pin_levels("idle");
        miso_bias_test("idle");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
