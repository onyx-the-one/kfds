#include "neo_m9n.h"
#include "node_config.h"

#include <cstring>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "NEO_M9N";

#define NMEA_MAX_LEN    128
#define SPI_POLL_SIZE   256   // bytes per SPI poll transaction

// SPI handle
static spi_device_handle_t s_spi = nullptr;

// Latest fix, protected by mutex
static gnss_fix_t s_fix;
static SemaphoreHandle_t s_fix_mutex;

// NMEA sentence accumulator
static char s_nmea_buf[NMEA_MAX_LEN];
static int  s_nmea_pos;

// -----------------------------------------------------------------------------
// NMEA checksum validation
// -----------------------------------------------------------------------------
static bool nmea_verify_checksum(const char *sentence)
{
    const char *star = strchr(sentence, '*');
    if (!star || star == sentence) return false;

    uint8_t calc = 0;
    for (const char *p = sentence + 1; p < star; p++) {
        calc ^= static_cast<uint8_t>(*p);
    }

    unsigned int recv = 0;
    if (sscanf(star + 1, "%2x", &recv) != 1) return false;

    return calc == static_cast<uint8_t>(recv);
}

// -----------------------------------------------------------------------------
// Field extraction helpers
// -----------------------------------------------------------------------------
static const char *nmea_field(const char *sentence, int n)
{
    const char *p = sentence;
    for (int i = 0; i < n; i++) {
        p = strchr(p, ',');
        if (!p) return nullptr;
        p++;
    }
    return p;
}

static bool field_empty(const char *f)
{
    return (!f || *f == ',' || *f == '*' || *f == '\0');
}

static int32_t parse_lat(const char *f, const char *ns)
{
    if (field_empty(f) || field_empty(ns)) return 0;
    double raw = atof(f);
    int deg = (int)(raw / 100.0);
    double min = raw - deg * 100.0;
    double lat = deg + min / 60.0;
    if (*ns == 'S' || *ns == 's') lat = -lat;
    return (int32_t)(lat * 1e7);
}

static int32_t parse_lon(const char *f, const char *ew)
{
    if (field_empty(f) || field_empty(ew)) return 0;
    double raw = atof(f);
    int deg = (int)(raw / 100.0);
    double min = raw - deg * 100.0;
    double lon = deg + min / 60.0;
    if (*ew == 'W' || *ew == 'w') lon = -lon;
    return (int32_t)(lon * 1e7);
}

static void parse_time(const char *f, uint8_t *h, uint8_t *m, uint8_t *s)
{
    if (field_empty(f)) return;
    int hh = (f[0] - '0') * 10 + (f[1] - '0');
    int mm = (f[2] - '0') * 10 + (f[3] - '0');
    int ss = (f[4] - '0') * 10 + (f[5] - '0');
    *h = static_cast<uint8_t>(hh);
    *m = static_cast<uint8_t>(mm);
    *s = static_cast<uint8_t>(ss);
}

static void parse_date(const char *f, uint8_t *day, uint8_t *month, uint16_t *year)
{
    if (field_empty(f) || strlen(f) < 6) return;
    *day   = static_cast<uint8_t>((f[0] - '0') * 10 + (f[1] - '0'));
    *month = static_cast<uint8_t>((f[2] - '0') * 10 + (f[3] - '0'));
    *year  = static_cast<uint16_t>(2000 + (f[4] - '0') * 10 + (f[5] - '0'));
}

// -----------------------------------------------------------------------------
// NMEA sentence parsers
// -----------------------------------------------------------------------------

static void parse_gga(const char *sentence)
{
    const char *f;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);

    f = nmea_field(sentence, 1);
    if (!field_empty(f)) {
        parse_time(f, &s_fix.hour, &s_fix.minute, &s_fix.second);
    }

    const char *lat_f = nmea_field(sentence, 2);
    const char *ns_f  = nmea_field(sentence, 3);
    s_fix.lat_deg_x1e7 = parse_lat(lat_f, ns_f);

    const char *lon_f = nmea_field(sentence, 4);
    const char *ew_f  = nmea_field(sentence, 5);
    s_fix.lon_deg_x1e7 = parse_lon(lon_f, ew_f);

    f = nmea_field(sentence, 6);
    if (!field_empty(f)) {
        s_fix.fix_type = static_cast<uint8_t>(atoi(f));
    }

    f = nmea_field(sentence, 7);
    if (!field_empty(f)) {
        s_fix.sat_count = static_cast<uint8_t>(atoi(f));
    }

    f = nmea_field(sentence, 8);
    if (!field_empty(f)) {
        s_fix.hdop_x100 = static_cast<uint16_t>(atof(f) * 100.0);
    }

    f = nmea_field(sentence, 9);
    if (!field_empty(f)) {
        s_fix.alt_m_x100 = (int32_t)(atof(f) * 100.0);
    }

    s_fix.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    xSemaphoreGive(s_fix_mutex);
}

static void parse_rmc(const char *sentence)
{
    const char *f;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);

    f = nmea_field(sentence, 2);
    s_fix.valid = (!field_empty(f) && *f == 'A');

    f = nmea_field(sentence, 7);
    if (!field_empty(f)) {
        float knots = (float)atof(f);
        s_fix.ground_speed_cmps = static_cast<uint16_t>(knots * 51.4444f);
    }

    f = nmea_field(sentence, 8);
    if (!field_empty(f)) {
        s_fix.course_deg_x100 = static_cast<uint16_t>(atof(f) * 100.0);
    }

    f = nmea_field(sentence, 9);
    if (!field_empty(f)) {
        parse_date(f, &s_fix.day, &s_fix.month, &s_fix.year);
    }

    s_fix.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    xSemaphoreGive(s_fix_mutex);
}

static void parse_gsa(const char *sentence)
{
    const char *f;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);

    f = nmea_field(sentence, 2);
    if (!field_empty(f)) {
        uint8_t ft = static_cast<uint8_t>(atoi(f));
        if (ft <= 1) s_fix.fix_type = 0;
        else s_fix.fix_type = ft;
    }

    f = nmea_field(sentence, 15);
    if (!field_empty(f)) {
        s_fix.pdop_x100 = static_cast<uint16_t>(atof(f) * 100.0);
    }

    f = nmea_field(sentence, 16);
    if (!field_empty(f)) {
        s_fix.hdop_x100 = static_cast<uint16_t>(atof(f) * 100.0);
    }

    f = nmea_field(sentence, 17);
    if (!field_empty(f)) {
        s_fix.vdop_x100 = static_cast<uint16_t>(atof(f) * 100.0);
    }

    xSemaphoreGive(s_fix_mutex);
}

// -----------------------------------------------------------------------------
// Process a complete NMEA sentence
// -----------------------------------------------------------------------------
static void process_sentence(const char *sentence)
{
    if (!nmea_verify_checksum(sentence)) return;

    if (sentence[0] != '$') return;
    if (sentence[1] != 'G') return;

    const char *type = sentence + 3; // skip "$GP" or "$GN"

    if (strncmp(type, "GGA,", 4) == 0) {
        parse_gga(sentence);
    } else if (strncmp(type, "RMC,", 4) == 0) {
        parse_rmc(sentence);
    } else if (strncmp(type, "GSA,", 4) == 0) {
        parse_gsa(sentence);
    }
}

// -----------------------------------------------------------------------------
// Feed a single byte into the NMEA accumulator
// -----------------------------------------------------------------------------
static void feed_byte(uint8_t byte)
{
    if (byte == '$') {
        s_nmea_pos = 0;
        s_nmea_buf[s_nmea_pos++] = static_cast<char>(byte);
    } else if (byte == '\n' || byte == '\r') {
        if (s_nmea_pos > 0) {
            s_nmea_buf[s_nmea_pos] = '\0';
            process_sentence(s_nmea_buf);
            s_nmea_pos = 0;
        }
    } else if (s_nmea_pos > 0 && s_nmea_pos < NMEA_MAX_LEN - 1) {
        s_nmea_buf[s_nmea_pos++] = static_cast<char>(byte);
    } else {
        s_nmea_pos = 0;
    }
}

// -----------------------------------------------------------------------------
// SPI poll task — reads data from NEO-M9N via SPI
// NEO-M9N SPI protocol: master sends 0xFF bytes, slave returns data.
// 0xFF from slave = no data available.
// -----------------------------------------------------------------------------
static void gnss_spi_task(void *arg)
{
    uint8_t tx_buf[SPI_POLL_SIZE];
    uint8_t rx_buf[SPI_POLL_SIZE];
    memset(tx_buf, 0xFF, sizeof(tx_buf));

    while (true) {
        spi_transaction_t t = {};
        t.length = SPI_POLL_SIZE * 8;
        t.tx_buffer = tx_buf;
        t.rx_buffer = rx_buf;

        esp_err_t err = spi_device_transmit(s_spi, &t);
        if (err != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Feed received bytes into NMEA parser, skipping 0xFF filler
        bool got_data = false;
        for (int i = 0; i < SPI_POLL_SIZE; i++) {
            if (rx_buf[i] != 0xFF) {
                feed_byte(rx_buf[i]);
                got_data = true;
            }
        }

        // Poll faster when data is flowing, slower when idle
        vTaskDelay(pdMS_TO_TICKS(got_data ? 10 : 100));
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

esp_err_t neo_m9n_init(spi_host_device_t host)
{
    s_fix_mutex = xSemaphoreCreateMutex();
    if (!s_fix_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }

    memset(&s_fix, 0, sizeof(s_fix));
    s_nmea_pos = 0;

    // Add NEO-M9N to the shared SPI bus — Mode 0 (CPOL=0, CPHA=0)
    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.clock_speed_hz = SPI_CLK_GPS;
    dev_cfg.mode = 0;
    dev_cfg.spics_io_num = PIN_CS_GPS;
    dev_cfg.queue_size = 4;

    esp_err_t err = spi_bus_add_device(host, &dev_cfg, &s_spi);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPI add device failed: %s", esp_err_to_name(err));
        return err;
    }

    // Start SPI polling task
    BaseType_t ret = xTaskCreate(gnss_spi_task, "gnss_spi", TASK_STACK_GNSS,
                                  nullptr, TASK_PRIO_GNSS, nullptr);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GNSS SPI task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "NEO-M9N SPI initialized at %d Hz", SPI_CLK_GPS);
    return ESP_OK;
}

esp_err_t neo_m9n_get_fix(gnss_fix_t *fix)
{
    if (!fix || !s_fix_mutex) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);
    *fix = s_fix;
    xSemaphoreGive(s_fix_mutex);

    return fix->valid ? ESP_OK : ESP_ERR_NOT_FOUND;
}
