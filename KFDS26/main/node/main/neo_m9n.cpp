#include "neo_m9n.h"
#include "node_config.h"

#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char *TAG = "NEO_M9N";

#define NMEA_MAX_LEN    128
#define UART_BUF_SIZE   1024

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
    // Sentence: $....*HH\r\n  or  $....*HH
    const char *star = strchr(sentence, '*');
    if (!star || star == sentence) return false;

    uint8_t calc = 0;
    // XOR everything between '$' and '*' (exclusive)
    for (const char *p = sentence + 1; p < star; p++) {
        calc ^= (uint8_t)*p;
    }

    // Parse hex checksum after '*'
    unsigned int recv = 0;
    if (sscanf(star + 1, "%2x", &recv) != 1) return false;

    return calc == (uint8_t)recv;
}

// -----------------------------------------------------------------------------
// Field extraction helpers
// -----------------------------------------------------------------------------
// Get the nth comma-separated field (0-indexed) from an NMEA sentence.
// Returns pointer to start of field within the sentence, or NULL.
static const char *nmea_field(const char *sentence, int n)
{
    const char *p = sentence;
    for (int i = 0; i < n; i++) {
        p = strchr(p, ',');
        if (!p) return NULL;
        p++; // skip comma
    }
    return p;
}

static bool field_empty(const char *f)
{
    return (!f || *f == ',' || *f == '*' || *f == '\0');
}

// Parse NMEA latitude: DDMM.MMMMM
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

// Parse NMEA longitude: DDDMM.MMMMM
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
    // HHMMSS.SS
    int hh = (f[0] - '0') * 10 + (f[1] - '0');
    int mm = (f[2] - '0') * 10 + (f[3] - '0');
    int ss = (f[4] - '0') * 10 + (f[5] - '0');
    *h = (uint8_t)hh;
    *m = (uint8_t)mm;
    *s = (uint8_t)ss;
}

static void parse_date(const char *f, uint8_t *day, uint8_t *month, uint16_t *year)
{
    if (field_empty(f) || strlen(f) < 6) return;
    // DDMMYY
    *day   = (uint8_t)((f[0] - '0') * 10 + (f[1] - '0'));
    *month = (uint8_t)((f[2] - '0') * 10 + (f[3] - '0'));
    *year  = (uint16_t)(2000 + (f[4] - '0') * 10 + (f[5] - '0'));
}

// -----------------------------------------------------------------------------
// NMEA sentence parsers
// -----------------------------------------------------------------------------

// $G?GGA — Fix information
static void parse_gga(const char *sentence)
{
    // Fields: 0=$GPGGA, 1=time, 2=lat, 3=N/S, 4=lon, 5=E/W,
    //         6=quality, 7=numSV, 8=HDOP, 9=alt, 10=M, 11=sep, 12=M, ...
    const char *f;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);

    // Time
    f = nmea_field(sentence, 1);
    if (!field_empty(f)) {
        parse_time(f, &s_fix.hour, &s_fix.minute, &s_fix.second);
    }

    // Latitude
    const char *lat_f = nmea_field(sentence, 2);
    const char *ns_f  = nmea_field(sentence, 3);
    s_fix.lat_deg_x1e7 = parse_lat(lat_f, ns_f);

    // Longitude
    const char *lon_f = nmea_field(sentence, 4);
    const char *ew_f  = nmea_field(sentence, 5);
    s_fix.lon_deg_x1e7 = parse_lon(lon_f, ew_f);

    // Fix quality
    f = nmea_field(sentence, 6);
    if (!field_empty(f)) {
        s_fix.fix_type = (uint8_t)atoi(f);
    }

    // Satellite count
    f = nmea_field(sentence, 7);
    if (!field_empty(f)) {
        s_fix.sat_count = (uint8_t)atoi(f);
    }

    // HDOP
    f = nmea_field(sentence, 8);
    if (!field_empty(f)) {
        s_fix.hdop_x100 = (uint16_t)(atof(f) * 100.0);
    }

    // Altitude
    f = nmea_field(sentence, 9);
    if (!field_empty(f)) {
        s_fix.alt_m_x100 = (int32_t)(atof(f) * 100.0);
    }

    s_fix.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    xSemaphoreGive(s_fix_mutex);
}

// $G?RMC — Recommended minimum
static void parse_rmc(const char *sentence)
{
    // Fields: 0=$GPRMC, 1=time, 2=status(A/V), 3=lat, 4=N/S, 5=lon, 6=E/W,
    //         7=speed(knots), 8=course, 9=date, ...
    const char *f;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);

    // Status
    f = nmea_field(sentence, 2);
    s_fix.valid = (!field_empty(f) && *f == 'A');

    // Speed (knots → cm/s: 1 knot = 51.4444 cm/s)
    f = nmea_field(sentence, 7);
    if (!field_empty(f)) {
        float knots = (float)atof(f);
        s_fix.ground_speed_cmps = (uint16_t)(knots * 51.4444f);
    }

    // Course over ground
    f = nmea_field(sentence, 8);
    if (!field_empty(f)) {
        s_fix.course_deg_x100 = (uint16_t)(atof(f) * 100.0);
    }

    // Date
    f = nmea_field(sentence, 9);
    if (!field_empty(f)) {
        parse_date(f, &s_fix.day, &s_fix.month, &s_fix.year);
    }

    s_fix.last_update_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    xSemaphoreGive(s_fix_mutex);
}

// $G?GSA — DOP and active satellites
static void parse_gsa(const char *sentence)
{
    // Fields: 0=$GPGSA, 1=mode1, 2=mode2(fix type 1/2/3),
    //         3-14=satellite PRN, 15=PDOP, 16=HDOP, 17=VDOP
    const char *f;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);

    // Fix type (1=no fix, 2=2D, 3=3D)
    f = nmea_field(sentence, 2);
    if (!field_empty(f)) {
        uint8_t ft = (uint8_t)atoi(f);
        // Map GSA fix type: 1=no fix→0, 2=2D fix→2, 3=3D fix→3
        if (ft <= 1) s_fix.fix_type = 0;
        else s_fix.fix_type = ft;
    }

    // PDOP
    f = nmea_field(sentence, 15);
    if (!field_empty(f)) {
        s_fix.pdop_x100 = (uint16_t)(atof(f) * 100.0);
    }

    // HDOP
    f = nmea_field(sentence, 16);
    if (!field_empty(f)) {
        s_fix.hdop_x100 = (uint16_t)(atof(f) * 100.0);
    }

    // VDOP
    f = nmea_field(sentence, 17);
    if (!field_empty(f)) {
        s_fix.vdop_x100 = (uint16_t)(atof(f) * 100.0);
    }

    xSemaphoreGive(s_fix_mutex);
}

// -----------------------------------------------------------------------------
// Process a complete NMEA sentence
// -----------------------------------------------------------------------------
static void process_sentence(const char *sentence)
{
    if (!nmea_verify_checksum(sentence)) return;

    // Match both $GP and $GN prefixes
    const char *type = sentence + 3; // skip "$GP" or "$GN"
    if (sentence[0] != '$') return;
    if (sentence[1] != 'G') return;

    if (strncmp(type, "GGA,", 4) == 0) {
        parse_gga(sentence);
    } else if (strncmp(type, "RMC,", 4) == 0) {
        parse_rmc(sentence);
    } else if (strncmp(type, "GSA,", 4) == 0) {
        parse_gsa(sentence);
    }
}

// -----------------------------------------------------------------------------
// UART read task — runs continuously, feeds NMEA parser
// -----------------------------------------------------------------------------
static void gnss_uart_task(void *arg)
{
    uint8_t byte;
    while (true) {
        int len = uart_read_bytes(GNSS_UART_NUM, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        if (byte == '$') {
            // Start of new sentence
            s_nmea_pos = 0;
            s_nmea_buf[s_nmea_pos++] = (char)byte;
        } else if (byte == '\n' || byte == '\r') {
            if (s_nmea_pos > 0) {
                s_nmea_buf[s_nmea_pos] = '\0';
                process_sentence(s_nmea_buf);
                s_nmea_pos = 0;
            }
        } else if (s_nmea_pos > 0 && s_nmea_pos < NMEA_MAX_LEN - 1) {
            s_nmea_buf[s_nmea_pos++] = (char)byte;
        } else {
            // Overflow — discard
            s_nmea_pos = 0;
        }
    }
}

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------

bool neo_m9n_init(void)
{
    s_fix_mutex = xSemaphoreCreateMutex();
    if (!s_fix_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return false;
    }

    memset(&s_fix, 0, sizeof(s_fix));
    s_nmea_pos = 0;

    // Configure UART
    uart_config_t uart_cfg = {};
    uart_cfg.baud_rate = GNSS_UART_BAUD;
    uart_cfg.data_bits = UART_DATA_8_BITS;
    uart_cfg.parity    = UART_PARITY_DISABLE;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_driver_install(GNSS_UART_NUM, UART_BUF_SIZE * 2, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_param_config(GNSS_UART_NUM, &uart_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(err));
        return false;
    }

    err = uart_set_pin(GNSS_UART_NUM, GNSS_UART_TX, GNSS_UART_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(err));
        return false;
    }

    // Start UART read task
    BaseType_t ret = xTaskCreate(gnss_uart_task, "gnss_uart", TASK_STACK_GNSS,
                                  NULL, TASK_PRIO_GNSS, NULL);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create GNSS UART task");
        return false;
    }

    ESP_LOGI(TAG, "NEO-M9N UART initialized at %d baud", GNSS_UART_BAUD);
    return true;
}

bool neo_m9n_get_fix(gnss_fix_t *fix)
{
    if (!fix || !s_fix_mutex) return false;

    xSemaphoreTake(s_fix_mutex, portMAX_DELAY);
    *fix = s_fix;
    xSemaphoreGive(s_fix_mutex);

    return fix->valid;
}
