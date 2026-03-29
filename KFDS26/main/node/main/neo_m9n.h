#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// GNSS fix data
typedef struct {
    // Position
    int32_t  lat_deg_x1e7;     // latitude  in degrees × 1e7
    int32_t  lon_deg_x1e7;     // longitude in degrees × 1e7
    int32_t  alt_m_x100;       // altitude  in meters × 100

    // Motion
    uint16_t ground_speed_cmps; // ground speed in cm/s
    uint16_t course_deg_x100;   // course over ground in degrees × 100

    // Quality
    uint16_t hdop_x100;         // HDOP × 100
    uint16_t pdop_x100;         // PDOP × 100
    uint16_t vdop_x100;         // VDOP × 100
    uint8_t  sat_count;         // number of satellites used
    uint8_t  fix_type;          // 0=no fix, 1=GPS, 2=DGPS, 3=3D fix

    // Time (UTC)
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    uint8_t  day;
    uint8_t  month;
    uint16_t year;

    bool     valid;             // true if fix is valid (RMC status 'A')
    uint32_t last_update_ms;    // uptime when last fix was received
} gnss_fix_t;

// Initialize UART and start NMEA parsing.
// Returns true on success.
bool neo_m9n_init(void);

// Get a copy of the latest GNSS fix (thread-safe).
// Returns true if a valid fix is available.
bool neo_m9n_get_fix(gnss_fix_t *fix);

#ifdef __cplusplus
}
#endif
