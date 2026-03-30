#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// ----------------------
// Protocol constants
// ----------------------

#define PS_SYNC_LO      0x55
#define PS_SYNC_HI      0xAA

#define PS_PROTO_VER    0x01

// Message types
typedef enum {
    PS_MSG_ENV_TELEM  = 0x01,
    PS_MSG_IMU_TELEM  = 0x02,
    PS_MSG_GPS_TELEM  = 0x03,
    PS_MSG_STATUS     = 0x04,
    PS_MSG_BEACON     = 0x05,

    PS_MSG_CMD        = 0x10,
    PS_MSG_CMD_ACK    = 0x11,

    PS_MSG_QKD_INITIATE = 0x20,
    PS_MSG_QKD_RESPONSE = 0x21,
    PS_MSG_QKD_SIFTED   = 0x22,
    PS_MSG_QKD_VERIFY   = 0x23
} ps_msg_type_t;

// Command classes
typedef enum {
    PS_CMD_CLASS_RECOVERY = 0,
    PS_CMD_CLASS_MOTOR    = 1,
    PS_CMD_CLASS_MISSION  = 2,
    PS_CMD_CLASS_CONFIG   = 3
} ps_cmd_class_t;

// Mission states
typedef enum {
    PS_MSTATE_IDLE      = 0,
    PS_MSTATE_PRELAUNCH = 1,
    PS_MSTATE_ASCENT    = 2,
    PS_MSTATE_DESCENT   = 3,
    PS_MSTATE_LANDED    = 4,
    PS_MSTATE_RECOVERY  = 5
} ps_mission_state_t;

// Bit manipulation macros for packed bit arrays (MSB first)
#define BIT_GET(arr, i)  (((arr)[(i)/8] >> (7 - ((i) % 8))) & 1)
#define BIT_SET(arr, i)  ((arr)[(i)/8] |= (0x80 >> ((i) % 8)))
#define BIT_CLR(arr, i)  ((arr)[(i)/8] &= ~(0x80 >> ((i) % 8)))

// CRC16-CCITT parameters (0x1021 poly, 0xFFFF init)
uint16_t ps_crc16_ccitt(const uint8_t *data, size_t len);

// ----------------------
// Frame header
// ----------------------

// On-wire header layout (excluding SYNC and CRC):
//  ver (1), msg_type (1), node_id (1), len (2)

typedef struct __attribute__((packed)) {
    uint8_t  ver;
    uint8_t  msg_type;
    uint8_t  node_id;
    uint16_t len;        // payload length
} ps_header_t;

// ----------------------
// Telemetry payloads
// ----------------------

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint32_t sample_id;
    int16_t  temperature_c_x100;
    uint32_t pressure_pa;
    uint16_t humidity_rh_x100;
    uint16_t env_flags;
    uint8_t  reserved[6];
} ps_env_telem_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;          // 4
    uint32_t sample_id;          // 4  -> 8
    int16_t  ax_mps2_x1000;      // 2  -> 10
    int16_t  ay_mps2_x1000;      // 2  -> 12
    int16_t  az_mps2_x1000;      // 2  -> 14
    int16_t  gx_dps_x1000;       // 2  -> 16
    int16_t  gy_dps_x1000;       // 2  -> 18
    int16_t  gz_dps_x1000;       // 2  -> 20
    uint16_t imu_flags;          // 2  -> 22
    int16_t  qw_x10000;          // 2  -> 24
    int16_t  qx_x10000;          // 2  -> 26
    int16_t  qy_x10000;          // 2  -> 28
    int16_t  qz_x10000;          // 2  -> 30
    uint8_t  reserved[10];       // 10 -> 40
} ps_imu_telem_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint32_t sample_id;
    int32_t  lat_deg_x1e7;
    int32_t  lon_deg_x1e7;
    int32_t  alt_m_x100;
    uint16_t ground_speed_cmps;
    uint16_t course_deg_x100;
    uint16_t hdop_x100;
    uint8_t  sat_count;
    uint8_t  fix_type;
    uint16_t gps_flags;
} ps_gps_telem_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t battery_mv;
    uint16_t battery_pct_x100;
    uint32_t storage_free_kb;
    uint8_t  mission_state;
    uint8_t  recovery_flags;
    int16_t  link_rssi_dbm;
    int16_t  link_snr_db_x10;
    uint16_t status_flags;
    uint8_t  reserved[2];
} ps_status_t;

typedef struct __attribute__((packed)) {
    uint32_t uptime_ms;
    uint16_t last_fix_age_ms;
    int32_t  lat_deg_x1e7;
    int32_t  lon_deg_x1e7;
    int32_t  alt_m_x100;
    uint16_t battery_mv;
    uint8_t  recovery_flags;
    uint8_t  beacon_seq;
    uint8_t  reserved[4];
} ps_beacon_t;

// ----------------------
// Command payloads
// ----------------------

// Common CMD envelope inside payload
typedef struct __attribute__((packed)) {
    uint16_t cmd_id;
    uint8_t  cmd_class;
    uint8_t  cmd_code;
    uint16_t param_len; // bytes after this header
    // uint8_t params[param_len]; // variable
} ps_cmd_hdr_t;

// Example specific param structs

// RECOVERY_SET_BEACON (class 0, code 0x01)
typedef struct __attribute__((packed)) {
    uint8_t  enable;      // 0/1
    uint32_t period_ms;
} ps_cmd_recovery_set_beacon_t;

// MOTOR_SET_SIDES (class 1, code 0x01)
typedef struct __attribute__((packed)) {
    int16_t left_speed_x1000;
    int16_t right_speed_x1000;
    uint8_t flags;
    uint8_t reserved;
} ps_cmd_motor_set_sides_t;

// MISSION_SET_STATE (class 2, code 0x01)
typedef struct __attribute__((packed)) {
    uint8_t mission_state;
} ps_cmd_mission_set_state_t;

// MISSION_SET_RATES (class 2, code 0x02)
typedef struct __attribute__((packed)) {
    uint16_t env_hz_x100;
    uint16_t imu_hz_x100;
    uint16_t gps_hz_x100;
    uint16_t status_hz_x100;
} ps_cmd_mission_set_rates_t;

// ----------------------
// CMD_ACK payload
// ----------------------

typedef struct __attribute__((packed)) {
    uint16_t cmd_id;
    uint8_t  status;
    uint8_t  detail;
    uint32_t timestamp_ms;
} ps_cmd_ack_t;

// Status codes
typedef enum {
    PS_CMD_STATUS_RECEIVED        = 0,
    PS_CMD_STATUS_ACCEPTED        = 1,
    PS_CMD_STATUS_DONE            = 2,
    PS_CMD_STATUS_REJECTED_INVALID= 3,
    PS_CMD_STATUS_REJECTED_STATE  = 4,
    PS_CMD_STATUS_ERROR_INTERNAL  = 5
} ps_cmd_status_t;

// ----------------------
// Frame builder helpers
// ----------------------

// Build a frame into dst; returns total frame length (bytes) or 0 on error.
// dst must be at least (2 + sizeof(ps_header_t) + payload_len + 2) bytes.
size_t ps_build_frame(uint8_t *dst,
                      size_t dst_size,
                      uint8_t  node_id,
                      uint8_t  msg_type,
                      const uint8_t *payload,
                      uint16_t payload_len);

// ----------------------
// QKD (BB84) payloads
// ----------------------

typedef struct __attribute__((packed)) {
    uint8_t raw_bits[13];    // 100 bits packed, MSB first
    uint8_t gs_bases[13];    // 100 bases packed, same format
    uint8_t num_bits;        // = 100
} ps_qkd_initiate_t;        // 27 bytes

typedef struct __attribute__((packed)) {
    uint8_t sat_bases[13];   // 100 satellite bases, packed
    uint8_t num_bits;        // = 100
} ps_qkd_response_t;        // 14 bytes

typedef struct __attribute__((packed)) {
    uint8_t match_mask[13];  // 100 bits: 1 = bases matched
    uint8_t check_mask[13];  // 100 bits: 1 = sacrificed for error check
    uint8_t check_bits[13];  // 100 bits: GS's bit values (check positions)
    uint8_t num_bits;        // = 100
} ps_qkd_sifted_t;          // 40 bytes

#define QKD_PROOF_MSG_MAX 32

typedef struct __attribute__((packed)) {
    uint8_t error_count;                      // check bit mismatches (should be 0)
    uint8_t check_count;                      // how many bits were checked
    uint8_t key_len_bits;                     // final usable key length in bits
    uint8_t encrypted_msg[QKD_PROOF_MSG_MAX]; // XOR-encrypted proof message
    uint8_t encrypted_len;                    // length of encrypted message bytes
    uint8_t status;                           // 0 = success, 1 = error check failed
} ps_qkd_verify_t;                           // 36 bytes

// Minimal streaming parser state (for optional use on ESP)
typedef struct {
    uint8_t  state;
    uint8_t  sync_seen;
    uint8_t  header_buf[sizeof(ps_header_t)];
    uint8_t  header_pos;
    uint16_t payload_len;
    uint16_t payload_pos;
    uint8_t  *payload_dst;
    uint16_t crc_recv;
    uint16_t crc_calc;
} ps_parser_t;

void ps_parser_init(ps_parser_t *p);

// Feed one byte; when a full valid frame is received, returns 1 and
// header/payload/crc can be read; returns 0 otherwise.
int ps_parser_feed(ps_parser_t *p,
                   uint8_t byte,
                   ps_header_t *out_hdr,
                   uint8_t *out_payload,
                   size_t out_payload_size);

#ifdef __cplusplus
}
#endif
