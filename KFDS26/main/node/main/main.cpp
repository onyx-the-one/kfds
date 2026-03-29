// KFDS26 Main Node Firmware
// FreeRTOS multi-task architecture for ESP32-S3
//
// Tasks:
//   sensor_task  — BME688 + ICM-42688-P telemetry
//   gnss_task    — NEO-M9N GNSS telemetry
//   lora_tx_task — Dequeue frames, transmit via LoRa
//   lora_rx_task — Listen for CMD frames, dispatch
//   status_task  — Periodic STATUS telemetry

#include <cstdio>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"

#include "node_config.h"
#include "polysense_proto.h"
#include "bme688.h"
#include "icm42688p.h"
#include "neo_m9n.h"
#include "e22_lora.h"

static const char *TAG = "MAIN";

// =============================================================================
// TX frame queue
// =============================================================================

typedef struct {
    uint8_t data[TX_FRAME_MAX_SIZE];
    size_t  len;
} tx_frame_t;

static QueueHandle_t s_tx_queue;

// Helper: build a polysense frame and enqueue it for LoRa TX
static bool enqueue_frame(uint8_t msg_type, const uint8_t *payload, uint16_t payload_len)
{
    tx_frame_t frame;
    frame.len = ps_build_frame(frame.data, sizeof(frame.data),
                               NODE_ID, msg_type, payload, payload_len);
    if (frame.len == 0) return false;
    return xQueueSend(s_tx_queue, &frame, pdMS_TO_TICKS(10)) == pdTRUE;
}

// =============================================================================
// Mission state machine
// =============================================================================

static volatile ps_mission_state_t s_mission_state = PS_MSTATE_IDLE;
static SemaphoreHandle_t s_state_mutex;

static ps_mission_state_t get_mission_state(void)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    ps_mission_state_t st = s_mission_state;
    xSemaphoreGive(s_state_mutex);
    return st;
}

static void set_mission_state(ps_mission_state_t new_state)
{
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (new_state != s_mission_state) {
        ESP_LOGI(TAG, "Mission state: %d -> %d", s_mission_state, new_state);
        s_mission_state = new_state;
    }
    xSemaphoreGive(s_state_mutex);
}

// =============================================================================
// Link quality cache (updated by lora_rx_task)
// =============================================================================

static e22_link_quality_t s_link_quality = {};
static SemaphoreHandle_t  s_lq_mutex;

static void update_link_quality(void)
{
    e22_link_quality_t lq;
    e22_get_link_quality(&lq);
    xSemaphoreTake(s_lq_mutex, portMAX_DELAY);
    s_link_quality = lq;
    xSemaphoreGive(s_lq_mutex);
}

static e22_link_quality_t get_link_quality(void)
{
    xSemaphoreTake(s_lq_mutex, portMAX_DELAY);
    e22_link_quality_t lq = s_link_quality;
    xSemaphoreGive(s_lq_mutex);
    return lq;
}

// =============================================================================
// Uptime helper
// =============================================================================

static inline uint32_t uptime_ms(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

// =============================================================================
// sensor_task — BME688 + ICM-42688-P
// =============================================================================

static void sensor_task(void *arg)
{
    uint32_t env_sample_id = 0;
    uint32_t imu_sample_id = 0;
    TickType_t last_env_tick = xTaskGetTickCount();
    TickType_t last_imu_tick = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // --- ENV telemetry (BME688) ---
        if ((now - last_env_tick) * portTICK_PERIOD_MS >= ENV_INTERVAL_MS) {
            last_env_tick = now;

            bme688_data_t env_data;
            bool env_ok = (bme688_read(&env_data) == ESP_OK);

            ps_env_telem_t env = {};
            env.uptime_ms          = uptime_ms();
            env.sample_id          = env_sample_id++;
            env.temperature_c_x100 = env_ok ? (int16_t)(env_data.temperature_c * 100.0f) : 0;
            env.pressure_pa        = env_ok ? (uint32_t)env_data.pressure_pa : 0;
            env.humidity_rh_x100   = env_ok ? (uint16_t)(env_data.humidity_rh * 100.0f) : 0;

            // env_flags: bit0=temp valid, bit1=press valid, bit2=hum valid, bit3=gas valid
            env.env_flags = 0;
            if (env_ok) {
                env.env_flags |= 0x07; // T+P+H valid
                if (env_data.gas_valid) env.env_flags |= 0x08;
            }

            // Pack gas resistance into reserved bytes (uint32_t, Ohms)
            if (env_ok && env_data.gas_valid) {
                uint32_t gas_ohm = (uint32_t)env_data.gas_resistance;
                memcpy(env.reserved, &gas_ohm, sizeof(gas_ohm));
            }

            enqueue_frame(PS_MSG_ENV_TELEM, (const uint8_t *)&env, sizeof(env));
        }

        // --- IMU telemetry (ICM-42688-P) ---
        if ((now - last_imu_tick) * portTICK_PERIOD_MS >= IMU_INTERVAL_MS) {
            last_imu_tick = now;

            icm42688p_data_t imu_data;
            bool imu_ok = (icm42688p_read(&imu_data) == ESP_OK);

            ps_imu_telem_t imu = {};
            imu.uptime_ms     = uptime_ms();
            imu.sample_id     = imu_sample_id++;
            imu.ax_mps2_x1000 = imu_ok ? (int16_t)(imu_data.ax * 1000.0f) : 0;
            imu.ay_mps2_x1000 = imu_ok ? (int16_t)(imu_data.ay * 1000.0f) : 0;
            imu.az_mps2_x1000 = imu_ok ? (int16_t)(imu_data.az * 1000.0f) : 0;
            imu.gx_dps_x1000  = imu_ok ? (int16_t)(imu_data.gx * 1000.0f) : 0;
            imu.gy_dps_x1000  = imu_ok ? (int16_t)(imu_data.gy * 1000.0f) : 0;
            imu.gz_dps_x1000  = imu_ok ? (int16_t)(imu_data.gz * 1000.0f) : 0;

            // imu_flags: bit0=accel valid, bit1=gyro valid
            imu.imu_flags = imu_ok ? 0x03 : 0x00;

            enqueue_frame(PS_MSG_IMU_TELEM, (const uint8_t *)&imu, sizeof(imu));
        }

        vTaskDelay(pdMS_TO_TICKS(5)); // Yield between checks
    }
}

// =============================================================================
// gnss_task — NEO-M9N GNSS telemetry
// =============================================================================

static void gnss_task(void *arg)
{
    uint32_t gps_sample_id = 0;

    while (true) {
        gnss_fix_t fix;
        neo_m9n_get_fix(&fix);

        ps_gps_telem_t gps = {};
        gps.uptime_ms          = uptime_ms();
        gps.sample_id          = gps_sample_id++;
        gps.lat_deg_x1e7       = fix.lat_deg_x1e7;
        gps.lon_deg_x1e7       = fix.lon_deg_x1e7;
        gps.alt_m_x100         = fix.alt_m_x100;
        gps.ground_speed_cmps  = fix.ground_speed_cmps;
        gps.course_deg_x100    = fix.course_deg_x100;
        gps.hdop_x100          = fix.hdop_x100;
        gps.sat_count          = fix.sat_count;
        gps.fix_type           = fix.fix_type;

        // gps_flags: bit0=fix valid
        gps.gps_flags = fix.valid ? 0x01 : 0x00;

        enqueue_frame(PS_MSG_GPS_TELEM, (const uint8_t *)&gps, sizeof(gps));

        vTaskDelay(pdMS_TO_TICKS(GPS_INTERVAL_MS));
    }
}

// =============================================================================
// lora_tx_task — Dequeue frames and transmit via LoRa
// =============================================================================

static void lora_tx_task(void *arg)
{
    tx_frame_t frame;

    while (true) {
        if (xQueueReceive(s_tx_queue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (e22_transmit(frame.data, frame.len) != ESP_OK) {
                ESP_LOGW(TAG, "LoRa TX failed (len=%d)", (int)frame.len);
            }
        }
    }
}

// =============================================================================
// lora_rx_task — Listen for CMD frames, dispatch
// =============================================================================

static void handle_command(const ps_header_t *hdr, const uint8_t *payload)
{
    if (hdr->len < sizeof(ps_cmd_hdr_t)) return;

    const ps_cmd_hdr_t *cmd = (const ps_cmd_hdr_t *)payload;
    ESP_LOGI(TAG, "CMD received: id=%u class=%u code=0x%02X",
             cmd->cmd_id, cmd->cmd_class, cmd->cmd_code);

    // Build ACK
    ps_cmd_ack_t ack = {};
    ack.cmd_id       = cmd->cmd_id;
    ack.timestamp_ms = uptime_ms();

    switch (cmd->cmd_class) {
    case PS_CMD_CLASS_MISSION:
        if (cmd->cmd_code == 0x01 && cmd->param_len >= 1) {
            // MISSION_SET_STATE
            const ps_cmd_mission_set_state_t *p =
                (const ps_cmd_mission_set_state_t *)(payload + sizeof(ps_cmd_hdr_t));
            if (p->mission_state <= PS_MSTATE_RECOVERY) {
                set_mission_state((ps_mission_state_t)p->mission_state);
                ack.status = PS_CMD_STATUS_DONE;
            } else {
                ack.status = PS_CMD_STATUS_REJECTED_INVALID;
            }
        } else if (cmd->cmd_code == 0x02 && cmd->param_len >= sizeof(ps_cmd_mission_set_rates_t)) {
            // MISSION_SET_RATES — acknowledge but rates are compile-time for now
            ack.status = PS_CMD_STATUS_ACCEPTED;
        } else {
            ack.status = PS_CMD_STATUS_REJECTED_INVALID;
        }
        break;

    case PS_CMD_CLASS_RECOVERY:
        ack.status = PS_CMD_STATUS_ACCEPTED;
        break;

    case PS_CMD_CLASS_MOTOR:
        ack.status = PS_CMD_STATUS_ACCEPTED;
        break;

    case PS_CMD_CLASS_CONFIG:
        ack.status = PS_CMD_STATUS_ACCEPTED;
        break;

    default:
        ack.status = PS_CMD_STATUS_REJECTED_INVALID;
        break;
    }

    enqueue_frame(PS_MSG_CMD_ACK, (const uint8_t *)&ack, sizeof(ack));
}

static void lora_rx_task(void *arg)
{
    e22_start_rx();

    uint8_t rx_buf[256];
    ps_parser_t parser;
    ps_parser_init(&parser);

    ps_header_t hdr;
    uint8_t payload_buf[128];

    while (true) {
        size_t rx_len = 0;
        esp_err_t rx_err = e22_receive(rx_buf, sizeof(rx_buf), &rx_len);
        if (rx_err == ESP_OK && rx_len > 0) {
            update_link_quality();

            // Feed bytes into protocol parser
            for (size_t i = 0; i < rx_len; i++) {
                if (ps_parser_feed(&parser, rx_buf[i], &hdr, payload_buf, sizeof(payload_buf))) {
                    if (hdr.msg_type == PS_MSG_CMD) {
                        handle_command(&hdr, payload_buf);
                    }
                }
            }

            // Re-enter RX mode after processing
            e22_start_rx();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =============================================================================
// status_task — Periodic STATUS telemetry
// =============================================================================

static void status_task(void *arg)
{
    while (true) {
        e22_link_quality_t lq = get_link_quality();

        ps_status_t status = {};
        status.uptime_ms        = uptime_ms();
        status.battery_mv       = 0;     // placeholder — no ADC configured yet
        status.battery_pct_x100 = 0;
        status.storage_free_kb  = 0;     // placeholder
        status.mission_state    = (uint8_t)get_mission_state();
        status.recovery_flags   = 0;
        status.link_rssi_dbm    = lq.rssi_dbm;
        status.link_snr_db_x10  = lq.snr_db_x10;
        status.status_flags     = 0;

        enqueue_frame(PS_MSG_STATUS, (const uint8_t *)&status, sizeof(status));

        vTaskDelay(pdMS_TO_TICKS(STATUS_INTERVAL_MS));
    }
}

// =============================================================================
// app_main — Initialize shared SPI bus and start tasks
// =============================================================================

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== KFDS26 Main Node Starting ===");
    ESP_LOGI(TAG, "Node ID: %d", NODE_ID);

    // Create synchronization primitives
    s_tx_queue   = xQueueCreate(TX_QUEUE_LEN, sizeof(tx_frame_t));
    s_state_mutex = xSemaphoreCreateMutex();
    s_lq_mutex    = xSemaphoreCreateMutex();

    if (!s_tx_queue || !s_state_mutex || !s_lq_mutex) {
        ESP_LOGE(TAG, "Failed to create RTOS primitives");
        return;
    }

    // Initialize shared SPI bus (SPI2_HOST) — all 4 devices share this bus
    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = PIN_SPI_MOSI;
    bus_cfg.miso_io_num = PIN_SPI_MISO;
    bus_cfg.sclk_io_num = PIN_SPI_SCLK;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = 4096;
    bus_cfg.flags = 0;

    ESP_LOGI(TAG, "Initializing shared SPI bus (SPI2_HOST)...");
    ESP_ERROR_CHECK(spi_bus_initialize(POLYSENSE_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    // Initialize drivers — each adds itself to the shared bus
    ESP_LOGI(TAG, "Initializing BME688...");
    bool bme_ok = (bme688_init(POLYSENSE_SPI_HOST) == ESP_OK);
    if (!bme_ok) ESP_LOGW(TAG, "BME688 init failed — ENV telemetry disabled");

    ESP_LOGI(TAG, "Initializing ICM-42688-P...");
    bool imu_ok = (icm42688p_init(POLYSENSE_SPI_HOST) == ESP_OK);
    if (!imu_ok) ESP_LOGW(TAG, "ICM-42688-P init failed — IMU telemetry disabled");

    ESP_LOGI(TAG, "Initializing NEO-M9N GNSS...");
    bool gnss_ok = (neo_m9n_init(POLYSENSE_SPI_HOST) == ESP_OK);
    if (!gnss_ok) ESP_LOGW(TAG, "NEO-M9N init failed — GPS telemetry disabled");

    ESP_LOGI(TAG, "Initializing E22 LoRa...");
    bool lora_ok = (e22_lora_init(POLYSENSE_SPI_HOST) == ESP_OK);
    if (!lora_ok) {
        ESP_LOGE(TAG, "E22 LoRa init failed — cannot transmit");
        return;
    }

    // Start tasks
    xTaskCreate(sensor_task,  "sensor",   TASK_STACK_SENSOR,  NULL, TASK_PRIO_SENSOR,  NULL);
    xTaskCreate(gnss_task,    "gnss",     TASK_STACK_GNSS,    NULL, TASK_PRIO_GNSS,    NULL);
    xTaskCreate(lora_tx_task, "lora_tx",  TASK_STACK_LORA_TX, NULL, TASK_PRIO_LORA_TX, NULL);
    xTaskCreate(lora_rx_task, "lora_rx",  TASK_STACK_LORA_RX, NULL, TASK_PRIO_LORA_RX, NULL);
    xTaskCreate(status_task,  "status",   TASK_STACK_STATUS,  NULL, TASK_PRIO_STATUS,  NULL);

    ESP_LOGI(TAG, "All tasks started. Mission state: IDLE");
}
