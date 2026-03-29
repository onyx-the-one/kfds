#pragma once

// =============================================================================
// KFDS26 Main Node Configuration
// =============================================================================
// All pin assignments are PLACEHOLDERS — update when PCB design is finalized.
// =============================================================================

// -----------------------------------------------------------------------------
// Node identity
// -----------------------------------------------------------------------------
#define NODE_ID                 0       // Main node (backup uses 1)

// -----------------------------------------------------------------------------
// I2C Bus — BME688
// -----------------------------------------------------------------------------
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       GPIO_NUM_5      // placeholder
#define I2C_MASTER_SCL_IO       GPIO_NUM_4      // placeholder
#define I2C_MASTER_FREQ_HZ      400000

// BME688 I2C addresses (try 0x76 first, then 0x77)
#define BME688_ADDR_PRIMARY     0x76
#define BME688_ADDR_SECONDARY   0x77

// -----------------------------------------------------------------------------
// SPI Bus 1 — ICM-42688-P (IMU)
// -----------------------------------------------------------------------------
#define IMU_SPI_HOST            SPI2_HOST
#define IMU_SPI_MOSI            GPIO_NUM_11     // placeholder
#define IMU_SPI_MISO            GPIO_NUM_13     // placeholder
#define IMU_SPI_SCLK            GPIO_NUM_12     // placeholder
#define IMU_SPI_CS              GPIO_NUM_10     // placeholder
#define IMU_SPI_CLOCK_HZ        (10 * 1000 * 1000)  // 10 MHz (max 24 MHz)

// -----------------------------------------------------------------------------
// SPI Bus 2 — E22-900M22S (LoRa / SX1262)
// -----------------------------------------------------------------------------
#define LORA_SPI_HOST           SPI3_HOST
#define LORA_SPI_MOSI           GPIO_NUM_35     // placeholder
#define LORA_SPI_MISO           GPIO_NUM_37     // placeholder
#define LORA_SPI_SCLK           GPIO_NUM_36     // placeholder
#define LORA_SPI_CS             GPIO_NUM_34     // placeholder
#define LORA_PIN_BUSY           GPIO_NUM_38     // placeholder
#define LORA_PIN_DIO1           GPIO_NUM_39     // placeholder
#define LORA_PIN_RST            GPIO_NUM_40     // placeholder

// -----------------------------------------------------------------------------
// UART1 — NEO-M9N (GNSS)
// -----------------------------------------------------------------------------
#define GNSS_UART_NUM           UART_NUM_1
#define GNSS_UART_TX            GPIO_NUM_17     // placeholder
#define GNSS_UART_RX            GPIO_NUM_18     // placeholder
#define GNSS_UART_BAUD          38400

// -----------------------------------------------------------------------------
// Telemetry rates (Hz)
// -----------------------------------------------------------------------------
#define ENV_HZ                  1       // BME688 environmental
#define IMU_HZ                  10      // ICM-42688-P accel+gyro
#define GPS_HZ                  1       // NEO-M9N GNSS
#define STATUS_HZ               1       // Status telemetry

// Derived intervals in ms
#define ENV_INTERVAL_MS         (1000 / ENV_HZ)
#define IMU_INTERVAL_MS         (1000 / IMU_HZ)
#define GPS_INTERVAL_MS         (1000 / GPS_HZ)
#define STATUS_INTERVAL_MS      (1000 / STATUS_HZ)

// -----------------------------------------------------------------------------
// LoRa radio parameters
// -----------------------------------------------------------------------------
#define LORA_FREQUENCY_HZ       868000000       // 868 MHz
#define LORA_SF                 7               // Spreading factor
#define LORA_BW                 125000          // Bandwidth in Hz (125 kHz)
#define LORA_CR                 5               // Coding rate 4/5
#define LORA_TX_POWER_DBM       22              // Max 22 dBm for E22-900M22S
#define LORA_PREAMBLE_LEN       8
#define LORA_SYNC_WORD          0x12            // Private network

// -----------------------------------------------------------------------------
// FreeRTOS queue / task config
// -----------------------------------------------------------------------------
#define TX_QUEUE_LEN            16              // Max frames waiting to TX
#define TX_FRAME_MAX_SIZE       128             // Max bytes per frame

#define TASK_STACK_SENSOR       4096
#define TASK_STACK_GNSS         4096
#define TASK_STACK_LORA_TX      4096
#define TASK_STACK_LORA_RX      4096
#define TASK_STACK_STATUS       3072

#define TASK_PRIO_SENSOR        5
#define TASK_PRIO_GNSS          5
#define TASK_PRIO_LORA_TX       6
#define TASK_PRIO_LORA_RX       7
#define TASK_PRIO_STATUS        3
