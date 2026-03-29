#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

// ── Shared SPI Bus (SPI2_HOST) ──────────────────────────────────────────
#define POLYSENSE_SPI_HOST    SPI2_HOST
#define PIN_SPI_MOSI          GPIO_NUM_11
#define PIN_SPI_MISO          GPIO_NUM_13
#define PIN_SPI_SCLK          GPIO_NUM_12

// ── Chip Select Lines (active low) ─────────────────────────────────────
#define PIN_CS_IMU            GPIO_NUM_10   // ICM-42688-P
#define PIN_CS_ENV            GPIO_NUM_9    // BME688
#define PIN_CS_LORA           GPIO_NUM_14   // E22-900M22S (NSS)
#define PIN_CS_GPS            GPIO_NUM_15   // NEO-M9N

// ── ICM-42688-P Interrupts ─────────────────────────────────────────────
#define PIN_IMU_INT1          GPIO_NUM_4
#define PIN_IMU_INT2          GPIO_NUM_5

// ── NEO-M9N Control ────────────────────────────────────────────────────
#define PIN_GPS_EXTINT        GPIO_NUM_6

// ── E22-900M22S Control ────────────────────────────────────────────────
#define PIN_LORA_BUSY         GPIO_NUM_7
#define PIN_LORA_DIO1         GPIO_NUM_8
#define PIN_LORA_RST          GPIO_NUM_16

// ── SPI Clock Speeds ───────────────────────────────────────────────────
#define SPI_CLK_IMU           1000000   // ICM-42688-P: 1 MHz (supports up to 24 MHz)
#define SPI_CLK_ENV           1000000   // BME688: 1 MHz (supports up to 10 MHz)
#define SPI_CLK_LORA          1000000   // E22/SX1262: 1 MHz (supports up to 16 MHz)
#define SPI_CLK_GPS           1000000   // NEO-M9N: 1 MHz (max 5.5 MHz)

// ── PolySense Protocol ─────────────────────────────────────────────────
#define NODE_ID               0x00      // Main system
#define POLYSENSE_SYNC_WORD   0x55AA

// ── Telemetry rates (Hz) ───────────────────────────────────────────────
#define ENV_HZ                1       // BME688 environmental
#define IMU_HZ                10      // ICM-42688-P accel+gyro
#define GPS_HZ                1       // NEO-M9N GNSS
#define STATUS_HZ             1       // Status telemetry

// Derived intervals in ms
#define ENV_INTERVAL_MS       (1000 / ENV_HZ)
#define IMU_INTERVAL_MS       (1000 / IMU_HZ)
#define GPS_INTERVAL_MS       (1000 / GPS_HZ)
#define STATUS_INTERVAL_MS    (1000 / STATUS_HZ)

// ── LoRa radio parameters ──────────────────────────────────────────────
#define LORA_FREQUENCY_HZ     868000000       // 868 MHz
#define LORA_SF               7               // Spreading factor
#define LORA_BW               125000          // Bandwidth in Hz (125 kHz)
#define LORA_CR               5               // Coding rate 4/5
#define LORA_TX_POWER_DBM     22              // Max 22 dBm for E22-900M22S
#define LORA_PREAMBLE_LEN     8
#define LORA_SYNC_WORD        0x12            // Private network

// ── FreeRTOS queue / task config ────────────────────────────────────────
#define TX_QUEUE_LEN          16              // Max frames waiting to TX
#define TX_FRAME_MAX_SIZE     128             // Max bytes per frame

#define TASK_STACK_SENSOR     4096
#define TASK_STACK_GNSS       4096
#define TASK_STACK_LORA_TX    4096
#define TASK_STACK_LORA_RX    4096
#define TASK_STACK_STATUS     3072

#define TASK_PRIO_SENSOR      5
#define TASK_PRIO_GNSS        5
#define TASK_PRIO_LORA_TX     6
#define TASK_PRIO_LORA_RX     7
#define TASK_PRIO_STATUS      3
