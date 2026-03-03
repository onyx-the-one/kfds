# KFDS Software Repository

KFDS (KFDS25, KFDS26, and future versions) is the software for a next-generation CanSat platform that transforms into a semiautonomous rover post-landing. It features modular payloads, dual-redundant LoRa communications, rich environmental telemetry, and autonomous wheel deployment via nitinol actuators.

## Quick Overview

The codebase powers:
- **Node firmware** (ESP32-S3): Sensor fusion (IMU, GNSS, environmental, radiation), motor control, state machines for flight-to-rover transition, and binary telemetry over dual LoRa radios (E22 primary + RFM95W backup) with CRC-32 integrity.
- **Ground Station (GS)**: Telemetry reception, logging, real-time dashboard, and FPV video integration.
- **Shared Protocols**: Compact binary packet format (preamble + header + CSV payload + CRC) used by both node and GS for zero-loss LoS communication.

Designed for reliability: RTOS tasks, error recovery (LoRa failover, GNSS dead-reckoning), OTA updates, and onboard microSD logging.

## Repository Structure

```
KFDS/
├── KFDS25/           # Previous iteration (archived)
├── KFDS26/
│   ├── main/         # Primary node firmware + shared protocols
│   │   ├── node/     # ESP32-S3 rover firmware
│   │   └── gs/       # Ground station receiver + dashboard
│   └── backupsys/    # Redundant backup node/GS implementations
│       ├── backupsys-node/
│       └── backupsys-gs/
├── KFDS27/           # Future development (empty)
├── docs/             # Technical reports (kfds26-pdr-6.2.pdf, kfds_1001-b.pdf)
└── .gitignore        # Ignores build artifacts, logs, etc.
```

Active development is in `KFDS26`. Protocols are shared between `main` and `backupsys` to avoid duplication.

## Tech Stack

| Component      | Tech                          | Purpose                          |
|----------------|-------------------------------|----------------------------------|
| Node Firmware | ESP32-S3, FreeRTOS, ESP-IDF  | Real-time control + telemetry   |
| Protocols     | Custom binary (C structs)    | LoRa packetization (500-5000 bps)|
| GS Software   | Python 3, PySerial, PyQt5    | Telemetry decode + visualization |
| Comms         | LoRa SX1262/SX1276 (868 MHz) | Dual-redundant + adaptive SF    |
| Logging       | SQLite + microSD (node)      | Post-mission analysis            |

## Quick Start

### Node (ESP32)
1. Clone: `git clone https://github.com/onyx-the-one/KFDS`
2. Node: `cd KFDS26/main/node && idf.py build flash monitor`

### Ground Station
1. `cd KFDS26/main/gs`
2. `pip install -r requirements.txt`
3. `python gs_main.py --port /dev/ttyUSB0`

See `KFDS26/main/node/README.md` for full build instructions.

## Development Status

- **KFDS26**: About to go into prototype validation, waiting on hardware.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

***

**Contributing**: Fork, branch (`feature/...`), PR to `main`. Issues welcome for bug reports or payload ideas.

