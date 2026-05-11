# LilyGo Zombie Telematics

ESP32-based CAN bus data logger for Zombie based EV tractor conversions, built on the [LILYGO T-CAN485](http://www.lilygo.cc) development board.

## Overview

This firmware runs on the ESP32 inside the T-CAN485 and sits on the vehicle CAN bus in listen-only mode, recording data from all major EV drivetrain components to an SD card. A companion Android app connects over BLE to mark trip boundaries and download session data.

## Hardware

- **Board:** LILYGO T-CAN485 (ESP32, onboard CAN transceiver, SD card slot)
- **CAN bus:** 500 kbps, listen-only
- **Storage:** MicroSD (FAT32, SPI)
- **Wireless:** Bluetooth LE (NUS — Nordic UART Service)

## Data Sources

| Component | CAN IDs | Data |
|---|---|---|
| Tesla M3 BMS (SomersetEV firmware) | 0x355, 0x356, 0x373 | SoC, pack voltage, cell voltages, temperatures |
| ISA IVT-S Current Shunt | 0x521, 0x522, 0x526, 0x527, 0x528 | Current, voltage, power, Ah, kWh |
| Nissan Leaf Inverter | 0x1DA, 0x55A | Motor RPM, motor temp, inverter temp |
| MG Gen2 V2L Charger | 0x39F, 0x323, 0x33B | LV voltage, LV current, plug state, charger temp |

## Logging

Every known CAN frame is timestamped and written to `raw_XXXX.csv` on the SD card at full bus rate. A new file is created on each power cycle. Trip start and end markers are inserted by the phone app, and a summary row is written at trip end:

```
TRIP_START,,,,,,,,,,,
...raw CAN frames...
TRIP_END,<duration_s>,<ah_used>,<kwh_used>,<soc_start%>,<soc_end%>,<peak_current_a>,,,,
```

## BLE Interface

The device advertises as `SomersetEV-Tractor`. The phone app communicates over NUS using simple text commands:

| Command | Action |
|---|---|
| `LIST` | List completed session files |
| `GET <id>` | Stream session CSV to phone |
| `DONE <id>` | Acknowledge successful sync |
| `TIME <unix>` | Set RTC offset from phone clock |
| `TRIP_START` | Insert trip start marker |
| `TRIP_END` | Insert trip end marker with summary |

## Built With

- [ESP-IDF](https://github.com/espressif/esp-idf) v6.0
- NimBLE (via ESP-IDF `bt` component)
- ESP TWAI (onchip CAN driver)
