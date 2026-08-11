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
| `STATUS` | Report trip state — `STATUS trip=<0\|1>` |
| `SUMMARY <id>` | Re-emit a session's trip summary as a `JOB` line |
| `SPD <centi_mph> <fix> [<heading_ddeg>]` | GPS ground speed from the phone. No reply. |
| `SPDCAN <0\|1>` | Enable/disable the CAN speed broadcast (persisted, needs reboot) |
| `SPDSTAT` | Report GPS speed cache and CAN TX counters |

## GPS speed relay

The phone app streams its GPS ground speed over BLE (`SPD`, 5 Hz) and the device
re-broadcasts it on CAN so the Leyland 255 Nextion dash can show road speed —
replacing an untested NEO-6M module on the dash.

Frame `0x3E8`, 8 bytes, 10 Hz:

| byte | field |
|---|---|
| 0-1 | speed, uint16 LE, 0.01 mph. `0xFFFF` = unknown (never 0) |
| 2-3 | heading, uint16 LE, 0.1 deg. `0xFFFF` = unknown |
| 4 | status: b0 valid, b1 gps fix, b2 phone connected, b3 stale |
| 5 | sample age in 10 ms units, saturating at `0xFF` |
| 6 | rolling counter, +1 per frame |
| 7 | XOR checksum of bytes 0-6 |

If the phone disconnects or its fix goes stale (>1 s), the frame keeps
transmitting with speed `0xFFFF` and the valid bit clear. It never publishes
0 mph on loss — a consumer must not mistake "unknown" for "stopped".

> **This is the only feature that makes the device transmit.** By default the
> TWAI node is created listen-only exactly as before. `SPDCAN 1` clears
> listen-only at the next boot, which makes the device an error-active bus
> participant that **ACKs every frame** on the vehicle bus. Enable it
> deliberately, stationary, and confirm `0x3E8` is unused on your bus first
> (grep the archived `session_*.csv` logs). Transmit is additionally gated on
> having received 20 frames, and latches off after 10 consecutive unACKed
> transmits or 5 bus-off events — check `SPDSTAT` for the reason.

## Built With

- [ESP-IDF](https://github.com/espressif/esp-idf) v5.5.2
- NimBLE (via ESP-IDF `bt` component)
- ESP TWAI (onchip CAN driver, node API — requires IDF >= 5.5)
