#pragma once
#include <stdint.h>

/*
 * vehicle_state.h
 * Central struct holding latest decoded values from all CAN sources.
 * Written exclusively by can_rx_task (Core 1).
 * Read by sd_logger_task and ble_nus_task.
 * No mutex needed on reads — single writer, values are individually atomic at this size.
 */

// ── Log record written to SD card and transmitted over BLE ──────────────────
// Keep fields int16_t/uint16_t where possible — compact and sufficient range with scaling.
// Scaling noted inline.

typedef struct __attribute__((packed)) {
    uint32_t tick_ms;           // ms since boot (no RTC — phone TIME command adds offset)
    int32_t  unix_offset;       // 0 until phone syncs time, then (unix_time - tick_ms/1000)

    // ── ZombieVerter / Leaf inverter ─────────────────────────────────────────
    // TODO: populate from ZombieVerter CAN messages once IDs confirmed
    int16_t  motor_rpm;         // RPM, signed (negative = reverse)
    int16_t  motor_temp;        // °C x10
    int16_t  inverter_temp;     // °C x10

    // ── ISA IVT-S Shunt (0x521–0x528, int32 bytes 2-5) ──────────────────────
    int32_t  pack_current_ma;   // mA  (÷1000 = A)   — 0x521
    int32_t  pack_voltage_mv;   // mV  (÷1000 = V)   — 0x522
    int32_t  isa_kw;            // W   (÷1000 = kW)  — 0x526
    int32_t  isa_ah;            // As  (÷3600 = Ah)  — 0x527
    int32_t  isa_kwh;           // Wh  (÷1000 = kWh) — 0x528

    // ── M3 BMS (0x355 SoC, 0x356 Pack, 0x373 Cells) ─────────────────────────
    uint8_t  soc;               // % (0–100), from 0x355 byte 0
    int16_t  bms_temp_max;      // °C x10, from 0x373 bytes 6-7 (Kelvin - 273) x10
    int16_t  bms_temp_min;      // °C x10, from 0x373 bytes 4-5 (Kelvin - 273) x10
    uint16_t cell_voltage_max;  // mV, from 0x373 bytes 2-3
    uint16_t cell_voltage_min;  // mV, from 0x373 bytes 0-1
    uint16_t pack_voltage_bms;  // 0.01V units, from 0x356 bytes 0-1

    // ── MG Gen2 V2L Charger (0x39F LV, 0x323 plug, 0x33B temp) ─────────────
    uint16_t lv_volts_mv;       // mV (bytes[1]*125), from 0x39F
    uint8_t  lv_amps;           // A  (bytes[4] raw), from 0x39F
    uint8_t  plug_state;        // 0=unplugged 1=plugged, from 0x323 byte 5
    int16_t  charger_temp;      // °C x10 ((byte[3]-50)*10), from 0x33B

} log_record_t;                 // ~46 bytes per record

// ── Raw CAN frame for high-rate logging ──────────────────────────────────────
typedef struct {
    uint32_t tick_ms;
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} raw_can_log_t;

// ── Log queue message — carries a data record, raw frame, or trip marker ─────
// Using a tagged union keeps the queue a single type so both the CAN task
// and the BLE task can post to it without a separate marker queue.

typedef enum {
    LOG_MSG_RAW_FRAME,  // every CAN frame as it arrives
    LOG_MSG_TRIP_START, // TRIP_START marker — write sentinel row to raw log
    LOG_MSG_TRIP_END,   // TRIP_END marker   — write sentinel row to raw log
} log_msg_type_t;

typedef struct {
    log_msg_type_t type;
    raw_can_log_t  frame; // only valid when type == LOG_MSG_RAW_FRAME
} log_msg_t;

// ── Live state (superset of log record, includes raw/debug fields) ───────────
typedef struct {
    log_record_t latest;
    uint8_t      bms_status;    // raw status byte from BMS if available
    uint8_t      inverter_status;
    uint8_t      can_rx_errors; // incremented on unknown/malformed frames
} vehicle_state_t;

void vehicle_state_init(void);
vehicle_state_t *vehicle_state_get(void);
