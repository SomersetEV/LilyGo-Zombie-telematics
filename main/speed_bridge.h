#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * speed_bridge.h
 * GPS ground speed relayed from the phone app over BLE, re-broadcast on CAN.
 *
 * The phone streams "SPD <centi_mph> <fix> [<heading_ddeg>]" over the NUS RX
 * characteristic. speed_bridge caches the latest sample with a monotonic
 * timestamp; speed_tx_task re-broadcasts it as a fixed-rate CAN frame so the
 * Leyland dash can show road speed without a GPS module of its own.
 *
 * Writer: NimBLE host task (Core 0), via speed_bridge_try_consume().
 * Reader: speed_tx_task (Core 1), via speed_bridge_get().
 * The pair is guarded by a spinlock — deliberately NOT part of vehicle_state_t,
 * whose single-writer (can_rx_task) invariant would otherwise be broken.
 */

// ── CAN OUTPUT CONTRACT — edit here only ────────────────────────────────────
// Consumer: ElectricLeyland255 Nextion dash (Arduino Due, due_can).
// 0x3E8 collides with nothing this firmware decodes (can_handler.c) nor
// anything the dash handles (0x1DA/0x323/0x33B/0x355/0x356/0x373/0x39F/
// 0x521-0x528/0x55A/0x603/0x666/0x6F6).
#define SPEED_CAN_ID            0x3E8u
#define SPEED_CAN_DLC           8
#define SPEED_CAN_PERIOD_MS     100     // 10 Hz. Must be a multiple of the 10ms FreeRTOS tick.
#define SPEED_CAN_STALE_MS      1000    // sample older than this is published as invalid
#define SPEED_UNKNOWN           0xFFFFu // sentinel for speed and heading — never 0

// Payload (little-endian, matching the BMS/Leaf/MG frames already decoded):
//   b0-1  speed    uint16 LE, 0.01 mph, 0xFFFF = invalid
//   b2-3  heading  uint16 LE, 0.1 deg,  0xFFFF = unknown
//   b4    status   b0 speed_valid, b1 gps_fix, b2 ble_connected, b3 stale
//   b5    age      uint8, sample age in 10ms units, saturating at 0xFF
//   b6    counter  uint8, +1 per frame, wraps
//   b7    checksum uint8, XOR of b0..b6
#define SPEED_ST_VALID          0x01u
#define SPEED_ST_FIX            0x02u
#define SPEED_ST_BLE            0x04u
#define SPEED_ST_STALE          0x08u

// Interlocks before the first transmit — see speed_bridge.c
#define SPEED_TX_MIN_RX_FRAMES  20      // proves bitrate is right and other nodes exist
#define SPEED_TX_MAX_ACK_FAILS  10      // consecutive ack failures before latching off
#define SPEED_TX_MAX_BUS_OFF    5       // bus-off events before latching off

typedef struct {
    uint16_t speed_cmph;    // 0.01 mph, SPEED_UNKNOWN if not known
    uint16_t heading_ddeg;  // 0.1 deg,  SPEED_UNKNOWN if not known
    uint8_t  fix;           // 1 = phone reports a usable GPS fix
    int64_t  rx_us;         // esp_timer_get_time() at receipt; 0 = never received
    uint32_t rx_count;
} gps_speed_t;

void     speed_bridge_init(void);
bool     speed_bridge_try_consume(const char *text, uint16_t len); // true => was a SPD command
void     speed_bridge_invalidate(void);
void     speed_bridge_get(gps_speed_t *out);
uint32_t speed_bridge_age_ms(const gps_speed_t *s);   // UINT32_MAX if never received

bool     speed_bridge_tx_enabled(void);
bool     speed_bridge_set_tx_enabled(bool enable);    // persists to NVS
void     speed_bridge_status_str(char *buf, size_t n);
void     speed_bridge_note_collision(void);           // called from can_rx_task on ID clash

void     speed_tx_task(void *pvParameters);
