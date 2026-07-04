#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * can_logger.h
 * Unfiltered raw CAN frame logger. Independent of trip logging.
 *
 * When active, every CAN frame received by can_rx_task is written verbatim to
 * a canlog_XXXX.csv file on the SD card. Started/stopped over BLE. The log
 * stays on the module and is never synced to the phone.
 */

// Dedicated Core-0 task — owns the log file and does all SD I/O.
void can_logger_task(void *pvParameters);

// Start / stop a capture. Called from the BLE command dispatcher.
void can_logger_start(void);
void can_logger_stop(void);

// True while a capture is running (reported in STATUS).
bool can_logger_active(void);

// Forward a received frame to the logger. Called unconditionally from
// can_rx_task; a no-op (early return) when logging is inactive.
void can_logger_submit(uint32_t id, uint8_t dlc, const uint8_t *data);
