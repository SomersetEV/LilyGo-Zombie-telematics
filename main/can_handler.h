#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_twai.h"
#include <stdbool.h>

void     can_rx_task(void *pvParameters);
uint32_t can_handler_frame_count(void);

// ── Transmit ─────────────────────────────────────────────────────────────────
// The node is created listen-only unless the NVS gate (SPDCAN) is set, in which
// case it becomes an error-active bus participant that ACKs every frame.
//
// NOTE: the driver queues a POINTER to `frame` (tx_mount_queue holds
// twai_frame_t*), so `frame` and its buffer must stay valid until the transmit
// completes. Do not pass a stack local.
esp_err_t can_handler_transmit(const twai_frame_t *frame, int timeout_ms);
bool      can_handler_tx_capable(void);     // node exists and is not listen-only
void      can_handler_service(void);        // task context: bus-off recovery

uint32_t  can_handler_tx_ok_count(void);
uint32_t  can_handler_tx_fail_count(void);
uint32_t  can_handler_ack_fail_streak(void);
uint32_t  can_handler_bus_off_count(void);
