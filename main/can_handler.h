#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_twai_onchip.h"
#include "esp_twai.h"

// LILYGO T-CAN485 pin assignments — also used by the web-interface pipeline's
// CANopen/SDO client (oi_can.c) so both pipelines address the same transceiver.
#define CAN_TX_PIN  GPIO_NUM_27
#define CAN_RX_PIN  GPIO_NUM_26

// Transceiver control, from the LilyGo T-CAN485 reference design. These matter
// specifically for TX: the transceivers on these boards (SN65HVD230 Rs,
// TJA1051 S) keep the receiver alive in standby while disabling the
// transmitter — the classic "RX fine, transmits nothing" symptom. That is why
// the listen-only logging pipeline has always worked without driving them.
// Originally added in bb0c816 and lost in the 6663c45 revert.
#define CAN_5V_EN_PIN  GPIO_NUM_16   // boost rail supplying the transceiver
#define CAN_SE_PIN     GPIO_NUM_23   // SE/Rs: low = normal (high-speed) mode

void     can_rx_task(void *pvParameters);
uint32_t can_handler_frame_count(void);

// Brings up a listen-only (non-transmitting) TWAI node at 500kbps.
// Used internally by can_rx_task(); exposed so other callers can query the
// same bring-up path/config without duplicating it.
esp_err_t can_handler_init_listen_only(twai_node_handle_t *out_node);
