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

void     can_rx_task(void *pvParameters);
uint32_t can_handler_frame_count(void);

// Brings up a listen-only (non-transmitting) TWAI node at 500kbps.
// Used internally by can_rx_task(); exposed so other callers can query the
// same bring-up path/config without duplicating it.
esp_err_t can_handler_init_listen_only(twai_node_handle_t *out_node);
