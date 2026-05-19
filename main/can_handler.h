#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void     can_rx_task(void *pvParameters);
uint32_t can_handler_frame_count(void);
