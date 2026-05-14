#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void     sd_logger_task(void *pvParameters);
bool     sd_logger_trip_active(void);
uint32_t sd_logger_can_frame_count(void);
bool     sd_logger_wait_rotate(uint32_t timeout_ms);
