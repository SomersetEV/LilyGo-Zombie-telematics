#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

void sd_logger_task(void *pvParameters);
bool sd_logger_trip_active(void);
