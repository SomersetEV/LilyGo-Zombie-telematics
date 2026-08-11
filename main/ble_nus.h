#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdbool.h>

typedef enum {
    APP_MODE_DETECTING,   // waiting to see if phone sends a command
    APP_MODE_TELEMATICS,  // command received — session sync protocol
    APP_MODE_SPEEDO,      // no command after timeout — live CAN streaming
} app_mode_t;

extern QueueHandle_t       g_ble_live_queue;  // raw CAN frames for Speedo mode
extern volatile app_mode_t g_app_mode;

void ble_nus_task(void *pvParameters);
bool ble_nus_is_connected(void);
