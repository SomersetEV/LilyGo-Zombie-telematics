#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    APP_MODE_DETECTING,      // waiting to see if phone sends a command
    APP_MODE_TELEMATICS,     // command received — session sync protocol
    APP_MODE_SPEEDO,         // no command after timeout — live CAN streaming
    APP_MODE_WEB_INTERFACE,  // WiFi + HTTP server active, CAN transmit-capable
} app_mode_t;

// NVS key (namespace "telematics", see ble_nus.c NVS_NAMESPACE) used to select
// which pipeline app_main() starts on the next boot
#define NVS_KEY_BOOT_MODE       "boot_mode"
typedef enum {
    BOOT_MODE_LOGGING = 0,   // default: BLE + listen-only CAN + SD logging
    BOOT_MODE_WEB_INTERFACE, // WiFi + HTTP server + transmit-capable CAN
} boot_mode_t;

extern QueueHandle_t       g_ble_live_queue;  // raw CAN frames for Speedo mode
extern volatile app_mode_t g_app_mode;

void ble_nus_task(void *pvParameters);
