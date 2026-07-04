/*
 * SomersetEV Tractor Telematics
 * Hardware: LILYGO T-CAN485
 * Framework: ESP-IDF
 *
 * Architecture:
 *   Core 1: CAN RX task — ISR-driven, decodes frames into vehicle_state only
 *   Core 0: SD logger task (trip summary on TRIP_END) + BLE NUS task
 *
 * Storage:
 *   SD card (SPI/FAT32) — one snap_XXXX.csv summary row per trip
 *   BLE NUS             — session sync to phone app (LIST/GET/DONE/SUMMARY)
 *   NVS                 — session counter, last synced session
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "can_handler.h"
#include "can_logger.h"
#include "sd_logger.h"
#include "ble_nus.h"
#include "vehicle_state.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "SomersetEV Telematics starting...");

    // Init NVS (needed for BLE and session counter)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // Initialise shared vehicle state
    vehicle_state_init();

    // Small queue — only TRIP_START/TRIP_END events (no per-frame logging)
    QueueHandle_t log_queue = xQueueCreate(8, sizeof(log_msg_t));
    configASSERT(log_queue);

    // Core 1: CAN RX — ISR-driven, decodes frames into vehicle_state only
    // Core 0: SD logger waits for trip events; BLE handles phone sync
    xTaskCreatePinnedToCore(can_rx_task,     "can_rx",     4096, NULL,       5, NULL, 1);
    xTaskCreatePinnedToCore(sd_logger_task,  "sd_logger",  4096, log_queue,  4, NULL, 0);
    xTaskCreatePinnedToCore(can_logger_task, "can_logger", 4096, NULL,       4, NULL, 0);
    xTaskCreatePinnedToCore(ble_nus_task,    "ble_nus",    8192, log_queue,  3, NULL, 0);

    ESP_LOGI(TAG, "All tasks started");
}
