/*
 * SomersetEV Tractor Telematics
 * Hardware: LILYGO T-CAN485
 * Framework: ESP-IDF
 *
 * Architecture:
 *   Core 1: CAN RX task, log writer task
 *   Core 0: BLE NUS task
 *
 * Storage:
 *   SD card (SPI/FAT32) — CSV, primary archive
 *   BLE NUS             — binary sync to phone app
 *   NVS                 — session counter, last synced session, config
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "can_handler.h"
#include "sd_logger.h"
#include "ble_nus.h"
#include "vehicle_state.h"
#include "web_interface.h"
#include "oi_can.h"
#include "nvs.h"

static const char *TAG = "MAIN";

// Reads and clears the boot-mode NVS flag set by ble_nus.c's WIFI_MODE
// command handler. Defaults to BOOT_MODE_LOGGING if unset/unreadable —
// the normal driving-mode pipeline is the safe default on first boot or
// after NVS erase.
static boot_mode_t read_and_clear_boot_mode(void)
{
    boot_mode_t mode = BOOT_MODE_LOGGING;
    nvs_handle_t nvs;
    if (nvs_open("telematics", NVS_READWRITE, &nvs) == ESP_OK) {
        uint8_t stored = BOOT_MODE_LOGGING;
        if (nvs_get_u8(nvs, NVS_KEY_BOOT_MODE, &stored) == ESP_OK) {
            mode = (boot_mode_t)stored;
        }
        // Always boot back into logging mode next time unless WIFI_MODE sets
        // the flag again — web-interface mode is opt-in per boot, not sticky.
        nvs_set_u8(nvs, NVS_KEY_BOOT_MODE, BOOT_MODE_LOGGING);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    return mode;
}

static void start_logging_pipeline(void)
{
    vehicle_state_init();

    // Deep queue — raw CAN frames at full bus rate plus 1Hz snapshots
    QueueHandle_t log_queue = xQueueCreate(512, sizeof(log_msg_t));
    configASSERT(log_queue);

    // Start tasks — CAN and SD writer pinned to Core 1, BLE on Core 0
    xTaskCreatePinnedToCore(can_rx_task,    "can_rx",    4096, log_queue, 5, NULL, 1);
    xTaskCreatePinnedToCore(sd_logger_task, "sd_logger", 8192, log_queue, 3, NULL, 1);
    xTaskCreatePinnedToCore(ble_nus_task,   "ble_nus",   8192, log_queue, 4, NULL, 0);

    ESP_LOGI(TAG, "Logging pipeline started (BLE + listen-only CAN + SD)");
}

static void web_interface_poll_task(void *pvParameters)
{
    (void)pvParameters;
    while (1) {
        oi_can_loop();
        // At least 1 tick — pdMS_TO_TICKS(5) is 0 at the 100Hz tick rate,
        // which busy-loops this task and starves IDLE1 (task_wdt trips).
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void start_web_interface_pipeline(void)
{
    g_app_mode = APP_MODE_WEB_INTERFACE;

    // BLE stays up so the phone app can still see status/trigger reboot,
    // but runs the (empty) logging pipeline's queue plumbing unused.
    QueueHandle_t log_queue = xQueueCreate(8, sizeof(log_msg_t));
    configASSERT(log_queue);
    xTaskCreatePinnedToCore(ble_nus_task, "ble_nus", 8192, log_queue, 4, NULL, 0);

    ESP_ERROR_CHECK(web_interface_start());
    xTaskCreatePinnedToCore(web_interface_poll_task, "oi_can_poll", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "Web-interface pipeline started (WiFi + HTTP + transmit-capable CAN)");
}

void app_main(void)
{
    ESP_LOGI(TAG, "SomersetEV Telematics starting...");

    // Init NVS (needed for BLE and session counter)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    boot_mode_t mode = read_and_clear_boot_mode();
    if (mode == BOOT_MODE_WEB_INTERFACE) {
        start_web_interface_pipeline();
    } else {
        start_logging_pipeline();
    }

    ESP_LOGI(TAG, "All tasks started");
}
