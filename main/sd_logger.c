#include "sd_logger.h"
#include "vehicle_state.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>

static const char *TAG    = "SD";
#define MOUNT_POINT       "/sdcard"
#define NVS_NAMESPACE     "telematics"
#define NVS_KEY_SESSION   "session_id"

#define CAN_ID_ISA_CURRENT  0x521

// ── LILYGO T-CAN485 SD SPI pins ──────────────────────────────────────────────
#define SD_MOSI  GPIO_NUM_15
#define SD_MISO  GPIO_NUM_2
#define SD_CLK   GPIO_NUM_14
#define SD_CS    GPIO_NUM_13

static uint32_t current_session_id = 0;
static FILE    *raw_file           = NULL;

// ── Trip tracking ─────────────────────────────────────────────────────────────
static bool     trip_active        = false;
static int32_t  trip_start_ah      = 0;
static int32_t  trip_start_kwh     = 0;
static uint8_t  trip_start_soc     = 0;
static uint32_t trip_start_tick    = 0;
static int32_t  trip_peak_current  = 0;   // peak absolute mA seen during trip

static uint32_t load_and_increment_session(void)
{
    nvs_handle_t nvs;
    uint32_t id = 0;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, NVS_KEY_SESSION, &id);
        nvs_set_u32(nvs, NVS_KEY_SESSION, id + 1);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    return id;
}

static bool sd_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 5,
        .allocation_unit_size   = 16 * 1024
    };

    sdmmc_card_t *card;
    sdmmc_host_t host         = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg  = {
        .mosi_io_num   = SD_MOSI,
        .miso_io_num   = SD_MISO,
        .sclk_io_num   = SD_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS;

    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_err_t ret = esp_vfs_fat_sdspi_mount(MOUNT_POINT, &host, &slot_cfg, &mount_cfg, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "SD mounted. Card: %s %lluMB",
             card->cid.name,
             ((uint64_t)card->csd.capacity * card->csd.sector_size) >> 20);
    return true;
}

static bool open_raw_file(uint32_t session_id)
{
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/raw_%04lu.csv", session_id);

    raw_file = fopen(path, "w");
    if (!raw_file) {
        ESP_LOGE(TAG, "Failed to open %s", path);
        return false;
    }

    fprintf(raw_file, "Time Stamp,ID,Extended,Bus,LEN,D1,D2,D3,D4,D5,D6,D7,D8\n");
    fflush(raw_file);

    ESP_LOGI(TAG, "Opened raw CAN file: %s", path);
    return true;
}

static void write_raw_frame(const raw_can_log_t *f)
{
    if (!raw_file) return;

    fprintf(raw_file, "%lu,0x%03lX,false,0,%u,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\n",
        f->tick_ms, f->id, f->dlc,
        f->data[0], f->data[1], f->data[2], f->data[3],
        f->data[4], f->data[5], f->data[6], f->data[7]);

    // Track peak current during a trip
    if (trip_active && f->id == CAN_ID_ISA_CURRENT && f->dlc >= 6) {
        int32_t current_ma = (int32_t)(
            ((uint32_t)f->data[5] << 24) |
            ((uint32_t)f->data[4] << 16) |
            ((uint32_t)f->data[3] <<  8) |
             (uint32_t)f->data[2]);
        int32_t abs_ma = current_ma < 0 ? -current_ma : current_ma;
        if (abs_ma > trip_peak_current) trip_peak_current = abs_ma;
    }

    // Flush every 100 frames to limit data loss on power cut
    static uint8_t flush_ctr = 0;
    if (++flush_ctr >= 100) {
        fflush(raw_file);
        flush_ctr = 0;
    }
}

static void handle_trip_start(void)
{
    vehicle_state_t *state = vehicle_state_get();

    trip_start_ah      = state->latest.isa_ah;
    trip_start_kwh     = state->latest.isa_kwh;
    trip_start_soc     = state->latest.soc;
    trip_start_tick    = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    trip_peak_current  = 0;
    trip_active        = true;

    if (!raw_file) return;
    fprintf(raw_file, "TRIP_START,,,,,,,,,,,\n");
    fflush(raw_file);
    ESP_LOGI(TAG, "Trip started — soc=%u%%", trip_start_soc);
}

static void handle_trip_end(void)
{
    trip_active = false;

    vehicle_state_t *state   = vehicle_state_get();
    uint32_t end_tick        = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t duration_s      = (end_tick - trip_start_tick) / 1000;
    float    ah_used         = (state->latest.isa_ah  - trip_start_ah)  / 3600.0f;
    float    kwh_used        = (state->latest.isa_kwh - trip_start_kwh) / 1000.0f;
    uint8_t  soc_end         = state->latest.soc;
    float    peak_current_a  = trip_peak_current / 1000.0f;

    if (!raw_file) return;

    // Summary row — columns: label, duration_s, ah, kwh, soc_start, soc_end, peak_a
    fprintf(raw_file,
        "TRIP_END,%lu,%.2f,%.3f,%u,%u,%.1f,,,,\n",
        duration_s, ah_used, kwh_used,
        trip_start_soc, soc_end, peak_current_a);
    fflush(raw_file);

    ESP_LOGI(TAG, "Trip ended — %lus, %.2fAh, %.3fkWh, SoC %u%%→%u%%, peak %.1fA",
             duration_s, ah_used, kwh_used, trip_start_soc, soc_end, peak_current_a);
}

void sd_logger_task(void *pvParameters)
{
    QueueHandle_t log_queue = (QueueHandle_t)pvParameters;

    while (!sd_init()) {
        ESP_LOGW(TAG, "SD init failed — retrying in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    current_session_id = load_and_increment_session();
    open_raw_file(current_session_id);

    log_msg_t msg;
    while (1) {
        if (xQueueReceive(log_queue, &msg, portMAX_DELAY) == pdTRUE) {
            switch (msg.type) {
                case LOG_MSG_RAW_FRAME:
                    write_raw_frame(&msg.frame);
                    break;
                case LOG_MSG_TRIP_START:
                    handle_trip_start();
                    break;
                case LOG_MSG_TRIP_END:
                    handle_trip_end();
                    break;
            }
        }
    }
}
