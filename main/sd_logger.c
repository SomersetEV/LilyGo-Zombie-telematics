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
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>

static const char *TAG    = "SD";
#define MOUNT_POINT       "/sdcard"
#define NVS_NAMESPACE     "telematics"
#define NVS_KEY_SESSION   "session_id"

// ── LILYGO T-CAN485 SD SPI pins ──────────────────────────────────────────────
#define SD_MOSI  GPIO_NUM_15
#define SD_MISO  GPIO_NUM_2
#define SD_CLK   GPIO_NUM_14
#define SD_CS    GPIO_NUM_13

static bool     s_spi_bus_ok       = false;
static uint32_t current_session_id = 0;

// ── Session rotation semaphore ────────────────────────────────────────────────
static SemaphoreHandle_t rotate_sem = NULL;

// ── Trip tracking ─────────────────────────────────────────────────────────────
static volatile bool trip_active      = false;
static log_record_t  trip_start_state = {0};  // full vehicle state snapshot at TRIP_START
static uint32_t      trip_start_tick  = 0;
static int32_t       trip_peak_current = 0;   // peak absolute mA during trip
static int16_t       trip_peak_rpm    = 0;
static int16_t       trip_peak_motor_c = 0;   // °C x10
static int16_t       trip_peak_inv_c  = 0;    // °C x10
static int16_t       trip_peak_bms_c  = 0;    // °C x10

// ── NVS helpers ───────────────────────────────────────────────────────────────

static uint32_t load_session_id(void)
{
    nvs_handle_t nvs;
    uint32_t id = 1;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, NVS_KEY_SESSION, &id);
        if (id == 0) id = 1;
        nvs_close(nvs);
    }
    return id;
}

static void save_session_id(uint32_t id)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, NVS_KEY_SESSION, id);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

// ── SD init ───────────────────────────────────────────────────────────────────

static bool sd_init(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024
    };

    sdmmc_card_t *card;
    sdmmc_host_t host             = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_cfg.gpio_cs = SD_CS;

    if (!s_spi_bus_ok) {
        spi_bus_config_t bus_cfg = {
            .mosi_io_num   = SD_MOSI,
            .miso_io_num   = SD_MISO,
            .sclk_io_num   = SD_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        esp_err_t r = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "SPI bus init failed: %s", esp_err_to_name(r));
            return false;
        }
        s_spi_bus_ok = true;
    }

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

// ── 1Hz trip tick ─────────────────────────────────────────────────────────────
// Called every ~1 second during a trip. Writes one SNAP1 row to the open snap
// file and updates running peak values. The phone's kWh formula sums packKw at
// 1Hz and divides by 3600 — correct results require 1Hz rows.

static FILE *snap_file = NULL;

static void trip_tick(void)
{
    const log_record_t *r = &vehicle_state_get()->latest;

    // Update peaks
    int32_t abs_ma = r->pack_current_ma < 0 ? -r->pack_current_ma : r->pack_current_ma;
    if (abs_ma           > trip_peak_current)  trip_peak_current  = abs_ma;
    if (r->motor_rpm     > trip_peak_rpm)      trip_peak_rpm      = r->motor_rpm;
    if (r->motor_temp    > trip_peak_motor_c)  trip_peak_motor_c  = r->motor_temp;
    if (r->inverter_temp > trip_peak_inv_c)    trip_peak_inv_c    = r->inverter_temp;
    if (r->bms_temp_max  > trip_peak_bms_c)    trip_peak_bms_c    = r->bms_temp_max;

    // Write 1Hz SNAP1 row
    if (!snap_file) return;
    uint32_t tick_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    fprintf(snap_file, "SNAP1,%lu,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%u,%u,%d\n",
        (unsigned long)tick_ms,
        (unsigned)r->soc, (unsigned)(r->pack_voltage_bms * 10u),
        (int)r->pack_current_ma, (int)r->isa_kw, (int)r->isa_ah,
        (int)r->motor_rpm, (int)r->motor_temp, (int)r->inverter_temp,
        (int)r->bms_temp_max, (int)r->bms_temp_min,
        (unsigned)r->cell_voltage_max, (unsigned)r->cell_voltage_min,
        (int)r->charger_temp);
}

// ── Trip handlers ─────────────────────────────────────────────────────────────

static void handle_trip_start(void)
{
    vehicle_state_t *state = vehicle_state_get();
    trip_start_state  = state->latest;
    trip_start_tick   = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    trip_peak_current = 0;
    trip_peak_rpm     = 0;
    trip_peak_motor_c = 0;
    trip_peak_inv_c   = 0;
    trip_peak_bms_c   = 0;

    // Open snap file and write header + TRIP_START marker
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv", current_session_id);
    snap_file = fopen(path, "w");
    if (snap_file) {
        fprintf(snap_file,
            "SNAP1,tick_ms,soc_pct,pack_v_bms_mv,pack_i_ma,"
            "isa_kw_w,isa_as,motor_rpm,motor_temp_c10,inv_temp_c10,"
            "bms_tmax_c10,bms_tmin_c10,cell_v_max_mv,cell_v_min_mv,charger_temp_c10\n");
        fprintf(snap_file, "TRIP_START,,,,,,,,,,,,,\n");
        fflush(snap_file);
    } else {
        ESP_LOGE(TAG, "Failed to open snap_%04lu.csv", current_session_id);
    }

    trip_active = true;
    ESP_LOGI(TAG, "Trip started — soc=%u%%", state->latest.soc);
}

static void handle_trip_end(void)
{
    if (!trip_active) {
        ESP_LOGW(TAG, "TRIP_END received with no active trip — ignored");
        if (rotate_sem) xSemaphoreGive(rotate_sem);
        return;
    }
    trip_active = false;

    vehicle_state_t *state  = vehicle_state_get();
    uint32_t end_tick       = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t duration_s     = (end_tick - trip_start_tick) / 1000;
    float    ah_used        = (state->latest.isa_ah  - trip_start_state.isa_ah)  / 3600.0f;
    float    kwh_used       = (state->latest.isa_kwh - trip_start_state.isa_kwh) / 1000.0f;
    uint8_t  soc_end        = state->latest.soc;
    float    peak_current_a = trip_peak_current / 1000.0f;

    int32_t  unix_offset = state->latest.unix_offset;
    uint32_t start_unix  = (unix_offset != 0)
        ? (uint32_t)((int32_t)(trip_start_tick / 1000) + unix_offset)
        : 0;

    // Append TRIP_END summary row to the snap file that was opened at TRIP_START
    // and has been accumulating 1Hz SNAP1 rows throughout the trip.
    if (snap_file) {
        fprintf(snap_file, "TRIP_END,%lu,%.2f,%.3f,%u,%u,%.1f,%lu,%d,%d,%d,%d,,\n",
            (unsigned long)duration_s, ah_used, kwh_used,
            trip_start_state.soc, soc_end, peak_current_a,
            (unsigned long)start_unix,
            (int)trip_peak_rpm, (int)trip_peak_motor_c,
            (int)trip_peak_inv_c, (int)trip_peak_bms_c);
        fclose(snap_file);
        snap_file = NULL;
        ESP_LOGI(TAG, "Trip written: snap_%04lu.csv — %lus, %.2fAh, %.3fkWh, "
                      "SoC %u%%→%u%%, peak %.1fA",
                 current_session_id, (unsigned long)duration_s,
                 ah_used, kwh_used, trip_start_state.soc, soc_end, peak_current_a);
    } else {
        ESP_LOGW(TAG, "TRIP_END: no open snap file for session %lu", current_session_id);
    }

    // Advance session so the just-written file appears in LIST on the next LIST command.
    current_session_id++;
    save_session_id(current_session_id);

    if (rotate_sem) xSemaphoreGive(rotate_sem);
}

// ── Public task ───────────────────────────────────────────────────────────────

void sd_logger_task(void *pvParameters)
{
    QueueHandle_t log_queue = (QueueHandle_t)pvParameters;

    rotate_sem = xSemaphoreCreateBinary();

    while (!sd_init()) {
        ESP_LOGW(TAG, "SD init failed — retrying in 5s");
        vTaskDelay(pdMS_TO_TICKS(5000));
    }

    current_session_id = load_session_id();
    ESP_LOGI(TAG, "Ready — next session will be snap_%04lu.csv", current_session_id);

    log_msg_t msg;
    while (1) {
        // 1-second timeout used for peak sampling during active trips
        bool got_msg = (xQueueReceive(log_queue, &msg, pdMS_TO_TICKS(1000)) == pdTRUE);

        if (got_msg) {
            switch (msg.type) {
                case LOG_MSG_TRIP_START: handle_trip_start(); break;
                case LOG_MSG_TRIP_END:   handle_trip_end();   break;
            }
        } else if (trip_active) {
            trip_tick();
        }
    }
}

bool sd_logger_trip_active(void) {
    return trip_active;
}

bool sd_logger_wait_rotate(uint32_t timeout_ms) {
    if (!rotate_sem) return false;
    return xSemaphoreTake(rotate_sem, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}
