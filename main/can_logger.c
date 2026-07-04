/*
 * can_logger.c — Unfiltered raw CAN frame logger
 * SomersetEV Tractor Telematics
 *
 * Architecture:
 *   - can_rx_task (Core 1) calls can_logger_submit() for every received frame.
 *     When logging is active the frame is copied into s_frame_queue with a
 *     non-blocking send (dropped on overflow — acceptable for a diagnostic log).
 *   - can_logger_task (Core 0) owns the file handle and does all SD I/O, so no
 *     blocking fwrite ever runs on the high-priority CAN task. This mirrors how
 *     sd_logger isolates SD I/O from the rest of the system.
 *
 * Start/stop are requested over BLE (CANLOG_START / CANLOG_STOP). The file is
 * opened lazily inside the task on start — by the time a phone issues a command
 * the SD card mounted by sd_logger is long since ready, so there is no boot-time
 * ordering dependency.
 *
 * The log is never offered via LIST/GET and never deleted by firmware. The SD
 * card is the master archive; retrieval is by pulling the card.
 */

#include "can_logger.h"
#include "esp_log.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>

static const char *TAG = "CANLOG";

#define MOUNT_POINT       "/sdcard"
#define NVS_NAMESPACE     "telematics"
#define NVS_KEY_CANLOG_ID "canlog_id"

// Frames can burst: ISA shunt alone is ~5 IDs at up to 100Hz. A 256-deep queue
// gives ~0.5s of headroom at worst-case ~500 fps on top of buffered stdio.
#define FRAME_QUEUE_DEPTH 256

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} raw_frame_t;

static QueueHandle_t     s_frame_queue = NULL;
static volatile bool     s_logging     = false;  // fast gate read by can_rx_task
static volatile bool     s_want_start  = false;
static volatile bool     s_want_stop   = false;
static uint32_t          s_canlog_id   = 0;
static volatile uint32_t s_drops       = 0;      // frames dropped on queue-full

// ── NVS helpers ───────────────────────────────────────────────────────────────

static uint32_t load_canlog_id(void)
{
    nvs_handle_t nvs;
    uint32_t id = 1;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, NVS_KEY_CANLOG_ID, &id);
        if (id == 0) id = 1;
        nvs_close(nvs);
    }
    return id;
}

static void save_canlog_id(uint32_t id)
{
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_set_u32(nvs, NVS_KEY_CANLOG_ID, id);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
}

// ── Public API (called from other tasks) ─────────────────────────────────────

void can_logger_start(void)
{
    s_want_stop  = false;
    s_want_start = true;
}

void can_logger_stop(void)
{
    s_logging    = false;   // stop can_rx_task feeding the queue immediately
    s_want_start = false;
    s_want_stop  = true;
}

bool can_logger_active(void)
{
    return s_logging;
}

void can_logger_submit(uint32_t id, uint8_t dlc, const uint8_t *data)
{
    if (!s_logging || s_frame_queue == NULL) return;

    raw_frame_t f = { .id = id, .dlc = (uint8_t)(dlc < 8 ? dlc : 8) };
    for (int i = 0; i < f.dlc; i++) f.data[i] = data[i];

    if (xQueueSend(s_frame_queue, &f, 0) != pdTRUE) {
        s_drops++;   // queue full — drop this frame (diagnostic log, tolerable)
    }
}

// ── Task ──────────────────────────────────────────────────────────────────────

static FILE *s_log_file = NULL;

static void open_log_file(void)
{
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/canlog_%04lu.csv", s_canlog_id);
    s_log_file = fopen(path, "w");
    if (s_log_file) {
        fprintf(s_log_file, "tick_ms,id,dlc,data\n");
        fflush(s_log_file);
        s_drops   = 0;
        s_logging = true;   // begin accepting frames only once the file is open
        ESP_LOGI(TAG, "CAN log started — canlog_%04lu.csv", s_canlog_id);
    } else {
        ESP_LOGE(TAG, "Failed to open canlog_%04lu.csv", s_canlog_id);
    }
}

static void close_log_file(void)
{
    if (!s_log_file) return;

    // Drain anything still queued so the tail of the capture isn't lost.
    raw_frame_t f;
    while (xQueueReceive(s_frame_queue, &f, 0) == pdTRUE) {
        uint32_t tick_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
        fprintf(s_log_file, "%lu,0x%03lX,%u,", (unsigned long)tick_ms,
                (unsigned long)f.id, (unsigned)f.dlc);
        for (int i = 0; i < f.dlc; i++) {
            fprintf(s_log_file, "%02X%s", f.data[i], i + 1 < f.dlc ? " " : "");
        }
        fprintf(s_log_file, "\n");
    }

    fflush(s_log_file);
    fclose(s_log_file);
    s_log_file = NULL;

    ESP_LOGI(TAG, "CAN log stopped — canlog_%04lu.csv (%lu frames dropped)",
             s_canlog_id, (unsigned long)s_drops);

    // Advance the counter so the next capture gets a fresh file.
    s_canlog_id++;
    save_canlog_id(s_canlog_id);
}

static void write_frame(const raw_frame_t *f)
{
    if (!s_log_file) return;
    uint32_t tick_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());
    fprintf(s_log_file, "%lu,0x%03lX,%u,", (unsigned long)tick_ms,
            (unsigned long)f->id, (unsigned)f->dlc);
    for (int i = 0; i < f->dlc; i++) {
        fprintf(s_log_file, "%02X%s", f->data[i], i + 1 < f->dlc ? " " : "");
    }
    fprintf(s_log_file, "\n");
}

void can_logger_task(void *pvParameters)
{
    (void)pvParameters;

    s_frame_queue = xQueueCreate(FRAME_QUEUE_DEPTH, sizeof(raw_frame_t));
    configASSERT(s_frame_queue);

    s_canlog_id = load_canlog_id();
    ESP_LOGI(TAG, "Ready — next capture will be canlog_%04lu.csv", s_canlog_id);

    raw_frame_t f;
    while (1) {
        // Honour a pending start request: open the file, which flips s_logging on.
        if (s_want_start) {
            s_want_start = false;
            if (!s_log_file) open_log_file();
        }

        // Honour a pending stop request: drain, flush and close the file.
        if (s_want_stop) {
            s_want_stop = false;
            close_log_file();
        }

        // Drain frames; on idle timeout flush so data survives a power loss.
        if (xQueueReceive(s_frame_queue, &f, pdMS_TO_TICKS(200)) == pdTRUE) {
            write_frame(&f);
        } else if (s_log_file) {
            fflush(s_log_file);
        }
    }
}
