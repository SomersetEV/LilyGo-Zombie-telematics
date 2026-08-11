#include "speed_bridge.h"
#include "can_handler.h"
#include "ble_nus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "SPDBR";

#define NVS_NAMESPACE   "telematics"
#define NVS_KEY_TX_EN   "spd_can_en"

// Bus-off backoff: 5s doubling to 60s
#define BUS_OFF_BACKOFF_MIN_MS  5000
#define BUS_OFF_BACKOFF_MAX_MS  60000

typedef enum {
    LATCH_NONE = 0,
    LATCH_ACK,        // no other node ACKing — wiring/transceiver/termination
    LATCH_BUS_OFF,    // repeated bus-off
    LATCH_COLLISION,  // another node already uses SPEED_CAN_ID
} latch_reason_t;

static gps_speed_t   s_gps;
static portMUX_TYPE  s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile bool           s_tx_enabled  = false;   // NVS gate, cached
static volatile latch_reason_t s_latch       = LATCH_NONE;
static uint8_t                 s_counter     = 0;
static uint32_t                s_tx_sent     = 0;
static int64_t                 s_backoff_until_us = 0;
static uint32_t                s_backoff_ms  = BUS_OFF_BACKOFF_MIN_MS;

// ── Init ─────────────────────────────────────────────────────────────────────

void speed_bridge_init(void)
{
    portENTER_CRITICAL(&s_mux);
    s_gps.speed_cmph   = SPEED_UNKNOWN;
    s_gps.heading_ddeg = SPEED_UNKNOWN;
    s_gps.fix          = 0;
    s_gps.rx_us        = 0;
    s_gps.rx_count     = 0;
    portEXIT_CRITICAL(&s_mux);

    uint32_t v = 0;
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK) {
        nvs_get_u32(h, NVS_KEY_TX_EN, &v);   // leaves v untouched if absent
        nvs_close(h);
    }
    s_tx_enabled = (v != 0);
    ESP_LOGI(TAG, "init, CAN TX %s", s_tx_enabled ? "ENABLED" : "disabled");
}

// ── Ingest ───────────────────────────────────────────────────────────────────

// "SPD <centi_mph> <fix> [<heading_ddeg>]"
// Runs in the NimBLE host task: integer parse and a small struct copy only.
// Deliberately not sscanf — this is called at the phone's GPS stream rate.
bool speed_bridge_try_consume(const char *text, uint16_t len)
{
    // Require the space so SPDCAN / SPDSTAT fall through to dispatch_command
    if (len < 5 || text[0] != 'S' || text[1] != 'P' || text[2] != 'D' || text[3] != ' ') {
        return false;
    }

    const char *p = text + 4;
    char *end;

    unsigned long spd = strtoul(p, &end, 10);
    if (end == p) return true;              // ours, but malformed — drop quietly
    p = end;

    unsigned long fix = strtoul(p, &end, 10);
    if (end == p) return true;
    p = end;

    unsigned long hdg = strtoul(p, &end, 10);
    if (end == p) hdg = SPEED_UNKNOWN;      // heading is optional

    if (spd > SPEED_UNKNOWN) spd = SPEED_UNKNOWN;
    if (hdg > 3599 && hdg != SPEED_UNKNOWN) hdg = SPEED_UNKNOWN;

    portENTER_CRITICAL(&s_mux);
    s_gps.speed_cmph   = (uint16_t)spd;
    s_gps.heading_ddeg = (uint16_t)hdg;
    s_gps.fix          = (fix != 0) ? 1 : 0;
    s_gps.rx_us        = esp_timer_get_time();
    s_gps.rx_count++;
    portEXIT_CRITICAL(&s_mux);

    return true;   // consumed — never reaches cmd_queue
}

void speed_bridge_invalidate(void)
{
    portENTER_CRITICAL(&s_mux);
    s_gps.speed_cmph   = SPEED_UNKNOWN;
    s_gps.heading_ddeg = SPEED_UNKNOWN;
    s_gps.fix          = 0;
    s_gps.rx_us        = 0;
    portEXIT_CRITICAL(&s_mux);
}

void speed_bridge_get(gps_speed_t *out)
{
    portENTER_CRITICAL(&s_mux);
    *out = s_gps;
    portEXIT_CRITICAL(&s_mux);
}

uint32_t speed_bridge_age_ms(const gps_speed_t *s)
{
    if (s->rx_us == 0) return UINT32_MAX;
    int64_t age_us = esp_timer_get_time() - s->rx_us;
    if (age_us < 0) return 0;
    return (uint32_t)(age_us / 1000);
}

// ── NVS gate ─────────────────────────────────────────────────────────────────

bool speed_bridge_tx_enabled(void) { return s_tx_enabled; }

bool speed_bridge_set_tx_enabled(bool enable)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return false;
    esp_err_t err = nvs_set_u32(h, NVS_KEY_TX_EN, enable ? 1 : 0);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) return false;

    // Takes effect on reboot: enable_listen_only is fixed at node creation.
    s_tx_enabled = enable;
    ESP_LOGI(TAG, "CAN TX gate set to %d (reboot required)", enable);
    return true;
}

void speed_bridge_note_collision(void)
{
    if (s_latch != LATCH_COLLISION) {
        s_latch = LATCH_COLLISION;
        ESP_LOGE(TAG, "ID collision: another node transmits 0x%03X — TX latched off",
                 (unsigned)SPEED_CAN_ID);
    }
}

// ── Status ───────────────────────────────────────────────────────────────────

static const char *latch_name(latch_reason_t r)
{
    switch (r) {
        case LATCH_ACK:       return "ack";
        case LATCH_BUS_OFF:   return "busoff";
        case LATCH_COLLISION: return "collision";
        default:              return "none";
    }
}

void speed_bridge_status_str(char *buf, size_t n)
{
    gps_speed_t s;
    speed_bridge_get(&s);
    uint32_t age = speed_bridge_age_ms(&s);
    bool valid = s.fix && (age <= SPEED_CAN_STALE_MS) && (s.speed_cmph != SPEED_UNKNOWN);

    snprintf(buf, n,
             "SPDSTAT spd=%u hdg=%u age=%lu valid=%d rx=%lu en=%d tx=%lu txok=%lu txerr=%lu latch=%s\n",
             (unsigned)s.speed_cmph, (unsigned)s.heading_ddeg,
             (unsigned long)(age == UINT32_MAX ? 0 : age), valid ? 1 : 0,
             (unsigned long)s.rx_count, s_tx_enabled ? 1 : 0,
             (unsigned long)s_tx_sent,
             (unsigned long)can_handler_tx_ok_count(),
             (unsigned long)can_handler_tx_fail_count(),
             latch_name(s_latch));
}

// ── Periodic CAN transmit ────────────────────────────────────────────────────

static void pack_speed_frame(twai_frame_t *fr, uint8_t *buf, const gps_speed_t *s)
{
    uint32_t age   = speed_bridge_age_ms(s);
    bool     stale = (age > SPEED_CAN_STALE_MS);
    bool     valid = s->fix && !stale && (s->speed_cmph != SPEED_UNKNOWN);

    // Invalid publishes 0xFFFF, never 0 — a consumer must not read "stopped".
    uint16_t spd = valid ? s->speed_cmph : SPEED_UNKNOWN;
    uint16_t hdg = valid ? s->heading_ddeg : SPEED_UNKNOWN;

    uint8_t st = 0;
    if (valid)                  st |= SPEED_ST_VALID;
    if (s->fix)                 st |= SPEED_ST_FIX;
    if (ble_nus_is_connected()) st |= SPEED_ST_BLE;
    if (stale)                  st |= SPEED_ST_STALE;

    uint32_t age10 = (age == UINT32_MAX) ? 0xFF : (age / 10);

    buf[0] = (uint8_t)(spd & 0xFF);
    buf[1] = (uint8_t)(spd >> 8);
    buf[2] = (uint8_t)(hdg & 0xFF);
    buf[3] = (uint8_t)(hdg >> 8);
    buf[4] = st;
    buf[5] = (age10 > 0xFF) ? 0xFF : (uint8_t)age10;
    buf[6] = s_counter++;

    uint8_t chk = 0;
    for (int i = 0; i < 7; i++) chk ^= buf[i];
    buf[7] = chk;

    memset(&fr->header, 0, sizeof(fr->header));   // ide/rtr/fdf clear = standard data frame
    fr->header.id  = SPEED_CAN_ID;
    fr->header.dlc = SPEED_CAN_DLC;
    fr->buffer     = buf;
    fr->buffer_len = SPEED_CAN_DLC;
}

// All interlocks that must hold before a single frame goes onto a vehicle bus.
static bool tx_armed(void)
{
    if (!s_tx_enabled)            return false;
    if (s_latch != LATCH_NONE)    return false;
    if (!can_handler_tx_capable()) return false;

    // Proves the bitrate is right and other nodes exist. Without this, enabling
    // TX on a mis-wired or single-node bus means no ACK ever arrives and the
    // transmit error counter climbs straight to bus-off.
    if (can_handler_frame_count() < SPEED_TX_MIN_RX_FRAMES) return false;

    if (esp_timer_get_time() < s_backoff_until_us) return false;

    return true;
}

static void check_latches(void)
{
    if (s_latch != LATCH_NONE) return;

    if (can_handler_ack_fail_streak() >= SPEED_TX_MAX_ACK_FAILS) {
        s_latch = LATCH_ACK;
        ESP_LOGE(TAG, "no ACK after %d consecutive transmits — TX latched off. "
                      "Check CAN_SE transceiver enable, 120 ohm termination at BOTH ends, "
                      "and that another node is powered.", SPEED_TX_MAX_ACK_FAILS);
        return;
    }

    if (can_handler_bus_off_count() >= SPEED_TX_MAX_BUS_OFF) {
        s_latch = LATCH_BUS_OFF;
        ESP_LOGE(TAG, "%d bus-off events — TX latched off until reboot", SPEED_TX_MAX_BUS_OFF);
    }
}

void speed_tx_task(void *pvParameters)
{
    (void)pvParameters;

    if (!s_tx_enabled) {
        ESP_LOGI(TAG, "CAN TX disabled (SPDCAN 0) — tx task exiting");
        vTaskDelete(NULL);
    }

    // can_rx_task creates the node inside itself; wait for it.
    while (!can_handler_tx_capable()) {
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ESP_LOGI(TAG, "TX task running, 0x%03X every %dms",
             (unsigned)SPEED_CAN_ID, SPEED_CAN_PERIOD_MS);

    // STATIC: the driver queues a pointer to the frame, not a copy
    // (tx_mount_queue holds twai_frame_t*), so it must outlive the call.
    // Ping-pong so a frame still pending in the driver is never overwritten.
    static twai_frame_t s_frames[2];
    static uint8_t      s_bufs[2][SPEED_CAN_DLC];
    int idx = 0;

    uint32_t   last_bus_off = 0;
    TickType_t last_wake    = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SPEED_CAN_PERIOD_MS));

        can_handler_service();      // task-context bus-off recovery

        uint32_t bo = can_handler_bus_off_count();
        if (bo != last_bus_off) {
            last_bus_off = bo;
            s_backoff_until_us = esp_timer_get_time() + (int64_t)s_backoff_ms * 1000;
            ESP_LOGW(TAG, "bus-off #%lu, backing off %lums",
                     (unsigned long)bo, (unsigned long)s_backoff_ms);
            s_backoff_ms *= 2;
            if (s_backoff_ms > BUS_OFF_BACKOFF_MAX_MS) s_backoff_ms = BUS_OFF_BACKOFF_MAX_MS;
        }

        check_latches();
        if (!tx_armed()) continue;

        gps_speed_t s;
        speed_bridge_get(&s);

        idx ^= 1;
        pack_speed_frame(&s_frames[idx], s_bufs[idx], &s);

        // timeout 0 — a periodic task must never block on a full TX queue.
        esp_err_t err = can_handler_transmit(&s_frames[idx], 0);
        if (err == ESP_OK) {
            s_tx_sent++;
        } else {
            // Rate-limited: at 10Hz an unconditional log floods the console.
            static int64_t last_log_us = 0;
            int64_t now = esp_timer_get_time();
            if (now - last_log_us > 5000000) {
                last_log_us = now;
                ESP_LOGW(TAG, "transmit failed: %s", esp_err_to_name(err));
            }
        }
    }
}
