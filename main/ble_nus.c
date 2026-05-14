/*
 * ble_nus.c — Full NimBLE NUS implementation
 * SomersetEV Tractor Telematics
 *
 * Architecture:
 *   - nimble_port_freertos_init() creates the NimBLE host task internally
 *   - ble_nus_task() does setup, then becomes the command processor loop
 *   - GATT RX callback posts commands to cmd_queue (non-blocking, runs in NimBLE context)
 *   - Command processor handles LIST/GET/DONE/TIME, does SD I/O, sends notifications
 *
 * Android notes:
 *   - Android auto-requests MTU 517 on connect — we honour whatever is negotiated
 *   - Chunk size is set dynamically from negotiated MTU
 *   - A 20ms delay between notification chunks prevents Android BLE stack overflow
 *   - Android requires CCCD subscription before notifications work — standard behaviour,
 *     handled automatically by most Android BLE libraries
 */

#include "ble_nus.h"
#include "vehicle_state.h"
#include "sd_logger.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <sys/stat.h>

static const char *TAG        = "BLE";
#define DEVICE_NAME             "SomersetEV-Tractor"
#define MOUNT_POINT             "/sdcard"
#define NVS_NAMESPACE           "telematics"
#define NVS_KEY_LAST_SYNCED     "last_synced"
#define CMD_QUEUE_DEPTH         8
#define CMD_MAX_LEN             32

// ── BLE chunk sizing ─────────────────────────────────────────────────────────
// Default 20 bytes (MTU 23) until Android negotiates MTU up on connect
#define DEFAULT_CHUNK_SIZE      20
#define MAX_CHUNK_SIZE          509     // MTU 512 - 3 bytes ATT overhead

// ── NUS Service and Characteristic UUIDs ────────────────────────────────────
// 128-bit UUIDs stored little-endian as required by NimBLE
// Service: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t nus_svc_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e
);
// RX: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E  (phone writes commands here)
static const ble_uuid128_t nus_rx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e
);
// TX: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E  (ESP32 notifies phone)
static const ble_uuid128_t nus_tx_uuid = BLE_UUID128_INIT(
    0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0,
    0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e
);

static int gap_event_handler(struct ble_gap_event *event, void *arg);

// ── State ────────────────────────────────────────────────────────────────────
static uint16_t      nus_tx_handle  = 0;
static uint16_t      conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t      negotiated_mtu = DEFAULT_CHUNK_SIZE + 3;
static QueueHandle_t cmd_queue      = NULL;
static QueueHandle_t log_queue      = NULL;  // shared with CAN and SD tasks

typedef struct {
    char     text[CMD_MAX_LEN];
    uint16_t len;
} ble_cmd_t;

// ── Notification helper ──────────────────────────────────────────────────────

static int nus_notify(const void *data, uint16_t len)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return -1;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
        ESP_LOGE(TAG, "mbuf alloc failed");
        return -1;
    }
    int rc = ble_gatts_notify_custom(conn_handle, nus_tx_handle, om);
    if (rc != 0) {
        ESP_LOGW(TAG, "notify failed: %d", rc);
    }
    return rc;
}

static int nus_notify_str(const char *str)
{
    return nus_notify(str, (uint16_t)strlen(str));
}

// ── Command handlers ─────────────────────────────────────────────────────────

static void handle_list_command(void)
{
    /*
     * Enumerate completed session files on SD card.
     * The currently-open session (still being written) is excluded.
     * Its ID = (NVS "session_id" - 1) since sd_logger increments on boot.
     *
     * Response: "LIST 0001,3600;0002,1800;\n"
     * Record count is approximate (file_size / 100 bytes per CSV row).
     */
    uint32_t current_session = 0;
    uint32_t last_synced     = 0;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, "session_id",       &current_session);
        nvs_get_u32(nvs, NVS_KEY_LAST_SYNCED, &last_synced);
        nvs_close(nvs);
    }
    uint32_t active_session = current_session > 0 ? current_session - 1 : 0;

    DIR *dir = opendir(MOUNT_POINT);
    if (!dir) {
        nus_notify_str("ERR no_sd\n");
        return;
    }

    char response[512] = "LIST ";
    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        uint32_t sid;
        if (sscanf(entry->d_name, "snap_%04lu.csv", &sid) != 1) continue;
        if (sid == active_session) continue;    // skip currently-open session
        if (sid <= last_synced)    continue;    // skip already-synced sessions

        char path[280];
        snprintf(path, sizeof(path), MOUNT_POINT "/%s", entry->d_name);
        struct stat st;
        if (stat(path, &st) != 0 || st.st_size == 0) continue;  // skip empty/unreadable files
        uint32_t records = (uint32_t)(st.st_size / 100);

        char entry_str[32];
        snprintf(entry_str, sizeof(entry_str), "%04lu,%lu;", sid, records);
        strncat(response, entry_str, sizeof(response) - strlen(response) - 1);
    }
    closedir(dir);
    strncat(response, "\n", sizeof(response) - strlen(response) - 1);
    nus_notify_str(response);
    ESP_LOGI(TAG, "LIST sent: %s", response);
}

static void handle_get_command(uint32_t session_id)
{
    /*
     * Stream a completed session CSV to the phone.
     *
     * Protocol:
     *   1. "DATA <id> <filesize>\n"   — header, phone allocates buffer
     *   2. Raw CSV bytes in chunks    — chunk size = negotiated_mtu - 3
     *   3. "END <id>\n"               — transfer complete
     *
     * 20ms inter-chunk delay prevents Android BLE stack dropping packets.
     * At MTU 512 (509 byte chunks) this gives ~25KB/s — fine for our data rates.
     * A 1-hour session at 1Hz / ~100 bytes/row = ~360KB ≈ 15 seconds transfer time.
     */
    char path[64];
    snprintf(path, sizeof(path), MOUNT_POINT "/snap_%04lu.csv", session_id);

    struct stat st;
    if (stat(path, &st) != 0) {
        char err[32];
        snprintf(err, sizeof(err), "ERR not_found %04lu\n", session_id);
        nus_notify_str(err);
        return;
    }

    FILE *f = fopen(path, "r");
    if (!f) {
        nus_notify_str("ERR open_failed\n");
        return;
    }

    // Send header — give Android 50ms to process before streaming begins
    char header[64];
    snprintf(header, sizeof(header), "DATA %04lu %ld\n", session_id, (long)st.st_size);
    nus_notify_str(header);
    vTaskDelay(pdMS_TO_TICKS(50));

    uint16_t chunk_size = (negotiated_mtu > 3) ? (negotiated_mtu - 3) : DEFAULT_CHUNK_SIZE;
    if (chunk_size > MAX_CHUNK_SIZE) chunk_size = MAX_CHUNK_SIZE;

    uint8_t  buf[MAX_CHUNK_SIZE];
    size_t   bytes_read;
    uint32_t total_sent = 0;

    while ((bytes_read = fread(buf, 1, chunk_size, f)) > 0) {
        if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            ESP_LOGW(TAG, "Disconnected mid-transfer, aborting GET %04lu", session_id);
            fclose(f);
            return;     // Phone discards incomplete session and re-requests on reconnect
        }
        nus_notify(buf, (uint16_t)bytes_read);
        total_sent += bytes_read;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    fclose(f);

    char end_marker[32];
    snprintf(end_marker, sizeof(end_marker), "END %04lu\n", session_id);
    nus_notify_str(end_marker);

    ESP_LOGI(TAG, "GET %04lu complete — %lu bytes sent", session_id, total_sent);
}

static void handle_done_command(uint32_t session_id)
{
    /*
     * Phone confirms successful receipt and local storage of session.
     * Update NVS last_synced. Never delete from SD — SD is the master archive.
     */
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        uint32_t last = 0;
        nvs_get_u32(nvs, NVS_KEY_LAST_SYNCED, &last);
        if (session_id > last) {
            nvs_set_u32(nvs, NVS_KEY_LAST_SYNCED, session_id);
            nvs_commit(nvs);
        }
        nvs_close(nvs);
    }
    nus_notify_str("OK\n");
    ESP_LOGI(TAG, "DONE %04lu acknowledged", session_id);
}

static void handle_time_command(uint32_t unix_epoch)
{
    /*
     * Phone sends current Unix timestamp on connect.
     * We store (unix_epoch - tick_ms/1000) as a soft RTC offset.
     * All subsequent log records can be timestamped: unix = offset + tick_ms/1000
     * Persists only in RAM — resets on next power cycle until phone reconnects.
     */
    vehicle_state_t *state = vehicle_state_get();
    state->latest.unix_offset = (int32_t)unix_epoch
                              - (int32_t)(state->latest.tick_ms / 1000);
    nus_notify_str("OK\n");
    ESP_LOGI(TAG, "TIME synced: unix=%lu offset=%ld",
             unix_epoch, (long)state->latest.unix_offset);
}

static void handle_trip_marker(log_msg_type_t type)
{
    /*
     * Post a marker message directly onto the log queue so the SD logger
     * task writes it to the CSV in the correct sequence relative to data records.
     * Respond OK immediately — the actual file write is async but near-instant.
     */
    if (!log_queue) {
        nus_notify_str("ERR no_logger\n");
        return;
    }
    log_msg_t msg = { .type = type };
    if (xQueueSend(log_queue, &msg, pdMS_TO_TICKS(200)) != pdTRUE) {
        ESP_LOGW(TAG, "Log queue full — trip marker dropped");
        nus_notify_str("ERR queue_full\n");
        return;
    }
    // Wait for sd_logger to finish closing the file and rotating the session
    // before responding OK. The phone then immediately syncs the new session.
    if (type == LOG_MSG_TRIP_END) {
        sd_logger_wait_rotate(2000);
    }
    nus_notify_str("OK\n");
    ESP_LOGI(TAG, "Trip marker queued: %s",
             type == LOG_MSG_TRIP_START ? "TRIP_START" : "TRIP_END");
}

static void handle_status_command(void)
{
    char buf[32];
    snprintf(buf, sizeof(buf), "STATUS trip=%d\n",
             sd_logger_trip_active() ? 1 : 0);
    nus_notify_str(buf);
}

static void dispatch_command(const char *cmd, uint16_t len)
{
    uint32_t arg;
    if      (strncmp(cmd, "LIST", 4) == 0)           { handle_list_command();                    }
    else if (sscanf(cmd, "GET %lu",  &arg) == 1)     { handle_get_command(arg);                  }
    else if (sscanf(cmd, "DONE %lu", &arg) == 1)     { handle_done_command(arg);                 }
    else if (sscanf(cmd, "TIME %lu", &arg) == 1)     { handle_time_command(arg);                 }
    else if (strncmp(cmd, "TRIP_START", 10) == 0)    { handle_trip_marker(LOG_MSG_TRIP_START);   }
    else if (strncmp(cmd, "TRIP_END",   8) == 0)     { handle_trip_marker(LOG_MSG_TRIP_END);     }
    else if (strncmp(cmd, "STATUS",     6) == 0)     { handle_status_command();                  }
    else {
        ESP_LOGW(TAG, "Unknown cmd: %.*s", len, cmd);
        nus_notify_str("ERR unknown_cmd\n");
    }
}

// ── GATT RX callback ─────────────────────────────────────────────────────────

static int nus_rx_access_cb(uint16_t conn_hdl, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_hdl; (void)attr_handle; (void)arg;
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    // This runs in NimBLE host task context — just copy data and post to queue
    ble_cmd_t cmd = {0};
    uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
    if (om_len >= CMD_MAX_LEN) om_len = CMD_MAX_LEN - 1;
    ble_hs_mbuf_to_flat(ctxt->om, cmd.text, om_len, &cmd.len);
    cmd.text[cmd.len] = '\0';

    // Strip trailing CR/LF
    while (cmd.len > 0 &&
           (cmd.text[cmd.len-1] == '\n' || cmd.text[cmd.len-1] == '\r')) {
        cmd.text[--cmd.len] = '\0';
    }

    if (xQueueSend(cmd_queue, &cmd, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Command queue full — dropped: %s", cmd.text);
    }
    return 0;
}

static int nus_tx_access_cb(uint16_t conn_hdl, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_hdl; (void)attr_handle; (void)ctxt; (void)arg;
    return 0;
}

// ── GATT service table ───────────────────────────────────────────────────────

static const struct ble_gatt_svc_def nus_gatt_svcs[] = {
    {
        .type            = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid            = &nus_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = &nus_rx_uuid.u,
                .access_cb = nus_rx_access_cb,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid       = &nus_tx_uuid.u,
                .access_cb  = nus_tx_access_cb,
                .val_handle = &nus_tx_handle,
                .flags      = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        }
    },
    { 0 }
};

// ── GAP event handler ────────────────────────────────────────────────────────

static void restart_advertising(void)
{
    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_handler, NULL);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle    = event->connect.conn_handle;
            negotiated_mtu = DEFAULT_CHUNK_SIZE + 3;
            ESP_LOGI(TAG, "Phone connected — handle=%d", conn_handle);
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            restart_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGI(TAG, "Disconnected — reason=%d", event->disconnect.reason);
        restart_advertising();
        break;

    case BLE_GAP_EVENT_MTU:
        /*
         * Android requests MTU 517 on connect. ESP32 NimBLE default max is 512.
         * Actual agreed value ends up here. Usable payload = mtu - 3.
         */
        negotiated_mtu = event->mtu.value;
        ESP_LOGI(TAG, "MTU=%d, chunk size=%d bytes", negotiated_mtu, negotiated_mtu - 3);
        break;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "TX notifications %s",
                 event->subscribe.cur_notify ? "enabled" : "disabled");
        break;

    default:
        break;
    }
    return 0;
}

// ── NimBLE host task ─────────────────────────────────────────────────────────

static void ble_host_task(void *param)
{
    (void)param;
    nimble_port_run();          // blocks until nimble_port_stop()
    nimble_port_freertos_deinit();
}

static void on_ble_sync(void)
{
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "No BT address");
        return;
    }

    // Advertising data: flags + name only (23 bytes — fits in 31-byte limit)
    // UUID128 goes in the scan response so it doesn't push us over the limit
    struct ble_hs_adv_fields adv_fields = {0};
    adv_fields.flags            = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.name             = (uint8_t *)DEVICE_NAME;
    adv_fields.name_len         = strlen(DEVICE_NAME);
    adv_fields.name_is_complete = 1;
    int rc2 = ble_gap_adv_set_fields(&adv_fields);
    if (rc2 != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed: %d", rc2);
        return;
    }

    // Scan response: NUS UUID128 (visible on active scan)
    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.uuids128             = &nus_svc_uuid;
    rsp_fields.num_uuids128         = 1;
    rsp_fields.uuids128_is_complete = 1;
    rc2 = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc2 != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields failed: %d", rc2);
        return;
    }

    struct ble_gap_adv_params adv_params = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &adv_params, gap_event_handler, NULL);

    ESP_LOGI(TAG, "Advertising as \"%s\"", DEVICE_NAME);
}

static void on_ble_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset — reason=%d", reason);
}

// ── Public entry point ───────────────────────────────────────────────────────

void ble_nus_task(void *pvParameters)
{
    log_queue = (QueueHandle_t)pvParameters;   // shared log queue from main
    ESP_LOGI(TAG, "BLE NUS task starting");

    cmd_queue = xQueueCreate(CMD_QUEUE_DEPTH, sizeof(ble_cmd_t));
    configASSERT(cmd_queue);

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);

    int rc = ble_gatts_count_cfg(nus_gatt_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(nus_gatt_svcs);
    assert(rc == 0);

    ble_hs_cfg.sync_cb  = on_ble_sync;
    ble_hs_cfg.reset_cb = on_ble_reset;

    // NimBLE host runs in its own task — this task becomes the command processor
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "Command processor running");
    ble_cmd_t cmd;
    while (1) {
        if (xQueueReceive(cmd_queue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            ESP_LOGI(TAG, "RX: [%s]", cmd.text);
            dispatch_command(cmd.text, cmd.len);
        } else if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            // Send periodic CAN activity heartbeat so the phone can show bus health
            char buf[32];
            snprintf(buf, sizeof(buf), "CAN %lu\n", sd_logger_can_frame_count());
            nus_notify_str(buf);
        }
    }
}
