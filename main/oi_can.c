// CANopen/SDO client for the Zombieverter inverter — ported from
// esp32-web-interface-lilygo_tcan/src/oi_can.cpp (Arduino/C++, ArduinoJson,
// legacy twai_transmit/twai_receive) to plain C against the IDF TWAI
// onchip-node API already used by can_handler.c.
//
// Differences from the source project, and why:
//  - twai_receive()/twai_transmit() (legacy driver) don't exist in this IDF
//    version; only the node API (twai_node_transmit / RX-done callback) is
//    available. twai_node_receive_from_isr() can only be called from the ISR
//    callback, so the RX callback here pushes frames onto a queue and every
//    "twai_receive(&f, pdMS_TO_TICKS(N))" call in the original becomes a
//    blocking xQueueReceive with the same timeout — same synchronous,
//    single-outstanding-request style as the source, just queue-mediated.
//  - ArduinoJson -> oi_json.h (hand-rolled minimal reader, see that file for
//    why: the schema is fixed-shape and small, not worth a managed component).
//  - SPIFFS.h -> POSIX file I/O via the spiffs VFS mount (esp_vfs_spiffs).
//  - String/WiFiClient -> plain buffers and a raw fd (from the httpd request).
#include "oi_can.h"
#include "oi_json.h"
#include "can_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_twai_onchip.h"
#include "esp_twai.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/socket.h>
#include <stdarg.h>

static const char *TAG = "OICAN";

// ── buffered response writer ─────────────────────────────────────────────────
// Response bodies (JSON, CAN mapping, streamed samples) are emitted as httpd
// chunks, buffered here so the many small fprintf-style writes in the ported
// code don't become one chunk each.
//
// An earlier version of this port wrote straight to the socket fd from
// httpd_req_to_sockfd(). That skipped the HTTP status line and headers
// entirely — httpd_resp_set_type() only records the type for a later
// httpd_resp_send*() call — so every successful response went out as a bare
// body the browser threw away, while only the error path (which does go
// through httpd_resp_send_err) was ever well formed.
typedef struct {
    httpd_req_t *req;
    char         buf[512];
    size_t       len;
} sock_writer_t;

static void sw_init(sock_writer_t *w, httpd_req_t *req) { w->req = req; w->len = 0; }

static void sw_flush(sock_writer_t *w)
{
    if (w->len == 0) return;
    httpd_resp_send_chunk(w->req, w->buf, w->len);
    w->len = 0;
}

// Flush and close the chunked response. Every body-producing path must end here.
static void sw_finish(sock_writer_t *w)
{
    sw_flush(w);
    httpd_resp_send_chunk(w->req, NULL, 0);
}

static void sw_write(sock_writer_t *w, const char *data, size_t n)
{
    while (n > 0) {
        size_t space = sizeof(w->buf) - w->len;
        size_t chunk = n < space ? n : space;
        memcpy(w->buf + w->len, data, chunk);
        w->len += chunk;
        data += chunk;
        n -= chunk;
        if (w->len == sizeof(w->buf)) sw_flush(w);
    }
}

static void sw_putc(sock_writer_t *w, char c) { sw_write(w, &c, 1); }

static void sw_printf(sock_writer_t *w, const char *fmt, ...)
{
    char tmp[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) sw_write(w, tmp, (size_t)(n < (int)sizeof(tmp) ? n : (int)sizeof(tmp) - 1));
}

#define SDO_REQUEST_DOWNLOAD  (1 << 5)
#define SDO_REQUEST_UPLOAD    (2 << 5)
#define SDO_REQUEST_SEGMENT   (3 << 5)
#define SDO_RESPONSE_UPLOAD   (2 << 5)
#define SDO_RESPONSE_DOWNLOAD (3 << 5)
#define SDO_EXPEDITED         (1 << 1)
#define SDO_SIZE_SPECIFIED    (1)
#define SDO_WRITE              (SDO_REQUEST_DOWNLOAD | SDO_EXPEDITED | SDO_SIZE_SPECIFIED)
#define SDO_READ                SDO_REQUEST_UPLOAD
#define SDO_ABORT               0x80

#define SDO_INDEX_PARAMS      0x2000
#define SDO_INDEX_PARAM_UID   0x2100
#define SDO_INDEX_MAP_TX      0x3000
#define SDO_INDEX_MAP_RX      0x3001
#define SDO_INDEX_MAP_RD      0x3100
#define SDO_INDEX_SERIAL      0x5000
#define SDO_INDEX_STRINGS     0x5001
#define SDO_INDEX_COMMANDS    0x5002
#define SDO_CMD_SAVE          0
#define SDO_CMD_RESET         2

#define PAGE_SIZE_BYTES  1024
#define JSON_MOUNT_POINT "/spiffs"

// The schema is downloaded here and renamed onto its real name only once the
// final segment arrives. Writing straight to the real name meant any
// interruption — a reset, a /nodeid re-init, an SDO abort mid-transfer — left a
// truncated file that every later boot accepted as "already downloaded" and
// then failed to parse, permanently.
#define JSON_TMP_PATH    JSON_MOUNT_POINT "/schema.tmp"
// Smallest plausible schema. Anything shorter is a leftover stub, not a schema.
#define JSON_MIN_BYTES   64

typedef enum { ST_IDLE, ST_ERROR, ST_OBTAINSERIAL, ST_OBTAIN_JSON } sdo_state_t;
typedef enum { UPD_IDLE, SEND_MAGIC, SEND_SIZE, SEND_PAGE, CHECK_CRC, REQUEST_JSON } upd_state_t;

static uint8_t            s_node_id;
static oi_baud_t          s_baud_rate;
static sdo_state_t        s_state = ST_IDLE;
static upd_state_t        s_upd_state = UPD_IDLE;
static uint32_t           s_serial[4];
static char                s_json_path[40];
static twai_node_handle_t s_node = NULL;
static QueueHandle_t      s_rx_queue = NULL;
static int                s_retries = 0;

// Serialises SDO transactions. The source project ran every request from one
// Arduino loop; the port has two concurrent callers — the oi_can_poll task and
// the httpd task — both draining s_rx_queue, so the poll task silently ate the
// replies the HTTP handlers were blocked on (in ST_IDLE handle_sdo_response
// falls through its default case and drops the frame). Back to one outstanding
// request at a time.
static SemaphoreHandle_t  s_sdo_lock = NULL;
// Tick of the last serial-number request, for the handshake retry in oi_can_loop().
static TickType_t         s_serial_req_tick = 0;
static int                s_serial_attempts = 0;

// ── low-level TX/RX ──────────────────────────────────────────────────────────

typedef struct { uint32_t id; uint8_t dlc; uint8_t data[8]; } oi_frame_t;

#define SDO_RESPONSE_ID   (0x580u | s_node_id)
#define BOOTLOADER_ID     0x7de

// Did the last frame we put on the bus get acknowledged by anyone? On CAN a
// frame is only ACKed if at least one other node received it cleanly, so this
// separates "the inverter is there but not answering us" from "we are alone on
// the wire". Counters, not a flag, so a stalled handshake shows its history.
static volatile uint32_t s_tx_ok_count = 0;
static volatile uint32_t s_tx_fail_count = 0;

// Every frame the controller accepts, including ones not addressed to us. This
// is the difference between "the bus is alive but the inverter won't answer"
// and "we are not electrically connected to anything". Counted before the
// SDO-relevance check below, so it reflects raw bus activity.
static volatile uint32_t s_rx_any_count = 0;
static volatile uint32_t s_rx_last_id = 0;

static bool IRAM_ATTR oi_tx_done_cb(twai_node_handle_t handle,
                                     const twai_tx_done_event_data_t *edata,
                                     void *user_ctx)
{
    (void)handle; (void)user_ctx;
    if (edata && edata->is_tx_success) s_tx_ok_count++;
    else                               s_tx_fail_count++;
    return false;
}

static bool IRAM_ATTR oi_rx_done_cb(twai_node_handle_t handle,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    (void)edata; (void)user_ctx;
    uint8_t buf[8] = {0};
    twai_frame_t rx = { .buffer = buf, .buffer_len = sizeof(buf) };

    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) return false;

    s_rx_any_count++;
    s_rx_last_id = rx.header.id;

    // Only SDO replies from our node and bootloader frames belong in this queue.
    // Every twai_recv() below is a synchronous "next frame is my reply" read, so
    // without this the inverter's normal periodic traffic is what those reads
    // return. Backs up the hardware acceptance filter set in init.
    if (rx.header.id != SDO_RESPONSE_ID && rx.header.id != BOOTLOADER_ID) return false;

    oi_frame_t f = {
        .id  = rx.header.id,
        .dlc = (uint8_t)(rx.header.dlc < 8 ? rx.header.dlc : 8),
    };
    memcpy(f.data, buf, sizeof(f.data));

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &f, &woken);
    return woken == pdTRUE;
}

static void twai_send(uint32_t id, const uint8_t *data, uint8_t dlc)
{
    twai_frame_t frame = {0};
    frame.header.id  = id;
    frame.header.dlc = dlc;
    frame.buffer      = (uint8_t *)data;
    frame.buffer_len  = dlc;
    // The result was previously discarded, which hid the difference between
    // "the inverter isn't answering" and "we never got the frame onto the bus".
    esp_err_t err = twai_node_transmit(s_node, &frame, 10);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "TX of 0x%lx failed: %s", (unsigned long)id, esp_err_to_name(err));
    }
}

// One-line summary of what the CAN controller itself thinks is going on. An
// unanswered request looks completely different depending on the cause:
//   error-active, tx_err 0    -> frames are being ACKed; nobody is replying
//                                (wrong node id, or the inverter isn't running)
//   tx_err climbing / bus-off -> nothing is ACKing us at all
//                                (wiring, termination, or wrong bitrate)
static void log_bus_status(const char *context)
{
    static const char *state_name[] = { "error-active", "error-warning",
                                        "error-passive", "BUS-OFF" };
    twai_node_status_t st = {0};
    twai_node_record_t rec = {0};
    if (twai_node_get_info(s_node, &st, &rec) != ESP_OK) return;

    ESP_LOGW(TAG, "%s: bus %s, tx_acked %lu, tx_unacked %lu, rx_frames %lu (last id 0x%lx), "
                  "tx_err %u, rx_err %u, bus_err %lu",
             context,
             st.state < (sizeof(state_name) / sizeof(state_name[0]))
                 ? state_name[st.state] : "?",
             (unsigned long)s_tx_ok_count, (unsigned long)s_tx_fail_count,
             (unsigned long)s_rx_any_count, (unsigned long)s_rx_last_id,
             (unsigned)st.tx_error_count, (unsigned)st.rx_error_count,
             (unsigned long)rec.bus_err_num);

    // Bus-off is latched — without an explicit recover the node stays offline
    // for good and every later request silently does nothing.
    if (st.state == TWAI_ERROR_BUS_OFF) {
        ESP_LOGW(TAG, "Node is bus-off, attempting recovery");
        twai_node_recover(s_node);
    }
}

// Blocking receive with timeout, mirroring the source's twai_receive(&f, pdMS_TO_TICKS(N)).
static bool twai_recv(oi_frame_t *out, int timeout_ms)
{
    return xQueueReceive(s_rx_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static bool sdo_lock(TickType_t wait)
{
    return s_sdo_lock && xSemaphoreTake(s_sdo_lock, wait) == pdTRUE;
}

static void sdo_unlock(void)
{
    if (s_sdo_lock) xSemaphoreGive(s_sdo_lock);
}

// Discard anything left over from a previous (timed-out) transaction so a stale
// reply can't be mistaken for the answer to the next request.
static void drain_rx_queue(void)
{
    oi_frame_t f;
    while (s_rx_queue && xQueueReceive(s_rx_queue, &f, 0) == pdTRUE) { }
}

static void request_sdo_element(uint16_t index, uint8_t sub_index)
{
    uint8_t d[8] = { SDO_READ, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub_index, 0, 0, 0, 0 };
    twai_send(0x600 | s_node_id, d, 8);
}

// Wait for the reply to one specific index/sub-index, discarding answers to
// earlier requests along the way.
//
// This matters when reading a few hundred parameters back to back. The inverter
// services CAN at its lowest interrupt priority, so under load the odd reply
// arrives after our timeout. Taking "the next frame" as the answer then shifts
// every subsequent read by one and the whole run fails from that point on —
// which is what "24 parameter reads unanswered" was: one late reply, then a
// cascade. Skipping stale replies re-synchronises instead.
static bool await_sdo_reply(uint16_t index, uint8_t sub_index, oi_frame_t *out,
                            int timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    do {
        if (!twai_recv(out, 5)) continue;
        if ((uint16_t)(out->data[1] | (out->data[2] << 8)) == index &&
            out->data[3] == sub_index) {
            return true;
        }
    } while ((int32_t)(deadline - xTaskGetTickCount()) > 0);
    return false;
}

static void set_value_sdo_u32(uint16_t index, uint8_t sub_index, uint32_t value)
{
    uint8_t d[8] = { SDO_WRITE, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub_index, 0, 0, 0, 0 };
    memcpy(&d[4], &value, 4);
    twai_send(0x600 | s_node_id, d, 8);
}

static void set_value_sdo_double(uint16_t index, uint8_t sub_index, double value)
{
    set_value_sdo_u32(index, sub_index, (uint32_t)(int32_t)(value * 32));
}

static void request_next_segment(bool toggle_bit)
{
    uint8_t d[8] = { (uint8_t)(SDO_REQUEST_SEGMENT | (toggle_bit << 4)), 0, 0, 0, 0, 0, 0, 0 };
    twai_send(0x600 | s_node_id, d, 8);
}

// ── parameter name -> SDO id lookup ──────────────────────────────────────────
// Scans the cached schema JSON for `"<name>": { ..., "id":N or "i":N, ... }`.
static int get_id_for_name(const char *name)
{
    FILE *f = fopen(s_json_path, "r");
    if (!f) return -1;

    struct stat st;
    if (stat(s_json_path, &st) != 0) { fclose(f); return -1; }

    char *buf = malloc((size_t)st.st_size + 1);
    if (!buf) { fclose(f); return -1; }
    size_t n = fread(buf, 1, (size_t)st.st_size, f);
    buf[n] = '\0';
    fclose(f);

    int result = -1;
    oi_json_reader_t r;
    oi_json_reader_init(&r, buf, n);
    const char *key, *obj;
    size_t klen, olen;
    while (oi_json_next_entry(&r, &key, &klen, &obj, &olen)) {
        if (strlen(name) == klen && memcmp(key, name, klen) == 0) {
            double id;
            if (oi_json_get_number(obj, olen, "id", &id) ||
                oi_json_get_number(obj, olen, "i", &id)) {
                result = (int)id;
            }
            break;
        }
    }
    free(buf);
    return result;
}

// ── SDO response handling (serial number + JSON download) ───────────────────

static void handle_sdo_response(const oi_frame_t *rx)
{
    static bool toggle_bit = false;
    static FILE *file = NULL;

    if (rx->data[0] == SDO_ABORT) {
        s_state = ST_ERROR;
        ESP_LOGW(TAG, "SDO abort — error obtaining serial number, try restarting");
        return;
    }

    switch (s_state) {
    case ST_OBTAINSERIAL: {
        uint16_t idx = rx->data[1] | (rx->data[2] << 8);
        if (idx == SDO_INDEX_SERIAL && rx->data[3] < 4) {
            uint32_t val;
            memcpy(&val, &rx->data[4], 4);
            s_serial[rx->data[3]] = val;

            if (rx->data[3] < 3) {
                request_sdo_element(SDO_INDEX_SERIAL, rx->data[3] + 1);
            } else {
                snprintf(s_json_path, sizeof(s_json_path), JSON_MOUNT_POINT "/%lx.json",
                          (unsigned long)s_serial[3]);
                ESP_LOGI(TAG, "Got serial %lX:%lX:%lX:%lX",
                         (unsigned long)s_serial[0], (unsigned long)s_serial[1],
                         (unsigned long)s_serial[2], (unsigned long)s_serial[3]);

                struct stat st;
                if (stat(s_json_path, &st) == 0 && st.st_size >= JSON_MIN_BYTES) {
                    s_state = ST_IDLE;
                    ESP_LOGI(TAG, "JSON schema already downloaded (%ld bytes)",
                             (long)st.st_size);
                } else {
                    s_state = ST_OBTAIN_JSON;
                    ESP_LOGI(TAG, "Downloading schema for %s", s_json_path);
                    unlink(JSON_TMP_PATH);
                    file = fopen(JSON_TMP_PATH, "w");
                    toggle_bit = false;
                    request_sdo_element(SDO_INDEX_STRINGS, 0);
                }
            }
        }
        break;
    }
    case ST_OBTAIN_JSON:
        if (!file) break;
        if ((rx->data[0] & SDO_SIZE_SPECIFIED) && (rx->data[0] & SDO_READ) == 0) {
            int size = 7 - ((rx->data[0] >> 1) & 0x7);
            fwrite(&rx->data[1], 1, (size_t)size, file);
            fclose(file);
            file = NULL;
            s_state = ST_IDLE;
            // Only now does it get its real name, so a partial transfer can
            // never be mistaken for a cached schema.
            unlink(s_json_path);
            if (rename(JSON_TMP_PATH, s_json_path) != 0) {
                ESP_LOGE(TAG, "Could not move schema into place");
            } else {
                struct stat st;
                stat(s_json_path, &st);
                ESP_LOGI(TAG, "Schema download complete (%ld bytes)", (long)st.st_size);
            }
        } else if (rx->data[0] == (uint8_t)(toggle_bit << 4) && (rx->data[0] & SDO_READ) == 0) {
            fwrite(&rx->data[1], 1, 7, file);
            toggle_bit = !toggle_bit;
            request_next_segment(toggle_bit);
        } else if ((rx->data[0] & SDO_READ) == SDO_READ) {
            request_next_segment(toggle_bit);
        }
        break;
    default:
        break;
    }
}

// ── firmware update over CAN ──────────────────────────────────────────────────

static uint32_t crc32_word(uint32_t crc, uint32_t data)
{
    crc ^= data;
    for (int i = 0; i < 32; i++) {
        crc = (crc & 0x80000000) ? (crc << 1) ^ 0x04C11DB7 : (crc << 1);
    }
    return crc;
}

static FILE      *s_update_file = NULL;
static int         s_current_page = 0;
static int         s_current_byte = 0;
static uint32_t    s_update_crc = 0;

static void handle_update(const oi_frame_t *rx)
{
    switch (s_upd_state) {
    case SEND_MAGIC:
        if (rx->data[0] == 0x33) {
            uint8_t d[4] = { rx->data[4], rx->data[5], rx->data[6], rx->data[7] };
            twai_send(0x7dd, d, 4);
            s_upd_state = SEND_SIZE;
            ESP_LOGI(TAG, "Update: sent id, waiting for size request");
            if (rx->data[1] < 1) vTaskDelay(pdMS_TO_TICKS(100));
        }
        break;
    case SEND_SIZE:
        if (rx->data[0] == 'S') {
            struct stat st;
            fstat(fileno(s_update_file), &st);
            uint8_t pages = (uint8_t)((st.st_size + PAGE_SIZE_BYTES - 1) / PAGE_SIZE_BYTES);
            uint8_t d[1] = { pages };
            twai_send(0x7dd, d, 1);
            s_upd_state = SEND_PAGE;
            s_update_crc = 0xFFFFFFFF;
            s_current_byte = 0;
            s_current_page = 0;
            ESP_LOGI(TAG, "Update: sending size %u pages", pages);
        }
        break;
    case SEND_PAGE:
        if (rx->data[0] == 'P') {
            uint8_t buffer[8];
            size_t bytes_read = 0;
            if (s_update_file) {
                fseek(s_update_file, s_current_byte, SEEK_SET);
                bytes_read = fread(buffer, 1, sizeof(buffer), s_update_file);
            }
            while (bytes_read < 8) buffer[bytes_read++] = 0xFF;
            s_current_byte += (int)bytes_read;

            uint32_t w0, w1;
            memcpy(&w0, &buffer[0], 4);
            memcpy(&w1, &buffer[4], 4);
            s_update_crc = crc32_word(s_update_crc, w0);
            s_update_crc = crc32_word(s_update_crc, w1);

            twai_send(0x7dd, buffer, 8);
            s_upd_state = SEND_PAGE;
        } else if (rx->data[0] == 'C') {
            uint8_t d[4] = {
                (uint8_t)(s_update_crc & 0xFF), (uint8_t)((s_update_crc >> 8) & 0xFF),
                (uint8_t)((s_update_crc >> 16) & 0xFF), (uint8_t)((s_update_crc >> 24) & 0xFF)
            };
            twai_send(0x7dd, d, 4);
            s_upd_state = CHECK_CRC;
        }
        break;
    case CHECK_CRC:
        s_update_crc = 0xFFFFFFFF;
        if (rx->data[0] == 'P') {
            s_upd_state = SEND_PAGE;
            s_current_page++;
            handle_update(rx);
        } else if (rx->data[0] == 'E') {
            s_upd_state = SEND_PAGE;
            s_current_byte = s_current_page * PAGE_SIZE_BYTES;
            handle_update(rx);
        } else if (rx->data[0] == 'D') {
            s_upd_state = REQUEST_JSON;
            s_state = ST_OBTAINSERIAL;
            s_retries = 50;
            if (s_update_file) { fclose(s_update_file); s_update_file = NULL; }
            ESP_LOGI(TAG, "Update complete");
        }
        break;
    default:
        break;
    }
}

// ── public API ────────────────────────────────────────────────────────────────

// Actual init body. Must run on core 1: twai_new_node_onchip() allocates its
// CPU interrupt on the calling core, and core 0's low/med interrupt slots are
// exhausted by WiFi + BLE in web-interface mode (alloc fails with NOT_FOUND).
// Node delete/re-create must also happen on the allocating core, so re-inits
// go through the same core-1 path.
static esp_err_t oi_can_init_on_this_core(uint8_t node_id, oi_baud_t baud)
{
    // Power and un-standby the transceiver. Driving SE low alone (as this did)
    // leaves the boost rail off, so the ESP32 transmits into a dead
    // transceiver: frames never reach the bus, nothing ACKs them, and TEC
    // climbs by 8 per attempt while rx stays at zero.
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << CAN_5V_EN_PIN) | (1ULL << CAN_SE_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(CAN_5V_EN_PIN, 1);   // boost rail on
    gpio_set_level(CAN_SE_PIN, 0);      // SE/Rs low = normal (high-speed) mode
    vTaskDelay(pdMS_TO_TICKS(20));      // let the rail come up before we transmit
    ESP_LOGI(TAG, "Transceiver enabled (5V_EN=GPIO%d high, SE=GPIO%d low)",
             CAN_5V_EN_PIN, CAN_SE_PIN);

    if (s_node) {
        twai_node_disable(s_node);
        twai_node_delete(s_node);
        s_node = NULL;
    }
    if (!s_rx_queue) {
        s_rx_queue = xQueueCreate(64, sizeof(oi_frame_t));
    }

    uint32_t bitrate = (baud == OI_BAUD_125K) ? 125000 : (baud == OI_BAUD_250K) ? 250000 : 500000;
    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = CAN_TX_PIN,
            .rx = CAN_RX_PIN,
            .quanta_clk_out    = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = { .bitrate = bitrate },
        // Transmit-capable (unlike the logging pipeline's listen-only node), so
        // the driver requires tx_queue_depth >= 1 — without it creation fails
        // with ESP_ERR_INVALID_ARG. SDO traffic is one outstanding request at a
        // time, so a shallow queue is plenty.
        .tx_queue_depth = 8,
        .flags = { .enable_listen_only = 0 },
    };
    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(err));
        return err;
    }

    // s_node_id gates the ISR's frame filter, so it must be live before enable().
    s_node_id   = node_id;
    s_baud_rate = baud;

    // Accept every ID in hardware; oi_rx_done_cb decides in software what
    // actually belongs in the SDO queue. A hardware filter here would save some
    // ISR work on a busy bus, but it also makes "no traffic at all" and "plenty
    // of traffic, none of it for us" look identical from the console — and that
    // distinction is what any bring-up problem turns on. Must be configured
    // while the node is still disabled.
    twai_mask_filter_config_t filter = { .id = 0, .mask = 0 };
    err = twai_node_config_mask_filter(s_node, 0, &filter);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Acceptance filter not applied: %s", esp_err_to_name(err));
    }

    twai_event_callbacks_t cbs = {
        .on_rx_done = oi_rx_done_cb,
        .on_tx_done = oi_tx_done_cb,
    };
    twai_node_register_event_callbacks(s_node, &cbs, NULL);
    twai_node_enable(s_node);

    s_state           = ST_OBTAINSERIAL;
    s_serial_attempts = 0;
    s_tx_ok_count     = 0;
    s_tx_fail_count   = 0;
    s_rx_any_count    = 0;
    s_rx_last_id      = 0;
    drain_rx_queue();

    request_sdo_element(SDO_INDEX_SERIAL, 0);
    s_serial_req_tick = xTaskGetTickCount();
    ESP_LOGI(TAG, "CAN initialized (node %u, %lu bps, transmit-capable)",
             node_id, (unsigned long)bitrate);
    return ESP_OK;
}

typedef struct {
    uint8_t           node_id;
    oi_baud_t         baud;
    esp_err_t         result;
    SemaphoreHandle_t done;
} init_req_t;

static void oi_can_init_task(void *arg)
{
    init_req_t *req = (init_req_t *)arg;
    req->result = oi_can_init_on_this_core(req->node_id, req->baud);
    xSemaphoreGive(req->done);
    vTaskDelete(NULL);
}

esp_err_t oi_can_init(uint8_t node_id, oi_baud_t baud)
{
    if (!s_sdo_lock) {
        s_sdo_lock = xSemaphoreCreateMutex();
        if (!s_sdo_lock) return ESP_ERR_NO_MEM;
    }
    // A /nodeid change deletes and re-creates the node, so it must not land in
    // the middle of another caller's SDO transaction.
    if (!sdo_lock(pdMS_TO_TICKS(5000))) return ESP_ERR_TIMEOUT;

    esp_err_t result;
    if (xPortGetCoreID() == 1) {
        result = oi_can_init_on_this_core(node_id, baud);
    } else {
        // Marshal onto a core-1 task (callers run on core 0: app_main at startup,
        // httpd task on a /nodeid settings change).
        init_req_t req = {
            .node_id = node_id,
            .baud    = baud,
            .result  = ESP_FAIL,
            .done    = xSemaphoreCreateBinary(),
        };
        if (!req.done) {
            sdo_unlock();
            return ESP_ERR_NO_MEM;
        }
        if (xTaskCreatePinnedToCore(oi_can_init_task, "oi_can_init", 4096,
                                    &req, 10, NULL, 1) != pdPASS) {
            vSemaphoreDelete(req.done);
            sdo_unlock();
            return ESP_ERR_NO_MEM;
        }
        xSemaphoreTake(req.done, portMAX_DELAY);
        vSemaphoreDelete(req.done);
        result = req.result;
    }
    sdo_unlock();
    return result;
}

void oi_can_deinit(void)
{
    if (s_node) {
        twai_node_disable(s_node);
        twai_node_delete(s_node);
        s_node = NULL;
    }
}

// Probe every CANopen node id for an SDO server. Runs once, after the
// configured node has stayed silent while the bus is otherwise healthy —
// frames arriving and our transmissions being acknowledged means something is
// out there, just not answering on the id we were told to use. With more than
// one openinverter board on the bus this also says which is which, since the
// acknowledgement that proves "someone is listening" carries no address.
// Temporarily retargets s_node_id, which is what the RX callback filters on.
static void scan_for_nodes(void)
{
    uint8_t saved = s_node_id;
    int found = 0;

    ESP_LOGW(TAG, "Bus is healthy but node %u is silent — scanning ids 1-63 for "
                  "SDO servers...", saved);

    for (uint8_t id = 1; id <= 63; id++) {
        s_node_id = id;
        drain_rx_queue();
        request_sdo_element(SDO_INDEX_SERIAL, 0);

        oi_frame_t rx;
        if (twai_recv(&rx, 20) && rx.id == (0x580u | id) && rx.data[0] != SDO_ABORT) {
            uint32_t serial_word;
            memcpy(&serial_word, &rx.data[4], 4);
            ESP_LOGW(TAG, "  --> node %u answered, serial word 0 = 0x%08lx",
                     id, (unsigned long)serial_word);
            found++;
        }
    }

    s_node_id = saved;
    drain_rx_queue();

    if (found) {
        ESP_LOGW(TAG, "Scan complete: %d node(s) answered. Set the Node ID field "
                      "in the web UI to one of those. The serial numbers tell the "
                      "boards apart if more than one replied.", found);
    } else {
        ESP_LOGW(TAG, "Scan complete: nothing answered on any id 1-63, though the "
                      "bus is carrying traffic. Whatever is transmitting is not "
                      "running an SDO server.");
    }
}

void oi_can_loop(void)
{
    if (!s_node) return;
    // Try-lock, never block: an HTTP handler owning the bus is the normal case,
    // and this task runs again in 10ms anyway. Blocking here would put the poll
    // task back to competing with the handler for the same replies.
    if (!sdo_lock(0)) return;

    oi_frame_t rx;
    bool recvd_response = false;

    if (twai_recv(&rx, 0)) {
        if (rx.id == SDO_RESPONSE_ID) {
            handle_sdo_response(&rx);
            recvd_response = true;
        } else if (rx.id == BOOTLOADER_ID) {
            handle_update(&rx);
        } else {
            ESP_LOGD(TAG, "Ignoring frame id=0x%lx", (unsigned long)rx.id);
        }
    }

    // The source project asks for the serial number exactly once, at init. If
    // that frame is lost — or the inverter powers up after the ESP does — the
    // state machine sticks at ST_OBTAINSERIAL and every /cmd?cmd=json 500s,
    // which the UI renders as the ESP<->STM communication error bar, forever.
    // ST_ERROR (an SDO abort) is equally terminal. Re-ask once a second instead.
    if ((s_state == ST_OBTAINSERIAL || s_state == ST_ERROR) &&
        (xTaskGetTickCount() - s_serial_req_tick) > pdMS_TO_TICKS(1000)) {
        s_state = ST_OBTAINSERIAL;

        // Say so out loud. A silent retry loop looks identical on the console
        // to a firmware that has stopped trying, which is exactly the state
        // this whole handshake fails into.
        char context[64];
        snprintf(context, sizeof(context), "No reply from node %u (attempt %d)",
                 s_node_id, ++s_serial_attempts);
        log_bus_status(context);

        // Once, after a few silent attempts, find out whether anything on the
        // bus does answer SDO — usually it does, on a different id.
        static bool scanned = false;
        if (!scanned && s_serial_attempts >= 5) {
            scanned = true;
            scan_for_nodes();
        }

        drain_rx_queue();
        request_sdo_element(SDO_INDEX_SERIAL, 0);
        s_serial_req_tick = xTaskGetTickCount();
    }

    bool pacing_delay = (s_upd_state == REQUEST_JSON);
    if (pacing_delay) {
        s_retries--;
        if (recvd_response || s_retries < 0) {
            s_upd_state = UPD_IDLE;
        } else {
            request_sdo_element(SDO_INDEX_SERIAL, 0);
        }
    }

    sdo_unlock();
    if (pacing_delay) vTaskDelay(pdMS_TO_TICKS(100));  // outside the lock
}

bool oi_can_send_json(httpd_req_t *req)
{
    if (!s_node) return false;

    // The browser's first request races the serial-number/schema handshake
    // kicked off at init. Wait it out rather than 500ing straight into the
    // communication error bar on every page load.
    for (int i = 0; i < 60 && s_state != ST_IDLE; i++) vTaskDelay(pdMS_TO_TICKS(50));
    if (s_state != ST_IDLE) {
        ESP_LOGW(TAG, "json: link not ready (state %d)", (int)s_state);
        return false;
    }

    FILE *f = fopen(s_json_path, "r");
    if (!f) {
        ESP_LOGW(TAG, "json: cannot open cached schema %s", s_json_path);
        return false;
    }
    struct stat st;
    fstat(fileno(f), &st);
    char *buf = malloc((size_t)st.st_size + 1);
    if (!buf) { fclose(f); return false; }
    size_t n = fread(buf, 1, (size_t)st.st_size, f);
    buf[n] = '\0';
    fclose(f);

    if (!sdo_lock(pdMS_TO_TICKS(5000))) { free(buf); return false; }
    drain_rx_queue();

    sock_writer_t w;
    sw_init(&w, req);

    int failed = 0;
    sw_putc(&w, '{');
    bool first = true;

    oi_json_reader_t r;
    oi_json_reader_init(&r, buf, n);
    const char *key, *obj;
    size_t klen, olen;
    while (oi_json_next_entry(&r, &key, &klen, &obj, &olen)) {
        double id_d;
        bool has_id = oi_json_get_number(obj, olen, "id", &id_d) ||
                       oi_json_get_number(obj, olen, "i", &id_d);

        if (!first) sw_putc(&w, ',');
        first = false;
        sw_printf(&w, "\"%.*s\":", (int)klen, key);

        if (has_id && (int)id_d > 0) {
            int id = (int)id_d;
            uint16_t want_index = (uint16_t)(SDO_INDEX_PARAM_UID | (id >> 8));
            request_sdo_element(want_index, id & 0xFF);

            oi_frame_t resp;
            bool answered = await_sdo_reply(want_index, (uint8_t)(id & 0xFF), &resp, 25);
            if (!answered) {
                // A reply lost to a busy inverter would otherwise leave this
                // parameter with no value, showing as a blank field that comes
                // and goes between refreshes. Ask once more before giving up —
                // it only costs anything on the rare miss.
                request_sdo_element(want_index, id & 0xFF);
                answered = await_sdo_reply(want_index, (uint8_t)(id & 0xFF), &resp, 40);
            }

            if (answered && resp.data[0] != SDO_ABORT) {
                int32_t raw;
                memcpy(&raw, &resp.data[4], 4);
                double value = raw / 32.0;
                // Re-emit the object with "value" set, everything else verbatim.
                sw_write(&w, obj, 1);  // '{'
                sw_printf(&w, "\"value\":%g,", value);
                sw_write(&w, obj + 1, olen - 1);
            } else {
                // An abort means the inverter answered and simply won't serve
                // this parameter — emit it without a value, but don't let it
                // count towards the comm-failure budget below.
                if (!answered) failed++;
                sw_write(&w, obj, olen);
            }
        } else {
            sw_write(&w, obj, olen);
        }
    }
    sw_putc(&w, '}');
    sw_finish(&w);
    sdo_unlock();
    free(buf);

    if (failed > 0) {
        // Report but don't act: an unanswered read means a slow or busy
        // inverter, not a bad schema. Deleting the cached schema here (as an
        // earlier version did) throws away a good file and forces a 16-second
        // re-download during which every request fails.
        ESP_LOGW(TAG, "json: %d parameter read(s) unanswered", failed);
    }
    // The body is already on the wire, so the caller can no longer substitute
    // an error response — those are decided by the pre-send checks above.
    return true;
}

void oi_can_send_can_mapping(httpd_req_t *req)
{
    enum { REQ_START, REQ_COBID, REQ_DATAPOSLEN, REQ_GAINOFS, REQ_DONE };

    oi_frame_t rx;
    int index = SDO_INDEX_MAP_RD, sub_index = 0;
    int32_t cobid = 0, pos = 0, len = 0, paramid = 0;
    bool rx_side = false;
    int req_state = REQ_START;

    sock_writer_t w;
    sw_init(&w, req);

    // ui.js fires /canmap from the same page load as /cmd?cmd=json. Without the
    // ST_IDLE guard this starts its own SDO walk on top of the serial/schema
    // handshake and both fail; an empty map is the honest answer until we're up.
    if (!s_node || s_state != ST_IDLE || !sdo_lock(pdMS_TO_TICKS(5000))) {
        sw_write(&w, "[]", 2);
        sw_finish(&w);
        return;
    }
    drain_rx_queue();

    sw_putc(&w, '[');
    bool first = true;

    while (req_state != REQ_DONE) {
        switch (req_state) {
        case REQ_START:
            request_sdo_element(index, 0);
            req_state = REQ_COBID;
            break;
        case REQ_COBID:
            if (twai_recv(&rx, 10)) {
                if (rx.data[0] != SDO_ABORT) {
                    memcpy(&cobid, &rx.data[4], 4);
                    sub_index++;
                    request_sdo_element(index, sub_index);
                    req_state = REQ_DATAPOSLEN;
                } else if (!rx_side) {
                    rx_side = true;
                    index = SDO_INDEX_MAP_RD + 0x80;
                    req_state = REQ_START;
                } else {
                    req_state = REQ_DONE;
                }
            } else {
                req_state = REQ_DONE;
            }
            break;
        case REQ_DATAPOSLEN:
            if (twai_recv(&rx, 10)) {
                if (rx.data[0] != SDO_ABORT) {
                    uint16_t pid;
                    memcpy(&pid, &rx.data[4], 2);
                    paramid = pid;
                    pos = rx.data[6];
                    len = (int8_t)rx.data[7];
                    sub_index++;
                    request_sdo_element(index, sub_index);
                    req_state = REQ_GAINOFS;
                } else {
                    index++;
                    sub_index = 0;
                    req_state = REQ_START;
                }
            } else {
                req_state = REQ_DONE;
            }
            break;
        case REQ_GAINOFS:
            if (twai_recv(&rx, 10)) {
                if (rx.data[0] != SDO_ABORT) {
                    int32_t raw;
                    memcpy(&raw, &rx.data[4], 4);
                    float gain = (float)(raw & 0xFFFFFF) / 1000.0f;
                    int offset = (int8_t)rx.data[7];

                    if (!first) sw_putc(&w, ',');
                    first = false;
                    sw_printf(&w,
                        "{\"isrx\":%s,\"id\":%ld,\"paramid\":%ld,\"position\":%ld,"
                        "\"length\":%ld,\"gain\":%g,\"offset\":%d,\"index\":%d,\"subindex\":%d}",
                        rx_side ? "true" : "false", (long)cobid, (long)paramid,
                        (long)pos, (long)len, (double)gain, offset, index, sub_index);

                    sub_index++;
                    request_sdo_element(index, sub_index);
                    req_state = REQ_DATAPOSLEN;
                } else {
                    req_state = REQ_DONE;
                }
            } else {
                req_state = REQ_DONE;
            }
            break;
        default:
            break;
        }
    }
    sw_putc(&w, ']');
    sw_finish(&w);
    sdo_unlock();
}

void oi_can_delete_params(void)
{
    unlink(s_json_path);
    s_upd_state = REQUEST_JSON;
}

oi_result_t oi_can_add_mapping(const char *json, size_t len)
{
    if (s_state != ST_IDLE) return OI_COMM_ERROR;

    double id = 0, paramid = 0, position = 0, length = 0, gain = 0, offset = 0;
    bool b_isrx;
    bool ok = oi_json_get_bool(json, len, "isrx", &b_isrx) &&
              oi_json_get_number(json, len, "id", &id) &&
              oi_json_get_number(json, len, "paramid", &paramid) &&
              oi_json_get_number(json, len, "position", &position) &&
              oi_json_get_number(json, len, "length", &length) &&
              oi_json_get_number(json, len, "gain", &gain) &&
              oi_json_get_number(json, len, "offset", &offset);
    if (!ok) {
        ESP_LOGW(TAG, "AddCanMapping: missing argument");
        return OI_UNKNOWN_INDEX;
    }

    int index = b_isrx ? SDO_INDEX_MAP_RX : SDO_INDEX_MAP_TX;
    oi_frame_t rx;

    if (!sdo_lock(pdMS_TO_TICKS(5000))) return OI_COMM_ERROR;
    drain_rx_queue();

    oi_result_t result = OI_COMM_ERROR;
    do {
        set_value_sdo_u32(index, 0, (uint32_t)(int32_t)id);
        if (!twai_recv(&rx, 10)) break;

        uint32_t v1 = ((uint32_t)paramid & 0xFFFF) | (((uint32_t)position & 0xFF) << 16) |
                      (((uint32_t)(int32_t)length & 0xFF) << 24);
        set_value_sdo_u32(index, 1, v1);
        if (rx.data[0] == SDO_ABORT || !twai_recv(&rx, 10)) break;

        uint32_t v2 = ((uint32_t)((int32_t)(gain * 1000)) & 0xFFFFFF) |
                      (((uint32_t)(int32_t)offset & 0xFF) << 24);
        set_value_sdo_u32(index, 2, v2);
        if (rx.data[0] == SDO_ABORT || !twai_recv(&rx, 10)) break;

        if (rx.data[0] != SDO_ABORT) result = OI_OK;
        else ESP_LOGW(TAG, "Mapping failed");
    } while (0);

    sdo_unlock();
    return result;
}

oi_result_t oi_can_remove_mapping(const char *json, size_t len)
{
    if (s_state != ST_IDLE) return OI_COMM_ERROR;

    double index_d, subindex_d;
    if (!oi_json_get_number(json, len, "index", &index_d) ||
        !oi_json_get_number(json, len, "subindex", &subindex_d)) {
        ESP_LOGW(TAG, "RemoveCanMapping: missing argument");
        return OI_UNKNOWN_INDEX;
    }

    if (!sdo_lock(pdMS_TO_TICKS(5000))) return OI_COMM_ERROR;
    drain_rx_queue();

    set_value_sdo_u32((uint16_t)index_d, (uint8_t)subindex_d, 0);
    oi_frame_t rx;
    oi_result_t result = OI_COMM_ERROR;
    if (twai_recv(&rx, 10)) {
        result = (rx.data[0] != SDO_ABORT) ? OI_OK : OI_UNKNOWN_INDEX;
    }
    sdo_unlock();
    return result;
}

oi_result_t oi_can_set_value(const char *name, double value)
{
    if (s_state != ST_IDLE) return OI_COMM_ERROR;
    int id = get_id_for_name(name);
    if (id < 0) return OI_UNKNOWN_INDEX;

    if (!sdo_lock(pdMS_TO_TICKS(5000))) return OI_COMM_ERROR;
    drain_rx_queue();

    set_value_sdo_double(SDO_INDEX_PARAM_UID | (id >> 8), (uint8_t)(id & 0xFF), value);
    oi_frame_t rx;
    bool answered = twai_recv(&rx, 10);
    sdo_unlock();

    if (!answered) return OI_COMM_ERROR;
    if (rx.data[0] == SDO_RESPONSE_DOWNLOAD) return OI_OK;
    uint32_t err;
    memcpy(&err, &rx.data[4], 4);
    if (err == 0x06090030u) return OI_VALUE_OUT_OF_RANGE;
    return OI_UNKNOWN_INDEX;
}

double oi_can_get_value(const char *name)
{
    if (s_state != ST_IDLE) return 0;
    int id = get_id_for_name(name);
    if (id < 0) return 0;

    if (!sdo_lock(pdMS_TO_TICKS(5000))) return 0;
    drain_rx_queue();

    request_sdo_element(SDO_INDEX_PARAM_UID | (id >> 8), (uint8_t)(id & 0xFF));
    oi_frame_t rx;
    bool answered = twai_recv(&rx, 10);
    sdo_unlock();

    if (!answered || rx.data[0] == SDO_ABORT) return 0;
    int32_t raw;
    memcpy(&raw, &rx.data[4], 4);
    return (double)raw / 32.0;
}

bool oi_can_save_to_flash(void)
{
    if (s_state != ST_IDLE) return false;
    if (!sdo_lock(pdMS_TO_TICKS(5000))) return false;
    drain_rx_queue();

    set_value_sdo_u32(SDO_INDEX_COMMANDS, SDO_CMD_SAVE, 0);
    oi_frame_t rx;
    bool ok = twai_recv(&rx, 200);
    sdo_unlock();
    return ok;
}

void oi_can_stream_values(httpd_req_t *req, const char *names, int samples)
{
    if (s_state != ST_IDLE) {
        httpd_resp_send(req, "", 0);
        return;
    }

    int ids[30];
    int num_items = 0;

    // names is a leading-comma list, e.g. ",udc,il1,speed" (matches source wire format)
    const char *p = names;
    while (*p == ',') p++;
    while (*p && num_items < 30) {
        const char *comma = strchr(p, ',');
        size_t len = comma ? (size_t)(comma - p) : strlen(p);
        char name[32];
        if (len >= sizeof(name)) len = sizeof(name) - 1;
        memcpy(name, p, len);
        name[len] = '\0';
        int id = get_id_for_name(name);
        ids[num_items++] = id;
        if (!comma) break;
        p = comma + 1;
    }

    sock_writer_t w;
    sw_init(&w, req);

    if (!sdo_lock(pdMS_TO_TICKS(5000))) {
        httpd_resp_send(req, "", 0);
        return;
    }

    for (int i = 0; i < samples; i++) {
        drain_rx_queue();
        for (int item = 0; item < num_items; item++) {
            int id = ids[item];
            if (id < 0) continue;
            request_sdo_element(SDO_INDEX_PARAM_UID | (id >> 8), (uint8_t)(id & 0xFF));
        }

        int item = 0;
        oi_frame_t rx;
        while (twai_recv(&rx, 10)) {
            if (item > 0) sw_putc(&w, ',');
            if (rx.data[0] == SDO_ABORT) {
                sw_putc(&w, '0');
            } else {
                // data[1] is the index low byte, i.e. id >> 8, and data[3] the
                // sub-index, i.e. id & 0xFF — so this reconstructs the full id.
                // Comparing it against `ids[item] & 0xFF` only ever matched for
                // ids below 256; anything higher reported 0.
                int received_item = (rx.data[1] << 8) + rx.data[3];
                if (item < num_items && received_item == ids[item]) {
                    int32_t raw;
                    memcpy(&raw, &rx.data[4], 4);
                    sw_printf(&w, "%.2f", raw / 32.0);
                } else {
                    sw_putc(&w, '0');
                }
            }
            item++;
        }
        sw_write(&w, "\r\n", 2);
    }
    sw_finish(&w);
    sdo_unlock();
}

int oi_can_start_update(const char *filename)
{
    s_update_file = fopen(filename, "r");
    if (sdo_lock(pdMS_TO_TICKS(5000))) {
        set_value_sdo_u32(SDO_INDEX_COMMANDS, SDO_CMD_RESET, 1);
        sdo_unlock();
    }
    s_upd_state = SEND_MAGIC;
    s_current_page = 0;
    if (!s_update_file) return 0;
    struct stat st;
    fstat(fileno(s_update_file), &st);
    return (int)((st.st_size + PAGE_SIZE_BYTES - 1) / PAGE_SIZE_BYTES);
}

int oi_can_get_current_update_page(void) { return s_current_page; }
int oi_can_get_node_id(void)             { return s_node_id; }
oi_baud_t oi_can_get_baud_rate(void)     { return s_baud_rate; }
