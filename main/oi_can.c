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

// ── buffered raw-socket writer ───────────────────────────────────────────────
// httpd_req_to_sockfd() hands back the underlying LWIP socket fd, not
// something fdopen()/dup() can wrap portably under IDF's VFS config — so
// response bodies (JSON, CAN mapping, streamed samples) are written straight
// to the fd via send(), buffered here to keep the many small fprintf-style
// writes in the ported code from becoming one syscall each.
typedef struct {
    int    fd;
    char   buf[512];
    size_t len;
} sock_writer_t;

static void sw_init(sock_writer_t *w, int fd) { w->fd = fd; w->len = 0; }

static void sw_flush(sock_writer_t *w)
{
    if (w->len == 0) return;
    send(w->fd, w->buf, w->len, 0);
    w->len = 0;
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

#define OI_CAN_TRANSCEIVER_STANDBY_PIN  GPIO_NUM_23
#define PAGE_SIZE_BYTES  1024
#define JSON_MOUNT_POINT "/spiffs"

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

// ── low-level TX/RX ──────────────────────────────────────────────────────────

typedef struct { uint32_t id; uint8_t dlc; uint8_t data[8]; } oi_frame_t;

static bool IRAM_ATTR oi_rx_done_cb(twai_node_handle_t handle,
                                     const twai_rx_done_event_data_t *edata,
                                     void *user_ctx)
{
    (void)edata; (void)user_ctx;
    uint8_t buf[8] = {0};
    twai_frame_t rx = { .buffer = buf, .buffer_len = sizeof(buf) };

    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) return false;

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
    twai_node_transmit(s_node, &frame, 10);
}

// Blocking receive with timeout, mirroring the source's twai_receive(&f, pdMS_TO_TICKS(N)).
static bool twai_recv(oi_frame_t *out, int timeout_ms)
{
    return xQueueReceive(s_rx_queue, out, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

static void request_sdo_element(uint16_t index, uint8_t sub_index)
{
    uint8_t d[8] = { SDO_READ, (uint8_t)(index & 0xFF), (uint8_t)(index >> 8), sub_index, 0, 0, 0, 0 };
    twai_send(0x600 | s_node_id, d, 8);
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
                if (stat(s_json_path, &st) == 0) {
                    s_state = ST_IDLE;
                    ESP_LOGI(TAG, "JSON schema already downloaded");
                } else {
                    s_state = ST_OBTAIN_JSON;
                    ESP_LOGI(TAG, "Downloading schema to %s", s_json_path);
                    file = fopen(s_json_path, "w");
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
            ESP_LOGI(TAG, "Schema download complete");
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

esp_err_t oi_can_init(uint8_t node_id, oi_baud_t baud)
{
    gpio_set_direction(OI_CAN_TRANSCEIVER_STANDBY_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(OI_CAN_TRANSCEIVER_STANDBY_PIN, 0);

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
        .flags = { .enable_listen_only = 0 },  // transmit-capable, unlike the logging pipeline
    };
    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TWAI node: %s", esp_err_to_name(err));
        return err;
    }

    twai_event_callbacks_t cbs = { .on_rx_done = oi_rx_done_cb };
    twai_node_register_event_callbacks(s_node, &cbs, NULL);
    twai_node_enable(s_node);

    s_node_id   = node_id;
    s_baud_rate = baud;
    s_state     = ST_OBTAINSERIAL;
    request_sdo_element(SDO_INDEX_SERIAL, 0);
    ESP_LOGI(TAG, "CAN initialized (node %u, %lu bps, transmit-capable)",
             node_id, (unsigned long)bitrate);
    return ESP_OK;
}

void oi_can_deinit(void)
{
    if (s_node) {
        twai_node_disable(s_node);
        twai_node_delete(s_node);
        s_node = NULL;
    }
}

void oi_can_loop(void)
{
    oi_frame_t rx;
    bool recvd_response = false;

    if (twai_recv(&rx, 0)) {
        if (rx.id == (0x580 | s_node_id)) {
            handle_sdo_response(&rx);
            recvd_response = true;
        } else if (rx.id == 0x7de) {
            handle_update(&rx);
        } else {
            ESP_LOGD(TAG, "Ignoring frame id=0x%lx", (unsigned long)rx.id);
        }
    }

    if (s_upd_state == REQUEST_JSON) {
        s_retries--;
        if (recvd_response || s_retries < 0) {
            s_upd_state = UPD_IDLE;
        } else {
            request_sdo_element(SDO_INDEX_SERIAL, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

bool oi_can_send_json(int client_fd)
{
    if (s_state != ST_IDLE) return false;

    FILE *f = fopen(s_json_path, "r");
    if (!f) return false;
    struct stat st;
    fstat(fileno(f), &st);
    char *buf = malloc((size_t)st.st_size + 1);
    if (!buf) { fclose(f); return false; }
    size_t n = fread(buf, 1, (size_t)st.st_size, f);
    buf[n] = '\0';
    fclose(f);

    sock_writer_t w;
    sw_init(&w, client_fd);

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
            request_sdo_element(SDO_INDEX_PARAM_UID | (id >> 8), id & 0xFF);
            oi_frame_t resp;
            if (twai_recv(&resp, 10) && resp.data[3] == (uint8_t)(id & 0xFF)) {
                int32_t raw;
                memcpy(&raw, &resp.data[4], 4);
                double value = raw / 32.0;
                // Re-emit the object with "value" set, everything else verbatim.
                sw_write(&w, obj, 1);  // '{'
                sw_printf(&w, "\"value\":%g,", value);
                sw_write(&w, obj + 1, olen - 1);
            } else {
                failed++;
                sw_write(&w, obj, olen);
            }
        } else {
            sw_write(&w, obj, olen);
        }
    }
    sw_putc(&w, '}');
    sw_flush(&w);
    free(buf);
    return failed < 5;
}

void oi_can_send_can_mapping(int client_fd)
{
    enum { REQ_START, REQ_COBID, REQ_DATAPOSLEN, REQ_GAINOFS, REQ_DONE };

    oi_frame_t rx;
    int index = SDO_INDEX_MAP_RD, sub_index = 0;
    int32_t cobid = 0, pos = 0, len = 0, paramid = 0;
    bool rx_side = false;
    int req_state = REQ_START;

    sock_writer_t w;
    sw_init(&w, client_fd);
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
    sw_flush(&w);
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

    set_value_sdo_u32(index, 0, (uint32_t)(int32_t)id);
    if (!twai_recv(&rx, 10)) return OI_COMM_ERROR;

    uint32_t v1 = ((uint32_t)paramid & 0xFFFF) | (((uint32_t)position & 0xFF) << 16) |
                  (((uint32_t)(int32_t)length & 0xFF) << 24);
    set_value_sdo_u32(index, 1, v1);
    if (rx.data[0] == SDO_ABORT || !twai_recv(&rx, 10)) return OI_COMM_ERROR;

    uint32_t v2 = ((uint32_t)((int32_t)(gain * 1000)) & 0xFFFFFF) |
                  (((uint32_t)(int32_t)offset & 0xFF) << 24);
    set_value_sdo_u32(index, 2, v2);
    if (rx.data[0] == SDO_ABORT || !twai_recv(&rx, 10)) return OI_COMM_ERROR;

    if (rx.data[0] != SDO_ABORT) return OI_OK;
    ESP_LOGW(TAG, "Mapping failed");
    return OI_COMM_ERROR;
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

    set_value_sdo_u32((uint16_t)index_d, (uint8_t)subindex_d, 0);
    oi_frame_t rx;
    if (twai_recv(&rx, 10)) {
        if (rx.data[0] != SDO_ABORT) return OI_OK;
        return OI_UNKNOWN_INDEX;
    }
    return OI_COMM_ERROR;
}

oi_result_t oi_can_set_value(const char *name, double value)
{
    if (s_state != ST_IDLE) return OI_COMM_ERROR;
    int id = get_id_for_name(name);
    if (id < 0) return OI_UNKNOWN_INDEX;

    set_value_sdo_double(SDO_INDEX_PARAM_UID | (id >> 8), (uint8_t)(id & 0xFF), value);
    oi_frame_t rx;
    if (!twai_recv(&rx, 10)) return OI_COMM_ERROR;

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

    request_sdo_element(SDO_INDEX_PARAM_UID | (id >> 8), (uint8_t)(id & 0xFF));
    oi_frame_t rx;
    if (!twai_recv(&rx, 10)) return 0;
    if (rx.data[0] == 0x80) return 0;
    uint32_t raw;
    memcpy(&raw, &rx.data[4], 4);
    return (double)raw / 32.0;
}

bool oi_can_save_to_flash(void)
{
    if (s_state != ST_IDLE) return false;
    set_value_sdo_u32(SDO_INDEX_COMMANDS, SDO_CMD_SAVE, 0);
    oi_frame_t rx;
    return twai_recv(&rx, 200);
}

void oi_can_stream_values(int client_fd, const char *names, int samples)
{
    if (s_state != ST_IDLE) return;

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
    sw_init(&w, client_fd);

    for (int i = 0; i < samples; i++) {
        for (int item = 0; item < num_items; item++) {
            int id = ids[item];
            if (id < 0) continue;
            request_sdo_element(SDO_INDEX_PARAM_UID | (id >> 8), (uint8_t)(id & 0xFF));
        }

        int item = 0;
        oi_frame_t rx;
        while (twai_recv(&rx, 10)) {
            if (item > 0) sw_putc(&w, ',');
            if (rx.data[0] == 0x80) {
                sw_putc(&w, '0');
            } else {
                int received_item = (rx.data[1] << 8) + rx.data[3];
                if (item < num_items && received_item == (ids[item] & 0xFF)) {
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
    sw_flush(&w);
}

int oi_can_start_update(const char *filename)
{
    s_update_file = fopen(filename, "r");
    set_value_sdo_u32(SDO_INDEX_COMMANDS, SDO_CMD_RESET, 1);
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
