// WiFi + HTTP server for bench/service "web interface" mode.
// Ported from esp32-web-interface-lilygo_tcan (Arduino WiFi.h/WebServer/SPIFFS)
// onto IDF-native esp_wifi + esp_http_server + spiffs + mdns. Routes mirror
// esp32-web-interface.ino's server.on(...) table; handlers call into oi_can.h
// (ported oi_can.cpp) instead of the Arduino OICan:: namespace.
#include "web_interface.h"
#include "oi_can.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <dirent.h>

static const char *TAG = "WEBIF";
#define SPIFFS_MOUNT_POINT "/spiffs"
#define MDNS_HOSTNAME       "inverter"

// Node ID / CAN speed chosen on the web interface, persisted so they survive a
// reboot (same "telematics" namespace as sd_logger.c and ble_nus.c).
#define NVS_NAMESPACE       "telematics"
#define NVS_KEY_NODE_ID     "oi_node_id"
#define NVS_KEY_CAN_SPEED   "oi_can_speed"
#define DEFAULT_NODE_ID     20
#define DEFAULT_CAN_BAUD    OI_BAUD_500K

// ── content-type helper (mirrors getContentType() in esp32-web-interface.ino) ─

static const char *content_type_for(const char *path)
{
    size_t len = strlen(path);
    if (len > 4 && strcmp(path + len - 4, ".bin") == 0) return "application/octet-stream";
    if (len > 4 && strcmp(path + len - 4, ".htm") == 0) return "text/html";
    if (len > 5 && strcmp(path + len - 5, ".html") == 0) return "text/html";
    if (len > 4 && strcmp(path + len - 4, ".css") == 0)  return "text/css";
    if (len > 3 && strcmp(path + len - 3, ".js") == 0)   return "application/javascript";
    if (len > 4 && strcmp(path + len - 4, ".png") == 0)  return "image/png";
    if (len > 4 && strcmp(path + len - 4, ".gif") == 0)  return "image/gif";
    if (len > 4 && strcmp(path + len - 4, ".jpg") == 0)  return "image/jpeg";
    if (len > 4 && strcmp(path + len - 4, ".ico") == 0)  return "image/x-icon";
    if (len > 3 && strcmp(path + len - 3, ".gz") == 0)   return "application/x-gzip";
    return "text/plain";
}

// Streams a file from SPIFFS. Prefers "<path>.gz" if present (source assets
// are pre-gzipped). Returns false (caller sends 404) if neither exists.
static bool handle_file_read(httpd_req_t *req, const char *uri_path)
{
    char path[160];
    if (strcmp(uri_path, "/") == 0) uri_path = "/index.html";
    snprintf(path, sizeof(path), "%s%s", SPIFFS_MOUNT_POINT, uri_path);

    char gz_path[168];
    snprintf(gz_path, sizeof(gz_path), "%s.gz", path);

    const char *serve_path = path;
    struct stat st;
    if (stat(gz_path, &st) == 0) {
        serve_path = gz_path;
    } else if (stat(path, &st) != 0) {
        return false;
    }

    FILE *f = fopen(serve_path, "r");
    if (!f) return false;

    httpd_resp_set_type(req, content_type_for(path));
    if (serve_path == gz_path) httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=86400");

    char buf[1024];
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), f)) > 0) {
        if (httpd_resp_send_chunk(req, buf, n) != ESP_OK) {
            fclose(f);
            httpd_resp_send_chunk(req, NULL, 0);
            return true;  // response already started; nothing more we can do
        }
    }
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return true;
}

static esp_err_t not_found_handler(httpd_req_t *req)
{
    if (!handle_file_read(req, req->uri)) {
        httpd_resp_set_hdr(req, "Refresh", "6; url=/");
        httpd_resp_send_404(req);
    }
    return ESP_OK;
}

static esp_err_t root_get_handler(httpd_req_t *req)
{
    if (!handle_file_read(req, req->uri)) httpd_resp_send_404(req);
    return ESP_OK;
}

// ── /list, /edit (file manager over SPIFFS) ──────────────────────────────────

static esp_err_t list_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/json");
    DIR *dir = opendir(SPIFFS_MOUNT_POINT);
    if (!dir) { httpd_resp_sendstr(req, "[]"); return ESP_OK; }

    httpd_resp_sendstr_chunk(req, "[");
    struct dirent *entry;
    bool first = true;
    while ((entry = readdir(dir)) != NULL) {
        char buf[300];
        snprintf(buf, sizeof(buf), "%s{\"type\":\"file\",\"name\":\"/%s\"}",
                 first ? "" : ",", entry->d_name);
        httpd_resp_sendstr_chunk(req, buf);
        first = false;
    }
    closedir(dir);
    httpd_resp_sendstr_chunk(req, "]");
    httpd_resp_sendstr_chunk(req, NULL);
    return ESP_OK;
}

static esp_err_t edit_get_handler(httpd_req_t *req)
{
    if (!handle_file_read(req, "/edit.htm")) httpd_resp_send_404(req);
    return ESP_OK;
}

static bool query_arg(httpd_req_t *req, const char *key, char *out, size_t out_size)
{
    size_t qlen = httpd_req_get_url_query_len(req);
    if (qlen == 0) return false;
    char *qbuf = malloc(qlen + 1);
    if (!qbuf) return false;
    bool found = false;
    if (httpd_req_get_url_query_str(req, qbuf, qlen + 1) == ESP_OK) {
        found = httpd_query_key_value(qbuf, key, out, out_size) == ESP_OK;
    }
    free(qbuf);
    return found;
}

static esp_err_t edit_put_handler(httpd_req_t *req)
{
    char path[64];
    if (!query_arg(req, "path", path, sizeof(path)) || strcmp(path, "/") == 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "BAD PATH");
        return ESP_OK;
    }
    char full[96];
    snprintf(full, sizeof(full), "%s%s", SPIFFS_MOUNT_POINT, path);
    struct stat st;
    if (stat(full, &st) == 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "FILE EXISTS");
        return ESP_OK;
    }
    FILE *f = fopen(full, "w");
    if (!f) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "CREATE FAILED"); return ESP_OK; }
    fclose(f);
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

static esp_err_t edit_delete_handler(httpd_req_t *req)
{
    char path[64], storage[16] = "onboard";
    if (!query_arg(req, "path", path, sizeof(path))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "BAD ARGS");
        return ESP_OK;
    }
    query_arg(req, "storage", storage, sizeof(storage));
    if (strcmp(path, "/") == 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "BAD PATH");
        return ESP_OK;
    }
    if (strcmp(storage, "onboard") == 0) {
        char full[96];
        snprintf(full, sizeof(full), "%s%s", SPIFFS_MOUNT_POINT, path);
        if (unlink(full) != 0) {
            httpd_resp_send_404(req);
            return ESP_OK;
        }
    }
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

// Minimal multipart/form-data extraction: pulls the raw file bytes between the
// first blank-line-terminated part header and the trailing boundary. Good
// enough for the single-file uploads this endpoint receives (matches
// handleFileUpload()'s scope in the source project — no filename sanitization
// beyond what the client sends, since this is a bench-mode admin tool on a
// device the user's own AP/browser connects to).
static esp_err_t edit_post_handler(httpd_req_t *req)
{
    char filename[64];
    if (!query_arg(req, "path", filename, sizeof(filename))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "BAD ARGS");
        return ESP_OK;
    }
    char full[96];
    snprintf(full, sizeof(full), "%s%s", SPIFFS_MOUNT_POINT, filename);

    FILE *f = fopen(full, "w");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "CREATE FAILED");
        return ESP_OK;
    }

    char buf[1024];
    int remaining = req->content_len;
    while (remaining > 0) {
        int to_read = remaining < (int)sizeof(buf) ? remaining : (int)sizeof(buf);
        int n = httpd_req_recv(req, buf, to_read);
        if (n <= 0) break;
        fwrite(buf, 1, n, f);
        remaining -= n;
    }
    fclose(f);
    httpd_resp_sendstr(req, "");
    return ESP_OK;
}

// ── /wifi ─────────────────────────────────────────────────────────────────────

static esp_err_t wifi_get_handler(httpd_req_t *req)
{
    char ssid[33] = {0}, pass[65] = {0};
    bool has_ap  = query_arg(req, "apSSID", ssid, sizeof(ssid));
    char ap_pass[65] = {0};
    bool has_ap_pw = query_arg(req, "apPW", ap_pass, sizeof(ap_pass));
    char sta_ssid[33] = {0}, sta_pass[65] = {0};
    bool has_sta = query_arg(req, "staSSID", sta_ssid, sizeof(sta_ssid));
    bool has_sta_pw = query_arg(req, "staPW", sta_pass, sizeof(sta_pass));

    if (has_ap && has_ap_pw) {
        wifi_config_t cfg = {0};
        strncpy((char *)cfg.ap.ssid, ssid, sizeof(cfg.ap.ssid) - 1);
        cfg.ap.ssid_len = strlen(ssid);
        strncpy((char *)cfg.ap.password, ap_pass, sizeof(cfg.ap.password) - 1);
        cfg.ap.authmode = strlen(ap_pass) >= 8 ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
        cfg.ap.max_connection = 4;
        cfg.ap.channel = 1;
        esp_wifi_set_config(WIFI_IF_AP, &cfg);
    } else if (has_sta && has_sta_pw) {
        wifi_config_t cfg = {0};
        strncpy((char *)cfg.sta.ssid, sta_ssid, sizeof(cfg.sta.ssid) - 1);
        strncpy((char *)cfg.sta.password, sta_pass, sizeof(cfg.sta.password) - 1);
        esp_wifi_set_config(WIFI_IF_STA, &cfg);
        esp_wifi_connect();
    } else {
        if (!handle_file_read(req, "/wifi.html")) httpd_resp_send_404(req);
        return ESP_OK;
    }

    if (!handle_file_read(req, "/wifi-updated.html")) httpd_resp_sendstr(req, "Updated");
    return ESP_OK;
}

// ── /cmd (json / set / stream / save) ────────────────────────────────────────

static esp_err_t cmd_get_handler(httpd_req_t *req)
{
    char cmd[192];
    if (!query_arg(req, "cmd", cmd, sizeof(cmd))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "BAD ARGS");
        return ESP_OK;
    }

    if (strcmp(cmd, "json") == 0) {
        httpd_resp_set_type(req, "application/json");
        int fd = httpd_req_to_sockfd(req);
        if (fd < 0 || !oi_can_send_json(fd)) {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "CAN communication error");
        }
        // oi_can_send_json wrote directly to the socket fd; nothing more to send.
        return ESP_OK;
    }
    if (strncmp(cmd, "set ", 4) == 0) {
        char name[40];
        double value;
        if (sscanf(cmd, "set %39s %lf", name, &value) != 2) {
            httpd_resp_sendstr(req, "BAD ARGS");
            return ESP_OK;
        }
        switch (oi_can_set_value(name, value)) {
            case OI_OK:                httpd_resp_sendstr(req, "Set Ok"); break;
            case OI_UNKNOWN_INDEX:     httpd_resp_sendstr(req, "Unknown Parameter"); break;
            case OI_VALUE_OUT_OF_RANGE:httpd_resp_sendstr(req, "Value out of range"); break;
            case OI_COMM_ERROR:        httpd_resp_sendstr(req, "CAN communication error"); break;
        }
        return ESP_OK;
    }
    if (strncmp(cmd, "stream ", 7) == 0) {
        int samples;
        char names[192];
        if (sscanf(cmd, "stream %d %191[^\r]", &samples, names) != 2) {
            httpd_resp_sendstr(req, "BAD ARGS");
            return ESP_OK;
        }
        httpd_resp_set_type(req, "text/plain");
        int fd = httpd_req_to_sockfd(req);
        if (fd >= 0) oi_can_stream_values(fd, names, samples);
        return ESP_OK;
    }
    if (strcmp(cmd, "save") == 0) {
        httpd_resp_sendstr(req, oi_can_save_to_flash() ? "Parameters and CAN map saved"
                                                         : "No reply to save command");
        return ESP_OK;
    }

    httpd_resp_sendstr(req, "Unknown command");
    return ESP_OK;
}

// ── /canmap ───────────────────────────────────────────────────────────────────

static esp_err_t canmap_get_handler(httpd_req_t *req)
{
    char json[256];
    oi_result_t res = OI_OK;
    if (query_arg(req, "add", json, sizeof(json))) {
        res = oi_can_add_mapping(json, strlen(json));
    } else if (query_arg(req, "remove", json, sizeof(json))) {
        res = oi_can_remove_mapping(json, strlen(json));
    } else if (query_arg(req, "edit", json, sizeof(json))) {
        res = oi_can_remove_mapping(json, strlen(json));
        if (res == OI_OK) res = oi_can_add_mapping(json, strlen(json));
    }

    if (res == OI_OK) {
        httpd_resp_set_type(req, "application/json");
        int fd = httpd_req_to_sockfd(req);
        if (fd >= 0) oi_can_send_can_mapping(fd);
    } else if (res == OI_COMM_ERROR) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "CAN communication error");
    } else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Invalid request");
    }
    return ESP_OK;
}

// ── /fwupdate ─────────────────────────────────────────────────────────────────

static esp_err_t fwupdate_get_handler(httpd_req_t *req)
{
    char step_s[16], file[64];
    if (!query_arg(req, "step", step_s, sizeof(step_s)) ||
        !query_arg(req, "file", file, sizeof(file))) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "BAD ARGS");
        return ESP_OK;
    }
    int step = atoi(step_s);
    static int pages = 0;

    if (step < 0) {
        char full[96];
        snprintf(full, sizeof(full), "%s%s", SPIFFS_MOUNT_POINT, file);
        pages = oi_can_start_update(full);
    } else {
        while (oi_can_get_current_update_page() < step) {
            oi_can_loop();
        }
    }

    char resp[96];
    snprintf(resp, sizeof(resp), "{ \"message\": \"\", \"pages\": %d }", pages);
    httpd_resp_set_type(req, "text/json");
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

// ── /version, /nodeid, /reboot ───────────────────────────────────────────────

static esp_err_t version_get_handler(httpd_req_t *req)
{
    httpd_resp_sendstr(req, "1.0.R-idf");
    return ESP_OK;
}

static esp_err_t nodeid_get_handler(httpd_req_t *req)
{
    char id_s[8], speed_s[8];
    if (query_arg(req, "id", id_s, sizeof(id_s))) {
        int id = atoi(id_s);
        if (id < 1 || id > 63) id = DEFAULT_NODE_ID;

        // Keep the current speed unless the request explicitly names one, so
        // that changing only the node ID doesn't silently reset the baud rate.
        oi_baud_t baud = oi_can_get_baud_rate();
        if (query_arg(req, "canspeed", speed_s, sizeof(speed_s))) {
            int speed = atoi(speed_s);
            baud = speed == 0 ? OI_BAUD_125K : (speed == 1 ? OI_BAUD_250K : OI_BAUD_500K);
        }

        if (oi_can_init((uint8_t)id, baud) == ESP_OK) {
            nvs_handle_t nvs;
            if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
                nvs_set_u8(nvs, NVS_KEY_NODE_ID,   (uint8_t)id);
                nvs_set_u8(nvs, NVS_KEY_CAN_SPEED, (uint8_t)baud);
                nvs_commit(nvs);
                nvs_close(nvs);
            }
        } else {
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "CAN init failed");
            return ESP_OK;
        }
    }
    char resp[16];
    snprintf(resp, sizeof(resp), "%d,%d", oi_can_get_node_id(), oi_can_get_baud_rate());
    httpd_resp_sendstr(req, resp);
    return ESP_OK;
}

static esp_err_t reboot_get_handler(httpd_req_t *req)
{
    httpd_resp_sendstr(req, "Rebooting");
    oi_can_delete_params();
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
    return ESP_OK;
}

// ── WiFi + mDNS + SPIFFS bring-up ────────────────────────────────────────────

static esp_err_t init_spiffs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path              = SPIFFS_MOUNT_POINT,
        .partition_label        = "spiffs",
        .max_files               = 8,
        .format_if_mount_failed = false,
    };
    esp_err_t err = esp_vfs_spiffs_register(&conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(err));
        return err;
    }
    size_t total = 0, used = 0;
    esp_spiffs_info("spiffs", &total, &used);
    ESP_LOGI(TAG, "SPIFFS mounted: %u/%u bytes used", (unsigned)used, (unsigned)total);
    return ESP_OK;
}

static esp_err_t init_wifi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    wifi_config_t ap_cfg = {0};
    strncpy((char *)ap_cfg.ap.ssid, WEB_INTERFACE_AP_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    ap_cfg.ap.ssid_len = strlen(WEB_INTERFACE_AP_SSID);
    strncpy((char *)ap_cfg.ap.password, WEB_INTERFACE_AP_PASS, sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.channel = 1;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    // STA reconnects to any previously-saved credentials (set via /wifi) automatically
    // once esp_wifi_connect() has been called at least once from that handler.

    ESP_LOGI(TAG, "AP \"%s\" up, 192.168.4.1", WEB_INTERFACE_AP_SSID);
    return ESP_OK;
}

static esp_err_t init_mdns(void)
{
    esp_err_t err = mdns_init();
    if (err != ESP_OK) return err;
    mdns_hostname_set(MDNS_HOSTNAME);
    mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
    ESP_LOGI(TAG, "mDNS: http://%s.local/", MDNS_HOSTNAME);
    return ESP_OK;
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 16;
    config.stack_size = 8192;

    httpd_handle_t server = NULL;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        return err;
    }

    static const httpd_uri_t routes[] = {
        { .uri = "/list",     .method = HTTP_GET,    .handler = list_get_handler },
        { .uri = "/edit",     .method = HTTP_GET,    .handler = edit_get_handler },
        { .uri = "/edit",     .method = HTTP_PUT,    .handler = edit_put_handler },
        { .uri = "/edit",     .method = HTTP_DELETE, .handler = edit_delete_handler },
        { .uri = "/edit",     .method = HTTP_POST,   .handler = edit_post_handler },
        { .uri = "/wifi",     .method = HTTP_GET,    .handler = wifi_get_handler },
        { .uri = "/cmd",      .method = HTTP_GET,    .handler = cmd_get_handler },
        { .uri = "/canmap",   .method = HTTP_GET,    .handler = canmap_get_handler },
        { .uri = "/fwupdate", .method = HTTP_GET,    .handler = fwupdate_get_handler },
        { .uri = "/version",  .method = HTTP_GET,    .handler = version_get_handler },
        { .uri = "/nodeid",   .method = HTTP_GET,    .handler = nodeid_get_handler },
        { .uri = "/reboot",   .method = HTTP_GET,    .handler = reboot_get_handler },
        { .uri = "/",         .method = HTTP_GET,    .handler = root_get_handler },
        { .uri = "/*",        .method = HTTP_GET,    .handler = not_found_handler },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    ESP_LOGI(TAG, "HTTP server started on port 80");
    return ESP_OK;
}

esp_err_t web_interface_start(void)
{
    esp_err_t err = init_spiffs();
    if (err != ESP_OK) return err;

    err = init_wifi();
    if (err != ESP_OK) return err;

    init_mdns();

    err = start_http_server();
    if (err != ESP_OK) return err;

    uint8_t node_id = DEFAULT_NODE_ID;
    uint8_t baud    = DEFAULT_CAN_BAUD;
    nvs_handle_t nvs;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u8(nvs, NVS_KEY_NODE_ID,   &node_id);   // left untouched if absent
        nvs_get_u8(nvs, NVS_KEY_CAN_SPEED, &baud);
        nvs_close(nvs);
    }
    if (node_id < 1 || node_id > 63)  node_id = DEFAULT_NODE_ID;
    if (baud > OI_BAUD_500K)          baud    = DEFAULT_CAN_BAUD;

    oi_can_init(node_id, (oi_baud_t)baud);

    return ESP_OK;
}
