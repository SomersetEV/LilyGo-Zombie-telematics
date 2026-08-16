#pragma once
// WiFi + HTTP server for bench/service "web interface" mode — lets a laptop
// or phone browser configure the Zombieverter inverter over CAN/SDO (see
// oi_can.h). Ported from esp32-web-interface-lilygo_tcan (Arduino/PlatformIO,
// WiFi.h + WebServer + SPIFFS) onto IDF-native esp_wifi/esp_http_server/spiffs.
//
// Only ever runs in APP_MODE_WEB_INTERFACE — see ble_nus.h boot_mode_t. Not
// used together with the logging pipeline's BLE/CAN tasks in the same boot.
#include "esp_err.h"

#define WEB_INTERFACE_AP_SSID  "SomersetEV-Inverter"
#define WEB_INTERFACE_AP_PASS  "zombieverter"

// Brings up WiFi (AP+STA), the HTTP server, mDNS, and the CANopen/SDO client
// (oi_can_init). Call once from the web-interface boot path in app_main().
esp_err_t web_interface_start(void);
