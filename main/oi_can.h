#pragma once
// CANopen/SDO client for talking to the Zombieverter inverter.
// Ported from esp32-web-interface-lilygo_tcan/src/oi_can.cpp (Arduino/C++)
// to plain C against the IDF TWAI onchip-node API. Only usable in
// APP_MODE_WEB_INTERFACE — brings up a transmit-capable TWAI node, which is
// mutually exclusive with the logging pipeline's listen-only node.
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

typedef enum {
    OI_OK,
    OI_UNKNOWN_INDEX,
    OI_VALUE_OUT_OF_RANGE,
    OI_COMM_ERROR,
} oi_result_t;

typedef enum {
    OI_BAUD_125K,
    OI_BAUD_250K,
    OI_BAUD_500K,
} oi_baud_t;

esp_err_t   oi_can_init(uint8_t node_id, oi_baud_t baud);
void        oi_can_deinit(void);
void        oi_can_loop(void);   // call periodically from the web_interface task

// Parameter schema JSON (as downloaded from the inverter) with live values
// filled in, written to the given fd. Returns false on CAN comm failure.
bool        oi_can_send_json(int client_fd);
void        oi_can_send_can_mapping(int client_fd);
void        oi_can_delete_params(void);

oi_result_t oi_can_add_mapping(const char *json, size_t len);
oi_result_t oi_can_remove_mapping(const char *json, size_t len);
oi_result_t oi_can_set_value(const char *name, double value);
double      oi_can_get_value(const char *name);
bool        oi_can_save_to_flash(void);

// Comma-led list of names, e.g. ",udc,il1,speed" — matches the source
// project's wire format where the leading comma is part of the request.
// Writes `samples` rows of CSV values to the given fd.
void        oi_can_stream_values(int client_fd, const char *names, int samples);

int         oi_can_start_update(const char *filename);
int         oi_can_get_current_update_page(void);

int         oi_can_get_node_id(void);
oi_baud_t   oi_can_get_baud_rate(void);
