#include "can_handler.h"
#include "vehicle_state.h"
#include "esp_twai_onchip.h"
#include "esp_twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "CAN";

// ── LILYGO T-CAN485 pin assignments ─────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_27
#define CAN_RX_PIN  GPIO_NUM_26

// ── M3 BMS CAN IDs (SomersetEV/Tesla-M3-Bms-Software CAN_Common.cpp) ────────
#define CAN_ID_BMS_SOC      0x355   // SoC / SoH
#define CAN_ID_BMS_PACK     0x356   // Pack voltage, current, temp max
#define CAN_ID_BMS_CELLS    0x373   // Cell Vmin/Vmax, temp min/max (Kelvin)

// ── Leaf inverter CAN IDs (leafinv.cpp) ──────────────────────────────────────
#define CAN_ID_LEAF_SPEED_ERR   0x1DA   // RPM (bytes 4-5), voltage (byte 0)
#define CAN_ID_LEAF_TEMPS       0x55A   // motor temp (byte 1), inv temp (byte 2), Fahrenheit

// ── ISA IVT-S Shunt CAN IDs (isa_shunt.cpp) ──────────────────────────────────
// All frames: bytes 0-1 = header, bytes 2-5 = int32 value, bytes 6-7 unused
#define CAN_ID_ISA_CURRENT  0x521   // mA, signed
#define CAN_ID_ISA_VOLTAGE  0x522   // mV, signed
#define CAN_ID_ISA_KW       0x526   // W,  signed
#define CAN_ID_ISA_AH       0x527   // Ah (ISA internal units)
#define CAN_ID_ISA_KWH      0x528   // kWh (ISA internal units)

// ── MG Gen2 V2L Charger CAN IDs (MGgen2V2Lcharger.cpp) ───────────────────────
#define CAN_ID_MG_LV        0x39F   // LV volts (byte1/8), LV amps (byte4)
#define CAN_ID_MG_PLUG      0x323   // plug state (byte5: 0=out, 1=in)
#define CAN_ID_MG_TEMP      0x33B   // temp_1 = byte3 - 50 (°C)

// TODO: ZombieVerter IDs to be confirmed

// ── Internal frame type passed from ISR → task via queue ─────────────────────
typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
} raw_frame_t;

static twai_node_handle_t s_node        = NULL;
static QueueHandle_t      s_rx_queue    = NULL;
static QueueHandle_t      s_log_queue   = NULL;  // shared with sd_logger

// ── ISR callback — called when a CAN frame arrives ───────────────────────────
static bool IRAM_ATTR twai_rx_done_cb(twai_node_handle_t handle,
                                      const twai_rx_done_event_data_t *edata,
                                      void *user_ctx)
{
    (void)edata; (void)user_ctx;
    uint8_t buf[8] = {0};
    twai_frame_t rx = { .buffer = buf, .buffer_len = sizeof(buf) };

    if (twai_node_receive_from_isr(handle, &rx) != ESP_OK) return false;

    raw_frame_t f = {
        .id  = rx.header.id,
        .dlc = (uint8_t)(rx.header.dlc < 8 ? rx.header.dlc : 8),
    };
    memcpy(f.data, buf, f.dlc);

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &f, &woken);
    return woken == pdTRUE;
}

// ── Frame parsers ─────────────────────────────────────────────────────────────

// ── ISA IVT-S parser ─────────────────────────────────────────────────────────
// All ISA messages share the same layout: int32 in bytes 2-5, byte 5 = MSB.
// Matches isa_shunt.cpp: (bytes[5]<<24)|(bytes[4]<<16)|(bytes[3]<<8)|bytes[2]
static inline int32_t isa_decode(const raw_frame_t *f)
{
    return (int32_t)(((uint32_t)f->data[5] << 24) |
                     ((uint32_t)f->data[4] << 16) |
                     ((uint32_t)f->data[3] <<  8) |
                      (uint32_t)f->data[2]);
}

// ── M3 BMS parsers ────────────────────────────────────────────────────────────

static void parse_bms_soc(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x355 — SoC / SoH (SomersetEV CAN_Common.cpp)
    // Byte 0:   SoC integer % (0–100)
    // Byte 2:   SoH % (fixed 100 = 0x64)
    // Bytes 4-5: SoC x100 little-endian (0.01% resolution, unused here)
    if (f->dlc < 1) return;
    state->latest.soc = f->data[0];
}

static void parse_bms_pack(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x356 — Pack voltage / current / temp max (SomersetEV CAN_Common.cpp)
    // Bytes 0-1: pack voltage x100 little-endian (0.01V units)
    // Bytes 2-3: pack current x10  little-endian (0.1A, signed)
    // Bytes 4-5: temp max x10      little-endian (0.1°C, signed)
    if (f->dlc < 6) return;
    state->latest.pack_voltage_bms = (uint16_t)(f->data[0] | (f->data[1] << 8));
    state->latest.bms_temp_max     = (int16_t) (f->data[4] | (f->data[5] << 8));
}

static void parse_bms_cells(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x373 — Cell voltages and temperatures (SomersetEV CAN_Common.cpp)
    // Bytes 0-1: cell Vmin little-endian (mV)
    // Bytes 2-3: cell Vmax little-endian (mV)
    // Bytes 4-5: temp min + 273 little-endian (Kelvin → subtract 273 for °C)
    // Bytes 6-7: temp max + 273 little-endian (Kelvin → subtract 273 for °C)
    if (f->dlc < 8) return;
    state->latest.cell_voltage_min = (uint16_t)(f->data[0] | (f->data[1] << 8));
    state->latest.cell_voltage_max = (uint16_t)(f->data[2] | (f->data[3] << 8));
    uint16_t tmin_k = (uint16_t)(f->data[4] | (f->data[5] << 8));
    uint16_t tmax_k = (uint16_t)(f->data[6] | (f->data[7] << 8));
    state->latest.bms_temp_min = (int16_t)((tmin_k - 273) * 10);
    state->latest.bms_temp_max = (int16_t)((tmax_k - 273) * 10);
}

// ── Leaf inverter parsers ─────────────────────────────────────────────────────

static void parse_leaf_speed(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x1DA bytes 4-5: 15-bit signed RPM (leafinv.cpp DecodeCAN)
    // Reconstruction: (byte4 << 7) | (byte5 >> 1), then sign-extend from 15 bits
    if (f->dlc < 6) return;
    int16_t rpm = (int16_t)(((uint16_t)f->data[4] << 7) | (f->data[5] >> 1));
    if (rpm == 0x7fff) rpm = 0;           // invalid sentinel → zero
    else if (rpm > 0x3fff) rpm -= 0x7fff; // 15-bit sign extension
    state->latest.motor_rpm = rpm;
}

static void parse_leaf_temps(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x55A byte 1 = motor temp (°F), byte 2 = inverter temp (°F)
    // Convert: °C×10 = ((°F - 32) × 50) / 9
    if (f->dlc < 3) return;
    state->latest.motor_temp    = (int16_t)(((int16_t)f->data[1] - 32) * 50 / 9);
    state->latest.inverter_temp = (int16_t)(((int16_t)f->data[2] - 32) * 50 / 9);
}

// ── MG Gen2 V2L Charger parsers ───────────────────────────────────────────────

static void parse_mg_lv(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x39F byte1 = LV volts (÷8 = V, stored as mV → byte1*125)
    //        byte4 = LV amps (raw A)
    if (f->dlc < 5) return;
    state->latest.lv_volts_mv = (uint16_t)f->data[1] * 125;
    state->latest.lv_amps     = f->data[4];
}

static void parse_mg_plug(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x323 byte5 = plug state (1=plugged, 0=unplugged)
    if (f->dlc < 6) return;
    state->latest.plug_state = (f->data[5] == 1) ? 1 : 0;
}

static void parse_mg_temp(const raw_frame_t *f, vehicle_state_t *state)
{
    // 0x33B byte3 - 50 = charger temp_1 in °C, stored as °C x10
    if (f->dlc < 4) return;
    state->latest.charger_temp = (int16_t)((int16_t)f->data[3] - 50) * 10;
}

// ── Task ──────────────────────────────────────────────────────────────────────

void can_rx_task(void *pvParameters)
{
    s_log_queue             = (QueueHandle_t)pvParameters;
    vehicle_state_t *state  = vehicle_state_get();

    // Deep ISR queue — ISA shunt alone can burst 5 IDs at up to 100Hz
    s_rx_queue = xQueueCreate(256, sizeof(raw_frame_t));
    configASSERT(s_rx_queue);

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = CAN_TX_PIN,
            .rx = CAN_RX_PIN,
            .quanta_clk_out   = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = { .bitrate = 500000 },
        .flags = { .enable_listen_only = 1 },
    };

    ESP_ERROR_CHECK(twai_new_node_onchip(&node_cfg, &s_node));

    twai_event_callbacks_t cbs = { .on_rx_done = twai_rx_done_cb };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));

    ESP_ERROR_CHECK(twai_node_enable(s_node));
    ESP_LOGI(TAG, "TWAI started, 500kbps listen-only");

    raw_frame_t f;

    while (1) {
        if (xQueueReceive(s_rx_queue, &f, portMAX_DELAY) == pdTRUE) {
            uint32_t tick_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());

            // Parse into vehicle state and flag known IDs for raw logging
            bool known = true;
            switch (f.id) {
                case CAN_ID_BMS_SOC:        parse_bms_soc(&f, state);                       break;
                case CAN_ID_BMS_PACK:       parse_bms_pack(&f, state);                      break;
                case CAN_ID_BMS_CELLS:      parse_bms_cells(&f, state);                     break;
                case CAN_ID_LEAF_SPEED_ERR: parse_leaf_speed(&f, state);                    break;
                case CAN_ID_LEAF_TEMPS:     parse_leaf_temps(&f, state);                    break;
                case CAN_ID_ISA_CURRENT:    state->latest.pack_current_ma = isa_decode(&f); break;
                case CAN_ID_ISA_VOLTAGE:    state->latest.pack_voltage_mv = isa_decode(&f); break;
                case CAN_ID_ISA_KW:         state->latest.isa_kw          = isa_decode(&f); break;
                case CAN_ID_ISA_AH:         state->latest.isa_ah          = isa_decode(&f); break;
                case CAN_ID_ISA_KWH:        state->latest.isa_kwh         = isa_decode(&f); break;
                case CAN_ID_MG_LV:          parse_mg_lv(&f, state);                         break;
                case CAN_ID_MG_PLUG:        parse_mg_plug(&f, state);                       break;
                case CAN_ID_MG_TEMP:        parse_mg_temp(&f, state);                       break;
                // TODO: ZombieVerter IDs to be confirmed
                default: known = false; break;
            }

            // Only log frames we recognise — drop unknown IDs
            if (known) {
                log_msg_t raw_msg = {
                    .type  = LOG_MSG_RAW_FRAME,
                    .frame = { .tick_ms = tick_ms, .id = f.id, .dlc = f.dlc },
                };
                memcpy(raw_msg.frame.data, f.data, f.dlc);
                if (xQueueSend(s_log_queue, &raw_msg, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Log queue full, raw frame dropped");
                }
            }
        }
    }
}
