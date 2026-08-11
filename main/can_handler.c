#include "can_handler.h"
#include "vehicle_state.h"
#include "ble_nus.h"
#include "speed_bridge.h"
#include "esp_twai_onchip.h"
#include "esp_twai.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "CAN";

// ── LILYGO T-CAN485 pin assignments ─────────────────────────────────────────
#define CAN_TX_PIN  GPIO_NUM_27
#define CAN_RX_PIN  GPIO_NUM_26

// Transceiver control. UNVERIFIED against the board schematic — these are from
// the LilyGo T-CAN485 reference design. Listen-only RX works today without
// touching them, so they are driven ONLY when CAN TX is explicitly enabled;
// the passive path stays bit-for-bit as it always was.
// This matters specifically for TX: the transceivers used on these boards
// (SN65HVD230 Rs, TJA1051 S) keep the receiver alive in standby while
// disabling the transmitter — the classic "RX fine, transmits nothing" symptom.
#define CAN_5V_EN_PIN  GPIO_NUM_16
#define CAN_SE_PIN     GPIO_NUM_23

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
static volatile uint32_t  s_frame_count = 0;

// ── TX state (see can_handler.h) ─────────────────────────────────────────────
static volatile bool     s_tx_capable      = false;  // node created non-listen-only
static volatile bool     s_bus_off_flag    = false;  // set in ISR, cleared in service()
static volatile uint32_t s_tx_ok           = 0;
static volatile uint32_t s_tx_fail         = 0;
static volatile uint32_t s_ack_fail_streak = 0;
static volatile uint32_t s_bus_off_count   = 0;
static volatile uint32_t s_last_err_flags  = 0;

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

// ── TX / error / state ISR callbacks ─────────────────────────────────────────
// ISR context: counter bumps and flag sets only. twai_node_recover() is NOT
// ISR-safe, so bus-off is flagged here and acted on in can_handler_service().

static bool IRAM_ATTR twai_tx_done_cb(twai_node_handle_t handle,
                                      const twai_tx_done_event_data_t *edata,
                                      void *user_ctx)
{
    (void)handle; (void)user_ctx;
    if (edata->is_tx_success) {
        s_tx_ok++;
        s_ack_fail_streak = 0;
    } else {
        s_tx_fail++;
        s_ack_fail_streak++;
    }
    return false;
}

static bool IRAM_ATTR twai_error_cb(twai_node_handle_t handle,
                                    const twai_error_event_data_t *edata,
                                    void *user_ctx)
{
    (void)handle; (void)user_ctx;
    s_last_err_flags = edata->err_flags.val;   // ack_err bit = nobody else on the bus
    return false;
}

static bool IRAM_ATTR twai_state_cb(twai_node_handle_t handle,
                                    const twai_state_change_event_data_t *edata,
                                    void *user_ctx)
{
    (void)handle; (void)user_ctx;
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        s_bus_off_flag = true;
        s_bus_off_count++;
    }
    return false;
}

// Drive the transceiver out of standby and enable the 5V boost rail.
static void can_transceiver_enable(void)
{
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
    ESP_LOGI(TAG, "transceiver enabled (5V_EN=GPIO%d high, SE=GPIO%d low)",
             CAN_5V_EN_PIN, CAN_SE_PIN);
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

    // Clearing enable_listen_only makes this an error-active bus participant
    // that ACKs every frame — a real change to how it affects a live vehicle
    // bus. It is gated behind the SPDCAN NVS flag, applied here at node
    // creation because the flag is fixed for the node's lifetime.
    bool tx_en = speed_bridge_tx_enabled();

    twai_onchip_node_config_t node_cfg = {
        .io_cfg = {
            .tx = CAN_TX_PIN,
            .rx = CAN_RX_PIN,
            .quanta_clk_out   = -1,
            .bus_off_indicator = -1,
        },
        .bit_timing = { .bitrate = 500000 },
    };

    if (tx_en) {
        // tx_queue_depth > 0 is MANDATORY when not listen-only — the driver
        // rejects the config otherwise (esp_twai_onchip.c: "tx_queue_depth at
        // least 1"), which under ESP_ERROR_CHECK would be a boot loop.
        node_cfg.tx_queue_depth = 4;
        // On ESP32 this is a single-shot switch, not a counter: the HAL does
        // `.ss = retry_cnt != -1`. 0 => drop a failed frame and retry next
        // cycle. -1 would retransmit forever and drive an unACKed bus to
        // bus-off.
        node_cfg.fail_retry_cnt = 0;
        can_transceiver_enable();
    } else {
        node_cfg.flags.enable_listen_only = 1;
    }

    esp_err_t err = twai_new_node_onchip(&node_cfg, &s_node);
    if (err != ESP_OK && tx_en) {
        // A bad TX config must degrade to today's passive behaviour, never to
        // a boot loop.
        ESP_LOGE(TAG, "TX-capable node failed (%s) — falling back to listen-only",
                 esp_err_to_name(err));
        tx_en = false;
        node_cfg.tx_queue_depth = 0;
        node_cfg.fail_retry_cnt = 0;
        node_cfg.flags.enable_listen_only = 1;
        err = twai_new_node_onchip(&node_cfg, &s_node);
    }
    ESP_ERROR_CHECK(err);

    twai_event_callbacks_t cbs = {
        .on_rx_done      = twai_rx_done_cb,
        .on_tx_done      = tx_en ? twai_tx_done_cb : NULL,
        .on_state_change = twai_state_cb,
        .on_error        = twai_error_cb,
    };
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(s_node, &cbs, NULL));

    ESP_ERROR_CHECK(twai_node_enable(s_node));
    s_tx_capable = tx_en;
    ESP_LOGI(TAG, "TWAI started, 500kbps %s", tx_en ? "TX-ENABLED (acks bus)" : "listen-only");

    raw_frame_t f;

    while (1) {
        if (xQueueReceive(s_rx_queue, &f, portMAX_DELAY) == pdTRUE) {
            s_frame_count++;
            uint32_t tick_ms = (uint32_t)pdTICKS_TO_MS(xTaskGetTickCount());

            // Decode known IDs into vehicle state
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
                // Without loopback the node never sees its own frames, so any
                // sighting of our TX id means another node already owns it.
                case SPEED_CAN_ID:          speed_bridge_note_collision();                  break;
                default: break;
            }

            // Stream ALL frames to Speedo app — it needs full CAN bus visibility
            if (g_app_mode == APP_MODE_SPEEDO && g_ble_live_queue != NULL) {
                raw_can_log_t live = { .tick_ms = tick_ms, .id = f.id, .dlc = f.dlc };
                memcpy(live.data, f.data, f.dlc);
                xQueueSendToBack(g_ble_live_queue, &live, 0);
            }

            // Log ALL frames to SD card (including unknown IDs)
            log_msg_t raw_msg = {
                .type  = LOG_MSG_RAW_FRAME,
                .frame = { .tick_ms = tick_ms, .id = f.id, .dlc = f.dlc },
            };
            memcpy(raw_msg.frame.data, f.data, f.dlc);
            if (xQueueSend(s_log_queue, &raw_msg, 0) != pdTRUE) {
                ESP_LOGW(TAG, "Log queue full, frame dropped");
            }
        }
    }
}

uint32_t can_handler_frame_count(void) {
    return s_frame_count;
}

// ── Transmit API ─────────────────────────────────────────────────────────────

esp_err_t can_handler_transmit(const twai_frame_t *frame, int timeout_ms)
{
    if (s_node == NULL)  return ESP_ERR_INVALID_STATE;
    if (!s_tx_capable)   return ESP_ERR_NOT_SUPPORTED;
    return twai_node_transmit(s_node, frame, timeout_ms);
}

bool can_handler_tx_capable(void) {
    return s_tx_capable && s_node != NULL;
}

// Task context only — twai_node_recover() is not ISR-safe.
void can_handler_service(void)
{
    if (!s_bus_off_flag || s_node == NULL) return;
    s_bus_off_flag = false;
    esp_err_t err = twai_node_recover(s_node);
    ESP_LOGW(TAG, "bus-off #%lu, recover: %s (err_flags=0x%lx)",
             (unsigned long)s_bus_off_count, esp_err_to_name(err),
             (unsigned long)s_last_err_flags);
}

uint32_t can_handler_tx_ok_count(void)      { return s_tx_ok; }
uint32_t can_handler_tx_fail_count(void)    { return s_tx_fail; }
uint32_t can_handler_ack_fail_streak(void)  { return s_ack_fail_streak; }
uint32_t can_handler_bus_off_count(void)    { return s_bus_off_count; }
