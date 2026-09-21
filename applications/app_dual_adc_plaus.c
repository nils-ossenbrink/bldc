/*
 * Dual-ADC joystick with plausibility check for VESC 6/75
 * Author: Nils (extended with VESC 6.06 custom-config & persistence)
 *
 * Reads ADC_EXT and ADC_EXT2, enforces complementary-sum plausibility,
 * outputs current command; on any fault -> stop (or brake).
 *
 * Adds:
 *  - Configurable parameters (min/max/center, deadband, margins, alpha, i_max, etc.)
 *  - Persistent storage in EEPROM (custom region) with magic word for version safety
 *  - VESC Tool UI via conf_custom (XML supplied by firmware)
 *  - Mutex-protected config access (no race condition between Tool callback and thread)
 *  - Calibration terminal command (dual_adc_cal left|mid|right)
 *
 * Firmware targets: VESC FW 6.x (tested with 6.06)
 */

#pragma GCC optimize ("Os")

#include "ch.h"
#include "hal.h"
#include "app.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "utils.h"
#include "hw.h"
#include "buffer.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "terminal.h"     // terminal_register_command_callback / unregister
#include "commands.h"     // commands_printf, etc.
#include "conf_custom.h"  // custom config channel (UI + transport)
#include "conf_general.h" // EEPROM custom var helpers

// ---------------------- ADC access ------------------------------

// Most VESC boards use Vref = 3.3V and 12-bit ADC (0..4095).
#ifndef V_ADC_REF
#define V_ADC_REF   3.3f
#endif

static inline float adc_to_volt(uint16_t raw) {
    return (V_ADC_REF * (float)raw) / 4095.0f;
}

// Which ADC indices to read (default: external ADC1/ADC2)
#ifndef ADC1_IDX
#define ADC1_IDX    ADC_IND_EXT
#endif
#ifndef ADC2_IDX
#define ADC2_IDX    ADC_IND_EXT2
#endif

// ---------------------- Helper functions ------------------------

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// dedicated integer clamp – avoids incorrect float cast on int32_t fields
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ---------------------- App configuration -----------------------

// Magic word identifies a valid / current EEPROM struct version.
// Change this value whenever persisted layout changes.
#define DUALADC_CFG_MAGIC   0xDA0C0002u
#define DUALADC_CAL_VALID_MAGIC 0xDA0CCA1Bu
#define DUALADC_CAL_VALID_EEPROM_INDEX 13

// Runtime / Tool-visible config layout. Order must match XML field order.
typedef struct {
    float    v_min;            // [V] expected minimum per channel
    float    v_center;         // [V] expected center (idle) per-channel average
    float    v_max;            // [V] expected maximum per channel
    float    deadband;         // [-] deadband around 0 after normalisation
    float    sum_target;       // [V] expected v1+v2
    float    sum_tol;          // [V] ± tolerance on sum plausibility
    float    v_margin;         // [V] extra window margin per channel
    float    alpha;            // [-] low-pass filter weight (0..1)
    float    i_max;            // [A] max current command at |pos|=1
    float    brake_on_fault;   // [A] if >0, active brake current on fault
    int32_t  clear_time_ok_ms; // [ms] inputs must be OK before clearing latched fault
    int32_t  loop_period_ms;   // [ms] main loop period (1..20 typical)
} dualadc_cfg_t;

// Persisted layout adds a magic word ahead of runtime config.
typedef struct __attribute__((packed)) {
    uint32_t      magic; // must equal DUALADC_CFG_MAGIC
    dualadc_cfg_t cfg;
} dualadc_cfg_store_t;

#define DUALADC_CFG_SERIALIZED_SIZE 48
#define DUALADC_CFG_FLOAT_SCALE     1000.0f

static const char dualadc_cfg_signature_text[] =
    "v_min18v_center18v_max18deadband18sum_target18sum_tol18"
    "v_margin18alpha18i_max18brake_on_fault18clear_time_ok_ms26"
    "loop_period_ms26";

static uint32_t dualadc_cfg_signature(void) {
    return utils_crc32c((uint8_t *)dualadc_cfg_signature_text,
            strlen(dualadc_cfg_signature_text));
}

// sizeof(dualadc_cfg_t) = 48 bytes, sizeof(dualadc_cfg_store_t) = 52 bytes

static void cfg_load_defaults(dualadc_cfg_t *c) {
    c->v_min            = 0.32f;
    c->v_center         = 1.53f;
    c->v_max            = 2.69f;
    c->deadband         = 0.2f;
    // sum_target ≈ v_min + v_max for truly complementary sensors
    c->sum_target       = 2.92f;
    c->sum_tol          = 0.3f;
    c->v_margin         = 0.05f;
    c->alpha            = 0.12f;
    c->i_max            = 3.0f;
    c->brake_on_fault   = 0.0f;
    c->clear_time_ok_ms = 50;
    c->loop_period_ms   = 2;
}

// Mutex protects g_cfg against concurrent access by Tool callback and thread
static mutex_t      g_cfg_mtx;
static dualadc_cfg_t g_cfg;
static volatile bool m_calibration_valid = false;

static void calibration_valid_store(bool valid) {
    eeprom_var value;
    value.as_u32 = valid ? DUALADC_CAL_VALID_MAGIC : 0u;
    conf_general_store_eeprom_var_custom(&value, DUALADC_CAL_VALID_EEPROM_INDEX);
}

static bool calibration_valid_load(void) {
    eeprom_var value;
    return conf_general_read_eeprom_var_custom(&value, DUALADC_CAL_VALID_EEPROM_INDEX) &&
            value.as_u32 == DUALADC_CAL_VALID_MAGIC;
}

// Safely copy current config into a local snapshot for use inside the thread
static inline dualadc_cfg_t cfg_snapshot(void) {
    chMtxLock(&g_cfg_mtx);
    dualadc_cfg_t snap = g_cfg;
    chMtxUnlock(&g_cfg_mtx);
    return snap;
}

// ---------------------- Persistence (custom EEPROM) --------------

/*
 * Store the whole struct as a raw byte blob across consecutive custom
 * EEPROM slots (each slot = 32 bits). struct is __packed__ so no padding.
 */
static void cfg_store_to_eeprom(const dualadc_cfg_t *c) {
    dualadc_cfg_store_t store;
    store.magic = DUALADC_CFG_MAGIC;
    store.cfg = *c;

    uint8_t buf[sizeof(dualadc_cfg_store_t)];
    memcpy(buf, &store, sizeof(dualadc_cfg_store_t));

    const int words = (sizeof(dualadc_cfg_store_t) + 3) / 4;
    eeprom_var v;
    for (int i = 0; i < words; i++) {
        uint32_t w = 0;
        memcpy(&w, buf + (i * 4), 4);
        v.as_u32 = w;
        conf_general_store_eeprom_var_custom(&v, i);
    }
}

// Returns true and fills *c only if magic matches; false -> caller loads defaults
static bool cfg_read_from_eeprom(dualadc_cfg_t *c) {
    uint8_t buf[sizeof(dualadc_cfg_store_t)];
    memset(buf, 0, sizeof(buf));

    const int words = (sizeof(dualadc_cfg_store_t) + 3) / 4;
    eeprom_var v;
    for (int i = 0; i < words; i++) {
        if (!conf_general_read_eeprom_var_custom(&v, i)) {
            return false; // slot not programmed
        }
        memcpy(buf + (i * 4), &v.as_u32, 4);
    }

    dualadc_cfg_store_t tmp;
    memcpy(&tmp, buf, sizeof(dualadc_cfg_store_t));

    if (tmp.magic != DUALADC_CFG_MAGIC) {
        return false;
    }

    *c = tmp.cfg;
    return true;
}

// ---------------------- Custom-config (UI + transport) ----------

static int my_get_cfg(uint8_t *data, bool is_default) {
    dualadc_cfg_t tmp;
    if (is_default) {
        cfg_load_defaults(&tmp);
    } else {
        chMtxLock(&g_cfg_mtx);
        tmp = g_cfg;
        chMtxUnlock(&g_cfg_mtx);
    }
    int32_t ind = 0;
    buffer_append_uint32(data, dualadc_cfg_signature(), &ind);
    buffer_append_float32(data, tmp.v_min, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.v_center, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.v_max, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.deadband, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.sum_target, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.sum_tol, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.v_margin, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.alpha, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.i_max, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_float32(data, tmp.brake_on_fault, DUALADC_CFG_FLOAT_SCALE, &ind);
    buffer_append_int32(data, tmp.clear_time_ok_ms, &ind);
    buffer_append_int32(data, tmp.loop_period_ms, &ind);
    return ind;
}

// Accept new config from VESC Tool: validate, apply (mutex-protected), persist
static bool my_set_cfg(uint8_t *data) {
    if (!data) return false;

    dualadc_cfg_t in;
    int32_t ind = 0;
    if (buffer_get_uint32(data, &ind) != dualadc_cfg_signature()) return false;

    in.v_min = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.v_center = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.v_max = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.deadband = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.sum_target = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.sum_tol = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.v_margin = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.alpha = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.i_max = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.brake_on_fault = buffer_get_float32(data, DUALADC_CFG_FLOAT_SCALE, &ind);
    in.clear_time_ok_ms = buffer_get_int32(data, &ind);
    in.loop_period_ms = buffer_get_int32(data, &ind);

    // Hard validation – reject nonsensical voltage range
    if (!(in.v_min < in.v_max))                            return false;
    if (in.v_center < in.v_min || in.v_center > in.v_max) return false;
    if (in.sum_target < 0.0f || in.sum_target > 6.6f)     return false;
    if (in.sum_tol < 0.0f || in.sum_tol > 2.0f)           return false;
    if (in.v_margin < 0.0f || in.v_margin > 0.5f)         return false;
    if (in.i_max < 0.0f || in.i_max > 300.0f)             return false;
    if (in.brake_on_fault < 0.0f || in.brake_on_fault > 300.0f) return false;

    in.deadband         = clampf(in.deadband, 0.0f, 0.3f);
    in.alpha            = clampf(in.alpha,    0.01f, 1.0f);

    in.loop_period_ms   = clampi(in.loop_period_ms,   1,     20);
    in.clear_time_ok_ms = clampi(in.clear_time_ok_ms, 0, 10000);

    chMtxLock(&g_cfg_mtx);
    g_cfg = in;
    chMtxUnlock(&g_cfg_mtx);

    cfg_store_to_eeprom(&in);
    return true;
}

static volatile bool m_custom_xml_read = false;

static int my_get_cfg_xml(uint8_t **data) {
    // no printf here: called on every FW-version query and XML chunk fetch,
    // and the print latency was causing the Tool's chunk requests to time out.
    m_custom_xml_read = true;

        /*
        static const char xml[] =
                "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
                "<ConfigParams name=\"DualADCJoystick\">"
                "<Params>"
                " <hw_name><longName>Dual ADC Joystick</longName><type>0</type><transmittable>0</transmittable></hw_name>"
                " <v_min><longName>ADC Min (V)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>2</editorDecimalsDouble>"
                    "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>3.3</maxDouble></v_min>"
                " <v_center><longName>ADC Center (V)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>2</editorDecimalsDouble>"
                    "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>3.3</maxDouble></v_center>"
                " <v_max><longName>ADC Max (V)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>2</editorDecimalsDouble>"
                    "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>3.3</maxDouble></v_max>"
                " <deadband><longName>Deadband</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>3</editorDecimalsDouble>"
                    "<stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>0.3</maxDouble></deadband>"
                " <sum_target><longName>Sum Target (V)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>3</editorDecimalsDouble>"
                    "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>6.6</maxDouble></sum_target>"
                " <sum_tol><longName>Sum Tolerance (V)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>3</editorDecimalsDouble>"
                    "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>2</maxDouble></sum_tol>"
                " <v_margin><longName>Margin (V)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>3</editorDecimalsDouble>"
                    "<stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>0.5</maxDouble></v_margin>"
                " <alpha><longName>Filter Alpha</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>3</editorDecimalsDouble>"
                    "<stepDouble>0.001</stepDouble><minDouble>0.01</minDouble><maxDouble>1</maxDouble></alpha>"
                " <i_max><longName>Max Current (A)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>1</editorDecimalsDouble>"
                    "<stepDouble>0.1</stepDouble><minDouble>0</minDouble><maxDouble>300</maxDouble></i_max>"
                " <brake_on_fault><longName>Brake on Fault (A)</longName><type>1</type><vTx>8</vTx><vTxDoubleScale>1000</vTxDoubleScale>"
                    "<editorDecimalsDouble>1</editorDecimalsDouble>"
                    "<stepDouble>0.1</stepDouble><minDouble>0</minDouble><maxDouble>300</maxDouble></brake_on_fault>"
                " <clear_time_ok_ms><longName>Fault Clear Time (ms)</longName><type>2</type><vTx>6</vTx>"
                      "<stepInt>1</stepInt><minInt>0</minInt><maxInt>10000</maxInt></clear_time_ok_ms>"
                " <loop_period_ms><longName>Loop Period (ms)</longName><type>2</type><vTx>6</vTx>"
                      "<stepInt>1</stepInt><minInt>1</minInt><maxInt>20</maxInt></loop_period_ms>"
                "</Params>"
                "<SerOrder>"
                "<ser>v_min</ser><ser>v_center</ser><ser>v_max</ser><ser>deadband</ser>"
                "<ser>sum_target</ser><ser>sum_tol</ser><ser>v_margin</ser><ser>alpha</ser>"
                "<ser>i_max</ser><ser>brake_on_fault</ser><ser>clear_time_ok_ms</ser>"
                "<ser>loop_period_ms</ser>"
                "</SerOrder>"
                "<Grouping><group><groupName>General</groupName><subgroup>"
                "<subgroupName>Dual ADC</subgroupName><subgroupParams>"
                "<param>v_min</param><param>v_center</param><param>v_max</param>"
                "<param>deadband</param><param>sum_target</param><param>sum_tol</param>"
                "<param>v_margin</param><param>alpha</param><param>i_max</param>"
                "<param>brake_on_fault</param><param>clear_time_ok_ms</param>"
                "<param>loop_period_ms</param></subgroupParams></subgroup>"
                "</group></Grouping>"
                "</ConfigParams>";
        */

    static const char compressed_hex[] =
        "00000e4d78dadd57516fdb2010fe2b284fddc3829368551faeaeba44a936ad5ba5a67bb5884d53141b2c6c77e9bf1f60"
        "6c03ce433bad53d71703df9d8eefbbb30f0c178722478f54564cf0f3c96c1a4d10e5a9c818df9d4fee36eb8f67938b18"
        "9682dfb3dd0d91a4a81027053d9fac1a925fae965fc55355b3743f89a135c7081e7e25da27865cf0dd773dd3ce4879a3"
        "ce1d706f83faa9a47104d88c504bc2ab82d535d9e62dec0180bbe8081e9382716717bdc135e3e8e4e78751fc5917ff71"
        "7388cf00eb41cf57a251616f53a262cfa228321617049ab15ac8154d5941f2aa35c573c04771a86a5ada79348dd4ae0e"
        "008a6e67033c2ca02076cb78315d284bbf546c8c46ad35a5bca63290bb34e03b536c959a029343586072786f05561a11"
        "6494645bc233f7abb1d06b6b5d3c4febcbc546a1d85e2482aa29929ac81dad1dc5b74d813606fc17355ebc528d4fa7a7"
        "be6c47ab152ef250b5c8a96a7429fd9f85cf8fc8564adb0f59eebc667d6d8037a4f64f5eef4fe36fd9c84440f2f28138"
        "72d72cd78dfa52c36f5fb0a9fd51cd335f712b13010b3ab5eed2cb464ad5c9d1c9e5ab9778f62cc52f6fd69a802b97d9"
        "66bd95644f13c1937bd2e46e03fbac0d4870b4d686f7243d908c20cd299149cd0a85ee1375f1735e76237ea91dd04639"
        "a093a21a2762ee26e2d42642b3fcc2ebd812d653cd568f2dd5162107e3144596a641f18812529c449994543291f91cbf"
        "2903ba3186bf436f36a23777b98544b0bd2ec32d953f64a66f76957a98fb9e0a3eacdbdb9007a9a0ce3aebaf093d341c"
        "382128f22092ee570e44da06d5af59b099ff1a388630f78ec9976e0d78d07d254553aa1f8e18767a6607538a2bcad5b1"
        "a8380f903a40b7d6af9b79ff18fadcd91e71eff25deab1cb73bbe8b12ed7016c52e06143ce3dd8cdfbd8a0733f8adce6"
        "df836d0d3c8c1d2111d6c2338eebe199c39a58230eb385876ce36e1cea85dd1fc3f837f4f41849";
    static uint8_t compressed_xml[711];
    static bool compressed_xml_ready = false;

    if (!compressed_xml_ready) {
        for (int i = 0; i < (int)sizeof(compressed_xml); i++) {
            uint8_t value = 0;
            for (int nibble = 0; nibble < 2; nibble++) {
                char c = compressed_hex[i * 2 + nibble];
                uint8_t digit = (c >= '0' && c <= '9') ? (uint8_t)(c - '0') :
                        (c >= 'a' && c <= 'f') ? (uint8_t)(c - 'a' + 10) : 0;
                value = (uint8_t)((value << 4) | digit);
            }
            compressed_xml[i] = value;
        }
        compressed_xml_ready = true;
    }

    *data = compressed_xml;
    return (int)sizeof(compressed_xml);
}

// ---------------------- Internal runtime state ------------------

static THD_WORKING_AREA(dual_adc_thread_wa, 1024);
static THD_FUNCTION(dual_adc_thread, arg);

static volatile bool     m_running      = false;
static volatile bool     m_fault_latched = false;
static volatile float    m_pos_f        = 0.0f;
static volatile float    m_v1           = 0.0f;
static volatile float    m_v2           = 0.0f;
static volatile float    m_sum          = 0.0f;
static volatile uint32_t m_ok_since_ms  = 0;

typedef struct {
    bool valid;
    float v1;
    float v2;
    float sum;
} calibration_point_t;

static calibration_point_t m_calibration[3];
static volatile bool m_calibrating = false;

static void calibration_reset(void) {
    memset(m_calibration, 0, sizeof(m_calibration));
    m_calibrating = false;
}

static bool calibration_apply(void) {
    const calibration_point_t *left = &m_calibration[0];
    const calibration_point_t *mid = &m_calibration[1];
    const calibration_point_t *right = &m_calibration[2];

    if (!left->valid || !mid->valid || !right->valid) {
        return false;
    }

    const float v_min = fminf(fminf(left->v1, left->v2),
                              fminf(right->v1, right->v2));
    const float v_max = fmaxf(fmaxf(left->v1, left->v2),
                              fmaxf(right->v1, right->v2));
    const float v_center = 0.5f * (mid->v1 + mid->v2);
    const float sum_target = (left->sum + mid->sum + right->sum) / 3.0f;
    const float sum_min = fminf(fminf(left->sum, mid->sum), right->sum);
    const float sum_max = fmaxf(fmaxf(left->sum, mid->sum), right->sum);
    const float sum_deviation = fmaxf(sum_target - sum_min, sum_max - sum_target);
    const float sum_tol = fmaxf(0.05f, 2.0f * sum_deviation + 0.02f);

    if (!isfinite(v_min) || !isfinite(v_max) || !isfinite(v_center) ||
            !isfinite(sum_target) || !isfinite(sum_tol) ||
            (v_max - v_min) < 0.2f || v_center < v_min || v_center > v_max ||
            sum_tol > 2.0f) {
        return false;
    }

    dualadc_cfg_t cfg = cfg_snapshot();
    cfg.v_min = v_min;
    cfg.v_max = v_max;
    cfg.v_center = v_center;
    cfg.sum_target = sum_target;
    cfg.sum_tol = sum_tol;

    chMtxLock(&g_cfg_mtx);
    g_cfg = cfg;
    chMtxUnlock(&g_cfg_mtx);
    cfg_store_to_eeprom(&cfg);
    calibration_valid_store(true);
    m_calibration_valid = true;

    commands_printf("Calibration applied: min=%.3f center=%.3f max=%.3f sum=%.3f tol=%.3f",
            (double)cfg.v_min, (double)cfg.v_center, (double)cfg.v_max,
            (double)cfg.sum_target, (double)cfg.sum_tol);
    return true;
}

// ---------------------- Logic helpers ---------------------------

static inline bool plaus_ok(float v1, float v2, const dualadc_cfg_t *c) {
    const float sum = v1 + v2;
    const bool sum_ok = fabsf(sum - c->sum_target) <= c->sum_tol;
    const bool v1_ok  = (v1 >= (c->v_min - c->v_margin)) && (v1 <= (c->v_max + c->v_margin));
    const bool v2_ok  = (v2 >= (c->v_min - c->v_margin)) && (v2 <= (c->v_max + c->v_margin));
    return (sum_ok && v1_ok && v2_ok);
}

static void command_stop_fault(const dualadc_cfg_t *c) {
    if (c->brake_on_fault > 0.0f) {
        mc_interface_set_brake_current(c->brake_on_fault);
    } else {
        mc_interface_set_current(0.0f);
    }
    m_fault_latched = true;
}

/*
 * Normalize joystick position to [-1..1].
 *
 * A small bias derived from (v1+v2)/2 vs v_center compensates for common-mode
 * drift (both channels shift together). Bias is clamped to ±v_margin so it
 * cannot override a real plausibility fault.
 *
 * bias is intentionally small (drift compensation only); it does NOT
 * fight the plausibility window because it is bounded by v_margin.
 */
static inline float norm_pos(float v1, float v2, const dualadc_cfg_t *c) {
    const float span = c->v_max - c->v_min;
    if (span <= 0.0f) return 0.0f;

    float bias = c->v_center - 0.5f * (v1 + v2);
    bias = clampf(bias, -c->v_margin, c->v_margin);

    const float v1c = v1 + bias;
    const float v2c = v2 - bias;

    float pos = (v1c - v2c) / span;
    pos = clampf(pos, -1.0f, 1.0f);
    if (fabsf(pos) < c->deadband) pos = 0.0f;
    return pos;
}

// ---------------------- Thread ----------------------------------

static THD_FUNCTION(dual_adc_thread, arg) {
    (void)arg;
    chRegSetThreadName("APP_DUAL_ADC");

    systime_t ts = chVTGetSystemTimeX();

    while (m_running) {
        // Take a consistent snapshot of config
        dualadc_cfg_t c = cfg_snapshot();

        const uint16_t lp = (c.loop_period_ms > 0) ? (uint16_t)c.loop_period_ms : 2u;

        // Read raw ADCs and convert to volts
        float v1 = adc_to_volt(ADC_Value[ADC1_IDX]);
        float v2 = adc_to_volt(ADC_Value[ADC2_IDX]);
        m_v1  = v1;
        m_v2  = v2;
        m_sum = v1 + v2;

        const bool ok_now = plaus_ok(v1, v2, &c);

        if (!m_calibration_valid || m_calibrating) {
            mc_interface_set_current(0.0f);
        } else if (!ok_now) {
            m_ok_since_ms = 0;
            command_stop_fault(&c);
        } else {
            if (!m_fault_latched) {
                // Normal operation
                float pos = norm_pos(v1, v2, &c);
                m_pos_f   = m_pos_f + c.alpha * (pos - m_pos_f);
                mc_interface_set_current(m_pos_f * c.i_max);
            } else {
                // Fault latched: require sustained OK before clearing
                if (m_ok_since_ms >= (uint32_t)c.clear_time_ok_ms) {
                    m_fault_latched = false;
                } else {
                    m_ok_since_ms += lp;
                    mc_interface_set_current(0.0f);
                }
            }
        }

        timeout_reset();

        ts += MS2ST(lp);
        chThdSleepUntilWindowed(ts, ts + MS2ST(lp));
    }

    mc_interface_set_current(0.0f);
}

// ---------------------- Terminal commands -----------------------

static void terminal_cmd_dual_adc(int argc, const char **argv) {
    float v1  = m_v1;
    float v2  = m_v2;
    float sum = m_sum;
    float pos = m_pos_f;

    commands_printf("Dual-ADC status:");
    commands_printf("  v1:            %.3f V", (double)v1);
    commands_printf("  v2:            %.3f V", (double)v2);
    commands_printf("  sum:           %.3f V", (double)sum);
    commands_printf("  pos (filt):    %.3f",   (double)pos);
    commands_printf("  xml_read:      %d", m_custom_xml_read ? 1 : 0);
    commands_printf("  fault_latched: %d", m_fault_latched  ? 1 : 0);

    if (argc >= 2 && strcmp(argv[1], "stream") == 0) {
        int loops = 100;
        if (argc >= 3) {
            loops = atoi(argv[2]);
            if (loops <= 0) {
                commands_printf("Invalid loop count, using default = 100");
                loops = 100;
            }
        }
        commands_printf("Streaming %d iterations at 10 Hz...", loops);
        for (int i = 0; i < loops; i++) {
            chThdSleepMilliseconds(100);
            commands_printf("v1=%.3f  v2=%.3f  sum=%.3f  pos=%.3f  fault=%d",
                (double)m_v1, (double)m_v2, (double)m_sum,
                (double)m_pos_f, (int)m_fault_latched);
        }
    }
}

static void terminal_cmd_dual_adc_cal(int argc, const char **argv) {
    if (argc < 2) {
        commands_printf("Usage: dual_adc_cal mid|left|right|status|reset");
        return;
    }

    const char *argument = argv[1];

    if (strcmp(argument, "reset") == 0) {
        calibration_reset();
        calibration_valid_store(false);
        m_calibration_valid = false;
        commands_printf("Calibration reset");
        return;
    }

    if (strcmp(argument, "status") == 0) {
        const char *names[] = {"left", "mid", "right"};
        commands_printf("Calibration active: %d valid: %d", m_calibrating ? 1 : 0,
            m_calibration_valid ? 1 : 0);
        for (int i = 0; i < 3; i++) {
            commands_printf("  %s: %s v1=%.3f v2=%.3f sum=%.3f", names[i],
                    m_calibration[i].valid ? "captured" : "missing",
                    (double)m_calibration[i].v1, (double)m_calibration[i].v2,
                    (double)m_calibration[i].sum);
        }
        return;
    }

    int point = -1;
    if (strcmp(argument, "left") == 0) point = 0;
    if (strcmp(argument, "mid") == 0) point = 1;
    if (strcmp(argument, "right") == 0) point = 2;
    if (point < 0) {
        commands_printf("Usage: dual_adc_cal mid|left|right|status|reset");
        return;
    }

    m_calibrating = true;
    m_calibration[point].v1 = m_v1;
    m_calibration[point].v2 = m_v2;
    m_calibration[point].sum = m_sum;
    m_calibration[point].valid = true;

    commands_printf("Captured %s: v1=%.3f v2=%.3f sum=%.3f", argument,
            (double)m_calibration[point].v1, (double)m_calibration[point].v2,
            (double)m_calibration[point].sum);

    if (m_calibration[0].valid && m_calibration[1].valid && m_calibration[2].valid) {
        if (calibration_apply()) {
            calibration_reset();
        } else {
            commands_printf("Calibration rejected; use status or reset");
            m_calibrating = false;
        }
    } else {
        commands_printf("Capture the remaining points, then calibration is applied automatically");
    }
}

// ---------------------- App hooks -------------------------------

void app_custom_start(void) {
    if (m_running) return;

    // initialise mutex before first use
    chMtxObjectInit(&g_cfg_mtx);
    calibration_reset();

    m_running       = true;
    m_fault_latched = false;
    m_pos_f         = 0.0f;
    m_ok_since_ms   = 0;
    m_calibration_valid = calibration_valid_load();

    // Load config from EEPROM; fall back to defaults if not found / version mismatch
    if (!cfg_read_from_eeprom(&g_cfg)) {
        cfg_load_defaults(&g_cfg);
        cfg_store_to_eeprom(&g_cfg);
    }

    conf_custom_add_config(my_get_cfg, my_set_cfg, my_get_cfg_xml);

    terminal_register_command_callback(
        "dual_adc",
        "dual_adc [stream [N]] – print or stream ADC/pos/fault status",
        "[stream [count]]",
        terminal_cmd_dual_adc
    );

    terminal_register_command_callback(
        "dual_adc_cal",
        "dual_adc_cal mid|left|right|status|reset – calibrate ADC inputs",
        "mid|left|right|status|reset",
        terminal_cmd_dual_adc_cal
    );

    chThdCreateStatic(dual_adc_thread_wa, sizeof(dual_adc_thread_wa),
                      NORMALPRIO, dual_adc_thread, NULL);
}

void app_custom_stop(void) {
    if (!m_running) return;

    m_running = false;
    m_calibrating = false;
    mc_interface_set_current(0.0f);

    // Give the thread one period to exit cleanly
    chMtxLock(&g_cfg_mtx);
    int32_t lp = g_cfg.loop_period_ms;
    chMtxUnlock(&g_cfg_mtx);
    chThdSleepMilliseconds((lp > 0) ? (uint32_t)(lp + 1) : 3u);

    mc_interface_set_current(0.0f);

    conf_custom_clear_configs();

    terminal_unregister_callback(terminal_cmd_dual_adc);
    terminal_unregister_callback(terminal_cmd_dual_adc_cal);
}

void app_custom_configure(app_configuration *conf) {
    (void)conf;
    // Configuration is handled exclusively via the custom-config channel.
    // This hook exists only to satisfy the app interface contract.
}