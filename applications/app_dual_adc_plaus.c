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
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "terminal.h"     // terminal_register_command_callback / unregister
#include "commands.h"     // commands_printf, etc.
#include "conf_custom.h"  // custom config channel (UI + transport)
#include "conf_general.h" // EEPROM custom var helpers
#include "buffer.h"       // pack/unpack helpers (included for completeness)

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
        memcpy(data, &tmp, sizeof(dualadc_cfg_t));
    } else {
        chMtxLock(&g_cfg_mtx);
        memcpy(data, &g_cfg, sizeof(dualadc_cfg_t));
        chMtxUnlock(&g_cfg_mtx);
    }
    return (int)sizeof(dualadc_cfg_t);
}

// Accept new config from VESC Tool: validate, apply (mutex-protected), persist
static bool my_set_cfg(uint8_t *data) {
    if (!data) return false;

    dualadc_cfg_t in;
    memcpy(&in, data, sizeof(dualadc_cfg_t));

    // Hard validation – reject nonsensical voltage range
    if (!(in.v_min < in.v_max))                            return false;
    if (in.v_center < in.v_min || in.v_center > in.v_max) return false;

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
    commands_printf("DEBUG: XML config requested by Tool");
    m_custom_xml_read = true;

    // Field order must exactly match struct member order (after magic, which is
    // internal and not exposed in the UI).
    static const char xml[] =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        "<ConfigParams name=\"DualADCJoystick\">"
        "<Params>"
        " <v_min><longName>ADC Min (V)</longName><type>1</type>"
          "<editorDecimalsDouble>2</editorDecimalsDouble>"
          "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>3.3</maxDouble></v_min>"
        " <v_center><longName>ADC Center (V)</longName><type>1</type>"
          "<editorDecimalsDouble>2</editorDecimalsDouble>"
          "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>3.3</maxDouble></v_center>"
        " <v_max><longName>ADC Max (V)</longName><type>1</type>"
          "<editorDecimalsDouble>2</editorDecimalsDouble>"
          "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>3.3</maxDouble></v_max>"
        " <deadband><longName>Deadband</longName><type>1</type>"
          "<editorDecimalsDouble>3</editorDecimalsDouble>"
          "<stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>0.3</maxDouble></deadband>"
        " <sum_target><longName>Sum Target (V)</longName><type>1</type>"
          "<editorDecimalsDouble>3</editorDecimalsDouble>"
          "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>6.6</maxDouble></sum_target>"
        " <sum_tol><longName>Sum Tolerance (V)</longName><type>1</type>"
          "<editorDecimalsDouble>3</editorDecimalsDouble>"
          "<stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>2</maxDouble></sum_tol>"
        " <v_margin><longName>Margin (V)</longName><type>1</type>"
          "<editorDecimalsDouble>3</editorDecimalsDouble>"
          "<stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>0.5</maxDouble></v_margin>"
        " <alpha><longName>Filter Alpha</longName><type>1</type>"
          "<editorDecimalsDouble>3</editorDecimalsDouble>"
          "<stepDouble>0.001</stepDouble><minDouble>0.01</minDouble><maxDouble>1</maxDouble></alpha>"
        " <i_max><longName>Max Current (A)</longName><type>1</type>"
          "<editorDecimalsDouble>1</editorDecimalsDouble>"
          "<stepDouble>0.1</stepDouble><minDouble>0</minDouble><maxDouble>300</maxDouble></i_max>"
        " <brake_on_fault><longName>Brake on Fault (A)</longName><type>1</type>"
          "<editorDecimalsDouble>1</editorDecimalsDouble>"
          "<stepDouble>0.1</stepDouble><minDouble>0</minDouble><maxDouble>300</maxDouble></brake_on_fault>"
        " <clear_time_ok_ms><longName>Fault Clear Time (ms)</longName><type>0</type>"
          "<step>1</step><min>0</min><max>10000</max></clear_time_ok_ms>"
        " <loop_period_ms><longName>Loop Period (ms)</longName><type>0</type>"
          "<step>1</step><min>1</min><max>20</max></loop_period_ms>"
        "</Params>"
        "</ConfigParams>";

    *data = (uint8_t *)xml;
    // return byte length WITHOUT null terminator
    return (int)strlen(xml);
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

        if (!ok_now) {
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

    if (argc >= 1 && strcmp(argv[0], "stream") == 0) {
        int loops = 100;
        if (argc >= 2) {
            loops = atoi(argv[1]);
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

/*
 * Calibration helper: capture ADC readings at each joystick extreme and center.
 * Usage: dual_adc_cal left | mid | right
 *
 * Currently logs values only; extend to write into g_cfg and persist if desired.
 *
 * argv[0] is the position argument (left/mid/right)
 */
static void terminal_cmd_dual_adc_cal(int argc, const char **argv) {
    if (argc < 1 ||
        !(strcmp(argv[0], "left")  == 0 ||
          strcmp(argv[0], "mid")   == 0 ||
          strcmp(argv[0], "right") == 0)) {
        commands_printf("Usage: dual_adc_cal left|mid|right");
        return;
    }

    float v1  = m_v1;
    float v2  = m_v2;
    float sum = m_sum;

    commands_printf("Calibrating position: %s", argv[0]);
    commands_printf("  v1:            %.3f V", (double)v1);
    commands_printf("  v2:            %.3f V", (double)v2);
    commands_printf("  sum:           %.3f V", (double)sum);
    commands_printf("  fault_latched: %d",     m_fault_latched ? 1 : 0);

    // TODO: write captured values into g_cfg and call cfg_store_to_eeprom()
    // e.g. for "left":  g_cfg.v_min = v1;  g_cfg.sum_target recalculated, etc.
}

// ---------------------- App hooks -------------------------------

void app_custom_start(void) {
    if (m_running) return;

    // initialise mutex before first use
    chMtxObjectInit(&g_cfg_mtx);

    m_running       = true;
    m_fault_latched = false;
    m_pos_f         = 0.0f;
    m_ok_since_ms   = 0;

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
        "dual_adc_cal left|mid|right – capture ADC voltages at joystick position",
        "left|mid|right",
        terminal_cmd_dual_adc_cal
    );

    chThdCreateStatic(dual_adc_thread_wa, sizeof(dual_adc_thread_wa),
                      NORMALPRIO, dual_adc_thread, NULL);
}

void app_custom_stop(void) {
    m_running = false;

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