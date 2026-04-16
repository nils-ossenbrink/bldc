
/*
 * Dual-ADC joystick with plausibility check for VESC 6/75
 * Author: Nils (extended with VESC 6.06 custom-config & persistence)
 *
 * Reads ADC_EXT and ADC_EXT2, enforces complementary-sum plausibility,
 * outputs current command; on any fault -> stop (or brake).
 *
 * Adds:
 *  - Configurable parameters (min/max/center, deadband, margins, alpha, i_max, etc.)
 *  - Persistent storage in EEPROM (custom region)
 *  - VESC Tool UI via conf_custom (XML supplied by firmware)
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

#include "terminal.h"   // terminal_register_command_callback / unregister
#include "commands.h"   // commands_printf, etc.
#include "conf_custom.h"// custom config channel (UI + transport)
#include "conf_general.h" // EEPROM custom var helpers
#include "buffer.h"     // if you want to pack/unpack; here we use memcpy

// ---------------------- ADC access ------------------------------

// Helper: convert raw ADC sample to volts.
// Most VESC boards use Vref = 3.3V and 12-bit ADC (0..4095).
#ifndef V_ADC_REF
#define V_ADC_REF       3.3f
#endif

static inline float adc_to_volt(uint16_t raw) {
    return (V_ADC_REF * (float)raw) / 4095.0f;
}

// Which ADC indices to read (default: external ADC1/ADC2)
#ifndef ADC1_IDX
#define ADC1_IDX        ADC_IND_EXT
#endif
#ifndef ADC2_IDX
#define ADC2_IDX        ADC_IND_EXT2
#endif

// Helper functions

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


// ---------------------- App configuration -----------------------

typedef struct {
    float v_min;          // [V] expected minimum per channel
    float v_center;       // [V] expected center (idle) average
    float v_max;          // [V] expected maximum per channel
    float deadband;       // [-] deadband around 0 after normalization
    float sum_target;     // [V] expected v1+v2
    float sum_tol;        // [V] ± tolerance on sum plausibility
    float v_margin;       // [V] extra window margin per channel
    float alpha;          // [-] low-pass filter weight (0..1)
    float i_max;          // [A] max torque (current) at |pos|=1
    float brake_on_fault; // [A] if >0, active brake on fault
    int32_t clear_time_ok_ms;  // ms inputs must be OK to clear latched fault
    int32_t  loop_period_ms;    // main loop period (1..20 ms typical)
} dualadc_cfg_t;

static void cfg_load_defaults(dualadc_cfg_t *c) {
    c->v_min           = 0.32f;
    c->v_center        = 1.53f;
    c->v_max           = 2.69f;

    c->deadband        = 0.2f;

    // If your sensors are truly complementary across the whole range,
    // sum should be ~ v_min + v_max. Adjust if your hardware differs.
    c->sum_target      = 2.92f;    // e.g. 0.50 + 2.50
    c->sum_tol         = 0.3f;

    c->v_margin        = 0.05f;
    c->alpha           = 0.12f;
    c->i_max           = 3.0f;
    c->brake_on_fault  = 0.0f;

    c->clear_time_ok_ms = 50;
    c->loop_period_ms   = 2;
}

static dualadc_cfg_t g_cfg;

// ---------------------- Persistence (custom EEPROM) --------------

/*
 * We store the whole struct as a raw byte blob across consecutive
 * custom EEPROM "variables". Each slot is 32 bits; we write N words.
 */

static void cfg_store_to_eeprom(const dualadc_cfg_t *c) {
    uint8_t buf[sizeof(dualadc_cfg_t)];
    memcpy(buf, c, sizeof(dualadc_cfg_t));

    const int words = (sizeof(dualadc_cfg_t) + 3) / 4;

    eeprom_var v;
    for (int i = 0; i < words; i++) {
        uint32_t w = 0;
        memcpy(&w, buf + (i * 4), 4);
        v.as_u32 = w;
        // Use addresses [0..words-1] in the custom area.
        // There are up to 64 available; this struct uses ~11.
        conf_general_store_eeprom_var_custom(&v, i);
    }
}

static bool cfg_read_from_eeprom(dualadc_cfg_t *c) {
    uint8_t buf[sizeof(dualadc_cfg_t)];
    memset(buf, 0, sizeof(buf));

    const int words = (sizeof(dualadc_cfg_t) + 3) / 4;

    eeprom_var v;
    for (int i = 0; i < words; i++) {
        if (!conf_general_read_eeprom_var_custom(&v, i)) {
            return false; // not programmed yet
        }
        memcpy(buf + (i * 4), &v.as_u32, 4);
    }

    memcpy(c, buf, sizeof(dualadc_cfg_t));
    return true;
}

// ---------------------- Custom-config (UI + transport) ----------

/*
 * VESC Tool queries these to show a dynamic "Custom" page and to
 * send/receive our config blob.
 */

// Return current (or default) config as bytes
static int my_get_cfg(uint8_t *data, bool is_default) {
    dualadc_cfg_t tmp;
    const dualadc_cfg_t *src = &g_cfg;
    if (is_default) {
        cfg_load_defaults(&tmp);
        src = &tmp;
    }
    memcpy(data, src, sizeof(dualadc_cfg_t));
    return (int)sizeof(dualadc_cfg_t);
}

// Accept new config, validate, apply, persist
static bool my_set_cfg(uint8_t *data) {
    if (!data) return false;

    dualadc_cfg_t in;
    memcpy(&in, data, sizeof(dualadc_cfg_t));

    // Basic validation / clamping
    if (!(in.v_min < in.v_max))                           return false;
    if (in.v_center < in.v_min || in.v_center > in.v_max) return false;
    
    in.deadband = clampf(in.deadband, 0.0f, 0.3f);
    in.alpha    = clampf(in.alpha,    0.0f, 1.0f);

    in.loop_period_ms = clampf((float)in.loop_period_ms, 1.0f, 20.0f);

    if (in.loop_period_ms < 1)                            in.loop_period_ms = 1;
    if (in.loop_period_ms > 20)                           in.loop_period_ms = 20;

    g_cfg = in;
    cfg_store_to_eeprom(&g_cfg);
    return true;
}

static volatile bool   m_custom_xml_read = false;

static int my_get_cfg_xml(uint8_t **data) {
    commands_printf("DEBUG: XML config requested by Tool"); 
    m_custom_xml_read = true;

    static const char xml[] =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
        "<ConfigParams name=\"DualADCJoystick\">"
        "<Params>"
        " <v_min><longName>ADC Min</longName><type>1</type><editorDecimalsDouble>2</editorDecimalsDouble><stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>5</maxDouble></v_min>"
        " <v_center><longName>ADC Center</longName><type>1</type><editorDecimalsDouble>2</editorDecimalsDouble><stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>5</maxDouble></v_center>"
        " <v_max><longName>ADC Max</longName><type>1</type><editorDecimalsDouble>2</editorDecimalsDouble><stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>5</maxDouble></v_max>"
        " <deadband><longName>Deadband</longName><type>1</type><editorDecimalsDouble>3</editorDecimalsDouble><stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>1</maxDouble></deadband>"
        " <sum_target><longName>Sum Target</longName><type>1</type><editorDecimalsDouble>3</editorDecimalsDouble><stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>10</maxDouble></sum_target>"
        " <sum_tol><longName>Sum Tol</longName><type>1</type><editorDecimalsDouble>3</editorDecimalsDouble><stepDouble>0.01</stepDouble><minDouble>0</minDouble><maxDouble>5</maxDouble></sum_tol>"
        " <v_margin><longName>Margin</longName><type>1</type><editorDecimalsDouble>3</editorDecimalsDouble><stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>5</maxDouble></v_margin>"
        " <alpha><longName>Alpha</longName><type>1</type><editorDecimalsDouble>3</editorDecimalsDouble><stepDouble>0.001</stepDouble><minDouble>0</minDouble><maxDouble>1</maxDouble></alpha>"
        " <i_max><longName>Max Current</longName><type>1</type><editorDecimalsDouble>1</editorDecimalsDouble><stepDouble>0.1</stepDouble><minDouble>0</minDouble><maxDouble>300</maxDouble></i_max>"
        " <brake_on_fault><longName>Brake Fault</longName><type>1</type><editorDecimalsDouble>1</editorDecimalsDouble><stepDouble>0.1</stepDouble><minDouble>0</minDouble><maxDouble>300</maxDouble></brake_on_fault>"
        " <clear_time_ok_ms><longName>Clear Time ms</longName><type>0</type><step>1</step><min>0</min><max>10000</max></clear_time_ok_ms>"
        " <loop_period_ms><longName>Loop Period ms</longName><type>0</type><step>1</step><min>1</min><max>1000</max></loop_period_ms>"
        "</Params>"
        "</ConfigParams>";

    *data = (uint8_t*)xml;
    return strlen(xml) + 1;
}

// ---------------------- Internal state --------------------------

static THD_WORKING_AREA(dual_adc_thread_wa, 1024);
static THD_FUNCTION(dual_adc_thread, arg);

static volatile bool   m_running = false;
static volatile bool   m_fault_latched = false;
static volatile float  m_pos_f = 0.0f;
static volatile float  m_v1 = 0.0f, m_v2 = 0.0f;
static volatile float  m_sum = 0.0f;
static volatile uint32_t m_ok_since_ms = 0;

// ---------------------- Logic helpers ---------------------------

static inline bool plaus_ok(float v1, float v2) {
    const float sum = v1 + v2;

    const bool sum_ok =
        fabsf(sum - g_cfg.sum_target) <= g_cfg.sum_tol;

    const bool v1_ok =
        (v1 >= (g_cfg.v_min - g_cfg.v_margin)) && (v1 <= (g_cfg.v_max + g_cfg.v_margin));
    const bool v2_ok =
        (v2 >= (g_cfg.v_min - g_cfg.v_margin)) && (v2 <= (g_cfg.v_max + g_cfg.v_margin));

    return (sum_ok && v1_ok && v2_ok);
}

static void command_stop_fault(void) {
    if (g_cfg.brake_on_fault > 0.0f) {
        mc_interface_set_brake_current(g_cfg.brake_on_fault);
    } else {
        mc_interface_set_current(0.0f);
    }
    m_fault_latched = true;
}

/*
 * Normalize joystick using complementary pair with small center bias correction.
 * We derive a small bias from the average (v1+v2)/2 vs configured v_center
 * to keep "zero" stable if the pair drifts together. Bias is clamped to v_margin.
 *
 * pos in [-1..1] ideally: full fwd -> +1, full rev -> -1
 */
static inline float norm_pos(float v1, float v2) {
    const float span = (g_cfg.v_max - g_cfg.v_min);
    float pos;

    if (span <= 0.0f) {
        return 0.0f;
    }

    // Bias to re-center around v_center without fighting plausibility windows
    float bias = g_cfg.v_center - 0.5f * (v1 + v2);
    
    bias = clampf(bias, -g_cfg.v_margin, g_cfg.v_margin);


    const float v1c = v1 + bias;
    const float v2c = v2 - bias;

    pos = (v1c - v2c) / span;

    // clamp, deadband
    pos  = clampf(pos,  -1.0f, 1.0f);
    if (fabsf(pos) < g_cfg.deadband) pos = 0.0f;

    return pos;
}

// ---------------------- Thread ---------------------------------

static THD_FUNCTION(dual_adc_thread, arg) {
    (void)arg;
    chRegSetThreadName("APP_DUAL_ADC");

    systime_t ts = chVTGetSystemTimeX();

    while (m_running) {
        // Read raw ADCs
        uint16_t r1 = ADC_Value[ADC1_IDX];
        uint16_t r2 = ADC_Value[ADC2_IDX];

         // loop timing
        const uint16_t lp = (g_cfg.loop_period_ms > 0) ? g_cfg.loop_period_ms : 2;

        // Convert to volts
        float v1 = adc_to_volt(r1);
        float v2 = adc_to_volt(r2);
        m_v1 = v1; m_v2 = v2;
        m_sum = v1 + v2;

        const bool ok_now = plaus_ok(v1, v2);

        if (!ok_now) {
            m_ok_since_ms = 0;
            command_stop_fault();
        } else {
            if (!m_fault_latched) {
                // normal operation
                float pos = norm_pos(v1, v2);
                // low-pass filter position
                m_pos_f = m_pos_f + g_cfg.alpha * (pos - m_pos_f);
                //mc_interface_set_duty(m_pos_f);
                mc_interface_set_current(m_pos_f * g_cfg.i_max);
            } else {
                // Fault latched; require sustained OK before clearing
                if (m_ok_since_ms >= g_cfg.clear_time_ok_ms) {
                    m_fault_latched = false;
                } else {
                    m_ok_since_ms += g_cfg.loop_period_ms;
                    mc_interface_set_current(0.0f);
                }
            }
        }

        // Keep the firmware watchdog happy
        timeout_reset();

        ts += MS2ST(lp);
        chThdSleepUntilWindowed(ts, ts + MS2ST(lp));
    }

    // On exit, ensure motor is released
    mc_interface_set_current(0.0f);
}

// ---------------------- Terminal command ------------------------

static void terminal_cmd_dual_adc(int argc, const char **argv) {
    float v1  = m_v1;
    float v2  = m_v2;
    float sum = m_sum;
    float pos = m_pos_f;

    commands_printf("Dual-ADC status:");
    commands_printf("  v1:  %.3f V", (double)v1);
    commands_printf("  v2:  %.3f V", (double)v2);
    commands_printf("  sum: %.3f V", (double)sum);
    commands_printf("  pos: %.3f V", (double)m_pos_f);
    if (m_custom_xml_read) {
        commands_printf("  Custom_xml_read successfully");
    }
    else {
        commands_printf("  Custom_xml_ not read");
    }
    commands_printf("  fault_latched: %d", m_fault_latched ? 1 : 0);

    if (argc >= 2 && strcmp(argv[1], "stream") == 0) {

        // Default number of iterations
        int loops = 100;

        // If third argument = loop count
        if (argc == 3) {
            loops = atoi(argv[2]);
            if (loops <= 0) {
                commands_printf("Invalid loop count, using default = 100");
                loops = 100;
            }
        }

        commands_printf("Streaming %d iterations at 10 Hz...", loops);

        for (int i = 0; i < loops; i++) {
            chThdSleepMilliseconds(100);
            v1  = m_v1;
            v2  = m_v2;
            sum = m_sum;
            pos = m_pos_f;
            commands_printf("v1=%.3f  v2=%.3f  sum=%.3f  pos=%.3f  fault=%d",
                (double)v1, (double)v2, (double)sum, (double)pos,
                (int)m_fault_latched);
        }
    }
}

static void terminal_cmd_dual_adc_cal(int argc, const char **argv) {
    float v1  = m_v1;
    float v2  = m_v2;
    float sum = m_sum;

    if (argc != 2 ||
        !(strcmp(argv[1], "left") == 0 || strcmp(argv[1], "mid") == 0 || strcmp(argv[1], "right") == 0)) {
        commands_printf("No or wrong argument given. Usage: dual_adc_cal left|mid|right ");
        return;
    }

    commands_printf("Calibrating %c", argv[1]);
    commands_printf("  v1:  %.3f V", (double)v1);
    commands_printf("  v2:  %.3f V", (double)v2);
    commands_printf("  sum: %.3f V", (double)sum);
    commands_printf("  fault_latched: %d", m_fault_latched ? 1 : 0);
}

// ---------------------- App hooks -------------------------------

void app_custom_start(void) {
    if (m_running) return;
    m_running = true;
    m_fault_latched = false;
    m_pos_f = 0.0f;
    m_ok_since_ms = 0;

    // Load config (from EEPROM or defaults) and register custom-config UI
    if (!cfg_read_from_eeprom(&g_cfg)) {
        cfg_load_defaults(&g_cfg);
        cfg_store_to_eeprom(&g_cfg);
    }
    conf_custom_add_config(my_get_cfg, my_set_cfg, my_get_cfg_xml);

    // Register terminal command:
    terminal_register_command_callback(
        "dual_adc",
        "dual_adc [now|stream] - Print or stream ADC values",
        "[now|stream]",
        terminal_cmd_dual_adc
    );

    chThdCreateStatic(dual_adc_thread_wa, sizeof(dual_adc_thread_wa),
                      NORMALPRIO, dual_adc_thread, NULL);
}

void app_custom_stop(void) {
    m_running = false;
    // give thread one period to exit cleanly
    chThdSleepMilliseconds((g_cfg.loop_period_ms > 0) ? (g_cfg.loop_period_ms + 1) : 3);
    mc_interface_set_current(0.0f);

    terminal_unregister_callback(terminal_cmd_dual_adc);
}

void app_custom_configure(app_configuration *conf) {
    (void)conf;
    // Not used: configuration is handled by the custom-config channel.
    // Keep this hook to satisfy the app interface.
}
