/*
 * Dual-ADC joystick with plausibility check for VESC 6/75
 * Author: Nils
 *
 * Reads ADC_EXT and ADC_EXT2, enforces complementary-sum plausibility,
 * outputs current command; on any fault -> stop (or brake).
 *
 * Sources used for API/structure:
 * - app_adc.c and ADC indices (ADC_IND_EXT/ADC_IND_EXT2)
 * - Custom app hooks: app_custom_start/stop/configure
 * - mc_interface_* motor control functions
 */

#pragma GCC optimize ("Os")

#include "ch.h"
#include "hal.h"
#include "app.h"
#include "mc_interface.h"
#include "timeout.h"
#include "utils_math.h"
#include "hw.h"
#include <math.h>
#include <string.h>

#include "terminal.h"   // for terminal_register_command_callback / unregister
#include "commands.h"   // for com



// ---------------------- User configuration ----------------------

// Expected joystick channel range (on the ESC ADC pins, in volts)
#define V_MIN           0.50f
#define V_MAX           2.50f

// Complementary-sum plausibility target and tolerance (in volts)
#define SUM_TARGET_V    3.00f   // If you truly need 1.50 V, change here
#define SUM_TOL_V       0.15f   // ±150 mV window

// Extra window margin per channel (in volts)
#define V_MARGIN        0.05f

// Command shaping
#define DEAD_BAND       0.03f   // deadband on normalized position [-1..1]
#define ALPHA           0.12f   // low-pass filter (0..1), higher = less filtering
#define I_MAX_A         30.0f   // max motor current command [A] at pos=±1
#define BRAKE_ON_FAULT_A 0.0f   // set >0 to apply active brake on fault

// Latching and timing
#define CLEAR_TIME_OK_MS  50    // how long inputs must be OK to clear a latched fault
#define LOOP_PERIOD_MS     2    // ~500 Hz (matches stock ADC app cadence)

// Which ADC indices to read (default: external ADC1/ADC2)
#ifndef ADC1_IDX
#define ADC1_IDX        ADC_IND_EXT
#endif
#ifndef ADC2_IDX
#define ADC2_IDX        ADC_IND_EXT2
#endif

// ---------------------- Internal state --------------------------

static THD_WORKING_AREA(dual_adc_thread_wa, 1024);
static THD_FUNCTION(dual_adc_thread, arg);

static volatile bool   m_running = false;
static volatile bool   m_fault_latched = false;
static volatile float  m_pos_f = 0.0f;
static volatile float  m_v1 = 0.0f, m_v2 = 0.0f;
static volatile float  m_sum = 0.0f;
static volatile uint32_t m_ok_since_ms = 0;

// Helper: convert raw ADC sample to volts.
// Many VESC boards use Vref = 3.3V and 12-bit ADC (0..4095).
// If your hardware scales differently, adjust V_ADC_REF.
#define V_ADC_REF       3.3f
static inline float adc_to_volt(uint16_t raw) {
    return (V_ADC_REF * (float)raw) / 4095.0f;
}

// Check plausibility: sum ~ SUM_TARGET_V and each channel in window
static inline bool plaus_ok(float v1, float v2) {
    const float sum = v1 + v2;
    const bool sum_ok =
        fabsf(sum - SUM_TARGET_V) <= SUM_TOL_V;

    const bool v1_ok =
        (v1 >= (V_MIN - V_MARGIN)) && (v1 <= (V_MAX + V_MARGIN));
    const bool v2_ok =
        (v2 >= (V_MIN - V_MARGIN)) && (v2 <= (V_MAX + V_MARGIN));

    return (sum_ok && v1_ok && v2_ok);
}

static void command_stop_fault(void) {
    // Immediately command zero (or brake) and latch fault
    if (BRAKE_ON_FAULT_A > 0.0f) {
        mc_interface_set_brake_current(BRAKE_ON_FAULT_A);
    } else {
        mc_interface_set_current(0.0f);
    }
    m_fault_latched = true;
}

// Normalize joystick: complementary pair mapping
// For 0.5..2.5V each with opposite slopes, (v1 - v2)/(V_MAX - V_MIN) => [-1..1]
static inline float norm_pos(float v1, float v2) {
    float p = (v1 - v2) / (V_MAX - V_MIN);
    // clamp
    if (p > 1.0f) p = 1.0f;
    if (p < -1.0f) p = -1.0f;
    // deadband
    if (fabsf(p) < DEAD_BAND) p = 0.0f;
    return p;
}

static THD_FUNCTION(dual_adc_thread, arg) {
    (void)arg;
    chRegSetThreadName("APP_DUAL_ADC");

    systime_t ts = chVTGetSystemTimeX();

    while (m_running) {
        // Read raw ADCs
        uint16_t r1 = ADC_Value[ADC1_IDX];
        uint16_t r2 = ADC_Value[ADC2_IDX];

        // Convert to volts
        float v1 = adc_to_volt(r1);
        float v2 = adc_to_volt(r2);
        m_v1 = v1; m_v2 = v2;
        m_sum = v1 + v2;

        const bool ok_now = plaus_ok(v1, v2);

        // Fault latching logic
        if (!ok_now) {
            m_ok_since_ms = 0;
            command_stop_fault();
        } else {
            // OK this cycle
            if (!m_fault_latched) {
                // normal operation: compute and command
                float pos = norm_pos(v1, v2);
                // low-pass filter position
                m_pos_f = m_pos_f + ALPHA * (pos - m_pos_f);
                mc_interface_set_current(m_pos_f * I_MAX_A);
            } else {
                // Fault was latched; require sustained OK before clearing
                if (m_ok_since_ms >= CLEAR_TIME_OK_MS) {
                    m_fault_latched = false;
                } else {
                    m_ok_since_ms += LOOP_PERIOD_MS;
                    // keep output safe while latched
                    mc_interface_set_current(0.0f);
                }
            }
        }

        // Keep the firmware watchdog happy
        timeout_reset();

        // ~500 Hz loop
        ts += MS2ST(LOOP_PERIOD_MS);
        chThdSleepUntilWindowed(ts, ts + MS2ST(LOOP_PERIOD_MS));
    }

    // On exit, ensure motor is released
    mc_interface_set_current(0.0f);
}


// Prints v1, v2, sum once (NOW) or streams at 10 Hz (STREAM)
static void terminal_cmd_dual_adc(int argc, const char **argv) {
    float v1  = m_v1;
    float v2  = m_v2;
    float sum = m_sum;

    commands_printf("Dual-ADC status:");
    commands_printf("  v1:  %.3f V", (double)v1);
    commands_printf("  v2:  %.3f V", (double)v2);
    commands_printf("  sum: %.3f V", (double)sum);
    commands_printf("  fault_latched: %d", m_fault_latched ? 1 : 0);

    // Optional streaming mode
    if (argc >= 1 && strcmp(argv[0], "stream") == 0) {
        commands_printf("Streaming at 10 Hz... CTRL+C to stop");

        while (1) {
            chThdSleepMilliseconds(100);

            v1  = m_v1;
            v2  = m_v2;
            sum = m_sum;

            commands_printf("v1=%.3f  v2=%.3f  sum=%.3f  fault=%d",
                (double)v1, (double)v2, (double)sum,
                (int)m_fault_latched);
        }
    }
}




// ---------------------- App hooks -------------------------------

void app_custom_start(void) {
    if (m_running) return;
    m_running = true;
    m_fault_latched = false;
    m_pos_f = 0.0f;
    m_ok_since_ms = 0;
    
    // Register terminal command:
    terminal_register_command_callback(
        "dual_adc",
        "dual_adc [now|stream] - Print or stream ADC values",
        "mode",
        terminal_cmd_dual_adc
    );

    chThdCreateStatic(dual_adc_thread_wa, sizeof(dual_adc_thread_wa),
                      NORMALPRIO, dual_adc_thread, NULL);
}

void app_custom_stop(void) {
    m_running = false;
    // give thread one period to exit cleanly
    chThdSleepMilliseconds(LOOP_PERIOD_MS + 1);
    mc_interface_set_current(0.0f);

    terminal_unregister_callback(terminal_cmd_dual_adc);
}

void app_custom_configure(app_configuration *conf) {
    (void)conf;
    // If you want to expose parameters via VESC Tool later,
    // parse conf->app_custom_conf here.
}