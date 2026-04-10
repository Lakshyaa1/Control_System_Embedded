/*
 * fsm_control.c
 *
 * FSM (Fast Steering Mirror) control system implementation.
 *
 * This file owns the complete signal chain run every 100 µs:
 *   1. Normalise sensor readings
 *   2. Fade detection (QPD signal loss)
 *   3. PAT state machine (IDLE → SCANNING → TRACKING → FAULT)
 *   4. Notch filter (biquad, 1500 Hz resonance suppression)
 *   5. PID controller (with integrator freeze during fade)
 *   6. DAC output conversion
 *
 * Architecture note:
 *   FSMControl_Update() is called from TIM2_IRQHandler with
 *   raw ADC values already read. It must complete in < 16 µs.
 *   All operations here are pure arithmetic — no HAL calls,
 *   no blocking waits, no dynamic allocation.
 */

#include "fsm_control.h"
#include <math.h>     /* sinf, cosf — available from math.h, FPU-accelerated */
#include <stddef.h>

/* =========================================================
 * Global context instance
 * ========================================================= */
FSMControlContext g_ctrl;

/* =========================================================
 * Static helpers
 * ========================================================= */

/* Clamp float to [lo, hi] */
static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* =========================================================
 * FSMControl_Init
 * =========================================================
 * Sets up PID controllers, biquad notch filters, and zeros
 * all state. Call once in main() before enabling TIM2.
 */
void FSMControl_Init(void)
{
    /* ---- PID: X axis ---- */
    PIDController_Init(&g_ctrl.pid_x);

    /* ---- PID: Y axis ---- (same gains, independent state) */
    PIDController_Init(&g_ctrl.pid_y);

    /* ---- Notch filters ----
     * Coefficients from bilinear transform (see header comments):
     *   fn = 1500 Hz,  fs = 10000 Hz,  Q = 5
     *   b0 =  0.92515351
     *   b1 = -1.08758318
     *   b2 =  0.92515351
     *   a1 = -1.08758318
     *   a2 =  0.85030702
     */
    biquad_init(&g_ctrl.notch_x, NOTCH_B0, NOTCH_B1, NOTCH_B2, NOTCH_A1, NOTCH_A2);
    biquad_init(&g_ctrl.notch_y, NOTCH_B0, NOTCH_B1, NOTCH_B2, NOTCH_A1, NOTCH_A2);

    /* ---- State machine ---- */
    g_ctrl.state = PAT_STATE_IDLE;

    /* ---- Sensor and output defaults ---- */
    g_ctrl.qpd_x       = 0.0f;
    g_ctrl.qpd_y       = 0.0f;
    g_ctrl.beam_power  = 0.0f;
    g_ctrl.dac_x       = 0.0f;
    g_ctrl.dac_y       = 0.0f;
    g_ctrl.dac_x_hold  = 0.0f;
    g_ctrl.dac_y_hold  = 0.0f;

    /* ---- Fade state ---- */
    g_ctrl.fade_active  = 0;
    g_ctrl.fade_samples = 0;
    g_ctrl.fade_total   = 0;

    /* ---- Spiral scan state ---- */
    g_ctrl.spiral_r      = 0.0f;
    g_ctrl.spiral_t      = 0.0f;
    g_ctrl.scan_timeout  = 0;

    g_ctrl.new_data_ready = 0;
    g_ctrl.cycle_count    = 0;
}

/* =========================================================
 * FSMControl_Update — THE HOT PATH
 * =========================================================
 * Called from TIM2_IRQHandler every 100 µs.
 * Complete within < 16 µs to leave slack for DMA and overhead.
 *
 * Argument:
 *   adc_qpd_x  — 12-bit ADC value 0..4095 for QPD X channel
 *   adc_qpd_y  — 12-bit ADC value 0..4095 for QPD Y channel
 *   adc_power  — 12-bit ADC for total beam power (0 = disabled)
 */
void FSMControl_Update(uint16_t adc_qpd_x,
                       uint16_t adc_qpd_y,
                       uint16_t adc_power)
{
    g_ctrl.cycle_count++;

    /* ============================================================
     * STEP 1: Normalise sensor readings
     *   ADC range: 0..4095 (12-bit)
     *   Midpoint (beam centred):  2048
     *   Normalised range: -1.0 to +1.0
     * ============================================================ */
    g_ctrl.qpd_x = ((float)adc_qpd_x - (float)QPD_ADC_MIDPOINT)
                   / QPD_FULL_SCALE;
    g_ctrl.qpd_y = ((float)adc_qpd_y - (float)QPD_ADC_MIDPOINT)
                   / QPD_FULL_SCALE;

    /* Beam power: normalised 0..1 (0 = no beam, 1 = full power) */
    g_ctrl.beam_power = (float)adc_power / 4095.0f;

    /* ============================================================
     * STEP 2: Fade detection
     *   Primary criterion: total beam power < threshold
     *   Debounced by FADE_CONFIRM_SAMPLES to avoid false triggers
     * ============================================================ */
    if (g_ctrl.beam_power < FADE_POWER_THRESHOLD)
    {
        g_ctrl.fade_samples++;

        if (g_ctrl.fade_samples >= FADE_CONFIRM_SAMPLES)
        {
            if (!g_ctrl.fade_active)
            {
                /* Transition INTO fade: save current output */
                g_ctrl.fade_active  = 1;
                g_ctrl.fade_total   = 0;
                g_ctrl.dac_x_hold   = g_ctrl.dac_x;
                g_ctrl.dac_y_hold   = g_ctrl.dac_y;

                /* Tell PID to freeze integrators */
                g_ctrl.pid_x.fade_active = 1;
                g_ctrl.pid_y.fade_active = 1;
            }
            g_ctrl.fade_total++;
        }
    }
    else
    {
        /* Power is OK */
        g_ctrl.fade_samples = 0;

        if (g_ctrl.fade_active)
        {
            /* Transition OUT of fade: unfreeze integrators */
            g_ctrl.fade_active = 0;
            g_ctrl.fade_total  = 0;
            g_ctrl.pid_x.fade_active = 0;
            g_ctrl.pid_y.fade_active = 0;
        }
    }

    /* ============================================================
     * STEP 3: PAT State Machine
     * ============================================================ */
    switch (g_ctrl.state)
    {
        /* ----------------------------------------------------------
         * IDLE: system on, motors at neutral, waiting for command
         * ---------------------------------------------------------- */
        case PAT_STATE_IDLE:
        {
            g_ctrl.dac_x = 0.0f;
            g_ctrl.dac_y = 0.0f;

            /* Transition to SCANNING when beam power is detected */
            if (g_ctrl.beam_power >= FADE_POWER_THRESHOLD)
            {
                g_ctrl.state        = PAT_STATE_SCANNING;
                g_ctrl.spiral_r     = 0.0f;
                g_ctrl.spiral_t     = 0.0f;
                g_ctrl.scan_timeout = 0;
            }
            break;
        }

        /* ----------------------------------------------------------
         * SCANNING: spiral search for beacon
         *   x[n] = r[n] * cos(omega * t[n])
         *   y[n] = r[n] * sin(omega * t[n])
         *   r grows linearly each sample
         *
         * Transition to TRACKING when QPD error is small
         *   (beam is roughly centred — PID can take over)
         * Transition to IDLE on timeout (beacon not found)
         * ---------------------------------------------------------- */
        case PAT_STATE_SCANNING:
        {
            g_ctrl.scan_timeout++;

            /* Update spiral */
            g_ctrl.spiral_t += PID_SAMPLE_TIME;  /* accumulate time */
            g_ctrl.spiral_r += SPIRAL_R_INCREMENT;
            if (g_ctrl.spiral_r > SPIRAL_R_MAX)
                g_ctrl.spiral_r = SPIRAL_R_MAX;  /* cap radius */

            float angle = SPIRAL_OMEGA * g_ctrl.spiral_t;
            g_ctrl.dac_x = g_ctrl.spiral_r * cosf(angle);
            g_ctrl.dac_y = g_ctrl.spiral_r * sinf(angle);

            /* Check if beam has entered QPD field (error small enough) */
            float err_mag = g_ctrl.qpd_x * g_ctrl.qpd_x
                          + g_ctrl.qpd_y * g_ctrl.qpd_y;
            if (err_mag < 0.04f)  /* |error| < 0.2 on both axes */
            {
                /* Reset PID state before engaging */
                PIDController_Init(&g_ctrl.pid_x);
                PIDController_Init(&g_ctrl.pid_y);
                g_ctrl.state = PAT_STATE_TRACKING;
            }

            /* Timeout → give up, return to IDLE */
            if (g_ctrl.scan_timeout > SPIRAL_TIMEOUT_SAMPLES)
            {
                g_ctrl.state = PAT_STATE_IDLE;
            }
            break;
        }

        /* ----------------------------------------------------------
         * TRACKING: closed-loop PID active
         *
         * Signal chain this cycle:
         *   1. Notch filter on error (suppress 1500 Hz resonance)
         *   2. PID update (with integrator freeze if fade_active)
         *   3. Output → DAC
         *
         * During fade:
         *   - Notch + PID still run but integrator is frozen
         *   - Output is overridden to last held value
         *
         * Long fade (> FADE_TIMEOUT) → re-acquire
         * ---------------------------------------------------------- */
        case PAT_STATE_TRACKING:
        {
            /* ---- Notch filter: suppress 1500 Hz resonance ----
             * Applied to the raw QPD error BEFORE PID.
             * This is the key step: removes the resonance frequency
             * from the error signal so the PID never drives it. */
            float filtered_x = biquad_filter(&g_ctrl.notch_x, g_ctrl.qpd_x);
            float filtered_y = biquad_filter(&g_ctrl.notch_y, g_ctrl.qpd_y);

            /* ---- PID update ----
             * Setpoint = 0 (beam centred)
             * Measurement = filtered QPD error
             * Output = normalised DAC command */
             g_ctrl.dac_x = PIDController_Update(&g_ctrl.pid_x,
                                                   0.0f, filtered_x, PID_SAMPLE_TIME);
             g_ctrl.dac_y = PIDController_Update(&g_ctrl.pid_y,
                                                   0.0f, filtered_y, PID_SAMPLE_TIME);

            /* ---- Fade ride-through: hold last valid output ----
             * If fade is active, override with pre-fade held value.
             * Mirror stays at last known good position. */
            if (g_ctrl.fade_active)
            {
                g_ctrl.dac_x = g_ctrl.dac_x_hold;
                g_ctrl.dac_y = g_ctrl.dac_y_hold;

                /* Long fade: signal won't return, go re-acquire */
                if (g_ctrl.fade_total > FADE_TIMEOUT_SAMPLES)
                {
                    /* Reset integrators (old value no longer valid) */
                    PIDController_Init(&g_ctrl.pid_x);
                    PIDController_Init(&g_ctrl.pid_y);
                    g_ctrl.fade_active = 0;
                    g_ctrl.pid_x.fade_active = 0;
                    g_ctrl.pid_y.fade_active = 0;
                    g_ctrl.state = PAT_STATE_SCANNING;
                }
            }

            /* Loss of tracking — beam flew off QPD entirely */
            if (!g_ctrl.fade_active &&
                (g_ctrl.qpd_x > 0.95f || g_ctrl.qpd_x < -0.95f ||
                 g_ctrl.qpd_y > 0.95f || g_ctrl.qpd_y < -0.95f))
            {
                g_ctrl.state = PAT_STATE_SCANNING;
            }
            break;
        }

        /* ----------------------------------------------------------
         * FAULT: hardware error or commanded shutdown
         * Drive outputs to safe neutral position, stop PID.
         * Only exits on explicit FSMControl_SetState(IDLE) call.
         * ---------------------------------------------------------- */
        case PAT_STATE_FAULT:
        {
            g_ctrl.dac_x = 0.0f;
            g_ctrl.dac_y = 0.0f;
            g_ctrl.pid_x.fade_active = 1;  /* freeze integrators */
            g_ctrl.pid_y.fade_active = 1;
            break;
        }

        default:
            g_ctrl.state = PAT_STATE_FAULT;
            break;
    }

    /* ============================================================
     * STEP 4: Clamp final outputs (safety)
     * ============================================================ */
    g_ctrl.dac_x = clampf(g_ctrl.dac_x, DAC_OUTPUT_MIN, DAC_OUTPUT_MAX);
    g_ctrl.dac_y = clampf(g_ctrl.dac_y, DAC_OUTPUT_MIN, DAC_OUTPUT_MAX);

    /* ============================================================
     * STEP 5: Signal background loop
     * ============================================================ */
    g_ctrl.new_data_ready = 1;
}

/* =========================================================
 * FSMControl_NormToDAC
 * =========================================================
 * Map normalised output [-1.0, +1.0] to 16-bit DAC [0, 65535].
 * Midpoint (0.0) → 32767 = neutral piezo position.
 */
uint16_t FSMControl_NormToDAC(float norm)
{
    float scaled = (norm + 1.0f) * 32767.5f;
    if (scaled < 0.0f)     return 0U;
    if (scaled > 65535.0f) return 65535U;
    return (uint16_t)scaled;
}

/* =========================================================
 * FSMControl_SetState
 * =========================================================
 * Commanded state transition from main loop / host command.
 */
void FSMControl_SetState(PATState new_state)
{
    g_ctrl.state = new_state;
    if (new_state == PAT_STATE_IDLE || new_state == PAT_STATE_FAULT)
    {
        g_ctrl.pid_x.fade_active = 0;
        g_ctrl.pid_y.fade_active = 0;
        PIDController_Init(&g_ctrl.pid_x);
        PIDController_Init(&g_ctrl.pid_y);
    }
}
