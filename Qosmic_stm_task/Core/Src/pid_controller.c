/*
 * pid_controller.c
 *
 * Discrete-time PID controller implementation.
 * Style matches pms67/PID. Extended for FSM application with:
 *   - Integrator freeze during fade (fade_active flag)
 *   - Derivative-on-measurement (avoids kick on setpoint change)
 *   - Trapezoidal (Tustin) integration for better accuracy
 *   - Anti-windup via integrator clamping
 */

#include "pid_controller.h"

/* Helper: clamp a float to [lo, hi] */
static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

/* ---------------------------------------------------------
 * PIDController_Init
 * ---------------------------------------------------------
 * Loads default gains from header #defines and zeros all state.
 * The caller can override individual fields after Init if needed
 * (e.g. different gains for X vs Y axis).
 */
void PIDController_Init(PIDController *pid)
{
    /* Gains from compile-time constants (match Python sim results) */
    pid->Kp  = PID_KP;
    pid->Ki  = PID_KI;
    pid->Kd  = PID_KD;
    pid->tau = PID_TAU;

    /* Limits */
    pid->limMin    = PID_LIM_MIN;
    pid->limMax    = PID_LIM_MAX;
    pid->limMinInt = PID_LIM_MIN_INT;
    pid->limMaxInt = PID_LIM_MAX_INT;

    /* Sample time */
    pid->T = PID_SAMPLE_TIME;

    /* Zero all state */
    pid->integrator      = 0.0f;
    pid->prevError       = 0.0f;
    pid->differentiator  = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->fade_active = 0;
    pid->out         = 0.0f;
}

/* ---------------------------------------------------------
 * PIDController_Update
 * ---------------------------------------------------------
 * Called once per ISR cycle. Returns the new control output.
 *
 * Matches pms67/PID logic with these additions:
 *   1. Integrator freeze when fade_active == 1
 *   2. Dynamic integrator clamp (anti-windup, pms67 method)
 *   3. Derivative on measurement (not error) — no setpoint kick
 *
 * Step-by-step:
 *   a) Compute error
 *   b) Proportional term
 *   c) Integral term (trapezoidal) — skip if fade_active
 *   d) Anti-windup clamp on integrator
 *   e) Derivative term (on measurement, with LP filter)
 *   f) Sum and clamp output
 *   g) Store state for next cycle
 */
float pid_update(pid_state_t *state,
                 float setpoint,
                 float measurement,
                 float dt)
{
    /* ---- a) Error ---- */
    if (dt > 0.0f)
        state->T = dt;

    float error = setpoint - measurement;

    /* ---- b) Proportional ---- */
    float proportional = state->Kp * error;

    /* ---- c) Integral (trapezoidal) ----
     * Trapezoidal rule: I += Ki * T * (e[n] + e[n-1]) / 2
     * This is more accurate than forward/backward Euler at 10 kHz.
     * FREEZE if fade_active — preserves last valid integrator value,
     * preventing windup from noise during QPD signal loss. */
    if (!state->fade_active)
    {
        state->integrator += 0.5f * state->Ki * state->T
                             * (error + state->prevError);

        /* ---- d) Anti-windup: clamp integrator ----
         * Dynamic clamp (pms67 method):
         * The integrator is only allowed to grow up to the point where
         * adding proportional would still stay within output limits.
         * This prevents windup without discontinuous jumps on recovery. */
        float limMinInt, limMaxInt;

        /* Upper integrator limit */
        if (state->limMax > proportional)
            limMaxInt = state->limMax - proportional;
        else
            limMaxInt = 0.0f;

        /* Lower integrator limit */
        if (state->limMin < proportional)
            limMinInt = state->limMin - proportional;
        else
            limMinInt = 0.0f;

        /* Apply clamp */
        state->integrator = clampf(state->integrator, limMinInt, limMaxInt);
    }
    /* If fade_active: do nothing — integrator stays frozen */

    /* ---- e) Derivative on measurement ----
     * Using measurement (not error) avoids a derivative spike when
     * setpoint changes abruptly (no "derivative kick").
     *
     * pms67 note: since e = 0 - measurement, derivative-on-error would be
     *   Kd*(e[n]-e[n-1]) = -Kd*(meas[n]-meas[n-1])
     * Using measurement directly and negating gives the same result.
     * The pms67 repo includes this negation — so does this code.
     *
     * First-order LP filter on derivative (time constant tau):
     *   D[n] = -(2*Kd*(meas[n]-meas[n-1]) + (2*tau-T)*D[n-1]) / (2*tau+T)
     */
    state->differentiator =
        -(2.0f * state->Kd * (measurement - state->prevMeasurement)
          + (2.0f * state->tau - state->T) * state->differentiator)
        / (2.0f * state->tau + state->T);

    /* ---- f) Compute output and clamp ---- */
    state->out = clampf(proportional + state->integrator + state->differentiator,
                        state->limMin, state->limMax);

    /* ---- g) Store state for next cycle ---- */
    state->prevError       = error;
    state->prevMeasurement = measurement;

    return state->out;
}

float PIDController_Update(PIDController *pid,
                           float setpoint,
                           float measurement,
                           float dt)
{
    return pid_update(pid, setpoint, measurement, dt);
}
