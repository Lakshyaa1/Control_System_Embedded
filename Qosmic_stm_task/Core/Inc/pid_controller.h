/*
 * pid_controller.h
 *
 * Discrete-time PID controller.
 * Based on pms67/PID (https://github.com/pms67/PID), extended with:
 *   - fade_active flag: freezes integrator during QPD signal loss
 *   - Derivative-on-measurement (not on error) to avoid setpoint kick
 *   - Separate integrator clamp limits (anti-windup)
 *   - Output clamp to DAC range
 *
 * Discrete equations used (Tustin / trapezoidal integration):
 *   Proportional:   P  = Kp * e[n]
 *   Integral:       I += Ki * T * (e[n] + e[n-1]) / 2     (trapezoidal)
 *   Derivative:     D  = -(2*Kd*(meas[n] - meas[n-1])
 *                        + (2*tau - T)*D[n-1]) / (2*tau + T)
 *                   (derivative-on-measurement with LP filter, time constant tau)
 *   Output:         u  = clamp(P + I + D, limMin, limMax)
 *
 * Note on derivative-on-measurement (from pms67 README):
 *   e[n] = 0 - measurement, so (e[n] - e[n-1]) = -(meas[n] - meas[n-1]).
 *   The minus sign is included in the code, so Kd behaves normally.
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* =========================================================
 * PID gains — tune these to match your Python simulation results
 *
 * From Python simulation (fsm_control_simulation.py):
 *   Kp = 0.8,  Ki = 200.0,  Kd = 3e-4
 *   tau = 1/(2*pi*1592) = 1e-4  (derivative filter @ 1592 Hz)
 *   T   = 1/10000 = 1e-4 s
 * ========================================================= */
#define PID_KP           (0.8f)
#define PID_KI           (200.0f)
#define PID_KD           (3e-4f)
#define PID_TAU          (1.0e-4f)   /* derivative LP filter time constant */
#define PID_SAMPLE_TIME  (1.0e-4f)   /* T = 1/fs = 100 µs */

#define PID_LIM_MIN      (-1.0f)     /* normalised output: -1 = min DAC */
#define PID_LIM_MAX      ( 1.0f)     /* normalised output: +1 = max DAC */
#define PID_LIM_MIN_INT  (-0.5f)     /* integrator anti-windup clamp */
#define PID_LIM_MAX_INT  ( 0.5f)

/* =========================================================
 * PID controller state struct
 *
 * Matches pms67/PID layout exactly, with two additions:
 *   fade_active  — set by state machine when QPD is invalid
 *   out          — last computed output (exposed for telemetry)
 * ========================================================= */
typedef struct {
    /* Gains */
    float Kp;
    float Ki;
    float Kd;

    /* Derivative low-pass filter time constant [s] */
    float tau;

    /* Output limits */
    float limMin;
    float limMax;

    /* Integrator limits (anti-windup) */
    float limMinInt;
    float limMaxInt;

    /* Sample time [s] */
    float T;

    /* Controller memory (pms67 naming preserved) */
    float integrator;
    float prevError;        /* used by trapezoidal integration */
    float differentiator;
    float prevMeasurement;  /* used by derivative-on-measurement */

    /* Fade protection — set 1 by state machine during signal loss */
    uint8_t fade_active;

    /* Last output — readable by telemetry without re-computing */
    float out;

} PIDController;

/* =========================================================
 * API
 * ========================================================= */

/**
 * @brief  Initialise PID controller with gains and limits.
 *         Clears all state (integrator, differentiator, memory).
 *         Call once during system startup.
 * @param  pid  Pointer to PIDController instance
 */
void PIDController_Init(PIDController *pid);

/**
 * @brief  Compute one PID update step.
 *         Call once per control loop cycle (inside TIM2 ISR).
 *
 * @param  pid          Pointer to PIDController instance
 * @param  setpoint     Desired beam position (normalised, 0 = centre)
 * @param  measurement  Actual QPD reading (normalised, from ADC)
 * @return              Control output u[n], clamped to [limMin, limMax]
 *
 * If pid->fade_active == 1:
 *   - Integrator is FROZEN (not updated) — no windup during fade
 *   - P and D still computed using last valid measurement
 *   - Output is held at last valid value (set externally by state machine)
 */
typedef PIDController pid_state_t;

float pid_update(pid_state_t *state,
                 float setpoint,
                 float measurement,
                 float dt);

float PIDController_Update(PIDController *pid,
                           float setpoint,
                           float measurement,
                           float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
