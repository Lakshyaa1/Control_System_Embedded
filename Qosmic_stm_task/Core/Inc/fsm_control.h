/*
 * fsm_control.h
 *
 * Top-level FSM control system header.
 * Ties together:
 *   - PAT (Point, Acquire, Track) state machine
 *   - Biquad notch filter (one per axis)
 *   - PID controller (one per axis)
 *   - Fade detection and integrator freeze
 *   - Spiral scan generator (SCANNING state)
 *   - Telemetry flag for background UART
 *
 * Signal path each ISR cycle:
 *   QPD raw → normalise → notch filter → PID → DAC output
 *
 * States:
 *   IDLE      — system on, not tracking, FSM at rest
 *   SCANNING  — spiral search for beacon
 *   TRACKING  — closed-loop PID active
 *   FAULT     — error condition, safe shutdown
 */

#ifndef FSM_CONTROL_H
#define FSM_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "pid_controller.h"
#include "biquad_filter.h"

/* =========================================================
 * Physical / calibration constants
 * ========================================================= */
#define QPD_ADC_MIDPOINT      (2048U)     /* 12-bit ADC midpoint = 0 error */
#define QPD_FULL_SCALE        (2048.0f)   /* ADC counts for full deflection */

/* Fade detection: signal is lost when total power < this fraction of nominal */
#define FADE_POWER_THRESHOLD  (0.15f)     /* 15% of nominal beam power */
#define FADE_CONFIRM_SAMPLES  (5U)        /* debounce: N consecutive samples */
#define FADE_TIMEOUT_SAMPLES  (2000U)     /* 200 ms at 10 kHz → go to SCAN */

/* Spiral scan parameters */
#define SPIRAL_OMEGA          (2.0f * 3.14159265f * 2.0f)  /* 2 Hz rotation */
#define SPIRAL_R_INCREMENT    (0.001f)    /* radius grows per sample */
#define SPIRAL_R_MAX          (0.8f)      /* max normalised radius */
#define SPIRAL_TIMEOUT_SAMPLES (50000U)  /* 5 s max scan before FAULT */

/* DAC output range (normalised -1 to +1 maps to DAC 0 to 65535) */
#define DAC_OUTPUT_MIN        (-1.0f)
#define DAC_OUTPUT_MAX        ( 1.0f)

/* =========================================================
 * PAT State machine states
 * ========================================================= */
typedef enum {
    PAT_STATE_IDLE     = 0,
    PAT_STATE_SCANNING = 1,
    PAT_STATE_TRACKING = 2,
    PAT_STATE_FAULT    = 3
} PATState;

/* =========================================================
 * Main control system context
 * One global instance, initialised in FSMControl_Init().
 * ========================================================= */
typedef struct {

    /* Current PAT state */
    PATState state;

    /* PID controllers — one per axis */
    PIDController pid_x;
    PIDController pid_y;

    /* Biquad notch filters — one per axis, placed BEFORE PID */
    BiquadFilter notch_x;
    BiquadFilter notch_y;

    /* Sensor readings (normalised -1.0 to +1.0) */
    float qpd_x;        /* QPD X error, normalised */
    float qpd_y;        /* QPD Y error, normalised */
    float beam_power;   /* Total QPD power (A+B+C+D), normalised */

    /* Control outputs (normalised -1.0 to +1.0 → maps to DAC) */
    float dac_x;
    float dac_y;

    /* Fade detection counters */
    uint32_t fade_samples;          /* consecutive low-power samples */
    uint32_t fade_total;            /* total samples since fade began */
    uint8_t  fade_active;           /* 1 = fade in progress */
    float    dac_x_hold;            /* output held during fade */
    float    dac_y_hold;

    /* Spiral scan state */
    float    spiral_r;              /* current radius */
    float    spiral_t;              /* phase accumulator (sample count × dt) */
    uint32_t scan_timeout;          /* samples since scan started */

    /* Telemetry flag — set by ISR, cleared by main loop */
    volatile uint8_t new_data_ready;

    /* Cycle counter for diagnostics */
    uint32_t cycle_count;

} FSMControlContext;

/* =========================================================
 * Global context — defined in fsm_control.c
 * ========================================================= */
extern FSMControlContext g_ctrl;

/* =========================================================
 * API
 * ========================================================= */

/**
 * @brief  Initialise all control system components.
 *         Call once in main() before starting TIM2.
 */
void FSMControl_Init(void);

/**
 * @brief  Main ISR update — call from TIM2_IRQHandler.
 *         Reads sensor values, runs notch + PID, writes DAC outputs,
 *         manages PAT state machine.
 *
 * @param  adc_qpd_x   Raw 12-bit ADC reading for QPD X channel
 * @param  adc_qpd_y   Raw 12-bit ADC reading for QPD Y channel
 * @param  adc_power   Raw 12-bit ADC reading for total QPD power
 *                     (or pass 0 if not available — disables fade detection)
 */
void FSMControl_Update(uint16_t adc_qpd_x,
                       uint16_t adc_qpd_y,
                       uint16_t adc_power);

/**
 * @brief  Convert normalised output [-1, +1] to 16-bit DAC count [0, 65535].
 * @param  norm  Normalised value in [-1.0, +1.0]
 * @return       DAC count 0–65535
 */
uint16_t FSMControl_NormToDAC(float norm);

/**
 * @brief  Force transition to a specific state.
 *         Use from main loop for commanded state changes (e.g. host PC command).
 */
void FSMControl_SetState(PATState new_state);

#ifdef __cplusplus
}
#endif

#endif /* FSM_CONTROL_H */
