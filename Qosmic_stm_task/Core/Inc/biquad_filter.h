/*
 * biquad_filter.h
 *
 * Direct Form II Transposed biquad IIR filter.
 * Approach follows Phil's Lab #39 (Notch Filters - Theory and Software
 * Implementation) applied to a second-order notch for the FSM resonance
 * at fn = 1500 Hz, sampled at fs = 10 kHz.
 *
 * Transfer function (notch):
 *   H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
 *
 * Coefficients are pre-computed offline (Python / MATLAB) using the
 * bilinear transform and stored here as compile-time constants.
 *
 * Direct Form II Transposed recurrence:
 *   y[n]   = b0*x[n] + z1[n-1]
 *   z1[n]  = b1*x[n] - a1*y[n] + z2[n-1]
 *   z2[n]  = b2*x[n] - a2*y[n]
 *
 * Why DFI Transposed?
 *   - Only 2 state variables (minimal memory)
 *   - Numerically better conditioned than Direct Form I
 *   - Phil's Lab and ARM CMSIS both use this form
 */

#ifndef BIQUAD_FILTER_H
#define BIQUAD_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 * Notch filter parameters
 *   fs = 10 000 Hz  (control loop sample rate)
 *   fn =  1 500 Hz  (piezo resonance to suppress)
 *   Q  =      5.0   (notch quality factor)
 *
 * Pre-computed via bilinear transform (see fsm_coeff_calc.py):
 *   K  = tan(pi * fn / fs) = 0.509525
 *   b0 =  0.92515351
 *   b1 = -1.08758318
 *   b2 =  0.92515351   (= b0 for notch)
 *   a1 = -1.08758318   (= b1 for notch)
 *   a2 =  0.85030702
 * ========================================================= */
#define NOTCH_B0  ( 0.92515351f)
#define NOTCH_B1  (-1.08758318f)
#define NOTCH_B2  ( 0.92515351f)
#define NOTCH_A1  (-1.08758318f)
#define NOTCH_A2  ( 0.85030702f)

/* =========================================================
 * Biquad state struct
 *
 * Coefficients are set once at init and state variables z1/z2
 * are updated each sample.
 * ========================================================= */
typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
} biquad_state_t;

/* Existing project type name kept as alias */
typedef biquad_state_t BiquadFilter;

/* =========================================================
 * API
 * ========================================================= */

/**
 * @brief  Initialise a biquad filter state.
 * @param  state  Pointer to biquad state
 */
void biquad_init(biquad_state_t *state,
                 float b0, float b1, float b2,
                 float a1, float a2);

/**
 * @brief  Process one sample through the biquad filter.
 *         Call this once per control loop cycle, per axis.
 *
 * @param  state  Pointer to the filter instance (holds state)
 * @param  input  Raw input sample (e.g. QPD error signal)
 * @return        Filtered output sample
 *
 * Execution time on STM32H743 @ 480 MHz with FPU:
 *   ~5 FP multiply-accumulate ops = ~10–15 ns
 */
float biquad_filter(biquad_state_t *state, float input);

/* Backward-compatible wrappers */
static inline void BiquadFilter_Init(BiquadFilter *f,
                                     float b0, float b1, float b2,
                                     float a1, float a2)
{
    biquad_init(f, b0, b1, b2, a1, a2);
}

static inline float BiquadFilter_Update(BiquadFilter *f, float input)
{
    return biquad_filter(f, input);
}

#ifdef __cplusplus
}
#endif

#endif /* BIQUAD_FILTER_H */
