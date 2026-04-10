/*
 * biquad_filter.c
 *
 * Direct Form II Transposed biquad IIR filter implementation.
 * Pattern follows Phil's Lab #39 approach: offline coefficient
 * calculation → struct init → single update function in ISR.
 *
 * Recurrence equations (Direct Form II Transposed):
 *   y[n]  = b0*x[n] + z1
 *   z1    = b1*x[n] - a1*y[n] + z2
 *   z2    = b2*x[n] - a2*y[n]
 *
 * State update happens in-place — no extra buffers needed.
 */

#include "biquad_filter.h"

/* ---------------------------------------------------------
 * biquad_init
 * ---------------------------------------------------------
 * Stores coefficients and zeroes the state.
 * Call once during system init, before the control loop starts.
 */
void biquad_init(biquad_state_t *state,
                 float b0, float b1, float b2,
                 float a1, float a2)
{
    state->b0 = b0;
    state->b1 = b1;
    state->b2 = b2;
    state->a1 = a1;
    state->a2 = a2;

    /* Clear state — equivalent to assuming zero past inputs/outputs */
    state->z1 = 0.0f;
    state->z2 = 0.0f;
}

/* ---------------------------------------------------------
 * biquad_filter
 * ---------------------------------------------------------
 * Process one input sample. Returns filtered output.
 *
 * This is the hot path — called 10,000 times per second per axis.
 * Keep it minimal: 5 multiplications, 4 additions.
 *
 * Direct Form II Transposed (DFI-T) step by step:
 *   1.  Compute output using current state z1:
 *           y = b0*x + z1
 *   2.  Shift state forward (z1 <- z2, but folded into update):
 *           z1_new = b1*x - a1*y + z2
 *   3.  Update last state:
 *           z2_new = b2*x - a2*y
 *   4.  Store new states, return y.
 *
 * The ordering matters — compute y before overwriting z1/z2.
 */
float biquad_filter(biquad_state_t *state, float input)
{
    /* Step 1: output */
    float output = state->b0 * input + state->z1;

    /* Step 2: update first state */
    state->z1 = state->b1 * input - state->a1 * output + state->z2;

    /* Step 3: update second state */
    state->z2 = state->b2 * input - state->a2 * output;

    return output;
}
