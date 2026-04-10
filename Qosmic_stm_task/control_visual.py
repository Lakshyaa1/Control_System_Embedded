"""
FSM Control System Simulation
QOSMIC — Fast Steering Mirror: PID + Notch Filter Design
=========================================================
System: STM32H7 controlling a piezoelectric FSM for free-space optical comms
Plant:  G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
        fn = 1500 Hz, zeta = 0.02  (sharp resonance, barely damped)

Design goals:
  - Closed-loop bandwidth >= 500 Hz
  - Phase margin > 45 degrees
  - Gain margin > 6 dB

Run: python fsm_control_simulation.py
Requires: pip install control matplotlib numpy scipy
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import control as ct
from pathlib import Path

# =============================================================================
# SECTION 1: SYSTEM PARAMETERS
# =============================================================================
# Plant parameters (given)
fn   = 1500                    # natural frequency [Hz]
wn   = 2 * np.pi * fn          # natural frequency [rad/s]  = 9424.8 rad/s
zeta = 0.02                    # damping ratio (very low — nearly undamped)

# Frequency axis for Bode plots: 10 Hz to 100 kHz, log spaced
omega = np.logspace(1, 5, 5000)   # rad/s (10 to 100,000)
freq_hz = omega / (2 * np.pi)     # same axis in Hz

print("=" * 60)
print("FSM CONTROL SYSTEM DESIGN")
print("=" * 60)
print(f"Natural frequency:  fn  = {fn} Hz")
print(f"                    wn  = {wn:.1f} rad/s")
print(f"Damping ratio:      zeta = {zeta}")
print(f"Resonance peak gain: {1/(2*zeta):.1f}x  = {20*np.log10(1/(2*zeta)):.1f} dB")
print()

# =============================================================================
# SECTION 2: DEFINE THE PLANT G(s)
# =============================================================================
# G(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2)
#
# Physical meaning:
#   - s^2 term: mirror inertia (mass)
#   - 2*zeta*wn*s term: damping (very small for piezo)
#   - wn^2 term: stiffness (spring restoring force)

num_G = [wn**2]
den_G = [1, 2*zeta*wn, wn**2]
G = ct.tf(num_G, den_G)
print("Plant G(s):")
print(G)

# =============================================================================
# SECTION 3: DESIGN THE NOTCH FILTER N(s)
# =============================================================================
# N(s) = (s^2 + 2*zeta_z*wn*s + wn^2) / (s^2 + 2*zeta_p*wn*s + wn^2)
#
# Physical meaning:
#   - Zeros at wn with damping zeta_z → creates deep attenuation at wn
#   - Poles at wn with damping zeta_p → shapes the recovery bandwidth
#   - Notch depth = zeta_z / zeta_p   (in linear ratio)
#   - At DC and high freq: N(s) → 1  (no effect outside notch)
#
# Design choices:
#   - zeta_z = 0.01: very small → deep notch (~-37 dB at wn)
#   - zeta_p = 0.7:  large → fast recovery from notch, minimal phase disturbance

zeta_z = 0.01   # zero damping ratio (controls notch depth)
zeta_p = 0.7    # pole damping ratio (controls notch width / recovery)

num_N = [1, 2*zeta_z*wn, wn**2]
den_N = [1, 2*zeta_p*wn, wn**2]
N = ct.tf(num_N, den_N)

notch_depth_dB = 20 * np.log10(zeta_z / zeta_p)
print(f"\nNotch Filter N(s):")
print(f"  zeta_z = {zeta_z}  (zero damping)")
print(f"  zeta_p = {zeta_p}  (pole damping)")
print(f"  Notch depth at wn = {notch_depth_dB:.1f} dB  (= 20*log10({zeta_z}/{zeta_p}))")
print(N)

# =============================================================================
# SECTION 4: DESIGN THE PID CONTROLLER
# =============================================================================
# Standard PID in s-domain:
#   C_PID(s) = Kp + Ki/s + Kd*s
#
# However, pure derivative Kd*s has infinite gain at high frequency → amplifies
# noise. Fix: add a first-order low-pass filter on the derivative term:
#   D_filtered(s) = Kd * s / (1 + s/N_filter)
#   where N_filter = derivative filter cutoff in rad/s
#
# This gives a practical "PID with derivative filter":
#   C(s) = Kp + Ki/s + Kd*s/(1 + s/N_filter)
#
# Tuning logic:
#   - Kp:      Sets the main loop gain → determines bandwidth
#   - Ki:      Removes steady-state error; too large → destabilizing
#   - Kd:      Adds phase lead near crossover → improves phase margin
#   - N_filter: Limits derivative gain at frequencies above this cutoff
#
# These values are tuned to meet all three specs simultaneously.
# Tuning was done iteratively: start with Kp only, then add Kd, then Ki.

Kp       = 0.8       # proportional gain
Ki       = 200.0     # integral gain [rad/s]
Kd       = 3e-4      # derivative gain [s]
N_filter = 10000.0   # derivative filter cutoff [rad/s] ~1.6 kHz

# Build PID using s = tf('s') shorthand
s = ct.tf('s')

P_term  = Kp
I_term  = Ki / s
D_term  = Kd * s / (1 + s / N_filter)

PID = P_term + I_term + D_term

print(f"\nPID Controller:")
print(f"  Kp = {Kp}")
print(f"  Ki = {Ki}")
print(f"  Kd = {Kd}")
print(f"  N_filter (derivative cutoff) = {N_filter} rad/s = {N_filter/(2*np.pi):.0f} Hz")

# =============================================================================
# SECTION 5: FORM OPEN-LOOP TRANSFER FUNCTION L(s)
# =============================================================================
# Controller C(s) = PID × Notch (in series / cascaded)
# Open-loop L(s) = C(s) × G(s)
#
# The open-loop Bode plot is what we use to check stability margins.
# Gain crossover frequency wgc ≈ closed-loop bandwidth (good approximation).

C = PID * N          # combined controller: PID cascaded with notch filter
L = C * G            # open-loop: controller × plant

# Compute stability margins
# ct.margin returns: (gm, pm, wpc, wgc)
#   gm  = gain margin (linear ratio)
#   pm  = phase margin (degrees)
#   wpc = phase crossover frequency (where phase = -180°)
#   wgc = gain crossover frequency  (where magnitude = 0 dB)

gm, pm, wpc, wgc = ct.margin(L)

bandwidth_hz = wgc / (2 * np.pi)
gm_dB = 20 * np.log10(gm)

print()
print("=" * 60)
print("STABILITY MARGIN RESULTS")
print("=" * 60)
print(f"Gain margin:          {gm_dB:.1f} dB    (need > 6 dB)   {'PASS' if gm_dB > 6 else 'FAIL'}")
print(f"Phase margin:         {pm:.1f}°      (need > 45°)   {'PASS' if pm > 45 else 'FAIL'}")
print(f"Gain crossover freq:  {bandwidth_hz:.0f} Hz    (need ≥ 500 Hz) {'PASS' if bandwidth_hz >= 500 else 'FAIL'}")
print(f"Phase crossover freq: {wpc/(2*np.pi):.0f} Hz")

# =============================================================================
# SECTION 6: CLOSED-LOOP TRANSFER FUNCTION T(s) AND STEP RESPONSE
# =============================================================================
# Closed-loop: T(s) = L(s) / (1 + L(s))
# This is the standard negative feedback formula.
# ct.feedback(L, 1) computes exactly this.

T = ct.feedback(L, 1)

# Step response — simulate over 10 ms
t_sim = np.linspace(0, 0.01, 20000)
t_out, y_out = ct.step_response(T, T=t_sim)

# Performance metrics
y_final  = y_out[-1]
y_peak   = np.max(y_out)
overshoot_pct = 100 * (y_peak - y_final) / y_final

# 2% settling time: first time |y - y_final| stays < 2% of y_final
tol = 0.02 * y_final
settled_idx = np.where(np.abs(y_out - y_final) < tol)[0]
if len(settled_idx) > 0:
    # Find the last crossing out of the band
    outside = np.where(np.abs(y_out - y_final) >= tol)[0]
    if len(outside) > 0:
        last_out = outside[-1]
        settling_time_ms = t_out[last_out + 1] * 1000 if last_out + 1 < len(t_out) else t_out[settled_idx[0]] * 1000
    else:
        settling_time_ms = t_out[settled_idx[0]] * 1000
else:
    settling_time_ms = float('nan')

print()
print("=" * 60)
print("CLOSED-LOOP STEP RESPONSE METRICS")
print("=" * 60)
print(f"Overshoot:      {overshoot_pct:.1f}%")
print(f"Settling time:  {settling_time_ms:.2f} ms  (2% criterion)")
print(f"Final value:    {y_final:.4f}")

# =============================================================================
# SECTION 7: GENERATE ALL PLOTS
# =============================================================================

fig = plt.figure(figsize=(16, 12))
fig.suptitle(
    'FSM Control Design: PID + Notch Filter\n'
    f'fn={fn} Hz, ζ={zeta} | '
    f'Kp={Kp}, Ki={Ki}, Kd={Kd} | '
    f'ζ_z={zeta_z}, ζ_p={zeta_p}',
    fontsize=13, fontweight='bold'
)
gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.5, wspace=0.35)

# ── Plot 1: Plant Bode (top left) ──────────────────────────────────────────
ax1a = fig.add_subplot(gs[0, 0])
ax1b = fig.add_subplot(gs[1, 0])

mag_G, phase_G, _ = ct.bode(G, omega=omega, plot=False)
mag_G_dB    = 20 * np.log10(np.abs(mag_G))
phase_G_deg = np.degrees(phase_G)

ax1a.semilogx(freq_hz, mag_G_dB, 'b', linewidth=1.8)
ax1a.axvline(fn, color='red', linestyle='--', alpha=0.6, label=f'fₙ = {fn} Hz')
ax1a.axhline(0,  color='k',   linestyle=':', alpha=0.4)
ax1a.annotate(f'+{20*np.log10(1/(2*zeta)):.0f} dB peak\n(= 25× gain)',
              xy=(fn, 20*np.log10(1/(2*zeta))),
              xytext=(fn*3, 20),
              arrowprops=dict(arrowstyle='->', color='red'),
              color='red', fontsize=9)
ax1a.set_ylabel('Magnitude (dB)')
ax1a.set_title('Plant G(s) — resonance at 1.5 kHz', fontsize=10)
ax1a.legend(fontsize=8); ax1a.grid(True, which='both', alpha=0.3)

ax1b.semilogx(freq_hz, phase_G_deg, 'b', linewidth=1.8)
ax1b.axvline(fn, color='red', linestyle='--', alpha=0.6)
ax1b.axhline(-180, color='red', linestyle=':', alpha=0.6, label='-180°')
ax1b.axhline(-90,  color='orange', linestyle=':', alpha=0.5, label='-90° at fₙ')
ax1b.set_xlabel('Frequency (Hz)')
ax1b.set_ylabel('Phase (°)')
ax1b.legend(fontsize=8); ax1b.grid(True, which='both', alpha=0.3)

# ── Plot 2: Notch filter Bode (top right) ─────────────────────────────────
ax2a = fig.add_subplot(gs[0, 1])
ax2b = fig.add_subplot(gs[1, 1])

mag_N, phase_N, _ = ct.bode(N, omega=omega, plot=False)
mag_N_dB    = 20 * np.log10(np.abs(mag_N))
phase_N_deg = np.degrees(phase_N)

ax2a.semilogx(freq_hz, mag_N_dB, 'g', linewidth=1.8)
ax2a.axvline(fn, color='red', linestyle='--', alpha=0.6, label=f'fₙ = {fn} Hz')
ax2a.axhline(0,  color='k',   linestyle=':', alpha=0.4)
ax2a.annotate(f'Notch depth\n{notch_depth_dB:.0f} dB',
              xy=(fn, notch_depth_dB),
              xytext=(fn*3, notch_depth_dB+5),
              arrowprops=dict(arrowstyle='->', color='green'),
              color='green', fontsize=9)
ax2a.set_ylabel('Magnitude (dB)')
ax2a.set_title(f'Notch Filter N(s) — ζ_z={zeta_z}, ζ_p={zeta_p}', fontsize=10)
ax2a.legend(fontsize=8); ax2a.grid(True, which='both', alpha=0.3)

ax2b.semilogx(freq_hz, phase_N_deg, 'g', linewidth=1.8)
ax2b.axvline(fn, color='red', linestyle='--', alpha=0.6)
ax2b.set_xlabel('Frequency (Hz)')
ax2b.set_ylabel('Phase (°)')
ax2b.grid(True, which='both', alpha=0.3)

# ── Plot 3: Open-loop Bode with margins (bottom left) ─────────────────────
ax3a = fig.add_subplot(gs[2, 0])

mag_L, phase_L, _ = ct.bode(L, omega=omega, plot=False)
mag_L_dB    = 20 * np.log10(np.abs(mag_L))
phase_L_deg = np.degrees(phase_L)

# Twin axis: magnitude on top, phase on bottom (in same subplot for space)
ax3b = ax3a.twinx()
ax3a.semilogx(freq_hz, mag_L_dB,    'b',  linewidth=2,   label='Magnitude')
ax3b.semilogx(freq_hz, phase_L_deg, 'r--', linewidth=1.5, label='Phase', alpha=0.8)

# Gain crossover (0 dB line)
ax3a.axhline(0, color='k', linestyle=':', alpha=0.5)
ax3a.axvline(bandwidth_hz, color='blue', linestyle='--', alpha=0.5,
             label=f'Gain crossover: {bandwidth_hz:.0f} Hz')
# Phase margin line
ax3b.axhline(-180, color='red', linestyle=':', alpha=0.5)
ax3b.axhline(-180 + pm, color='orange', linestyle='--', alpha=0.6,
             label=f'PM = {pm:.0f}°')

# Annotate margins
ax3a.annotate(f'GM = {gm_dB:.1f} dB\nPM = {pm:.0f}°\nBW = {bandwidth_hz:.0f} Hz',
              xy=(0.02, 0.05), xycoords='axes fraction',
              bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
              fontsize=9)

ax3a.set_xlabel('Frequency (Hz)')
ax3a.set_ylabel('Magnitude (dB)', color='blue')
ax3b.set_ylabel('Phase (°)', color='red')
ax3a.set_title('Open-loop L(s) = C(s)×G(s) with stability margins', fontsize=10)
lines1, labels1 = ax3a.get_legend_handles_labels()
lines2, labels2 = ax3b.get_legend_handles_labels()
ax3a.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc='upper right')
ax3a.grid(True, which='both', alpha=0.3)
ax3a.set_ylim([-80, 60])
ax3b.set_ylim([-360, 10])

# ── Plot 4: Closed-loop step response (bottom right) ──────────────────────
ax4 = fig.add_subplot(gs[2, 1])

ax4.plot(t_out * 1000, y_out, 'b', linewidth=2, label='Step response')
ax4.axhline(y_final, color='k', linestyle='--', alpha=0.5, linewidth=1, label='Final value')
ax4.axhline(y_final * 1.02, color='g', linestyle=':', alpha=0.6)
ax4.axhline(y_final * 0.98, color='g', linestyle=':', alpha=0.6, label='±2% band')
ax4.axhline(y_final * (1 + overshoot_pct/100), color='orange', linestyle=':', alpha=0.6)

if not np.isnan(settling_time_ms):
    ax4.axvline(settling_time_ms, color='purple', linestyle='--', alpha=0.6,
                label=f'Settling: {settling_time_ms:.2f} ms')

ax4.annotate(f'Overshoot: {overshoot_pct:.1f}%',
             xy=(t_out[np.argmax(y_out)] * 1000, y_peak),
             xytext=(t_out[np.argmax(y_out)] * 1000 + 0.5, y_peak + 0.02),
             arrowprops=dict(arrowstyle='->', color='orange'),
             color='orange', fontsize=9)

ax4.set_xlabel('Time (ms)')
ax4.set_ylabel('Normalized response')
ax4.set_title('Closed-loop T(s) step response', fontsize=10)
ax4.legend(fontsize=8)
ax4.grid(True, alpha=0.3)
ax4.set_xlim([0, 10])

output_dir = Path(__file__).resolve().parent / 'outputs'
output_dir.mkdir(parents=True, exist_ok=True)
output_path = output_dir / 'fsm_bode_step_response.png'
plt.savefig(output_path, dpi=150, bbox_inches='tight')
print(f"\nPlot saved: {output_path}")
plt.show()

# =============================================================================
# SECTION 8: FINAL SUMMARY
# =============================================================================
print()
print("=" * 60)
print("FINAL DESIGN PARAMETERS")
print("=" * 60)
print("\n[Plant]")
print(f"  fn   = {fn} Hz  (natural frequency)")
print(f"  zeta = {zeta}   (damping ratio)")

print("\n[Notch Filter]")
print(f"  Center freq: {fn} Hz  (matches plant resonance exactly)")
print(f"  zeta_z = {zeta_z}  (zero damping  → deep notch)")
print(f"  zeta_p = {zeta_p}   (pole damping  → fast recovery)")
print(f"  Notch depth = {notch_depth_dB:.1f} dB at resonance")

print("\n[PID Controller]")
print(f"  Kp = {Kp}    (proportional gain)")
print(f"  Ki = {Ki}  (integral gain)")
print(f"  Kd = {Kd}  (derivative gain)")
print(f"  N_filter = {N_filter:.0f} rad/s = {N_filter/(2*np.pi):.0f} Hz  (derivative filter)")

print("\n[Performance vs Specs]")
print(f"  Bandwidth:    {bandwidth_hz:.0f} Hz   (spec: ≥500 Hz)  {'✓ PASS' if bandwidth_hz >= 500 else '✗ FAIL'}")
print(f"  Phase margin: {pm:.1f}°   (spec: >45°)    {'✓ PASS' if pm > 45 else '✗ FAIL'}")
print(f"  Gain margin:  {gm_dB:.1f} dB   (spec: >6 dB)   {'✓ PASS' if gm_dB > 6 else '✗ FAIL'}")
print(f"  Overshoot:    {overshoot_pct:.1f}%")
print(f"  Settling:     {settling_time_ms:.2f} ms  (2% criterion)")

print()
print("=" * 60)
print("WHY THESE VALUES WORK")
print("=" * 60)
print("""
1. Notch filter (zeta_z=0.01, zeta_p=0.7):
   - Centered at exactly fn = 1500 Hz — the dangerous resonance
   - Deep notch (zeta_z/zeta_p = 0.014 = -37 dB) eliminates the
     resonance peak from the open-loop gain
   - Large zeta_p = 0.7 means the filter recovers quickly above
     the notch — no lingering phase distortion at our 500 Hz crossover
   - DC gain = 1 (no effect at low frequencies where we need control)

2. PID gains (Kp=0.8, Ki=200, Kd=3e-4):
   - Kp=0.8: pushes gain crossover to ~500 Hz without exciting resonance
     (safe because notch has tamed the 1.5 kHz spike)
   - Kd=3e-4: adds phase lead near the crossover frequency → boosts
     phase margin above 45°. Derivative filter at 10,000 rad/s limits
     high-frequency amplification from the D term
   - Ki=200: provides integral action to zero out steady-state error
     while being small enough not to cause phase margin degradation
     below the crossover

3. The interplay:
   - Without notch: any Kp that achieves 500 Hz bandwidth would also
     produce |L| > 0 dB at 1.5 kHz where phase = -180° → unstable
   - With notch: |L| at 1.5 kHz is suppressed below 0 dB → the
     phase crossover at -180° has no corresponding gain crossing
     → gain margin becomes large → safe
""")