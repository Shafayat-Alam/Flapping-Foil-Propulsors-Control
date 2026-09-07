#!/usr/bin/env python3
"""Design + emit the harmonic control-authority sweep.

GOAL: prove, on the rig, that each of the six waveform features can be
driven by a named knob -- separately for Fx and for Fy (not simultaneously;
the rig is underactuated and simultaneous control is not claimed).

  feature                      knob                block
  ---------------------------- ------------------- -----
  1 magnitude                  freq_scale          A
  2 peak count                 freq_ratio          B
  3 lobe width                 a3  (= pitch_k)     C
  4 +/- peak scaling           a2                  D / E
  5 which lobe dominates       phi2                D / E
  6 skewness                   phi2                D / E

Blocks D and E are the same design applied to the PITCH servo (to shape Fx)
and the HEAVE servo (to shape Fy).

DESIGN DECISIONS AND WHY
------------------------
* Amplitude normalisation. The composite angle is rescaled so peak |theta|
  equals the block's nominal amplitude REGARDLESS of a2/a3. Without this,
  adding a harmonic also raises the stroke amplitude, and a change in force
  could not be attributed to shape rather than to size -- the sweep would
  confound the very thing it exists to separate.

* Slew feasibility is computed per point, numerically, from the normalised
  waveform. Harmonics raise peak velocity super-linearly (the n-th harmonic
  contributes n*omega*a_n), so a shape that looks harmless can exceed the
  5.5 rad/s limit. Points over budget are FLAGGED, not silently clipped --
  clipping would distort the very waveform under test.

* Randomised run order with periodic baseline re-measurement. The 3D
  campaign's rest offsets drifted (Fz0 moved several N over the campaign),
  so a sweep run in nested order would alias that drift onto whichever knob
  moves slowest. Randomising decorrelates drift from the knobs; the
  repeated baseline measures what drift remains.

* Replicates at the baseline point. Every claim of the form "knob X moved
  feature Y" needs a noise floor to be judged against. Prior hardware runs
  showed waveform_match scatter of about +-0.05 between identical-parameter
  cycles, so effects smaller than the replicate spread are not claimable.

* 3 cycles commanded, only the SECOND analysed. Cycle 1 carries the
  start-up transient and cycle 3 can catch the ramp-down, so the middle
  cycle is the only one that is unambiguously steady-state at the
  commanded parameters. A 4 s quiet delay separates each 3-cycle set so
  fluid motion from the previous point has decayed before the next one
  starts -- otherwise the wake left by one shape biases the next, which
  matters here precisely because the blocks differ only in shape.

* Idle tail after each mission. Per-mission taring needs post-motion quiet
  data -- only 1 of 35 cells in the previous campaign saved a calibration
  block, which forced a rewrite of the analysis. 2 s of idle is cheap
  insurance.

usage: python3 design_harmonic_sweep.py [--out sweep_plan.csv]
"""
import argparse
import itertools
import numpy as np

# ---------------------------------------------------------------- constants
SLEW_LIMIT = 5.5          # rad/s, per servo
PITCH_LIMIT = np.pi       # rad
HEAVE_LIMIT = np.pi / 2   # rad

CENTER_AMP = 0.6283185307   # rad, geometric mean used by the 3D campaign
CENTER_FREQ = 0.5           # Hz

# Baseline operating point: the 1-peak regime, which is where the drag
# target lives and also the regime with the least measured thrust -- so it
# is the hardest case and the one worth characterising.
BASE_FREQ_RATIO = 0.4
BASE_AMP_RATIO = 1.0
BASE_FREQ_SCALE = 1.0

N_CYCLES = 3            # 3 commanded; only cycle 2 is analysed
IDLE_TAIL_S = 2.0
SETTLE_S = 4.0          # quiet delay between each 3-cycle set
N_REPLICATES = 4


def composite(a2, phi2, a3, phi3=0.0, n=4000):
    """One normalised period of fundamental + 2nd + 3rd harmonic.

    Returned with peak |theta| == 1 so amplitude is decoupled from shape.
    """
    t = np.arange(n) / n
    w = 2 * np.pi
    th = (np.sin(w * t)
          + a2 * np.sin(2 * w * t + np.radians(phi2))
          + a3 * np.sin(3 * w * t + np.radians(phi3)))
    peak = np.max(np.abs(th))
    return t, th / (peak if peak > 1e-9 else 1.0)


def peak_velocity(a2, phi2, a3, amp_rad, freq_hz):
    """Max |dtheta/dt| in rad/s for the normalised composite at this amp/freq."""
    t, th = composite(a2, phi2, a3)
    v = np.gradient(th, t) * amp_rad * freq_hz   # t is in periods -> scale by f
    return float(np.max(np.abs(v)))


def kinematics(amp_ratio, freq_ratio, freq_scale):
    """Geometric-mean-preserving decode, matching the 3D campaign exactly."""
    s_a, s_f = np.sqrt(amp_ratio), np.sqrt(freq_ratio)
    pitch_amp = CENTER_AMP * s_a
    heave_amp = CENTER_AMP / s_a
    pitch_freq = CENTER_FREQ * freq_scale / s_f
    heave_freq = CENTER_FREQ * freq_scale * s_f
    return pitch_amp, heave_amp, pitch_freq, heave_freq


def max_feasible_freq_scale(amp_ratio, p_a2=0.0, p_phi2=0.0, p_a3=0.0,
                            h_a2=0.0, h_phi2=0.0, h_a3=0.0,
                            freq_ratio=BASE_FREQ_RATIO):
    """Largest freq_scale that keeps BOTH servos inside the slew limit.

    Peak velocity is linear in freq_scale, so the ceiling is exact rather
    than searched: measure each servo's peak velocity at freq_scale = 1 and
    divide the limit by the larger demand.
    """
    pa, ha, pf, hf = kinematics(amp_ratio, freq_ratio, 1.0)
    pv = peak_velocity(p_a2, p_phi2, p_a3, pa, pf)
    hv = peak_velocity(h_a2, h_phi2, h_a3, ha, hf)
    worst = max(pv, hv)
    return SLEW_LIMIT / worst if worst > 1e-9 else 1.0


def derate_to_slew(point_kwargs, block, label, target):
    """Build a point, lowering freq_scale only as far as slew requires.

    Used for the SHAPE blocks (C-G). There, absolute force is irrelevant --
    the claim under test is that a knob moves a scale-free descriptor
    (asymmetry, skew, width). Harmonics raise peak velocity super-linearly,
    so several shape points would otherwise breach the limit. Slowing the
    gait preserves the waveform shape exactly while bringing velocity into
    budget, whereas clipping would deform the very waveform being tested.
    """
    fs_max = max_feasible_freq_scale(
        point_kwargs.get("amp_ratio", BASE_AMP_RATIO),
        point_kwargs.get("p_a2", 0.0), point_kwargs.get("p_phi2", 0.0),
        point_kwargs.get("p_a3", 0.0), point_kwargs.get("h_a2", 0.0),
        point_kwargs.get("h_phi2", 0.0), point_kwargs.get("h_a3", 0.0))
    fs = min(BASE_FREQ_SCALE, round(fs_max * 0.97, 3))
    return make_point(block, label, target=target, freq_scale=fs, **point_kwargs)


def make_point(block, label, *, amp_ratio=BASE_AMP_RATIO,
               freq_ratio=BASE_FREQ_RATIO, freq_scale=BASE_FREQ_SCALE,
               p_a2=0.0, p_phi2=0.0, p_a3=0.0,
               h_a2=0.0, h_phi2=0.0, h_a3=0.0,
               bias_frac=0.0, target="Fx"):
    pa, ha, pf, hf = kinematics(amp_ratio, freq_ratio, freq_scale)
    pv = peak_velocity(p_a2, p_phi2, p_a3, pa, pf)
    hv = peak_velocity(h_a2, h_phi2, h_a3, ha, hf)
    return {
        "block": block, "label": label, "target": target,
        "amp_ratio": round(amp_ratio, 4), "freq_ratio": round(freq_ratio, 4),
        "freq_scale": round(freq_scale, 3),
        "pitch_amp_rad": round(pa, 4), "heave_amp_rad": round(ha, 4),
        "pitch_freq_hz": round(pf, 4), "heave_freq_hz": round(hf, 4),
        # pitch_bias is expressed as a FRACTION of pitch_amp, so the same
        # sweep means the same thing at every amp_ratio: 1.0 is exactly the
        # point where the stroke stops crossing neutral.
        "bias_frac": round(bias_frac, 3),
        "pitch_bias_rad": round(bias_frac * pa, 4),
        "p_a2": p_a2, "p_phi2": p_phi2, "p_a3": p_a3,
        "h_a2": h_a2, "h_phi2": h_phi2, "h_a3": h_a3,
        "pitch_vpk": round(pv, 3), "heave_vpk": round(hv, 3),
        "slew_ok": bool(pv <= SLEW_LIMIT and hv <= SLEW_LIMIT),
        # bias consumes servo range: the stroke now spans
        # bias +/- pitch_amp, so the far end is what must clear the limit.
        "amp_ok": bool(abs(bias_frac) * pa + pa <= PITCH_LIMIT and ha <= HEAVE_LIMIT),
        "cycles": N_CYCLES,
    }


def build():
    pts = []

    # --- A: magnitude, as a FRONTIER rather than a single line.
    #
    # Frequency, not amplitude: heave already saturates (commanded 1.094 rad
    # -> achieved 0.813) and 2.1x amplitude would exceed the pi/2 heave
    # limit. But how much frequency headroom exists depends strongly on
    # amp_ratio, because peak velocity is amp * 2*pi*freq: at amp_ratio 3.0
    # the pitch servo is ALREADY at 5.41 rad/s of the 5.5 limit with no
    # headroom at all, while at amp_ratio 0.33 it sits at 1.79 rad/s.
    # Since amp_ratio 3.0 is where the best measured 1-peak thrust was found
    # (1.99 N), the highest-thrust operating point is also the one that
    # cannot be sped up -- so the sweep has to trade the two against each
    # other rather than fix one. For each amp_ratio it walks freq_scale up
    # to that ratio's own slew-feasible ceiling.
    for ar in (0.33, 0.67, 1.0, 1.5, 3.0):
        fs_max = max_feasible_freq_scale(ar)
        for frac in (0.55, 0.70, 0.85, 0.98):
            fs = round(fs_max * frac, 3)
            pts.append(make_point("A_magnitude", f"A_ar{ar:.2f}_fs{fs:.2f}",
                                  amp_ratio=ar, freq_scale=fs))

    # --- B: peak count. Spans the measured 1 -> 2 -> 4+ transitions.
    for fr in (0.4, 0.5, 0.667, 1.0, 1.5):
        pts.append(make_point("B_peakcount", f"B_fr{fr:.3f}", freq_ratio=fr))

    # --- C: lobe width via the odd harmonic. a3 values chosen to match the
    # 3rd-harmonic content of pitch_k = 0, 0.5, 1, 2, 4 (measured earlier as
    # 0, 0.091, 0.143, 0.200, 0.250 of the fundamental).
    for a3 in (0.0, 0.091, 0.143, 0.200, 0.250):
        pts.append(derate_to_slew({"p_a3": a3}, "C_width", f"C_a3_{a3:.3f}", "Fx"))

    # --- D: Fx +/- scaling, lobe dominance and skew, on the PITCH servo.
    for a2, phi2 in itertools.product((0.1, 0.2, 0.3), (0, 45, 90, 135, 180, 270)):
        pts.append(derate_to_slew({"p_a2": a2, "p_phi2": phi2},
                                  "D_Fx_shape", f"D_a2{a2:.1f}_ph{phi2:03d}", "Fx"))

    # --- E: the same design on the HEAVE servo, for Fy.
    for a2, phi2 in itertools.product((0.1, 0.2, 0.3), (0, 45, 90, 135, 180, 270)):
        pts.append(derate_to_slew({"h_a2": a2, "h_phi2": phi2},
                                  "E_Fy_shape", f"E_a2{a2:.1f}_ph{phi2:03d}", "Fy"))

    # --- F / G: CROSS-CHANNEL. Shaping one channel while nulling the other.
    #
    # An even harmonic works by making the two half-strokes unequal, and an
    # unequal stroke is precisely what produces a NET force. So the same
    # knob that buys the Fx shape is expected to push Fy's net away from
    # zero -- these blocks measure that leakage and test whether it can be
    # trimmed back out.
    #
    # amp_ratio is the trim knob because it is close to selective: over the
    # 3D grid it moved Fy peak-to-peak by -1.801 N per doubling but Fx by
    # only -0.235 N, a 7.7:1 ratio. So it can pull Fy back toward zero
    # while largely leaving the Fx shape that block D established.
    # freq_ratio is NOT usable for this -- it moves Fx and Fy almost
    # equally (+0.895 vs +1.445, 1.6:1) and additionally changes peak count.
    for a2, ar in itertools.product((0.2, 0.3), (0.33, 0.67, 1.0, 1.5, 3.0)):
        pts.append(derate_to_slew({"p_a2": a2, "p_phi2": 0, "amp_ratio": ar},
                                  "F_Fx_null_Fy", f"F_a2{a2:.1f}_ar{ar:.2f}", "Fx"))
    # G: Fy shaped on its own servo, with Fx as the trim. Kept because the
    # requirement is "either channel at a time", but note the asymmetry in
    # what each block is FOR: F asks "can I hold Fy at net zero while
    # shaping Fx" (the operating case), G asks "can I shape Fy at all".
    for a2, ar in itertools.product((0.2, 0.3), (0.33, 0.67, 1.0, 1.5, 3.0)):
        pts.append(derate_to_slew({"h_a2": a2, "h_phi2": 0, "amp_ratio": ar},
                                  "G_Fy_shape", f"G_a2{a2:.1f}_ar{ar:.2f}", "Fy"))

    # --- H: BIAS. The remaining feature with no knob is SKEW, and the
    # harmonic route to it was dropped, so this tests the other candidate:
    # offsetting the pitch neutral without altering the waveform at all.
    #
    # A position bias does not change velocity (a constant differentiates to
    # zero), so it cannot change the v^2 magnitude directly. What it changes
    # is angle of attack -- the blade meets the heave-induced flow at a
    # different angle on the up- versus down-stroke, so the two half-strokes
    # produce different force. The interesting region is bias ~ pitch_amp,
    # where the sweep stops crossing neutral entirely and one stroke is
    # feathered; the response is expected to be strongly NONLINEAR there,
    # which is why the sampling is fine around 1.0x rather than uniform.
    #
    # Run at two a2 levels so bias is separable from harmonic asymmetry:
    # both produce a one-sided force, and a single-level sweep could not
    # tell which mechanism was responsible.
    for a2 in (0.0, 0.2):
        for frac in (0.0, 0.4, 0.7, 0.9, 1.0, 1.1, 1.3):
            pts.append(derate_to_slew({"p_a2": a2, "p_phi2": 0, "bias_frac": frac},
                                      "H_bias", f"H_a2{a2:.1f}_bias{frac:.1f}", "Fx"))

    # --- R: replicates of the untouched baseline, for the noise floor.
    for i in range(N_REPLICATES):
        pts.append(make_point("R_replicate", f"R_base_{i+1}"))

    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="harmonic_sweep_plan.csv")
    ap.add_argument("--seed", type=int, default=7)
    args = ap.parse_args()

    pts = build()

    # Randomise execution order so slow drift cannot masquerade as a knob
    # effect; keep the replicates spread through the run rather than
    # clustered, so they sample drift instead of one moment of it.
    rng = np.random.default_rng(args.seed)
    order = rng.permutation(len(pts))
    for rank, idx in enumerate(order, 1):
        pts[idx]["run_order"] = rank

    import pandas as pd
    df = pd.DataFrame(pts).sort_values("run_order")
    df.to_csv(args.out, index=False)

    n_bad_slew = int((~df.slew_ok).sum())
    n_bad_amp = int((~df.amp_ok).sum())

    print(f"{len(df)} missions across {df.block.nunique()} blocks -> {args.out}\n")
    print(df.groupby("block").agg(n=("label", "size"),
                                  pitch_vpk_max=("pitch_vpk", "max"),
                                  heave_vpk_max=("heave_vpk", "max")).round(2).to_string())
    print(f"\nslew limit {SLEW_LIMIT} rad/s -> {n_bad_slew} point(s) over budget")
    print(f"amplitude limits -> {n_bad_amp} point(s) over budget")
    if n_bad_slew:
        print("\nOVER SLEW BUDGET (drop, or lower freq_scale for these):")
        print(df[~df.slew_ok][["label", "pitch_vpk", "heave_vpk"]].to_string(index=False))

    est_s = len(df) * (N_CYCLES / (CENTER_FREQ * 0.7) + SETTLE_S + IDLE_TAIL_S)
    print(f"\nestimated rig time: {est_s/60:.0f} min "
          f"({N_CYCLES} cycles, cycle 2 analysed + {SETTLE_S:.0f} s delay + {IDLE_TAIL_S:.0f} s idle tail)")


if __name__ == "__main__":
    main()
