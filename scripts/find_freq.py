#!/usr/bin/env python3
"""
find_freq.py — max frequency and efficiency frequency
=====================================================
Sweeps frequency with BOTH amplitudes and the phase held constant, and reports
three frequencies:

    [1] max frequency, efficiency ignored
    [2] efficiency-optimal frequency
    [3] max frequency that still keeps ≥{band:.0%} of best efficiency

Run this ONCE, after find_amp.py has given you the pitch and heave amplitudes.

    python3 find_freq.py                    # run a fresh frequency sweep
    python3 find_freq.py --from <folder>    # analyze an existing frequency sweep

Definitions:

  thrust impulse  : ∫Fx dt per cycle [N·s] — NOT max(Fx).  A tall narrow spike is
                    useless; a wide sustained curve moves the robot.
                    impulse/cycle = mean_Fx/f, so unlike the amplitude sweep, the
                    1/f here genuinely reorders the samples: it is per-cycle work,
                    while mean Fx is per-second thrust.  Both are shown.

  efficiency      : ∫Fx dt / ∫(V·I) dt  [N·s/J], both servos billed.
                    ∫I dt is CHARGE (coulombs), not energy — voltage is recorded,
                    so true joules are used.  The charge form and the heave-only
                    form are reported alongside.
                    The integration window cancels, so this equals
                    mean_Fx / mean_power exactly.

A sample must pass every gate to be selectable:
  force quality   : Fz symmetric peak/trough, net Fy ≈ 0
  tracking        : achieved/commanded amplitude ≥ {track:.0%}
  smooth curve    : position THD ≤ {thd:.2f}.  A clean tracked sine has THD ≈ 0;
                    slew clipping or resonance injects harmonics and raises it.
                    ("sharp edges → possible resonance → discard")
  not jagged      : force high-harmonic (3f+) and broadband energy ≤ {noise:.2f} of
                    the COHERENT 1f/2f signal.  2f is NOT counted as noise: for
                    lift-based propulsion 2f IS the thrust (two peaks per cycle).

Plots are live matplotlib windows (zoom/pan); nothing is written as an image.
"""

import os, sys, math, argparse
import numpy as np
import matplotlib.pyplot as plt

import analysis_common as ac

D2R = math.pi / 180.0
SLEW_LIMIT = 5.5      # rad/s — measured servo slew ceiling
__doc__ = __doc__.format(thd=ac.THD_SMOOTH, noise=ac.NOISE_REJECT,
                         track=ac.TRACK_MIN, band=ac.EFF_BAND)


# ===========================================================================
# Report
# ===========================================================================
def report_freq(results):
    print("\n" + "=" * 118)
    print("MAX FREQUENCY (ignoring efficiency) and EFFICIENCY-AWARE FREQUENCY")
    print("  efficiency = ∫Fx dt / ∫(V·I) dt  [N·s/J], both servos billed")
    print("=" * 118)
    print(f"{'freq':>8} {'imp/cyc':>9} {'mean Fx':>9} {'J/cyc':>8} {'eff N·s/J':>10} "
          f"{'eff(chg)':>9} {'track':>6} {'THD':>6} {'jag':>5} {'style':>10}  verdict")
    print("-" * 118)
    ok_rows = []
    for r in sorted(results, key=lambda r: r["frequency"] or 0):
        ok_q, why_q = ac.quality_ok(r)
        ok_t, why_t = ac.tracking_ok(r)
        ok_s, why_s = ac.smooth_ok(r)
        ok_f, why_f = ac.force_smooth_ok(r)
        why = why_q or why_t or why_s or why_f
        if ok_q and ok_t and ok_s and ok_f:
            ok_rows.append(r); why = "usable"
        print(f"{r['frequency']:7.3f}H {r.get('impulse_per_cycle',float('nan')):9.4f} "
              f"{r.get('mean_Fx',float('nan')):9.4f} "
              f"{r.get('energy_per_cycle',float('nan')):8.3f} "
              f"{r.get('efficiency_energy',float('nan')):10.4f} "
              f"{r.get('efficiency_charge',float('nan')):9.4f} "
              f"{r.get('min_track',float('nan')):6.2f} {r.get('max_thd',float('nan')):6.3f} "
              f"{r.get('noise_ratio',float('nan')):5.2f} "
              f"{r.get('style','—'):>10}  {why}")
    print("-" * 118)

    if not ok_rows:
        print("  !! No frequency passed the quality/tracking/smoothness/jaggedness gates.")
        if not any(r.get("has_force") for r in results):
            print("     Every mission is missing load-cell data. Check LabVIEW is "
                  "streaming UDP to 192.168.137.1:5005.")
        return None, None, None

    # ---- [1] MAX FREQUENCY, efficiency ignored entirely --------------------
    fmax = max(ok_rows, key=lambda r: r["frequency"])
    print(f"\n  [1] MAX FREQUENCY (efficiency ignored) = {fmax['frequency']:.4f} Hz")
    print(f"      highest frequency that still holds a smooth curve and tracks the "
          f"command.")
    print(f"      THD {fmax.get('max_thd',float('nan')):.3f}, tracking "
          f"{fmax.get('min_track',float('nan')):.0%}, "
          f"impulse {fmax.get('impulse_per_cycle',float('nan')):.4f} N·s/cycle, "
          f"efficiency {fmax.get('efficiency_energy',float('nan')):.4f} N·s/J")
    swept_max = max(r["frequency"] for r in results)
    if fmax["frequency"] >= swept_max - 1e-9:
        print("      ! this is the HIGHEST frequency swept and it still passed — the "
              "real ceiling is above it. Extend the sweep to find where it breaks.")

    eff_rows = [r for r in ok_rows
                if r.get("efficiency_energy") == r.get("efficiency_energy")]
    if not eff_rows:
        print("\n  (no current/voltage data — efficiency frequencies unavailable)")
        return fmax, None, None

    # ---- [2] EFFICIENCY PEAK ----------------------------------------------
    feff = max(eff_rows, key=lambda r: r["efficiency_energy"])
    print(f"\n  [2] EFFICIENCY-OPTIMAL FREQUENCY = {feff['frequency']:.4f} Hz")
    print(f"      peak of ∫Fx dt / ∫(V·I) dt = {feff['efficiency_energy']:.4f} N·s/J "
          f"({feff.get('impulse_per_cycle',float('nan')):.4f} N·s ÷ "
          f"{feff.get('energy_per_cycle',float('nan')):.3f} J per cycle)")
    if feff.get("efficiency_charge_heave") == feff.get("efficiency_charge_heave"):
        print(f"      heave-only charge variant (as literally specified): "
              f"{feff['efficiency_charge_heave']:.4f} N·s/C — ignores what pitch costs")
    eff_sorted = sorted(eff_rows, key=lambda r: r["frequency"])
    if feff is eff_sorted[-1] and len(eff_sorted) > 1:
        print("      ! boundary, not a peak: efficiency was still rising where the "
              "sweep ended. Extend it to find the real optimum.")
    elif feff is eff_sorted[0] and len(eff_sorted) > 1:
        print("      ! boundary, not a peak: efficiency was still rising toward the "
              "bottom of the range. Extend the sweep downward.")

    # ---- [3] MAX FREQUENCY THAT KEEPS NEAR-BEST EFFICIENCY ----------------
    thr = ac.EFF_BAND * feff["efficiency_energy"]
    band = [r for r in eff_rows if r["efficiency_energy"] >= thr]
    fmax_eff = max(band, key=lambda r: r["frequency"]) if band else feff
    print(f"\n  [3] MAX FREQUENCY KEEPING ≥{ac.EFF_BAND:.0%} OF BEST EFFICIENCY = "
          f"{fmax_eff['frequency']:.4f} Hz")
    print(f"      the fastest you can run without giving up efficiency: "
          f"{fmax_eff['efficiency_energy']:.4f} N·s/J "
          f"({fmax_eff['efficiency_energy']/feff['efficiency_energy']:.0%} of best), "
          f"impulse {fmax_eff.get('impulse_per_cycle',float('nan')):.4f} N·s/cycle")
    if fmax_eff["frequency"] < fmax["frequency"]:
        print(f"      running at [1] {fmax['frequency']:.3f} Hz instead costs "
              f"{100*(1-fmax.get('efficiency_energy',0)/feff['efficiency_energy']):.0f}% "
              f"of peak efficiency.")
    else:
        print(f"      same as [1] — efficiency never falls below the band across the "
              f"usable range.")

    sharp = [r for r in results if not ac.smooth_ok(r)[0]]
    if sharp:
        print(f"\n  discarded as SHARP position curve (possible resonance, THD > "
              f"{ac.THD_SMOOTH:.2f}): "
              + ", ".join(f"{r['frequency']:.2f}Hz(THD {r['max_thd']:.2f})" for r in sharp))
    jag = [r for r in results if not ac.force_smooth_ok(r)[0]]
    if jag:
        print(f"  discarded as JAGGED force (3f+/broadband > {ac.NOISE_REJECT:.2f} of "
              f"coherent 1f/2f signal — suspect mechanical failure): "
              + ", ".join(f"{r['frequency']:.2f}Hz({r['noise_ratio']:.2f})" for r in jag))
    return fmax, feff, fmax_eff


def plots_freq(results, fmax, feff, fmax_eff):
    have = [r for r in results if r.get("has_force")]
    if not have:
        print("  (no force data — nothing to plot)"); return
    have.sort(key=lambda r: r["frequency"])
    fs = [r["frequency"] for r in have]

    def _marks(ax):
        if fmax is not None:
            ax.axvline(fmax["frequency"], color="tab:green", ls="--", alpha=0.8,
                       label=f"[1] max f @ {fmax['frequency']:.2f} Hz")
        if feff is not None:
            ax.axvline(feff["frequency"], color="tab:purple", ls="--", alpha=0.8,
                       label=f"[2] efficiency peak @ {feff['frequency']:.2f} Hz")
        if fmax_eff is not None:
            ax.axvline(fmax_eff["frequency"], color="tab:orange", ls=":", alpha=0.9,
                       label=f"[3] max f @ ≥{ac.EFF_BAND:.0%} eff @ "
                             f"{fmax_eff['frequency']:.2f} Hz")

    fig, ax = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    ax[0].plot(fs, [r.get("impulse_per_cycle", np.nan) for r in have], "o-",
               label="∫Fx dt per cycle (N·s)")
    ax[0].plot(fs, [r.get("mean_Fx", np.nan) for r in have], "s--", alpha=0.5,
               label="mean Fx (N) — impulse×f")
    _marks(ax[0])
    ax[0].set_ylabel("impulse / thrust"); ax[0].grid(True, alpha=0.3); ax[0].legend(fontsize=8)
    ax[0].set_title("Frequency sweep")

    ax[1].plot(fs, [r.get("efficiency_energy", np.nan) for r in have], "o-",
               color="tab:purple", label="∫Fx dt / ∫(V·I) dt  (N·s/J)")
    if feff is not None:
        ax[1].axhline(ac.EFF_BAND * feff["efficiency_energy"], color="tab:orange",
                      ls=":", label=f"{ac.EFF_BAND:.0%} of best")
    _marks(ax[1])
    ax[1].set_ylabel("efficiency (N·s/J)"); ax[1].grid(True, alpha=0.3); ax[1].legend(fontsize=8)

    ax[2].plot(fs, [r.get("max_thd", np.nan) for r in have], "o-", label="position THD")
    ax[2].axhline(ac.THD_SMOOTH, color="tab:red", ls="--",
                  label=f"sharp gate {ac.THD_SMOOTH}")
    ax[2].plot(fs, [r.get("min_track", np.nan) for r in have], "^-", alpha=0.6,
               label="tracking ratio")
    ax[2].axhline(ac.TRACK_MIN, color="tab:brown", ls="--", alpha=0.6,
                  label=f"track gate {ac.TRACK_MIN}")
    _marks(ax[2])
    ax[2].set_ylabel("THD / tracking"); ax[2].grid(True, alpha=0.3); ax[2].legend(fontsize=8)

    ax[3].plot(fs, [r.get("noise_ratio", np.nan) for r in have], "o-", color="tab:red",
               label="force jaggedness: (3f+ & broadband) / coherent 1f,2f")
    ax[3].axhline(ac.NOISE_REJECT, color="k", ls="--",
                  label=f"reject gate {ac.NOISE_REJECT}")
    _marks(ax[3])
    ax[3].set_xlabel("frequency (Hz)"); ax[3].set_ylabel("noise ratio")
    ax[3].grid(True, alpha=0.3); ax[3].legend(fontsize=8)
    fig.tight_layout()
    print("\n  showing plots — close the windows to finish")
    plt.show()


# ===========================================================================
# Sweep
# ===========================================================================
def run_freq_sweep():
    import sweep_common as sc
    sc.banner("FREQUENCY (max + efficiency)")
    outdir = sc.ask_outdir()
    print("\n  --- constants (use the amplitudes find_amp.py found for each axis) ---")
    pitch_amp = sc.ask_float("pitch amplitude (rad)", 0.235619449)
    heave_amp = sc.ask_float("heave amplitude (rad)", 0.235619449)
    phase     = sc.ask_float("phase (rad)", math.pi / 2)

    largest = max(pitch_amp, heave_amp)
    f_slew = SLEW_LIMIT / (2 * math.pi * largest) if largest > 1e-6 else float("inf")
    print(f"\n  slew note: largest amplitude {largest:.4f} rad -> servo follows to "
          f"~{f_slew:.3f} Hz; above that the stroke clips and the sample is excluded "
          f"by the tracking gate.")

    print("\n  --- sweep (frequency) ---")
    cycles    = sc.ask_int("cycles per mission command", 10)
    delay     = sc.ask_delay()
    start     = sc.ask_float("frequency start (Hz)", round(max(0.25, f_slew * 0.4), 3))
    increment = sc.ask_float("increment between samples (Hz)",
                             round(max(0.05, (f_slew - max(0.25, f_slew * 0.4)) / 5), 3))
    n         = sc.ask_int("number of samples", 6)

    freqs = sc.build_points(start, increment, n)
    points = [{"label": f"FQ_{int(round(f*1000)):05d}", "frequency": f,
               "pitch_amp": pitch_amp, "heave_amp": heave_amp, "phase": phase}
              for f in freqs]

    print(f"\n  {n} missions, frequency {freqs[0]:.4f}..{freqs[-1]:.4f} Hz")
    over = [f for f in freqs if 2 * math.pi * f * largest > SLEW_LIMIT]
    if over:
        print(f"  ! WARNING: {len(over)} sample(s) exceed the slew limit "
              f"({', '.join(f'{f:.3f}Hz' for f in over)}) — they will clip and be "
              f"excluded by the tracking gate.")
    est = sum(cycles / f for f in freqs) / 60.0 + n * (delay + 5) / 60.0
    print(f"  estimated runtime: ~{est:.0f} min")
    if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        sys.exit(0)

    node = sc.start_ros()
    try:
        sc.run_missions(node, outdir, points, cycles, delay)
    finally:
        sc.stop_ros(node)
    sc.write_info(outdir, "frequency (max + efficiency)",
                  {"pitch_amp (rad)": pitch_amp, "heave_amp (rad)": heave_amp,
                   "phase (rad)": phase},
                  f"frequency {freqs[0]:.4f}..{freqs[-1]:.4f} Hz, step {increment}",
                  points, cycles)
    return outdir


def analyze_folder(folder):
    missions = ac.find_missions(folder)
    if not missions:
        sys.exit(f"No mission folders found under {folder}")
    return [r for r in (ac.analyze(mdir, label) for label, mdir in missions) if r]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="src", default=None,
                    help="analyze an existing frequency-sweep folder instead of running one")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    folder = (os.path.abspath(os.path.expanduser(args.src)) if args.src
              else run_freq_sweep())
    results = analyze_folder(folder)
    fmax, feff, fmax_eff = report_freq(results)
    if not args.no_plots:
        plots_freq(results, fmax, feff, fmax_eff)


if __name__ == "__main__":
    main()
