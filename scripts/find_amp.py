#!/usr/bin/env python3
"""
find_amp.py — max thrust-impulse amplitude, for ONE servo axis
==============================================================
Sweeps ONE amplitude axis (pitch or heave) and picks the amplitude with the
largest thrust impulse per cycle.  The OTHER amplitude, the frequency and the
phase are held constant — you are asked for all three.

Run it once per axis (pitch, then heave), then use both winners in find_freq.py.

    python3 find_amp.py                     # run a fresh amplitude sweep
    python3 find_amp.py --from <folder>     # analyze an existing amplitude sweep

Definitions:

  thrust impulse  : ∫Fx dt per cycle [N·s] — NOT max(Fx).  A tall narrow spike
                    is useless; a wide sustained curve moves the robot.
                    Computed as mean_Fx × period, with mean_Fx taken from the
                    harmonic fit's DC term (unbiased under bursty sampling).
                    NOTE: impulse/cycle = mean_Fx/f.  This sweep holds f fixed,
                    so impulse ranks amplitudes identically to mean Fx here —
                    the 1/f is a constant.  It only reorders across frequencies.

A sample must pass every gate to be selectable:
  force quality   : Fz symmetric peak/trough, net Fy ≈ 0
  tracking        : achieved/commanded amplitude ≥ {track:.0%} — a stroke the servo
                    cannot reach is not a datapoint; the slew limit clipped it,
                    so its forces belong to a smaller stroke than requested.
  not jagged      : force high-harmonic (3f+) and broadband energy ≤ {noise:.2f} of
                    the COHERENT 1f/2f signal.  2f is NOT counted as noise: for
                    lift-based propulsion 2f IS the thrust (two peaks per cycle).
                    Mechanical jaggedness lives at 3f and above, and in the
                    non-harmonic residual.

Plots are live matplotlib windows (zoom/pan); nothing is written as an image.
"""

import os, sys, math, argparse
import numpy as np
import matplotlib.pyplot as plt

import analysis_common as ac

D2R = math.pi / 180.0
__doc__ = __doc__.format(noise=ac.NOISE_REJECT, track=ac.TRACK_MIN)


# ===========================================================================
# Report
# ===========================================================================
def report_amp(results, axis):
    print("\n" + "=" * 112)
    print(f"MAX THRUST-IMPULSE {axis.upper()} AMPLITUDE")
    print("  selection = max ∫Fx dt per cycle (N·s), NOT max peak Fx.")
    print("  (frequency is fixed here, so impulse/cycle = mean_Fx/f ranks the same "
          "as mean Fx)")
    print("=" * 112)
    print(f"{'amp':>9} {'imp/cyc':>9} {'mean Fx':>9} {'peak Fx':>9} {'track':>6} "
          f"{'THD':>6} {'jag':>5} {'fz_asym':>8} {'fy_net':>9} {'style':>10}  verdict")
    print("-" * 112)
    usable = []
    for r in results:
        amp = r["pitch_amp"] if axis == "pitch" else r["heave_amp"]
        ok_q, why_q = ac.quality_ok(r)
        ok_t, why_t = ac.tracking_ok(r)
        ok_f, why_f = ac.force_smooth_ok(r)
        why = why_q or why_t or why_f
        if ok_q and ok_t and ok_f:
            usable.append(r); why = "usable"
        print(f"{amp/D2R:8.1f}° {r.get('impulse_per_cycle',float('nan')):9.4f} "
              f"{r.get('mean_Fx',float('nan')):9.4f} "
              f"{r.get('peak_Fx',float('nan')):9.4f} "
              f"{r.get('min_track',float('nan')):6.2f} {r.get('max_thd',float('nan')):6.3f} "
              f"{r.get('noise_ratio',float('nan')):5.2f} "
              f"{r.get('fz_asym',float('nan')):8.3f} {r.get('fy_net',float('nan')):9.4f} "
              f"{r.get('style','—'):>10}  {why}")
    print("-" * 112)
    if not usable:
        print("  !! No usable samples — cannot pick a max thrust-impulse amplitude.")
        if not any(r.get("has_force") for r in results):
            print("     Every mission is missing load-cell data. Check LabVIEW is "
                  "streaming UDP to 192.168.137.1:5005.")
        return None
    best = max(usable, key=lambda r: r["impulse_per_cycle"])
    amp = best["pitch_amp"] if axis == "pitch" else best["heave_amp"]
    print(f"\n  MAX THRUST IMPULSE at {axis} amplitude = {amp:.4f} rad ({amp/D2R:.2f}°)")
    print(f"    impulse {best['impulse_per_cycle']:.4f} N·s/cycle   "
          f"mean Fx {best['mean_Fx']:.4f} N   peak Fx {best['peak_Fx']:.4f} N   "
          f"style {best['style']}")
    # peak-force winner, shown only to expose the difference the change makes
    pk = max(usable, key=lambda r: r.get("peak_Fx", float("-inf")))
    if pk is not best:
        pa = pk["pitch_amp"] if axis == "pitch" else pk["heave_amp"]
        print(f"    (max PEAK Fx would have chosen {pa/D2R:.1f}° instead — a taller "
              f"but narrower Fx curve. Impulse wins: it moves the robot.)")
    rejected = [r for r in results if r not in usable]
    if rejected:
        print(f"    ({len(rejected)} of {len(results)} samples excluded by the "
              f"quality/tracking/jaggedness gates — see the table)")
    amps = sorted((r["pitch_amp"] if axis == "pitch" else r["heave_amp"])
                  for r in usable)
    if amp >= amps[-1] - 1e-9:
        print("    ! this is the LARGEST usable amplitude swept — the true maximum "
              "may lie beyond it. Extend the sweep (slew limit permitting).")
    print(f"\n  -> feed this into find_freq.py as the {axis} amplitude "
          f"(run find_amp.py for the other axis first).")
    return best


def plots_amp(results, axis, best):
    have = [r for r in results if r.get("has_force")]
    if not have:
        print("  (no force data — nothing to plot)"); return
    amps = [(r["pitch_amp"] if axis == "pitch" else r["heave_amp"]) / D2R for r in have]
    fig, ax = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    ax[0].plot(amps, [r.get("impulse_per_cycle", np.nan) for r in have], "o-",
               label="∫Fx dt per cycle (N·s)  ← selection metric")
    ax[0].plot(amps, [r.get("mean_Fx", np.nan) for r in have], "s--", alpha=0.5,
               label="mean Fx (N)")
    ax[0].plot(amps, [r.get("peak_Fx", np.nan) for r in have], "^:", alpha=0.5,
               label="peak Fx (N)  ← NOT used")
    if best is not None:
        b = (best["pitch_amp"] if axis == "pitch" else best["heave_amp"]) / D2R
        ax[0].axvline(b, color="tab:green", ls="--", label=f"max impulse @ {b:.1f}°")
    ax[0].set_ylabel("impulse / force"); ax[0].legend(fontsize=8); ax[0].grid(True, alpha=0.3)
    ax[0].set_title(f"Thrust impulse vs {axis} amplitude")
    ax[1].plot(amps, [r.get("min_track", np.nan) for r in have], "o-",
               label="achieved/commanded")
    ax[1].axhline(ac.TRACK_MIN, color="tab:red", ls="--", label=f"track gate {ac.TRACK_MIN}")
    ax[1].set_ylabel("tracking ratio"); ax[1].legend(fontsize=8); ax[1].grid(True, alpha=0.3)
    ax[2].plot(amps, [r.get("noise_ratio", np.nan) for r in have], "o-", color="tab:red",
               label="force jaggedness (3f+ & broadband)/coherent")
    ax[2].axhline(ac.NOISE_REJECT, color="k", ls="--", label=f"jagged gate {ac.NOISE_REJECT}")
    ax[2].plot(amps, [r.get("fz_asym", np.nan) for r in have], "s-", alpha=0.6,
               label="Fz asymmetry (→0)")
    ax[2].plot(amps, [abs(r.get("fy_ratio", np.nan)) for r in have], "^-", alpha=0.6,
               label="|net Fy|/|mean Fx| (→0)")
    ax[2].set_xlabel(f"{axis} amplitude (deg)"); ax[2].set_ylabel("dimensionless")
    ax[2].legend(fontsize=8); ax[2].grid(True, alpha=0.3)
    fig.tight_layout()
    print("\n  showing plots — close the windows to finish")
    plt.show()


# ===========================================================================
# Sweep
# ===========================================================================
def run_amp_sweep():
    import sweep_common as sc
    sc.banner("AMPLITUDE (max thrust impulse)")
    outdir = sc.ask_outdir()
    axis = sc.ask_choice("sweep which axis", ["pitch", "heave"])

    print("\n  --- constants (held while the swept axis varies) ---")
    other = {}
    if axis == "pitch":
        other["heave_amp"] = sc.ask_float("heave amplitude (rad) [held]", 0.235619449)
    else:
        other["pitch_amp"] = sc.ask_float("pitch amplitude (rad) [held]", 0.235619449)
    frequency = sc.ask_float("frequency (Hz)", 0.75)
    phase     = sc.ask_float("phase (rad)", math.pi / 2)

    print(f"\n  --- sweep ({axis} amplitude) ---")
    cycles    = sc.ask_int("cycles per mission command", 10)
    delay     = sc.ask_delay()
    start     = sc.ask_float("amplitude start (rad)",
                             0.261799 if axis == "pitch" else 0.785398)
    increment = sc.ask_float("increment between samples (rad)", 5 * D2R)
    n         = sc.ask_int("number of samples", 7)

    amps = sc.build_points(start, increment, n)
    tag = "PA" if axis == "pitch" else "HA"
    points = []
    for a in amps:
        p = {"label": f"{tag}_{int(round(a/D2R)):04d}", "frequency": frequency,
             "phase": phase}
        p["pitch_amp"] = a if axis == "pitch" else other["pitch_amp"]
        p["heave_amp"] = a if axis == "heave" else other["heave_amp"]
        points.append(p)

    print(f"\n  {n} missions, {axis} amplitude {amps[0]:.4f}..{amps[-1]:.4f} rad "
          f"({amps[0]/D2R:.1f}°..{amps[-1]/D2R:.1f}°)")
    peak = 2 * math.pi * frequency * amps[-1]
    if peak > 5.5:
        print(f"  ! WARNING: largest sample needs {peak:.2f} rad/s (> 5.5 rad/s slew "
              f"limit) — it will clip and be excluded by the tracking gate.")
    est = n * (cycles / frequency + delay + 5) / 60.0
    print(f"  estimated runtime: ~{est:.0f} min")
    if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        sys.exit(0)

    node = sc.start_ros()
    try:
        sc.run_missions(node, outdir, points, cycles, delay)
    finally:
        sc.stop_ros(node)
    consts = {"frequency (Hz)": frequency, "phase (rad)": phase}
    consts.update({k: v for k, v in other.items()})
    sc.write_info(outdir, f"{axis} amplitude (max thrust impulse)", consts,
                  f"{axis}_amp {amps[0]:.4f}..{amps[-1]:.4f} rad, step {increment}",
                  points, cycles)
    return outdir, axis


def _infer_axis(results):
    """Which amplitude actually varied across the sweep."""
    pv = {r["pitch_amp"] for r in results if r["pitch_amp"] is not None}
    hv = {r["heave_amp"] for r in results if r["heave_amp"] is not None}
    if len(pv) > 1 and len(hv) <= 1:
        return "pitch"
    if len(hv) > 1 and len(pv) <= 1:
        return "heave"
    return "pitch" if len(pv) >= len(hv) else "heave"


def analyze_folder(folder):
    missions = ac.find_missions(folder)
    if not missions:
        sys.exit(f"No mission folders found under {folder}")
    return [r for r in (ac.analyze(mdir, label) for label, mdir in missions) if r]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="src", default=None,
                    help="analyze an existing amplitude-sweep folder instead of running one")
    ap.add_argument("--axis", choices=["pitch", "heave"], default=None,
                    help="with --from: which axis was swept (default: infer from the data)")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    if args.src:
        folder = os.path.abspath(os.path.expanduser(args.src))
        results = analyze_folder(folder)
        axis = args.axis or _infer_axis(results)
    else:
        folder, axis = run_amp_sweep()
        results = analyze_folder(folder)

    results.sort(key=lambda r: (r["pitch_amp"] if axis == "pitch"
                                else r["heave_amp"]) or 0)
    best = report_amp(results, axis)
    if not args.no_plots:
        plots_amp(results, axis, best)


if __name__ == "__main__":
    main()
