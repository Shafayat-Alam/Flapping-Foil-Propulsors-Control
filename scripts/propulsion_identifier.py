#!/usr/bin/env python3
"""
propulsion_identifier.py — drag-based vs lift-based, per phase shift
====================================================================
Runs a phase-shift sweep (or re-reads one you already ran) and identifies the
propulsion style each phase produces, from the load-cell force data.

    python3 propulsion_identifier.py                 # run a fresh phase sweep
    python3 propulsion_identifier.py --from <folder> # analyze an existing sweep

How the style is identified, per cycle (cycles 3..N, the first 3 discarded):

  drag-based : ONE Fx peak, then Fx ≈ 0 for the rest of the cycle
               → the thrust energy sits at the flapping frequency, 1f
  lift-based : TWO symmetric Fx peaks (possibly with troughs)
               → the thrust energy sits at 2f; a symmetric peak pair cancels
                 the fundamental, so r21 = A(2f)/A(1f) goes large

r21 is the primary discriminator; the peak count on the phase-averaged cycle
and the fraction of the cycle spent near zero Fx corroborate it.

Independent of style, these must hold — they are reported as quality checks,
NOT used to pick the style:
  * Fz symmetric peak/trough   (fz_asym → 0)
  * net Fy ≈ 0                 (fy_net  → 0)

Plots are live matplotlib windows (zoom/pan); nothing is written as an image.
"""

import os, sys, math, argparse
import numpy as np
import matplotlib.pyplot as plt

import analysis_common as ac

D2R = math.pi / 180.0


# ===========================================================================
# Report
# ===========================================================================
def report(results):
    print("\n" + "=" * 108)
    print("PROPULSION STYLE BY PHASE SHIFT   (cycles "
          f"{ac.IGNORE_CYCLES}..N; r21 = Fx 2f/1f amplitude ratio)")
    print("=" * 108)
    hdr = (f"{'phase':>10} {'style':>10} {'conf':>5} {'r21':>9} {'pk':>3} "
           f"{'dead':>6} {'mean Fx':>9} {'fz_asym':>8} {'fy_net':>9} {'Fx@2f lag':>10}")
    print(hdr); print("-" * 108)
    for r in results:
        if not r.get("has_force"):
            print(f"{r['phase']/D2R:9.1f}° {'—':>10} {'':>5} {'no load-cell data':>40}")
            continue
        r21 = r.get("r21", float("nan"))
        r21s = "   inf" if not np.isfinite(r21) else f"{r21:9.2f}"
        lag = r.get("Fx_phase2", float("nan"))
        lags = f"{lag/D2R:9.1f}°" if lag == lag else "        —"
        print(f"{r['phase']/D2R:9.1f}° {r['style']:>10} {r['confidence']:5.2f} {r21s} "
              f"{r.get('n_peaks',0):3d} {r.get('dead_frac',float('nan')):6.2f} "
              f"{r.get('mean_Fx',float('nan')):9.4f} {r.get('fz_asym',float('nan')):8.3f} "
              f"{r.get('fy_net',float('nan')):9.4f} {lags}")
    print("-" * 108)

    have = [r for r in results if r.get("has_force")]
    if not have:
        print("\n  !! No load-cell data in ANY mission — style cannot be identified.")
        print("     Check that LabVIEW is streaming UDP to 192.168.137.1:5005.")
        return
    for style in ("lift", "drag"):
        hits = [r for r in have if r["style"] == style]
        if hits:
            best = max(hits, key=lambda r: r["confidence"])
            phs = ", ".join(f"{r['phase']/D2R:.0f}°" for r in hits)
            print(f"  {style}-based at: {phs}")
            print(f"      strongest: {best['phase']/D2R:.0f}°  ({best['reason']})")
    amb = [r for r in have if r["style"] == "ambiguous"]
    if amb:
        print(f"  ambiguous at: {', '.join(f'{r['phase']/D2R:.0f}°' for r in amb)}")

    # quality flags — loud, because they invalidate the physics if violated
    print("\n  quality checks (should hold at every phase):")
    bad_fz = [r for r in have if r.get("fz_asym", 0) > 0.25]
    bad_fy = [r for r in have if abs(r.get("fy_ratio", 0)) > 0.25]
    print("    Fz peak/trough symmetry: "
          + ("OK" if not bad_fz else
             "VIOLATED at " + ", ".join(f"{r['phase']/D2R:.0f}°" for r in bad_fz)))
    print("    net Fy ≈ 0 (vs mean Fx): "
          + ("OK" if not bad_fy else
             "VIOLATED at " + ", ".join(f"{r['phase']/D2R:.0f}°" for r in bad_fy)))
    clipped = [r for r in have if r.get("min_track", 1.0) < ac.TRACK_MIN]
    if clipped:
        print(f"    ! servo under-tracked (<{ac.TRACK_MIN:.0%} of commanded stroke) at: "
              + ", ".join(f"{r['phase']/D2R:.0f}°" for r in clipped)
              + "\n      forces there reflect a SMALLER stroke than commanded.")

    # a measured fundamental far from the command means bad timing or bad tracking
    for r in have[:1]:
        m = r["_mission"]
        fdom = ac.dominant_freq(m["t_lc"], m["Fx"])
        exp = r["frequency"]
        if fdom == fdom and min(abs(fdom - exp), abs(fdom - 2 * exp)) > 0.25 * exp:
            print(f"\n  ! measured Fx fundamental {fdom:.2f} Hz matches neither "
                  f"{exp:.2f} Hz nor {2*exp:.2f} Hz — suspect the load-cell rate "
                  f"used by split_missions (--loadcell-rate) or servo tracking.")


# ===========================================================================
# Plots (live)
# ===========================================================================
def plots(results):
    have = [r for r in results if r.get("has_force") and r.get("_cls", {}).get("fold")]
    if not have:
        print("\n  (no force data — nothing to plot)")
        return

    # 1. folded Fx per phase — the shape the classification is actually reading
    n = len(have)
    cols = min(4, n); rows = int(math.ceil(n / cols))
    fig, axes = plt.subplots(rows, cols, figsize=(4 * cols, 2.8 * rows),
                             squeeze=False, sharex=True)
    for i, r in enumerate(have):
        ax = axes[i // cols][i % cols]
        ph, mean, std = r["_cls"]["fold"]
        ax.plot(ph, mean, lw=1.4)
        ax.fill_between(ph, mean - std, mean + std, alpha=0.2)
        ax.axhline(0, color="k", lw=0.6)
        col = {"lift": "tab:green", "drag": "tab:red"}.get(r["style"], "tab:gray")
        ax.set_title(f"{r['phase']/D2R:.0f}° — {r['style']} (r21={r.get('r21',0):.2f})",
                     fontsize=9, color=col)
        ax.grid(True, alpha=0.3)
    for j in range(n, rows * cols):
        axes[j // cols][j % cols].axis("off")
    fig.suptitle("Fx over one averaged cycle (cycles "
                 f"{ac.IGNORE_CYCLES}..N folded)  —  1 peak = drag, 2 peaks = lift",
                 fontsize=11)
    fig.supxlabel("cycle phase (0..1)"); fig.supylabel("Fx (N)")
    fig.tight_layout()

    # 2. discriminator + thrust vs phase
    fig2, ax2 = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    phs = [r["phase"] / D2R for r in have]
    r21 = [min(r.get("r21", np.nan), 100) for r in have]     # clip inf for display
    ax2[0].plot(phs, r21, "o-")
    ax2[0].axhline(ac.R21_LIFT, color="tab:green", ls="--", label=f"lift  (r21≥{ac.R21_LIFT})")
    ax2[0].axhline(ac.R21_DRAG, color="tab:red", ls="--", label=f"drag  (r21≤{ac.R21_DRAG})")
    ax2[0].set_yscale("log"); ax2[0].set_ylabel("r21 = Fx A(2f)/A(1f)")
    ax2[0].set_title("Propulsion style discriminator vs phase shift  (clipped at 100)")
    ax2[0].grid(True, alpha=0.3, which="both"); ax2[0].legend()
    ax2[1].plot(phs, [r.get("mean_Fx", np.nan) for r in have], "o-", label="mean Fx (thrust)")
    ax2[1].plot(phs, [r.get("fy_net", np.nan) for r in have], "s-", label="net Fy (→0)")
    ax2[1].plot(phs, [r.get("fz_net", np.nan) for r in have], "^-", label="net Fz (→0)")
    ax2[1].axhline(0, color="k", lw=0.6)
    ax2[1].set_xlabel("commanded phase shift (deg)"); ax2[1].set_ylabel("force (N)")
    ax2[1].grid(True, alpha=0.3); ax2[1].legend()
    fig2.tight_layout()

    # 3. quality
    fig3, ax3 = plt.subplots(figsize=(10, 4))
    ax3.plot(phs, [r.get("fz_asym", np.nan) for r in have], "o-", label="Fz asymmetry (→0)")
    ax3.plot(phs, [abs(r.get("fy_ratio", np.nan)) for r in have], "s-",
             label="|net Fy| / |mean Fx| (→0)")
    ax3.axhline(0.25, color="k", ls=":", label="0.25 flag line")
    ax3.set_xlabel("commanded phase shift (deg)"); ax3.set_ylabel("dimensionless")
    ax3.set_title("Force-quality invariants vs phase shift")
    ax3.grid(True, alpha=0.3); ax3.legend()
    fig3.tight_layout()

    print("\n  showing plots — close the windows to finish")
    plt.show()


# ===========================================================================
# Main
# ===========================================================================
def run_sweep():
    """Run a fresh phase sweep, reusing the sweep machinery."""
    import sweep_common as sc
    sc.banner("PHASE SHIFT (propulsion identification)")
    outdir = sc.ask_outdir()
    print("\n  --- constants ---")
    pitch_amp = sc.ask_float("pitch amplitude (rad)", 0.235619449)
    heave_amp = sc.ask_float("heave amplitude (rad)", 0.235619449)
    frequency = sc.ask_float("frequency (Hz)", 0.75)
    print("\n  --- sweep (phase) ---")
    cycles    = sc.ask_int("cycles per mission command", 10)
    delay     = sc.ask_delay()
    start     = sc.ask_float("phase start (rad)", 0.0)
    increment = sc.ask_float("increment between samples (rad)", 0.174533)
    n         = sc.ask_int("number of samples", 19)

    phases = sc.build_points(start, increment, n)
    points = [{"label": f"PH_{int(round(p / D2R)):04d}", "frequency": frequency,
               "pitch_amp": pitch_amp, "heave_amp": heave_amp, "phase": p}
              for p in phases]
    est = n * (cycles / frequency + delay + 5) / 60.0
    print(f"\n  {n} missions, phase {phases[0]:.4f}..{phases[-1]:.4f} rad; ~{est:.0f} min")
    if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        sys.exit(0)

    node = sc.start_ros()
    try:
        sc.run_missions(node, outdir, points, cycles, delay)
    finally:
        sc.stop_ros(node)
    sc.write_info(outdir, "phase shift (propulsion identification)",
                  {"pitch_amp (rad)": pitch_amp, "heave_amp (rad)": heave_amp,
                   "frequency (Hz)": frequency},
                  f"phase: {phases[0]:.4f}..{phases[-1]:.4f} rad, step {increment}",
                  points, cycles)
    return outdir


def analyze_folder(folder):
    missions = ac.find_missions(folder)
    if not missions:
        sys.exit(f"No mission folders found under {folder}")
    results = []
    for label, mdir in missions:
        r = ac.analyze(mdir, label)
        if r is None:
            print(f"  (skipped {label}: no feedback CSV)"); continue
        r.update(ac.phase_vs_motion(r["_mission"]))
        results.append(r)
    results.sort(key=lambda r: (r["phase"] if r["phase"] is not None else 0))
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from", dest="src", default=None,
                    help="analyze an existing phase-sweep folder instead of running one")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    folder = os.path.abspath(os.path.expanduser(args.src)) if args.src else run_sweep()
    results = analyze_folder(folder)
    report(results)
    if not args.no_plots:
        plots(results)


if __name__ == "__main__":
    main()
