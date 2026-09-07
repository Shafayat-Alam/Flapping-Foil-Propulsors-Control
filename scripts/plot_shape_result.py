#!/usr/bin/env python3
"""Plot the outcome of a shape-matching run.

Three panels, chosen to answer the three questions a shape match raises:

  1. The best waveform itself, over one gait cycle, with the trough
     tolerance drawn on it -- does the achieved curve look like what was
     asked for, and by how much does the trough miss?
  2. Trough depth versus phase -- the knob that did the work, shown as a
     curve rather than a number, so a real optimum can be told from a
     search that simply railed at an edge.
  3. Error against trial number, marking the stage boundaries, so the
     convergence is visible as a process rather than asserted.

The waveform is re-derived from the stored RAW capture using exactly the
same window logic as the controller used (analyze_harmonic_sweep), so the
picture and the reported numbers cannot disagree.
"""
import argparse
import csv
import json
import os
import sys
from fractions import Fraction

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
import analyze_harmonic_sweep as aha   # noqa: E402
import design_harmonic_sweep as dhs    # noqa: E402


def cycle_trace(folder, label, freq_ratio, amp_ratio, freq_scale):
    """One analysed cycle of Fx/Fy/Fz from a stored raw capture."""
    path = os.path.join(folder, "data", f"{label}.csv")
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"]
    fs = (len(t) - 1) / (t[-1] - t[0])
    _, _, pf, _ = dhs.kinematics(amp_ratio, freq_ratio, freq_scale)
    beat = Fraction(freq_ratio).limit_denominator(20).denominator / pf

    i0, i1 = aha.motion_window(d["Fx_raw"], fs)
    motion_t0, motion_t1 = t[i0], t[min(i1, len(t) - 1)]
    span = motion_t1 - motion_t0
    mid = 0.5 * (motion_t0 + motion_t1)
    win = min(beat, 0.6 * span)
    t0 = mid - 0.5 * win
    sl = (t >= t0) & (t < t0 + win)

    out = {}
    for ch in ("Fx", "Fy", "Fz"):
        raw = d[f"{ch}_raw"].astype(float)
        tail = raw[i1:]
        base = float(np.mean(tail[len(tail) // 5:])) if len(tail) > int(0.4 * fs) \
            else float(np.mean(raw[:int(0.2 * fs)]))
        out[ch] = aha.lowpass(raw - base, 10.0 * pf, fs)[sl]
    tt = t[sl] - t[sl][0]
    return tt, out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--name", default=None)
    args = ap.parse_args()
    folder = args.folder if os.path.isabs(args.folder) else \
        os.path.join(WORKSPACE_ROOT, args.folder)
    name = args.name or os.path.basename(folder)

    res = json.load(open(os.path.join(folder, "result.json")))
    tgt = res["target"]
    # Use the RE-SCORED history: the live run ranked trials with a peak
    # count normalised by the beat denominator rather than by the window
    # actually analysed, which overstated nothing consistently but did
    # pick a different winner. Re-scoring from the stored raw captures
    # costs nothing and makes the plot agree with the corrected numbers.
    hcorr = os.path.join(folder, "history_corrected.csv")
    hist = list(csv.DictReader(open(hcorr if os.path.exists(hcorr)
                                    else os.path.join(folder, "history.csv"))))
    if "err_corr" in hist[0]:
        for h in hist:
            h["err"] = h["err_corr"]
            h["peaks_per_cycle"] = h["pk_corr"]
            h["trough_frac"] = h["trough_corr"]
            h["pos_peak"], h["neg_peak"] = h["pos"], h["neg"]
        bh = min(hist, key=lambda h: float(h["err"]))
        best = {"label": bh["label"], "err": float(bh["err"]),
                "peaks_per_cycle": float(bh["pk_corr"]),
                "trough_frac": float(bh["trough_corr"]),
                "pos_peak": float(bh["pos"]), "neg_peak": float(bh["neg"]),
                "fy_net": float(bh["fy_net"]), "fy_frac": float(bh["fyf"]),
                "fz_net": float(bh["fz_net"]),
                "params": {k: bh[k] for k in
                           ("freq_ratio", "phase_deg", "amp_ratio", "freq_scale")}}
    else:
        best = res["best"]

    bp = best["params"]
    tt, tr = cycle_trace(folder, best["label"], float(bp["freq_ratio"]),
                         float(bp["amp_ratio"]), float(bp["freq_scale"]))

    fig, ax = plt.subplots(1, 3, figsize=(16, 4.6))

    # --- panel 1: the achieved waveform
    a = ax[0]
    a.plot(tt, tr["Fx"], lw=2.0, color="#1f77b4", label="Fx (thrust)")
    a.plot(tt, tr["Fy"], lw=1.0, color="#ff7f0e", alpha=.75, label="Fy (lateral)")
    a.axhline(0, color="k", lw=.8)
    pk = float(best["pos_peak"])
    a.axhline(-tgt["trough_frac"] * pk, color="crimson", ls="--", lw=1.2,
              label=f"trough tolerance ({tgt['trough_frac']:.2f}x peak)")
    a.axhline(float(best["neg_peak"]), color="crimson", ls=":", lw=1.2,
              label=f"achieved trough ({best['trough_frac']:.2f}x)")
    a.set_xlabel("time within one gait cycle (s)")
    a.set_ylabel("force (N)")
    a.set_title(f"{name}: achieved shape\n"
                f"{best['peaks_per_cycle']:.2f} peaks/cycle "
                f"(target {tgt['peaks_per_cycle']:.0f})")
    a.legend(fontsize=7.5, loc="best")
    a.grid(alpha=.3)

    # --- panel 2: the knob that did the work
    a = ax[1]
    ph = [(float(h["phase_deg"]), float(h["trough_frac"]), float(h["peaks_per_cycle"]))
          for h in hist if h["label"].split("_")[-1].startswith("phase")]
    if ph:
        ph.sort()
        x = [p[0] for p in ph]
        a.plot(x, [p[1] for p in ph], "o-", color="#1f77b4", label="trough / peak")
        a.axhline(tgt["trough_frac"], color="crimson", ls="--", lw=1.2,
                  label="target")
        a.axvline(float(bp["phase_deg"]), color="green", ls=":", lw=1.5,
                  label=f"chosen {float(bp['phase_deg']):.0f}°")
        a2 = a.twinx()
        a2.plot(x, [p[2] for p in ph], "s--", color="#999999", ms=4, lw=1,
                label="peaks/cycle")
        a2.set_ylabel("peaks per cycle", color="#777777", fontsize=9)
        a2.tick_params(axis="y", labelcolor="#777777")
        a.set_xlabel("phase (deg)")
        a.set_ylabel("trough / peak")
        a.set_title("phase rotation: trough depth\n"
                    "(grey = peak count, held while trough moves)")
        a.legend(fontsize=7.5, loc="upper right")
        a.grid(alpha=.3)

    # --- panel 3: convergence
    a = ax[2]
    err = [float(h["err"]) for h in hist]
    a.plot(range(1, len(err) + 1), err, "o-", color="#2ca02c")
    running = np.minimum.accumulate(err)
    a.plot(range(1, len(err) + 1), running, lw=2, color="#d62728",
           label="best so far")
    a.set_xlabel("trial")
    a.set_ylabel("shape error")
    a.set_title(f"convergence: {err[0]:.2f} -> {min(err):.2f}")
    a.legend(fontsize=8)
    a.grid(alpha=.3)

    fig.tight_layout()
    out = os.path.join(folder, f"{name}_shape_result.png")
    fig.savefig(out, dpi=135)
    print(f"wrote {out}")

    print(f"\n{name}: {best['peaks_per_cycle']:.2f} peaks/cycle "
          f"(target {tgt['peaks_per_cycle']:.0f})")
    print(f"  trough {best['trough_frac']:.2f} of peak (target <= {tgt['trough_frac']:.2f})")
    print(f"  +peak {best['pos_peak']:+.3f} N   -peak {best['neg_peak']:+.3f} N")
    print(f"  Fy net {best['fy_net']:+.3f} N ({best['fy_frac']*100:.0f}% of peak)")


if __name__ == "__main__":
    main()
