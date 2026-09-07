#!/usr/bin/env python3
"""What the thrust objective actually bought.

Three panels, because "more thrust" is three separate claims:

  1. Fx over one cycle, start gait versus best gait, with each one's CYCLE
     MEAN drawn as a horizontal line. Net thrust is that mean -- not the
     crest, and not crest-minus-trough -- so drawing it is the only honest way
     to show the quantity being maximised.

  2. Fy over the same cycle, with the null tolerance band. Thrust bought by
     letting the vertical channel drift is not thrust that was asked for, so
     the constraint is shown being kept rather than asserted.

  3. Net thrust against evaluation, with the running best. Points outside the
     null tolerance are marked hollow: they are gaits the optimiser SAW and
     REJECTED, and hiding them would make the search look tidier than it was.

usage:  plot_thrust_result.py <folder> [out.png]
"""
from __future__ import annotations

import glob
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_analyze as AN      # noqa: E402
import amfm_shaper as SH       # noqa: E402


def scan(folder):
    """Every capture, scored with the same objective the run used."""
    out = []
    for fp in glob.glob(os.path.join(folder, "data", "*_force.csv")):
        m = SH.evaluate((fp.replace("_force", "_kin"), fp), "Fx", None)
        if m is None:
            continue
        err, terms = SH.thrust_error(m)
        base = os.path.basename(fp)[:-10]
        try:
            ev = int(base.split("_")[-1])
        except ValueError:
            ev = 0
        out.append({"ev": ev, "label": base, "err": err, "path": fp,
                    "net": terms["net_thrust"], "other": terms["other_bias"],
                    "excess": terms["null_excess"],
                    "crest": m.get("crest_height", 0.0),
                    "trough": m.get("trough_depth", 0.0)})
    out.sort(key=lambda r: r["ev"])
    return out


def trace(path):
    r = AN.load_force(path)
    if r is None:
        return None
    t, fx, fy, fz = r
    return np.linspace(0, 1, len(fx), endpoint=False), fx, fy


def main():
    folder = sys.argv[1] if len(sys.argv) > 1 else "amfm_thrust"
    fpath = folder if os.path.isabs(folder) else os.path.join(WORKSPACE_ROOT, folder)
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(fpath, "thrust_result.png")

    rows = scan(fpath)
    if not rows:
        print("no captures found")
        return 1
    start = rows[0]
    tol = SH.THRUST_SPEC["other_tol"]
    # Report the best gait that actually KEEPS the null. The hinge penalty is
    # deliberately soft, so the lowest-scoring gait overall can sit slightly
    # outside tolerance when the thrust gain outweighs it -- useful for the
    # search, misleading as a headline. Both are printed.
    feasible = [r for r in rows if r["excess"] <= 0]
    best = min(feasible or rows, key=lambda r: r["err"])
    best_any = min(rows, key=lambda r: r["err"])

    fig = plt.figure(figsize=(13.5, 7.2))
    gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], hspace=0.42, wspace=0.22)
    ax1, ax2, ax3 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, :])
    fig.suptitle("Thrust objective: maximise net Fx, hold vertical at zero",
                 fontsize=14, y=0.97)

    for ax, ch, title in ((ax1, 1, "Fx  —  thrust"), (ax2, 2, "Fy  —  vertical")):
        for rec, col, nm in ((start, "#6b7280", f"start ({start['label']})"),
                             (best, "#c2410c", f"best (eval {best['ev']})")):
            tr = trace(rec["path"])
            if tr is None:
                continue
            x, fx, fy = tr
            y = fx if ch == 1 else fy
            ax.plot(x, y, color=col, lw=1.5, label=nm)
            ax.axhline(np.mean(y), color=col, lw=1.4, ls=":",
                       label=f"   mean {np.mean(y):+.3f} N")
        ax.axhline(0, color="0.75", lw=0.8)
        if ch == 2:
            ax.axhspan(-tol, tol, color="#16a34a", alpha=0.10, lw=0,
                       label=f"null tolerance ±{tol:.2f} N")
        ax.set_title(title, fontsize=11)
        ax.set_xlabel("cycle phase")
        ax.set_ylabel("N")
        ax.legend(fontsize=8, loc="upper right", framealpha=0.9)
        ax.margins(x=0)

    ok = [r for r in rows if r["excess"] <= 0]
    bad = [r for r in rows if r["excess"] > 0]
    ax3.scatter([r["ev"] for r in ok], [r["net"] for r in ok], s=26,
                color="#c2410c", label="null satisfied", zorder=3)
    ax3.scatter([r["ev"] for r in bad], [r["net"] for r in bad], s=26,
                facecolors="none", edgecolors="#9ca3af",
                label="null violated (rejected)", zorder=2)
    # NaN, not a sentinel, until the first null-satisfying gait exists -- a
    # large negative placeholder gets drawn and flattens the whole axis.
    run, run_best = [], np.nan
    for r in rows:
        if r["excess"] <= 0:
            run_best = r["net"] if np.isnan(run_best) else max(run_best, r["net"])
        run.append(run_best)
    ax3.step([r["ev"] for r in rows], run, where="post", color="#0f766e",
             lw=1.8, label="best so far", zorder=4)
    ax3.axhline(start["net"], color="#6b7280", ls="--", lw=1.2,
                label=f"start {start['net']:+.3f} N")
    ax3.set_title(f"net thrust per evaluation   —   best {best['net']:+.3f} N "
                  f"at eval {best['ev']}  "
                  f"({100*(best['net']/max(start['net'],1e-9)-1):+.0f}% vs start)",
                  fontsize=11)
    ax3.set_xlabel("evaluation")
    ax3.set_ylabel("net Fx  [N]")
    ax3.legend(fontsize=8.5, loc="lower right", ncol=2, framealpha=0.9)
    ax3.grid(alpha=0.25)

    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"wrote {out}")
    print(f"   start: net {start['net']:+.3f}  vert {start['other']:+.3f}  "
          f"crest {start['crest']:.3f}  trough {start['trough']:.3f}")
    print(f"   best (null kept): net {best['net']:+.3f}  vert {best['other']:+.3f}  "
          f"crest {best['crest']:.3f}  trough {best['trough']:.3f}  "
          f"ratio {best['trough']/max(best['crest'],1e-9):.3f}")
    if best_any["ev"] != best["ev"]:
        print(f"   best (any)      : net {best_any['net']:+.3f}  "
              f"vert {best_any['other']:+.3f}  "
              f"-> OUTSIDE the ±{tol:.2f} N null by {best_any['excess']:.3f} N")
    return 0


if __name__ == "__main__":
    sys.exit(main())
