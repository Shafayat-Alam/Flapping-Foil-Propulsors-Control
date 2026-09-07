#!/usr/bin/env python3
"""Target versus achieved, for every shaper campaign.

One panel per campaign: the mathematically-defined target curve drawn against
the force waveform the rig actually produced at the campaign's best knobs.

Two choices worth stating, because they decide what the picture means:

  SCALE.  The target is defined shape-only (its crest is 1.0 by construction),
  so it is drawn scaled to the measured crest -- exactly the `force_scale` the
  objective uses. Drawing it in raw units instead would show a magnitude gap
  that the optimiser was never asked to close, and would hide the shape
  agreement that it WAS asked for.

  PHASE.  Neither curve has an absolute phase: the target is a periodic shape
  and the measured cycle starts wherever the capture window opened. They are
  aligned on their largest crest, so the comparison is of shape rather than of
  an arbitrary time origin.

The measured trace is re-derived from the stored raw capture with the same
window and tare logic the analysis used, so the picture and the reported
metrics cannot disagree.

usage:  plot_shaper_result.py <out.png> <folder>:<curve>:<channel> [...]
"""
from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_analyze as AN        # noqa: E402
import target_curves as tc       # noqa: E402


def _best_from_captures(folder, curve, channel):
    """Rank the stored captures directly, by re-scoring them.

    history.csv is only written when a campaign finishes, so a run still in
    progress would otherwise be unplottable. Re-scoring every capture with the
    shaper's own evaluate()/error() gives the same ranking it would have
    produced, and works at any point during the run.
    """
    import glob
    import amfm_shaper as SH
    target, _ = SH.target_metrics_from_curve(curve)
    rows = []
    for fp in sorted(glob.glob(os.path.join(folder, "data", "*_force.csv"))):
        kp = fp.replace("_force.csv", "_kin.csv")
        m = SH.evaluate((kp, fp), channel, None)
        if m is None:
            continue
        err, _ = SH.error(m, target, max(m.get("crest_height", 0.0), 1e-9))
        # 'nominal_probe' is the pre-seed reference measurement, not a search
        # step, so it carries no evaluation number. Call it evaluation 0: it is
        # the starting point the run is measured against.
        try:
            ev = int(os.path.basename(fp).split("_")[-2])
        except ValueError:
            ev = 0
        rows.append((ev, err, fp))
    if not rows:
        return None
    rows.sort(key=lambda r: r[0])              # evaluation order
    err0 = rows[0][1]
    ev, err, fp = min(rows, key=lambda r: r[1])
    return fp, err, err0, len(rows), os.path.basename(fp)[:-10]


def best_trace(folder, curve, channel):
    """The measured cycle at the campaign's lowest-error evaluation."""
    hp = os.path.join(folder, "history.csv")
    if os.path.exists(hp):
        import pandas as pd
        h = pd.read_csv(hp)
        b = h.loc[h.err.idxmin()]
        label = f"{b['tag']}_{int(b['eval']):03d}"
        fp = os.path.join(folder, "data", f"{label}_force.csv")
        err, err0, nev = float(b["err"]), float(h.iloc[0]["err"]), len(h)
    else:
        got = _best_from_captures(folder, curve, channel)
        if got is None:
            return None
        fp, err, err0, nev, label = got
    if not os.path.exists(fp):
        return None
    r = AN.load_force(fp)
    if r is None:
        return None
    t, fx, fy, fz = r
    F = {"Fx": fx, "Fy": fy, "Fz": fz}[channel]
    return t - t[0], F, err, err0, nev, label


def aligned(target_y, meas_y):
    """Resample the target onto the measured grid and align on the crest."""
    n = len(meas_y)
    tgt = np.interp(np.linspace(0, 1, n, endpoint=False),
                    np.linspace(0, 1, len(target_y), endpoint=False), target_y)
    # scale the shape-only target to the measured crest (the objective's
    # force_scale), then roll it onto the measured peak
    tgt = tgt / max(tgt.max(), 1e-9) * max(meas_y.max(), 1e-9)
    return np.roll(tgt, int(np.argmax(meas_y) - np.argmax(tgt)))


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 1
    out = sys.argv[1]
    specs = [s.split(":") for s in sys.argv[2:]]

    n = len(specs)
    ncol = 2 if n > 1 else 1
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(7.0 * ncol, 3.6 * nrow),
                             squeeze=False)
    fig.suptitle("Target vs achieved force waveform", fontsize=14, y=0.985)

    for ax, (folder, curve, channel) in zip(axes.ravel(), specs):
        fpath = folder if os.path.isabs(folder) else os.path.join(WORKSPACE_ROOT, folder)
        got = best_trace(fpath, curve, channel)
        if got is None:
            ax.text(0.5, 0.5, f"{folder}\n(no data)", ha="center", va="center",
                    transform=ax.transAxes)
            ax.set_axis_off()
            continue
        t, F, err, err0, nev, label = got
        sp, _ = (tc.drag_target() if curve == "drag" else tc.lift_target())
        _, T = tc.evaluate(sp)
        tgt = aligned(np.asarray(T), F)
        x = np.linspace(0, 1, len(F), endpoint=False)

        ax.axhline(0, color="0.75", lw=0.8, zorder=1)
        ax.plot(x, tgt, color="0.35", lw=2.0, ls="--", label="target", zorder=3)
        ax.plot(x, F, color="#c2410c", lw=1.6, label="achieved", zorder=4)
        ax.fill_between(x, np.minimum(F, 0), 0, color="#c2410c", alpha=0.16,
                        lw=0, zorder=2, label="trough (to minimise)")

        seeded = "seeded" if "stage3" in folder else "no seed"
        ax.set_title(f"{os.path.basename(fpath)}  ·  {curve} target on {channel}"
                     f"  ({seeded})\nerr {err0:.1f} → {err:.1f} in {nev} evals",
                     fontsize=10.5)
        ax.set_xlabel("cycle phase")
        ax.set_ylabel(f"{channel}  [N]")
        ax.legend(fontsize=8.5, loc="upper right", framealpha=0.9)
        ax.margins(x=0)

    for ax in axes.ravel()[n:]:
        ax.set_axis_off()
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
