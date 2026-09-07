#!/usr/bin/env python3
"""Replay the two best gaits found this session, 3 cycles each, and plot both.

DRAG  = amfm_stage23_envelope/stageA_result.json -- the best drag-curve shape
        match found (err 22.9), including its phase offset (45 deg).
LIFT  = amfm_thrust2/result.json -- the best validated thrust gait (crest_1
        2.786N, crest_2 2.782N, 0.1% imbalance, Fy nulled), the same gait
        replicated 6x earlier this session at +0.508 +/- 0.017 N.

Both replayed through the SAME phase-preserving pipeline (stage23_final's
measure_phased), so drag's phase offset is honored exactly; lift's phase
defaults to 0 since that gait was never phase-optimized.

usage:  best_gaits_demo.py <folder> [--period 2.0]
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX                      # noqa: E402
import amfm_shaper as SH                            # noqa: E402
import amfm_analyze as AN                           # noqa: E402
import target_curves as TC                          # noqa: E402
from amfm_waveform import Knobs                     # noqa: E402
from stage23_final import measure_phased            # noqa: E402


def build_knobs(qualified, extra1=None, extra2=None):
    d1 = {k.split(".", 1)[1]: v for k, v in qualified.items() if k.startswith("s1.")}
    d2 = {k.split(".", 1)[1]: v for k, v in qualified.items() if k.startswith("s2.")}
    d1.setdefault("n", 1); d2.setdefault("n", 1)
    d1["n"] = int(d1["n"]); d2["n"] = int(d2["n"])
    if extra1: d1.update(extra1)
    if extra2: d2.update(extra2)
    k1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
    k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
    return k1, k2


def plot_drag(fp, out_png):
    r = AN.load_force(fp)
    t, fx, fy, fz = r
    x = np.linspace(0, 1, len(fx), endpoint=False)
    sp, _ = TC.drag_target()
    _, T = TC.evaluate(sp)
    tgt = np.interp(x, np.linspace(0, 1, len(T), endpoint=False), T)
    tgt = tgt / max(tgt.max(), 1e-9) * max(fx.max(), 1e-9)
    tgt = np.roll(tgt, int(np.argmax(fx) - np.argmax(tgt)))

    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.axhline(0, color="0.75", lw=0.8)
    ax.plot(x, tgt, color="0.35", lw=2.0, ls="--", label="target (drag)")
    ax.plot(x, fx, color="#c2410c", lw=1.8, label="achieved Fx")
    ax.fill_between(x, np.minimum(fx, 0), 0, color="#c2410c", alpha=0.15, lw=0)
    ax.set_title("Best DRAG gait  —  crest+trough shape match (err 22.9, phase 45°)\n"
                f"net Fx {fx.mean():+.3f} N   net Fy {fy.mean():+.3f} N", fontsize=11)
    ax.set_xlabel("cycle phase"); ax.set_ylabel("Fx  [N]")
    ax.legend(fontsize=9, loc="upper right"); ax.margins(x=0)
    fig.tight_layout(); fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")


def plot_lift(fp, out_png):
    r = AN.load_force(fp)
    t, fx, fy, fz = r
    x = np.linspace(0, 1, len(fx), endpoint=False)

    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.axhline(0, color="0.75", lw=0.8)
    ax.plot(x, fx, color="#c2410c", lw=1.8, label="Fx (thrust)")
    ax.axhline(fx.mean(), color="#c2410c", lw=1.2, ls=":",
              label=f"mean Fx {fx.mean():+.3f} N")
    ax.plot(x, fy, color="#2563eb", lw=1.4, label="Fy (vertical, nulled)")
    ax.axhline(fy.mean(), color="#2563eb", lw=1.2, ls=":",
              label=f"mean Fy {fy.mean():+.3f} N")
    ax.set_title("Best LIFT gait  —  symmetric 2-crest thrust, vertical nulled\n"
                f"crest_1/crest_2 imbalance 0.1%", fontsize=11)
    ax.set_xlabel("cycle phase"); ax.set_ylabel("N")
    ax.legend(fontsize=9, loc="upper right"); ax.margins(x=0)
    fig.tight_layout(); fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    out_dir = os.path.join(folder, "data")
    os.makedirs(out_dir, exist_ok=True)

    drag = json.load(open(os.path.join(WORKSPACE_ROOT, "amfm_stage23_envelope", "stageA_result.json")))
    lift = json.load(open(os.path.join(WORKSPACE_ROOT, "amfm_thrust2", "result.json")))["best_knobs"]

    k1d, k2d = build_knobs(drag["knobs"])
    k1l, k2l = build_knobs(lift)

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop(); return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    try:
        print(f"--- DRAG: replaying 3 cycles, phase={drag['phase_deg']:.0f} deg")
        paths_d, err = measure_phased(rig, k1d, k2d, drag["phase"], out_dir, "drag_final",
                                      period=drag["period"])
        if paths_d is None:
            print("   DRAG replay failed:", err)

        print(f"--- LIFT: replaying 3 cycles, phase=0 (never phase-optimized)")
        paths_l, err = measure_phased(rig, k1l, k2l, 0.0, out_dir, "lift_final",
                                      period=a.period)
        if paths_l is None:
            print("   LIFT replay failed:", err)
    finally:
        rig.stop()

    if paths_d:
        plot_drag(paths_d[1], os.path.join(folder, "drag_result.png"))
    if paths_l:
        plot_lift(paths_l[1], os.path.join(folder, "lift_result.png"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
