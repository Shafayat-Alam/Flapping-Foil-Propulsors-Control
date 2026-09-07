#!/usr/bin/env python3
"""Pitch-heave RELATIVE PHASE: the parameter this session could never reach.

    theta_heave(t) = A_h * sin(2*pi*t)
    theta_pitch(t) = A_p * sin(2*pi*t + 2*pi*phase)

WHY THIS IS NOT JUST ANOTHER GAIT
---------------------------------
The AM/FM parameterisation used by every campaign this session writes

    Phi(t) = 2*pi * ( n*t + w(t) ),   w(t) = sum (c_k/(2 pi k)) sin(2 pi k t + g_k)

Every warp term is a ZERO-MEAN sinusoid. There is no constant term in w, so
no setting of any of the 18 knobs can shift one servo's waveform in time
relative to the other's -- both servos are driven from the same t and are
therefore permanently phase-locked. The warp knobs bend timing WITHIN a
cycle symmetrically; they cannot produce a net offset.

So the pitch-heave phase offset -- the single most important parameter in the
flapping-foil literature, where thrust peaks near 90 degrees -- was pinned at
0 for the entire session by construction, not by choice. Every "best gait"
found so far is the best gait AT ZERO PHASE OFFSET.

That also makes this the cheapest remaining explanation for why net thrust
stayed small: a flapping foil at 0 phase is close to the worst case, because
the blade presents nearly the same face on both strokes instead of feathering
one and loading the other.

METHOD
------
Phase is swept COARSELY FIRST rather than optimised, because the answer of
interest is the shape of thrust-versus-phase (a curve with a known expected
optimum), not merely the single best point -- and a sweep cannot get stuck in
a local optimum the way coordinate descent can. Amplitudes are then refined
at the winning phase with the remaining budget.

usage:  sine_gait_phase.py <folder> [--period 2.0] [--n-phase 12] [--refine 9]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX                       # noqa: E402
import amfm_shaper as SH                            # noqa: E402
from triangle_gait import measure_raw               # noqa: E402

AMP_BOUNDS = {"amp_pitch": (0.10, EX.PITCH_LIMIT * 0.95),
              "amp_heave": (0.10, EX.HEAVE_LIMIT * 0.95)}


def build(amp_pitch, amp_heave, phase, period, hw_rate):
    """Heave is the reference; pitch leads it by `phase` cycles."""
    n_s = int(hw_rate * period)
    u = np.linspace(0.0, 1.0, n_s, endpoint=False)
    th_heave = amp_heave * np.sin(2 * np.pi * u)
    th_pitch = amp_pitch * np.sin(2 * np.pi * (u + phase))
    return th_pitch, th_heave          # servo1 = pitch, servo2 = heave


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--n-phase", type=int, default=12)
    ap.add_argument("--refine", type=int, default=9)
    ap.add_argument("--amp-pitch", type=float, default=0.45)
    ap.add_argument("--amp-heave", type=float, default=0.35)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    history = []

    def run(amp_p, amp_h, phase, tag):
        th1, th2 = build(amp_p, amp_h, phase, a.period, EX.HW_RATE)
        paths, err = measure_raw(rig, th1, th2, os.path.join(folder, "data"), tag)
        if paths is None:
            print(f"   {tag}: {err}")
            return None
        m = SH.evaluate(paths, "Fx", None)
        if m is None:
            print(f"   {tag}: no force data")
            return None
        e, terms = SH.thrust_error(m)
        rec = {"tag": tag, "err": e, "amp_pitch": amp_p, "amp_heave": amp_h,
               "phase": phase, "phase_deg": phase * 360.0,
               "net_Fx": terms["net_thrust"], "net_Fy": terms["other_bias"],
               "crest": m.get("crest_height", 0.0), "trough": m.get("trough_depth", 0.0)}
        history.append(rec)
        print(f"   {tag:18s} phase {phase*360:6.1f}d   err {e:8.3f}   "
              f"net_Fx {rec['net_Fx']:+.3f}   net_Fy {rec['net_Fy']:+.3f}")
        return e

    best = None
    try:
        print(f"--- PHASE SWEEP at A_p={a.amp_pitch:.2f} A_h={a.amp_heave:.2f} "
              f"({a.n_phase} points over a full cycle)")
        for i in range(a.n_phase):
            ph = i / a.n_phase
            e = run(a.amp_pitch, a.amp_heave, ph, f"ph_{int(ph*360):03d}")
            if e is not None and (best is None or e < best[0]):
                best = (e, {"amp_pitch": a.amp_pitch, "amp_heave": a.amp_heave, "phase": ph})

        if best is None:
            print("no usable phase-sweep results — aborting")
            return 2
        print(f"\n--- best phase {best[1]['phase']*360:.0f} deg; refining amplitudes")
        step = {n: 0.25 * (AMP_BOUNDS[n][1] - AMP_BOUNDS[n][0]) / 2 for n in AMP_BOUNDS}
        ev = 0
        while ev < a.refine and max(step.values()) > 0.01:
            improved = False
            for name in ("amp_pitch", "amp_heave"):
                if ev >= a.refine:
                    break
                for sgn in (+1, -1):
                    if ev >= a.refine:
                        break
                    lo, hi = AMP_BOUNDS[name]
                    trial = dict(best[1])
                    trial[name] = float(np.clip(trial[name] + sgn * step[name], lo, hi))
                    ev += 1
                    e = run(trial["amp_pitch"], trial["amp_heave"], trial["phase"],
                            f"ref{ev:02d}_{name}{'+' if sgn > 0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, trial)
                        improved = True
                        break
            if not improved:
                for n in step:
                    step[n] *= 0.5
    finally:
        rig.stop()

    if history:
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(history[0]))
            w.writeheader()
            w.writerows(history)
        sweep = [r for r in history if r["tag"].startswith("ph_")]
        sweep.sort(key=lambda r: r["phase"])
        print("\n" + "=" * 66)
        print("THRUST vs PITCH-HEAVE PHASE")
        print("=" * 66)
        print(f"{'phase':>8}{'net_Fx':>10}{'net_Fy':>10}{'crest':>9}{'trough':>9}")
        for r in sweep:
            print(f"{r['phase_deg']:7.0f}d{r['net_Fx']:10.3f}{r['net_Fy']:10.3f}"
                  f"{r['crest']:9.3f}{r['trough']:9.3f}")
        if sweep:
            b = max(sweep, key=lambda r: r["net_Fx"])
            z = min(sweep, key=lambda r: abs(r["phase"]))
            print(f"\n   best phase for thrust: {b['phase_deg']:.0f} deg "
                  f"-> net_Fx {b['net_Fx']:+.3f} N")
            print(f"   at 0 deg (what every earlier campaign was locked to): "
                  f"net_Fx {z['net_Fx']:+.3f} N")
            if abs(z["net_Fx"]) > 1e-9:
                print(f"   ratio: {b['net_Fx']/z['net_Fx']:.2f}x")
        print(f"\nbest overall: err {best[0]:.3f}  " +
              "  ".join(f"{k}={v:+.4f}" for k, v in best[1].items()))
        json.dump({"best": best[1], "err_best": best[0], "n_evals": len(history)},
                  open(os.path.join(folder, "result.json"), "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
