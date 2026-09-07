#!/usr/bin/env python3
"""A synchronised triangular gait: pitch and heave both ramp

    0 -> +full -> -full -> 0

hitting each breakpoint at the SAME instant, each at its own amplitude. Not
the AM/FM sine family used everywhere else this session -- this is a genuine
piecewise-LINEAR waveform (constant velocity within each leg), which the
smooth AM/FM parameterisation cannot represent exactly regardless of knob
settings.

Three free parameters, searched with the same thrust objective (net Fx,
Fy held near zero) used all session:

    amp_pitch   peak pitch angle             (0, PITCH_LIMIT]
    amp_heave   peak heave angle              (0, HEAVE_LIMIT]
    t1          fraction of the cycle spent on the RISE leg (0 -> +full).
                The RETURN leg (-full -> 0) is forced equal to t1 by
                construction (same distance, same reason to take the same
                time); the middle leg (+full -> -full, twice the distance)
                gets whatever is left: t2 = 1 - 2*t1.

usage:  triangle_gait.py <folder> [--period 2.0] [--max-evals 20]
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX                    # noqa: E402
import amfm_shaper as SH                         # noqa: E402

# SMOOTH version of the requested gait: 0 -> +full -> -full -> 0, pitch and
# heave hitting every breakpoint at the same instant. A plain sine already
# does exactly that in one period (rises to +1 at quarter-period, falls
# through zero to -1 at three-quarter, returns to 0) -- so the smooth form of
# this gait is simply an IN-PHASE sine pair with independent amplitudes, not
# a new waveform. 'warp' bends the timing of that same rise/fall/return
# smoothly (still zero velocity discontinuity anywhere) rather than forcing
# hard corners the way the piecewise-linear version did.
BOUNDS = {"amp_pitch": (0.10, EX.PITCH_LIMIT * 0.95),
          "amp_heave": (0.10, EX.HEAVE_LIMIT * 0.95),
          "warp": (-0.80, 0.80)}
TUNABLE = ["amp_pitch", "amp_heave", "warp"]


def smooth_wave(u, warp):
    """sin(2*pi*phi(u)), phi a smooth monotone re-timing of u via a single
    warp harmonic -- same mechanism as amfm_waveform's phase warp, kept to
    one free parameter here since pitch and heave share the SAME timing by
    construction (that is what 'hit every breakpoint together' means)."""
    phi = u + (warp / (2 * np.pi)) * np.sin(2 * np.pi * u)
    return np.sin(2 * np.pi * phi)


def build(amp_pitch, amp_heave, warp, period, hw_rate):
    n_s = int(hw_rate * period)
    u = np.linspace(0.0, 1.0, n_s, endpoint=False)
    v = smooth_wave(u, warp)
    return amp_pitch * v, amp_heave * v


def measure_raw(rig, th1, th2, out_dir, label):
    """Same pipeline as Rig.measure(), for a raw (th1, th2) pair instead of
    a Knobs-generated one -- the triangular wave has no AM/FM parameters to
    convert from."""
    if (np.max(np.abs(th1)) > EX.PITCH_LIMIT
            or np.max(np.abs(th2)) > EX.HEAVE_LIMIT):
        return None, "commanded array exceeds position limits"
    for th in (th1, th2):
        if np.max(np.abs(np.gradient(th, rig.period / len(th)))) > EX.SLEW_LIMIT:
            return None, "commanded array exceeds slew limit"

    p1 = rig.fb.get(1, (0.0, 0.0))[0]
    p2 = rig.fb.get(2, (0.0, 0.0))[0]
    for a in np.linspace(0, 1, int(EX.HW_RATE * 1.5)):
        rig.send((1 - a) * p1 + a * th1[0], (1 - a) * p2 + a * th2[0])
        time.sleep(1.0 / EX.HW_RATE)

    t_end = time.time() + EX.SETTLE_S
    while time.time() < t_end:
        rig.send(th1[0], th2[0])
        time.sleep(1.0 / EX.HW_RATE)

    rig.lc, rig.rec = [], True
    t0 = time.time() + EX.PRE_QUIET_S
    while time.time() < t0:
        rig.send(th1[0], th2[0])
        time.sleep(1.0 / EX.HW_RATE)

    kin = []
    for _ in range(EX.N_CYCLES):
        for j in range(len(th1)):
            rig.send(th1[j], th2[j])
            kin.append((time.time() - t0, th1[j], th2[j],
                       rig.fb.get(1, (np.nan,) * 2)[0], rig.fb.get(2, (np.nan,) * 2)[0]))
            time.sleep(max(0.0, 1.0 / EX.HW_RATE - (time.time() - t0 - kin[-1][0])))
    t_end = time.time() + EX.IDLE_TAIL_S
    while time.time() < t_end:
        time.sleep(0.005)
    rig.rec = False

    os.makedirs(out_dir, exist_ok=True)
    kp = os.path.join(out_dir, f"{label}_kin.csv")
    np.savetxt(kp, np.array(kin), delimiter=",", header="t,cmd1,cmd2,meas1,meas2", comments="")
    lp = os.path.join(out_dir, f"{label}_force.csv")
    import csv
    with open(lp, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "payload"])
        for wt, d in rig.lc:
            w.writerow([wt - t0, " ".join(f"{x:.6f}" for x in d)])
    return (kp, lp), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--max-evals", type=int, default=20)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    k = {"amp_pitch": 0.35, "amp_heave": 0.25, "warp": 0.0}
    step = {n: 0.25 * (BOUNDS[n][1] - BOUNDS[n][0]) / 2 for n in TUNABLE}
    history, best, ev = [], None, 0

    def score(kd, tag):
        nonlocal ev
        ev += 1
        th1, th2 = build(kd["amp_pitch"], kd["amp_heave"], kd["warp"], a.period, EX.HW_RATE)
        paths, err = measure_raw(rig, th1, th2, os.path.join(folder, "data"), f"{tag}_{ev:03d}")
        if paths is None:
            print(f"   [{ev:02d}] {tag}: {err}")
            return None
        m = SH.evaluate(paths, "Fx", None)
        if m is None:
            print(f"   [{ev:02d}] {tag}: no force data")
            return None
        e, terms = SH.thrust_error(m)
        history.append({"eval": ev, "tag": tag, "err": e, **kd, **m})
        print(f"   [{ev:02d}] {tag:14s} err {e:8.3f}   net_Fx {terms['net_thrust']:+.3f}"
              f"   net_Fy {terms['other_bias']:+.3f}")
        return e

    try:
        base = score(k, "start")
        if base is None:
            print("start infeasible or no feedback — aborting")
            return 2
        best = (base, dict(k))
        while ev < a.max_evals and max(step.values()) > 0.01:
            improved = False
            for name in TUNABLE:
                if ev >= a.max_evals:
                    break
                for sgn in (+1, -1):
                    lo, hi = BOUNDS[name]
                    trial = dict(best[1])
                    trial[name] = float(np.clip(trial[name] + sgn * step[name], lo, hi))
                    e = score(trial, f"{name}{'+' if sgn > 0 else '-'}")
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
        import csv, json
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=sorted({k for r in history for k in r}))
            w.writeheader(); w.writerows(history)
        errs = [r["err"] for r in history]
        print(f"\n{len(history)} evaluations   err {errs[0]:.3f} -> {min(errs):.3f}")
        print("   best params: " + "  ".join(f"{k}={v:+.4f}" for k, v in best[1].items()))
        json.dump({"best_params": best[1], "err_start": errs[0], "err_best": min(errs),
                   "n_evals": len(history)},
                  open(os.path.join(folder, "result.json"), "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
