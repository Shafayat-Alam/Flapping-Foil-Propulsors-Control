#!/usr/bin/env python3
"""Revisit the ORIGINAL stage-2/3 problem -- matching the drag target curve,
crest AND trough -- with everything found since that original attempt failed.

WHY THIS IS A DIFFERENT EXPERIMENT FROM final_combined_test.py
----------------------------------------------------------------
final_combined_test.py chases net THRUST across stage-4's directions. This
chases the ORIGINAL objective stage 2/3 were built for -- matching the
drag target's SHAPE (crest_height, trough_depth, crest_width, crest_skew,
trough_width, trough_skew, crest_count, bias all scored against
target_curves.drag_target()) -- because that is the problem that was never
actually solved: stage 2 got the crest right and the trough never moved
below ~50% of the crest no matter what was tried.

Stage 2/3 originally had THREE handicaps, discovered only after they ran:
  1. servo 2 (heave) was frozen at nominal the whole time -- only servo 1
     (pitch) could be tuned. This run tunes both.
  2. of the knobs that WERE reachable, three of heave's showed strong,
     clean, monotone isolated relationships (see amfm_shaping/metrics.csv,
     stage J+S) that the coordinate-descent search never actually reached in
     their favoured direction once combined with everything else -- flagged
     here as a WARM START, not assumed as gospel; the search is free to move
     away from it.
  3. pitch and heave were structurally locked to zero relative phase -- no
     knob could change it. That gap is closed here the same way as in
     final_combined_test.py: an exact circular roll of heave's precomputed
     array, reusable for any waveform shape.

STAGE A (blind).   Nominal start, both servos, phase included. Same
                    conditions original stage 2 had, minus all three
                    handicaps -- isolates how much of stage 2's failure was
                    the missing capability versus the search itself.
STAGE B (seeded).  Jacobian-seeded (as stage 3 was) PLUS the heave
                    correction applied on top PLUS phase warm-started near
                    the thrust-optimal 120 degrees (the best available prior
                    for phase on THIS objective -- it was only ever measured
                    against net thrust, not curve shape, so this is a
                    starting guess for the optimiser to refine, not a
                    settled answer).

Both stages run to convergence (coordinate descent, shrinking steps) so each
produces an actual BEST WAVEFORM at the end, not only a relationship report
-- the winning knob set is written to result.json and can be replayed
directly.

Writes to a NEW folder only -- does not touch amfm_stage2, amfm_stage3, or
any other existing experiment's data.

usage:  stage23_final.py <folder> [--period 2.0] [--max-evals 40]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX                                    # noqa: E402
import amfm_shaper as SH                                         # noqa: E402
from amfm_waveform import Knobs, cycle                           # noqa: E402

BOUND = 0.70
RATE_KNOBS = ["s_com", "w_com", "s_diff", "w_diff"]
# ENVELOPE_KNOBS shape AMPLITUDE over time within the cycle (crest taller /
# trough deeper) -- distinct from RATE_KNOBS, which shape SPEED over time
# within the cycle. The first run of this script omitted these entirely (A0
# was tunable as a flat, constant-per-cycle scalar, but not its within-cycle
# envelope), so "does amplitude varying over time help" was never actually
# tested, only "does rate varying over time help."
ENVELOPE_KNOBS = ["h_diff", "h_com"]
TUNABLE = ["phase", "period"] + \
          [f"s1.{n}" for n in RATE_KNOBS + ENVELOPE_KNOBS + ["A0", "C"]] + \
          [f"s2.{n}" for n in RATE_KNOBS + ENVELOPE_KNOBS + ["A0", "C"]]
# phase/period are FIRST, not last: coordinate descent works through TUNABLE
# in order, and final_combined_test.py put phase last with a budget too small
# to ever reach it -- 15 evals against 10 knobs ahead of it in the list, so
# phase silently never got tried in any of that run's 15 directions. Putting
# the newest, least-tested capability first guarantees it gets a fair chance
# regardless of how much budget is left over for the rest.
BOUNDS = {"s_com": (-BOUND, BOUND), "w_com": (-BOUND, BOUND),
          "s_diff": (-BOUND, BOUND), "w_diff": (-BOUND, BOUND),
          "h_diff": (-0.55, 0.55), "h_com": (-0.55, 0.55),   # amfm_shaper.py's own bound
          "A0": (0.15, 0.80), "C": (-0.30, 0.30), "phase": (0.0, 1.0),
          "period": (1.3, 3.0)}

# stage-1 isolated evidence, servo2, established against Fx_bias (see the
# session's own audit of amfm_shaping/metrics.csv stage J+S) -- a WARM START
# for stage B, not a constraint; the optimiser can move away from it.
HEAVE_CORRECTION = {"s_com": +0.60, "w_diff": -0.55, "h_diff": -0.45}
# af_ratio_test.py: heave amplitude correlated +0.56 to +0.89 with net thrust
# across EVERY frequency combination tested -- the single most reliable
# lever found this session. Seeded here so stage B starts near it.
HEAVE_A0_SEED = 0.70
PHASE_WARM_START = 120.0 / 360.0   # best net-thrust phase found this session


def measure_phased(rig, k1: Knobs, k2: Knobs, phase, out_dir, label, period=None):
    """Rig.measure(), with servo2's array circularly rolled by `phase`
    (0..1 of a cycle) -- exact for any shape, since cycle() already returns
    one full period. `period` overrides rig.period so it can be searched
    per-trial."""
    period = rig.period if period is None else period
    n_s = int(EX.HW_RATE * period)
    _, th1, _ = cycle(k1.to_params(), period, n_s)
    _, th2, _ = cycle(k2.to_params(), period, n_s)
    th2 = np.roll(th2, int(round(phase * n_s)))
    if (np.max(np.abs(th1)) > EX.PITCH_LIMIT or np.max(np.abs(th2)) > EX.HEAVE_LIMIT):
        return None, "commanded array exceeds position limits"
    for th in (th1, th2):
        if np.max(np.abs(np.gradient(th, period / len(th)))) > EX.SLEW_LIMIT:
            return None, "commanded array exceeds slew limit"
    p1 = rig.fb.get(1, (0.0, 0.0))[0]
    p2 = rig.fb.get(2, (0.0, 0.0))[0]
    for a in np.linspace(0, 1, int(EX.HW_RATE * 1.5)):
        rig.send((1 - a) * p1 + a * th1[0], (1 - a) * p2 + a * th2[0])
        time.sleep(1.0 / EX.HW_RATE)
    t_end = time.time() + EX.SETTLE_S
    while time.time() < t_end:
        rig.send(th1[0], th2[0]); time.sleep(1.0 / EX.HW_RATE)
    rig.lc, rig.rec = [], True
    t0 = time.time() + EX.PRE_QUIET_S
    while time.time() < t0:
        rig.send(th1[0], th2[0]); time.sleep(1.0 / EX.HW_RATE)
    kin = []
    for _ in range(EX.N_CYCLES):
        for j in range(n_s):
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
    with open(lp, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["wall", "payload"])
        for wt, d in rig.lc:
            w.writerow([wt - t0, " ".join(f"{x:.6f}" for x in d)])
    return (kp, lp), None


def optimise_shape(rig, folder, target, k_start, phase_start, max_evals, log=print,
                   period_start=None):
    """Coordinate descent against the SHAPE objective (amfm_shaper.error),
    over both servos' rate/amplitude/bias knobs plus phase and period."""
    out_dir = os.path.join(folder, "data")
    k = dict(k_start)
    phase = phase_start
    period = EX.PERIOD_S if period_start is None else period_start
    step = {n: 0.25 * (BOUNDS[n.split(".", 1)[1] if "." in n else n][1]
                       - BOUNDS[n.split(".", 1)[1] if "." in n else n][0]) / 2
           for n in TUNABLE}
    history, ev = [], 0

    def score(kd, ph, pd, tag):
        nonlocal ev
        ev += 1
        d1 = {n: kd.get(f"s1.{n}", 0.0) for n in RATE_KNOBS + ENVELOPE_KNOBS + ["A0", "C"]}
        d2 = {n: kd.get(f"s2.{n}", 0.0) for n in RATE_KNOBS + ENVELOPE_KNOBS + ["A0", "C"]}
        d1["A0"] = d1["A0"] or EX.NOMINAL[1].A0
        d2["A0"] = d2["A0"] or EX.NOMINAL[2].A0
        d1["n"] = 1; d2["n"] = 1
        k1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
        k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
        paths, err = measure_phased(rig, k1, k2, ph, out_dir, f"{tag}_{ev:03d}", period=pd)
        if paths is None:
            log(f"   [{ev:03d}] {tag}: {err}")
            return None
        m = SH.evaluate(paths, "Fx", None)
        if m is None:
            return None
        scale = max(m.get("crest_height", 0.0), 1e-6)
        e, terms = SH.error(m, target, scale)
        rec = {"eval": ev, "tag": tag, "err": e, "phase": ph, "period": pd, **kd,
              **{f"m_{a}": b for a, b in m.items()}}
        history.append(rec)
        log(f"   [{ev:03d}] {tag:20s} err {e:8.3f}  phase {ph:.2f}  period {pd:.2f}  "
            + "  ".join(f"{a}:{b:+.2f}" for a, b in list(terms.items())[:4]))
        return e

    base = score(k, phase, period, "start")
    if base is None:
        return None, history
    best = (base, dict(k), phase, period)
    while ev < max_evals and max(step.values()) > 0.01:
        improved = False
        for name in TUNABLE:
            if ev >= max_evals: break
            for sgn in (+1, -1):
                if ev >= max_evals: break
                if name == "phase":
                    trial_ph = (best[2] + sgn * step["phase"]) % 1.0
                    e = score(best[1], trial_ph, best[3], f"phase{'+' if sgn>0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, dict(best[1]), trial_ph, best[3]); improved = True; break
                elif name == "period":
                    lo, hi = BOUNDS["period"]
                    trial_pd = float(np.clip(best[3] + sgn * step["period"], lo, hi))
                    e = score(best[1], best[2], trial_pd, f"period{'+' if sgn>0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, dict(best[1]), best[2], trial_pd); improved = True; break
                else:
                    lo, hi = BOUNDS[name.split(".", 1)[1]]
                    trial = dict(best[1])
                    trial[name] = float(np.clip(trial.get(name, 0.0) + sgn * step[name], lo, hi))
                    e = score(trial, best[2], best[3], f"{name}{'+' if sgn>0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, trial, best[2], best[3]); improved = True; break
        if not improved:
            for n in step: step[n] *= 0.5
    return best, history


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--max-evals", type=int, default=40)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    assert not os.path.exists(folder), f"{folder} already exists -- refusing to overwrite"
    os.makedirs(folder)

    target, _ = SH.target_metrics_from_curve("drag")
    print("target (drag):", {k: round(v, 3) for k, v in target.items()})

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop(); return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    results = {}
    try:
        print("=" * 60); print("STAGE A -- blind, nominal start"); print("=" * 60)
        kA = {f"s1.A0": EX.NOMINAL[1].A0, f"s2.A0": EX.NOMINAL[2].A0}
        bestA, histA = optimise_shape(rig, os.path.join(folder, "stageA"),
                                      target, kA, 0.0, a.max_evals)
        results["stageA"] = bestA

        print("\n" + "=" * 60); print("STAGE B -- seeded (Jacobian + heave correction + phase)")
        print("=" * 60)
        kB = {"s1.A0": EX.NOMINAL[1].A0, "s2.A0": EX.NOMINAL[2].A0}
        jac_path = os.path.join(WORKSPACE_ROOT, "amfm_shaping", "jacobian.json")
        if os.path.exists(jac_path):
            seeded = SH.seed_from_jacobian(jac_path, target, {}, kB,
                                           set(TUNABLE) - {"phase", "period"},
                                           channel="Fx", log=print)
            kB.update(seeded)
        for n, v in HEAVE_CORRECTION.items():
            kB[f"s2.{n}"] = v
        kB["s2.A0"] = HEAVE_A0_SEED
        bestB, histB = optimise_shape(rig, os.path.join(folder, "stageB"),
                                      target, kB, PHASE_WARM_START, a.max_evals)
        results["stageB"] = bestB
    finally:
        rig.stop()

    for stage_name, best in results.items():
        if best is None:
            print(f"\n{stage_name}: no usable result")
            continue
        err, knobs, phase, period = best
        print(f"\n{stage_name} BEST: err {err:.3f}  phase {phase:.3f} "
              f"({phase*360:.0f} deg)  period {period:.3f}s")
        print("   " + "  ".join(f"{k}={v:+.4f}" for k, v in knobs.items()))
        json.dump({"err": err, "knobs": knobs, "phase": phase, "phase_deg": phase * 360,
                   "period": period},
                  open(os.path.join(folder, f"{stage_name}_result.json"), "w"), indent=2)

    if results.get("stageA") and results.get("stageB"):
        eA = results["stageA"][0]; eB = results["stageB"][0]
        winner = "stageB (seeded)" if eB < eA else "stageA (blind)"
        print(f"\nWINNER: {winner}   (stageA err {eA:.3f}  vs  stageB err {eB:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
