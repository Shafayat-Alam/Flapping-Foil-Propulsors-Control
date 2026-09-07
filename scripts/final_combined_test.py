#!/usr/bin/env python3
"""The final combined test: every direction that worked, refined with every
lever this session found, including the one structurally absent until now.

SCOPE -- draws on stage 1 through 4:
  stage 1   the isolated per-knob sensitivity data (amfm_shaping/metrics.csv)
            that flagged which heave knobs were pushed the WRONG way later
  stage 2/3 established the shape-matching objective and its limits
  stage 4   15 of 16 directions that actually converged (every one except
            pure +Fz, which never moved regardless of any knob -- excluded,
            not "forgotten": that direction is a confirmed mechanical dead
            end, re-testing it would not be informative)

WHAT'S NEW IN THIS RUN, per direction, seeded from that direction's own
stage-4 result:
  1. PHASE -- a genuine, shape-preserving offset between servo1 and servo2,
     added here for the first time to a real AM/FM waveform (not the plain
     sine used in the standalone phase test). Implemented as a circular
     ROLL of the precomputed heave array: since cycle() already returns one
     exact period, np.roll(th2, shift) IS the exactly-phase-shifted
     waveform, for ANY shape -- no new math, no approximation, reusable for
     every direction here.
  2. RATE KNOBS on both axes (s_com, w_com, s_diff, w_diff) and BOTH
     amplitudes, all refinable together -- so the search can find whatever
     split between the two axes actually works, not a fixed guess.
  3. For +Fx and -Fx specifically: warm-started from the heave-knob
     correction found in heave_flip_test.py's design (s2.s_com, s2.w_diff,
     s2.h_diff flipped toward their isolated-favoured sign) -- direct
     evidence exists for that objective. For the other 13 directions no
     such isolated sweep exists (it was only ever measured for Fx), so they
     start from their own stage-4 knobs unmodified and let the search find
     the correction itself, same mechanism, no presumed sign.

FREQUENCY is not folded in here, same reasoning as heave_flip_test.py: it
scales in an already-known, separate way (~1/T^2.34) and combining it would
roughly double an already large run. Layer it on after, per direction, if a
given direction's winner is worth chasing further.

usage:  final_combined_test.py <folder> [--period 2.0] [--evals-per-dir 15]
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX                    # noqa: E402
import amfm_shaper as SH                         # noqa: E402
from amfm_waveform import Knobs, cycle           # noqa: E402

BOUND = 0.70
RATE_KNOBS = ["s_com", "w_com", "s_diff", "w_diff"]
TUNABLE = [f"s1.{n}" for n in RATE_KNOBS + ["A0"]] + \
          [f"s2.{n}" for n in RATE_KNOBS + ["A0"]] + ["phase", "period"]
BOUNDS = {"s_com": (-BOUND, BOUND), "w_com": (-BOUND, BOUND),
          "s_diff": (-BOUND, BOUND), "w_diff": (-BOUND, BOUND),
          "A0": (0.15, 0.80), "phase": (0.0, 1.0),
          # period floor is NOT a fixed number -- feasibility (slew) depends
          # on whatever amplitude/rate knobs are active at the same trial, so
          # the search's own feasibility check (not this bound) is what
          # actually enforces the true floor at each point. This upper bound
          # just keeps it from drifting to unhelpfully slow.
          "period": (1.3, 3.0)}

# isolated-favoured heave correction, established from stage-1 data for the
# +Fx objective specifically (see heave_flip_test.py) -- applied only where
# that evidence actually applies.
HEAVE_FX_CORRECTION = {"s_com": +0.60, "w_diff": -0.55, "h_diff": -0.45, "A0": 0.70}
# A0 bump is scoped to +Fx/-Fx deliberately, not applied to the other 13
# directions: af_ratio_test.py proved heave amplitude strongly predicts net
# Fx specifically (+0.56 to +0.89 across every frequency tested). Whether it
# holds for a force PROJECTED onto a different axis (+Fy, +FxFy, ...) was
# never tested, and applying it blind there would repeat the exact mistake
# just corrected in this same investigation (transferring a validated
# finding into a context it was never measured in). The other 13 directions
# still have A0 as a free, searchable parameter -- if high heave amplitude
# helps them too, their own refinement will find it on the evidence, not an
# assumption.

ALL_DIRECTIONS = [
    "+Fx", "-Fx", "+Fy", "-Fy", "-Fz",
    "+FxFy", "-FxFy", "+FxFz", "-FxFz", "+FyFz", "-FyFz",
    "+Fx+Fy+Fz", "+Fx-Fy+Fz", "-Fx+Fy+Fz", "-Fx-Fy+Fz",
]   # +Fz excluded: confirmed dead (err 0.16 -> -0.06, 40 evals, no motion)

DIRECTION_VECTORS = {
    "+Fx": (1,0,0), "-Fx": (-1,0,0), "+Fy": (0,1,0), "-Fy": (0,-1,0), "-Fz": (0,0,-1),
}
S2 = 1/math.sqrt(2); S3 = 1/math.sqrt(3)
DIRECTION_VECTORS.update({
    "+FxFy": (S2,S2,0), "-FxFy": (-S2,-S2,0), "+FxFz": (S2,0,S2), "-FxFz": (-S2,0,-S2),
    "+FyFz": (0,S2,S2), "-FyFz": (0,-S2,-S2),
    "+Fx+Fy+Fz": (S3,S3,S3), "+Fx-Fy+Fz": (S3,-S3,S3),
    "-Fx+Fy+Fz": (-S3,S3,S3), "-Fx-Fy+Fz": (-S3,-S3,S3),
})


def load_seed(name):
    rp = os.path.join(WORKSPACE_ROOT, "amfm_stage4", f"dir_{name}", "result.json")
    kb = json.load(open(rp))["best_knobs"]
    d1 = {k.split(".",1)[1]: v for k,v in kb.items() if k.startswith("s1.")}
    d2 = {k.split(".",1)[1]: v for k,v in kb.items() if k.startswith("s2.")}
    d1["n"] = int(d1["n"]); d2["n"] = int(d2["n"])
    if name in ("+Fx", "-Fx"):
        d2 = dict(d2, **HEAVE_FX_CORRECTION)
    return d1, d2


def measure_phased(rig, k1: Knobs, k2: Knobs, phase, out_dir, label, period=None):
    """Same pipeline as Rig.measure(), with servo2's array circularly
    rolled by `phase` (0..1 fraction of a cycle) -- exact for any shape,
    since cycle() already returns one full period. `period` is an explicit
    override (not rig.period) so it can be searched per-trial instead of
    fixed for the whole run."""
    period = rig.period if period is None else period
    n_s = int(EX.HW_RATE * period)
    _, th1, _ = cycle(k1.to_params(), period, n_s)
    _, th2, _ = cycle(k2.to_params(), period, n_s)
    th2 = np.roll(th2, int(round(phase * n_s)))
    if (np.max(np.abs(th1)) > EX.PITCH_LIMIT or np.max(np.abs(th2)) > EX.HEAVE_LIMIT):
        return None, "commanded array exceeds position limits"
    for th in (th1, th2):
        if np.max(np.abs(np.gradient(th, period/len(th)))) > EX.SLEW_LIMIT:
            return None, "commanded array exceeds slew limit"

    p1 = rig.fb.get(1, (0.0, 0.0))[0]
    p2 = rig.fb.get(2, (0.0, 0.0))[0]
    for a in np.linspace(0, 1, int(EX.HW_RATE * 1.5)):
        rig.send((1-a)*p1 + a*th1[0], (1-a)*p2 + a*th2[0])
        time.sleep(1.0/EX.HW_RATE)
    t_end = time.time() + EX.SETTLE_S
    while time.time() < t_end:
        rig.send(th1[0], th2[0]); time.sleep(1.0/EX.HW_RATE)
    rig.lc, rig.rec = [], True
    t0 = time.time() + EX.PRE_QUIET_S
    while time.time() < t0:
        rig.send(th1[0], th2[0]); time.sleep(1.0/EX.HW_RATE)
    kin = []
    for _ in range(EX.N_CYCLES):
        for j in range(n_s):
            rig.send(th1[j], th2[j])
            kin.append((time.time()-t0, th1[j], th2[j],
                       rig.fb.get(1,(np.nan,)*2)[0], rig.fb.get(2,(np.nan,)*2)[0]))
            time.sleep(max(0.0, 1.0/EX.HW_RATE - (time.time()-t0-kin[-1][0])))
    t_end = time.time() + EX.IDLE_TAIL_S
    while time.time() < t_end:
        time.sleep(0.005)
    rig.rec = False
    os.makedirs(out_dir, exist_ok=True)
    kp = os.path.join(out_dir, f"{label}_kin.csv")
    np.savetxt(kp, np.array(kin), delimiter=",", header="t,cmd1,cmd2,meas1,meas2", comments="")
    lp = os.path.join(out_dir, f"{label}_force.csv")
    with open(lp, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["wall","payload"])
        for wt, d in rig.lc:
            w.writerow([wt-t0, " ".join(f"{x:.6f}" for x in d)])
    return (kp, lp), None


def run_direction(rig, name, evals, out_root, summary):
    u = DIRECTION_VECTORS[name]
    d1, d2 = load_seed(name)
    folder = os.path.join(out_root, f"dir_{name}")
    out_dir = os.path.join(folder, "data")
    history = []

    def score(kd, phase, period, tag):
        d1x, d2x = dict(d1), dict(d2)
        for key, v in kd.items():
            (d1x if key.startswith("s1.") else d2x)[key.split(".",1)[1]] = v
        k1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1x})
        k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2x})
        paths, err = measure_phased(rig, k1, k2, phase, out_dir, tag, period=period)
        if paths is None:
            print(f"      {tag}: {err}")
            return None
        m = SH.evaluate(paths, "Fx", u)
        if m is None:
            return None
        e, terms = SH.thrust_error(m)
        rec = {"tag": tag, "err": e, "net": terms["net_thrust"],
               "offaxis": terms["other_bias"], "phase": phase, "period": period, **kd}
        history.append(rec)
        print(f"      {tag:20s} err {e:8.3f}  net {rec['net']:+.3f}  "
              f"offaxis {rec['offaxis']:+.3f}  phase {phase:.2f}  period {period:.2f}")
        return e

    k = {f"s1.{n}": d1.get(n,0.0) for n in RATE_KNOBS+["A0"]}
    k.update({f"s2.{n}": d2.get(n,0.0) for n in RATE_KNOBS+["A0"]})
    step = {n: 0.25*(BOUNDS[n.split(".",1)[1] if "." in n else n][1]
                     - BOUNDS[n.split(".",1)[1] if "." in n else n][0])/2
           for n in TUNABLE}
    phase0, period0 = 0.0, EX.PERIOD_S
    base = score(k, phase0, period0, f"{name}_start")
    if base is None:
        print(f"      {name}: seed infeasible, skipping")
        return
    best = (base, dict(k), phase0, period0)
    ev = 0
    while ev < evals and max(step.values()) > 0.01:
        improved = False
        for name_k in TUNABLE:
            if ev >= evals: break
            for sgn in (+1, -1):
                if ev >= evals: break
                ev += 1
                if name_k == "phase":
                    trial_ph = (best[2] + sgn*step["phase"]) % 1.0
                    e = score(best[1], trial_ph, best[3],
                             f"{name}_r{ev:02d}_phase{'+' if sgn>0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, dict(best[1]), trial_ph, best[3]); improved = True; break
                elif name_k == "period":
                    lo, hi = BOUNDS["period"]
                    trial_pd = float(np.clip(best[3] + sgn*step["period"], lo, hi))
                    e = score(best[1], best[2], trial_pd,
                             f"{name}_r{ev:02d}_period{'+' if sgn>0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, dict(best[1]), best[2], trial_pd); improved = True; break
                else:
                    lo, hi = BOUNDS[name_k.split(".",1)[1]]
                    trial = dict(best[1])
                    trial[name_k] = float(np.clip(trial[name_k]+sgn*step[name_k], lo, hi))
                    e = score(trial, best[2], best[3],
                             f"{name}_r{ev:02d}_{name_k}{'+' if sgn>0 else '-'}")
                    if e is not None and e < best[0]:
                        best = (e, trial, best[2], best[3]); improved = True; break
        if not improved:
            for n in step: step[n] *= 0.5

    if history:
        os.makedirs(folder, exist_ok=True)
        with open(os.path.join(folder,"history.csv"),"w",newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(history[0]))
            w.writeheader(); w.writerows(history)
        best_rec = min(history, key=lambda r: r["err"])
        print(f"   {name}: {history[0]['net']:+.3f} -> {best_rec['net']:+.3f} N "
              f"({len(history)} evals)")
        summary.append({"direction": name, "start_net": history[0]["net"],
                        "best_net": best_rec["net"], "best_offaxis": best_rec["offaxis"],
                        "best_phase": best_rec["phase"], "best_period": best_rec["period"],
                        "n_evals": len(history)})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--evals-per-dir", type=int, default=15)
    ap.add_argument("--only", default=None, help="comma-separated subset of directions")
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)
    directions = a.only.split(",") if a.only else ALL_DIRECTIONS

    per = EX.N_CYCLES*EX.PERIOD_S + EX.SETTLE_S + EX.PRE_QUIET_S + EX.IDLE_TAIL_S
    total = len(directions) * (a.evals_per_dir + 1)
    print(f"{len(directions)} directions x ~{a.evals_per_dir+1} evals = "
          f"~{total} missions, ~{total*per/60:.0f} min\n")

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    summary = []
    try:
        for i, name in enumerate(directions, 1):
            print(f"\n===== [{i}/{len(directions)}] {name} =====")
            run_direction(rig, name, a.evals_per_dir, folder, summary)
    finally:
        rig.stop()

    if summary:
        with open(os.path.join(folder, "summary.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(summary[0]))
            w.writeheader(); w.writerows(summary)
        print(f"\n{'direction':12s}{'start net':>11}{'best net':>10}{'gain':>8}"
              f"{'phase':>8}{'period':>8}")
        for s in summary:
            gain = s["best_net"] - s["start_net"]
            print(f"{s['direction']:12s}{s['start_net']:11.3f}{s['best_net']:10.3f}"
                  f"{gain:+8.3f}{s['best_phase']:8.2f}{s['best_period']:8.2f}")
        json.dump(summary, open(os.path.join(folder,"summary.json"),"w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
