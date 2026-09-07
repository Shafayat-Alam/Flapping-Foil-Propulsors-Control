#!/usr/bin/env python3
"""Correct the 3 flipped heave knobs, then let a joint search find the real
combined optimum -- not just test one guessed point.

BACKGROUND (see amfm_shaping/metrics.csv, stage J+S, no new hardware needed
to establish this): every strong isolated (this-knob-alone, other axis flat)
relationship on PITCH matches what thrust2's optimizer converged to. Three of
four strong ones on HEAVE do not -- the optimizer landed on the opposite sign
from what the isolated sweep favours:

    knob         isolated favours     thrust2 used
    s2.s_com            +                 -0.175   FLIPPED
    s2.w_diff            -                 +0.700   FLIPPED
    s2.h_diff            -                 +0.320   FLIPPED
    (s1.s_com, s1.w_com, s1.w_diff, s2.h_com all matched -- pitch was fine)

Two explanations, indistinguishable without a real test: a genuine
interaction effect (heave's isolated optimum moves once amplitude and the
other knobs are also active), or coordinate descent simply never finding
that region while 15 other knobs were in play. This test is built to tell
them apart, not just report a single fixed-point comparison.

PHASE 1  ISOLATE. Fine sweep of EACH flipped knob alone (heave warped,
         pitch flat), full bound, to pin the true isolated optimum precisely
         (stage 1 only ever checked +-0.45 of a +-0.70 bound).

PHASE 2  CORRECTED STARTING POINT. All three flipped signs corrected
         together in thrust2's knob set (everything else unchanged).

PHASE 3  JOINT REFINEMENT. Coordinate descent from the corrected point,
         over BOTH axes' rate knobs (s_com, w_com, s_diff, w_diff) AND
         amplitude (A0) on both servos -- so if amplitude needs to move to
         accommodate the correction (the interaction-effect explanation),
         the search can find that instead of being blocked from it.

FREQUENCY is deliberately NOT folded in here -- it is already characterised
on its own (net thrust ~ 1/T^2.34, amfm_freq/) and combining it would roughly
double this search's cost for a dimension that scales in a known, separate
way. Once this converges, replaying the winning shape at a faster period
(as attempted, inconclusively, in amfm_thrust_fast/) is the natural next
step, not a fourth simultaneous dimension here.

usage:  heave_flip_test.py <folder> [--period 2.0] [--iso-levels 7] [--refine-evals 30]
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

import amfm_experiment as EX          # noqa: E402
import amfm_shaper as SH               # noqa: E402
from amfm_waveform import Knobs        # noqa: E402

BOUND = 0.70
FLIPPED = {"s_com": +1, "w_diff": -1, "h_diff": -1}   # isolated-favoured SIGN, servo2
REFINE_KNOBS = ["s1.s_com", "s1.w_com", "s1.s_diff", "s1.w_diff",
                "s2.s_com", "s2.w_com", "s2.s_diff", "s2.w_diff",
                "s1.A0", "s2.A0"]
REFINE_BOUNDS = {"s_com": (-BOUND, BOUND), "w_com": (-BOUND, BOUND),
                 "s_diff": (-BOUND, BOUND), "w_diff": (-BOUND, BOUND),
                 "A0": (0.15, 0.80)}


def load_thrust2():
    r = json.load(open(os.path.join(WORKSPACE_ROOT, "amfm_thrust2", "result.json")))
    kb = r["best_knobs"]
    d1 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s1.")}
    d2 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s2.")}
    d1["n"] = int(d1["n"]); d2["n"] = int(d2["n"])
    return d1, d2


def make_knobs(d1, d2):
    return (Knobs(**{**EX.NOMINAL[1].as_dict(), **d1}),
            Knobs(**{**EX.NOMINAL[2].as_dict(), **d2}))


def run(rig, k1, k2, out_dir, label, history):
    paths, err = rig.measure(k1, k2, out_dir, label)
    if paths is None:
        print(f"   {label}: {err}")
        return None
    m = SH.evaluate(paths, "Fx", None)
    if m is None:
        print(f"   {label}: no force data")
        return None
    e, terms = SH.thrust_error(m)
    rec = {"label": label, "err": e, "net_Fx": terms["net_thrust"],
           "net_Fy": terms["other_bias"],
           "crest": m.get("crest_height", 0.0), "trough": m.get("trough_depth", 0.0)}
    history.append(rec)
    ratio = rec["trough"] / max(rec["crest"], 1e-9)
    print(f"   {label:26s} net_Fx {rec['net_Fx']:+.3f}   net_Fy {rec['net_Fy']:+.3f}"
          f"   ratio {ratio:.3f}")
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--iso-levels", type=int, default=7)
    ap.add_argument("--refine-evals", type=int, default=30)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    out_dir = os.path.join(folder, "data")
    os.makedirs(out_dir, exist_ok=True)

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    history = []
    try:
        # ---- PHASE 1: isolate each flipped knob precisely
        print(f"--- PHASE 1: isolate the 3 flipped heave knobs, "
              f"{a.iso_levels} levels each, full +-{BOUND}")
        levels = np.linspace(-BOUND, BOUND, a.iso_levels)
        iso_best = {}
        for kn in FLIPPED:
            print(f"  {kn}:")
            recs = []
            for v in levels:
                k1 = Knobs(A0=EX.NOMINAL[1].A0, n=1)
                k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), kn: float(v)})
                r = run(rig, k1, k2, out_dir, f"iso_{kn}_{v:+.3f}", history)
                if r:
                    recs.append((v, r["net_Fx"]))
            if recs:
                iso_best[kn] = max(recs, key=lambda x: x[1])[0]
                print(f"     isolated best {kn} = {iso_best[kn]:+.3f}")

        # ---- PHASE 2: corrected starting point
        d1, d2 = load_thrust2()
        d2_fixed = dict(d2)
        for kn, v in iso_best.items():
            d2_fixed[kn] = v
        # informed by af_ratio_test.py: heave amplitude (A2) correlated
        # strongly and CONSISTENTLY with net thrust (+0.56 to +0.89 across
        # every frequency combination tested), far more reliably than pitch
        # amplitude -- start the refinement near that validated region
        # (0.70) instead of thrust2's lower value (~0.42), so the search
        # begins close to where the evidence points rather than re-finding
        # it from scratch.
        d2_fixed["A0"] = 0.70
        k1c, k2c = make_knobs(d1, d2_fixed)
        print("\n--- PHASE 2: thrust2 with all 3 heave knobs corrected")
        run(rig, k1c, k2c, out_dir, "corrected_start", history)
        # reference for a fair same-session comparison
        k1r, k2r = make_knobs(d1, d2)
        run(rig, k1r, k2r, out_dir, "reference_thrust2", history)

        # ---- PHASE 3: joint refinement from the corrected point
        print(f"\n--- PHASE 3: joint refinement, {a.refine_evals} evals, "
              f"both axes' rate knobs + amplitude")
        k = {**{f"s1.{n}": d1.get(n, 0.0) for n in ("s_com","w_com","s_diff","w_diff","A0")},
             **{f"s2.{n}": d2_fixed.get(n, 0.0) for n in ("s_com","w_com","s_diff","w_diff","A0")}}
        step = {n: 0.25 * (REFINE_BOUNDS[n.split(".",1)[1]][1]
                           - REFINE_BOUNDS[n.split(".",1)[1]][0]) / 2
               for n in REFINE_KNOBS}

        def score(kd, tag):
            d1x = dict(d1); d2x = dict(d2_fixed)
            for key, v in kd.items():
                (d1x if key.startswith("s1.") else d2x)[key.split(".",1)[1]] = v
            k1x, k2x = make_knobs(d1x, d2x)
            r = run(rig, k1x, k2x, out_dir, tag, history)
            return r["net_Fx"] if r else None

        best_val = score(k, "refine_start")
        if best_val is None:
            print("   corrected start infeasible — stopping at phase 2 result")
        else:
            best = (best_val, dict(k))
            ev = 0
            while ev < a.refine_evals and max(step.values()) > 0.01:
                improved = False
                for name in REFINE_KNOBS:
                    if ev >= a.refine_evals:
                        break
                    for sgn in (+1, -1):
                        if ev >= a.refine_evals:
                            break
                        lo, hi = REFINE_BOUNDS[name.split(".",1)[1]]
                        trial = dict(best[1])
                        trial[name] = float(np.clip(trial[name] + sgn*step[name], lo, hi))
                        ev += 1
                        v = score(trial, f"ref{ev:02d}_{name}{'+' if sgn>0 else '-'}")
                        if v is not None and v > best[0]:
                            best = (v, trial)
                            improved = True
                            break
                if not improved:
                    for n in step:
                        step[n] *= 0.5
            print(f"\n   refined best net_Fx = {best[0]:+.3f}")
            print("   knobs: " + "  ".join(f"{k}={v:+.4f}" for k,v in best[1].items()))
    finally:
        rig.stop()

    if history:
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(history[0]))
            w.writeheader(); w.writerows(history)
        best = max(history, key=lambda r: r["net_Fx"])
        ref = next((r for r in history if r["label"] == "reference_thrust2"), None)
        print("\n" + "=" * 66)
        print(f"BEST OVERALL: {best['label']}  net_Fx {best['net_Fx']:+.3f}")
        if ref:
            print(f"vs thrust2 (this session, same conditions): net_Fx {ref['net_Fx']:+.3f}")
            print(f"delta: {best['net_Fx']-ref['net_Fx']:+.3f} N "
                  f"({100*(best['net_Fx']/max(ref['net_Fx'],1e-9)-1):+.0f}%)")
        json.dump({"best": best, "n_evals": len(history)},
                  open(os.path.join(folder, "result.json"), "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
