#!/usr/bin/env python3
"""Chase the s2.s_com relationship stage 1 found and nobody exploited.

Stage-1's OWN sweep (this servo warped, the OTHER held flat) showed a strong,
clean, monotone trend for BOTH warp knobs:
    corr(s2.s_com, ratio)     = -0.97
    corr(s2.s_com, net Fx)    = +0.997
but was only ever tested to +-0.45 (its true bound is +-0.70), and never
carried into a later campaign in its favoured direction: thrust2's actual
optimum landed on s2.s_com=-0.175, the OPPOSITE sign from what the isolated
sweep favours. Two explanations fit that fact and only a real test tells them
apart: a genuine interaction effect (the isolated optimum moves once other
knobs are also active), or coordinate descent simply never finding that
region while 15 other knobs were also in play.

PHASE 1  ISOLATE.  Fine sweep of s2.s_com alone (pitch held at plain nominal
         sine) across its FULL bound, and the same for s1.s_com (heave held
         flat) -- stage 1 only ever checked one side of this for either knob
         at coarse resolution; this finds the true isolated optimum for both.

PHASE 2  COMBINE.  Three combinations, each backed by a specific number from
         this session rather than guessed:
           (a) both isolated optima together, nominal otherwise
           (b) isolated-best s2.s_com substituted into thrust2's full best
               knob set (does the fix beat interaction, holding everything
               else at its already-optimized value?)
           (c) isolated-best s1.s_com AND s2.s_com both substituted into
               thrust2's set

usage:  scom_isolate_combine.py <folder> [--period 2.0] [--levels 9]
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

BOUND = 0.70   # s_com's actual BOUNDS in amfm_shaper.py


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
           "crest": m.get("crest_height", 0.0), "trough": m.get("trough_depth", 0.0),
           "s1_s_com": k1.s_com, "s2_s_com": k2.s_com}
    history.append(rec)
    ratio = rec["trough"] / max(rec["crest"], 1e-9)
    print(f"   {label:26s} net_Fx {rec['net_Fx']:+.3f}   net_Fy {rec['net_Fy']:+.3f}"
          f"   ratio {ratio:.3f}")
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--levels", type=int, default=9)
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
        print(f"--- PHASE 1a: isolate s2.s_com (heave warped, PITCH FLAT), "
              f"{a.levels} points across full +-{BOUND}")
        levels = np.linspace(-BOUND, BOUND, a.levels)
        for v in levels:
            k1 = Knobs(A0=EX.NOMINAL[1].A0, n=1)                 # pitch: plain flat sine
            k2 = Knobs(A0=EX.NOMINAL[2].A0, n=1, s_com=float(v))  # heave: warped
            run(rig, k1, k2, out_dir, f"s2_scom_{v:+.3f}", history)

        print(f"\n--- PHASE 1b: isolate s1.s_com (pitch warped, HEAVE FLAT), "
              f"{a.levels} points across full +-{BOUND}")
        for v in levels:
            k1 = Knobs(A0=EX.NOMINAL[1].A0, n=1, s_com=float(v))  # pitch: warped
            k2 = Knobs(A0=EX.NOMINAL[2].A0, n=1)                  # heave: plain flat sine
            run(rig, k1, k2, out_dir, f"s1_scom_{v:+.3f}", history)

        s2_sweep = [r for r in history if r["label"].startswith("s2_scom")]
        s1_sweep = [r for r in history if r["label"].startswith("s1_scom")]
        best_s2 = max(s2_sweep, key=lambda r: r["net_Fx"])
        best_s1 = max(s1_sweep, key=lambda r: r["net_Fx"])
        print(f"\n   isolated best: s2.s_com={best_s2['s2_s_com']:+.3f} "
              f"(net_Fx {best_s2['net_Fx']:+.3f})")
        print(f"   isolated best: s1.s_com={best_s1['s1_s_com']:+.3f} "
              f"(net_Fx {best_s1['net_Fx']:+.3f})")

        print("\n--- PHASE 2: combinations backed by this session's own data")
        # (a) both isolated optima together, nominal otherwise
        k1 = Knobs(A0=EX.NOMINAL[1].A0, n=1, s_com=best_s1["s1_s_com"])
        k2 = Knobs(A0=EX.NOMINAL[2].A0, n=1, s_com=best_s2["s2_s_com"])
        run(rig, k1, k2, out_dir, "combo_a_both_isolated", history)

        # (b)/(c): substitute into thrust2's actual optimum
        rp = os.path.join(WORKSPACE_ROOT, "amfm_thrust2", "result.json")
        if os.path.exists(rp):
            kb = json.load(open(rp))["best_knobs"]
            d1 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s1.")}
            d2 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s2.")}
            d1["n"] = int(d1["n"]); d2["n"] = int(d2["n"])

            d2b = dict(d2); d2b["s_com"] = best_s2["s2_s_com"]
            k1b = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
            k2b = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2b})
            run(rig, k1b, k2b, out_dir, "combo_b_thrust2_fixed_s2", history)

            d1c = dict(d1); d1c["s_com"] = best_s1["s1_s_com"]
            d2c = dict(d2); d2c["s_com"] = best_s2["s2_s_com"]
            k1c = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1c})
            k2c = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2c})
            run(rig, k1c, k2c, out_dir, "combo_c_thrust2_fixed_both", history)

            # reference point: thrust2 itself, replayed here for a fair
            # same-session, same-conditions comparison
            k1r = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
            k2r = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
            run(rig, k1r, k2r, out_dir, "reference_thrust2_asis", history)
        else:
            print("   amfm_thrust2/result.json not found — skipping combos b/c")
    finally:
        rig.stop()

    if history:
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(history[0]))
            w.writeheader()
            w.writerows(history)
        print("\n" + "=" * 66)
        print("SUMMARY")
        print("=" * 66)
        for r in history:
            ratio = r["trough"] / max(r["crest"], 1e-9)
            print(f"   {r['label']:26s} net_Fx {r['net_Fx']:+.3f}   "
                  f"net_Fy {r['net_Fy']:+.3f}   ratio {ratio:.3f}")
        best = max(history, key=lambda r: r["net_Fx"])
        print(f"\n   BEST OVERALL: {best['label']}  net_Fx {best['net_Fx']:+.3f}")
        json.dump({"history_best": best, "n_evals": len(history)},
                  open(os.path.join(folder, "result.json"), "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
