#!/usr/bin/env python3
"""Does period matter more than shape?

Every mission in this entire session ran at the same 2.0 s period. Quasi-
steady drag goes as C_d*|v|*v, so force should scale roughly with velocity
SQUARED -- and velocity scales as 1/period for a fixed angular shape. If that
holds, halving the period should roughly quadruple the force, which would
dwarf anything found by shaping the waveform at a fixed period.

This holds the BEST KNOWN SHAPE fixed (from --result, default amfm_thrust2
falling back to amfm_thrust) and sweeps only PERIOD_S, from slow down to the
fastest period the shape can run at within the servo slew limit. Everything
else -- gait shape, both servos, the Fy null question -- is held constant, so
period is isolated as the one variable.

usage:  freq_sweep.py <folder> [--result amfm_thrust2] [--levels 10] [--reps 2]
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

import amfm_experiment as EX                                   # noqa: E402
from amfm_waveform import Knobs, cycle, max_feasible_period    # noqa: E402
import amfm_shaper as SH                                       # noqa: E402


def load_best(result_path):
    r = json.load(open(result_path))
    kb = r["best_knobs"]
    d1 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s1.")}
    d2 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s2.")}
    d1["n"] = int(d1.get("n", EX.NOMINAL[1].n))
    d2["n"] = int(d2.get("n", EX.NOMINAL[2].n))
    k1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
    k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
    return k1, k2


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--result", default="amfm_thrust2")
    ap.add_argument("--levels", type=int, default=10)
    ap.add_argument("--reps", type=int, default=2,
                    help="replicate the fastest and slowest period this many "
                         "times, to separate a real trend from rig noise")
    ap.add_argument("--margin", type=float, default=1.05,
                    help="safety margin above the computed slew floor")
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)

    rp = os.path.join(WORKSPACE_ROOT, a.result, "result.json")
    if not os.path.exists(rp):
        rp = os.path.join(WORKSPACE_ROOT, "amfm_thrust", "result.json")
        print(f"   {a.result}/result.json not found, falling back to amfm_thrust")
    k1, k2 = load_best(rp)
    p1, p2 = k1.to_params(), k2.to_params()

    floor1 = max_feasible_period(p1, EX.PITCH_LIMIT, EX.SLEW_LIMIT)
    floor2 = max_feasible_period(p2, EX.HEAVE_LIMIT, EX.SLEW_LIMIT)
    floor = max(floor1, floor2) * a.margin
    slow = 3.0
    periods = np.linspace(slow, floor, a.levels)
    print(f"shape from {rp}")
    print(f"slew floor: servo1 {floor1:.3f}s  servo2 {floor2:.3f}s  "
          f"-> sweeping {slow:.2f}s down to {floor:.3f}s ({a.levels} levels)")

    plan = [{"period": float(pd), "rep": 0} for pd in periods]
    for pd in (periods[0], periods[-1]):
        for r in range(1, a.reps):
            plan.append({"period": float(pd), "rep": r})

    rig = SH.Rig()
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    rows = []
    try:
        for i, item in enumerate(plan, 1):
            period = item["period"]
            rig.period = period          # Rig.measure() reads self.period
            n_s = int(EX.HW_RATE * period)
            _, th1, dth1 = cycle(p1, period, n_s)
            _, th2, dth2 = cycle(p2, period, n_s)
            if (np.max(np.abs(dth1)) > EX.SLEW_LIMIT
                    or np.max(np.abs(dth2)) > EX.SLEW_LIMIT):
                print(f"[{i}/{len(plan)}] T={period:.3f}s: SKIPPED (slew limit)")
                continue
            label = f"T{period:.3f}_r{item['rep']}".replace(".", "p")
            paths, err = rig.measure(k1, k2, os.path.join(folder, "data"), label)
            if paths is None:
                print(f"[{i}/{len(plan)}] T={period:.3f}s: {err}")
                continue
            m = SH.evaluate(paths, "Fx", None)
            if m is None:
                print(f"[{i}/{len(plan)}] T={period:.3f}s: no force data")
                continue
            rows.append({"period": period, "rep": item["rep"], "label": label,
                        "net_Fx": m.get("bias", 0.0),
                        "net_Fy": m.get("other_bias", 0.0),
                        "crest": m.get("crest_height", 0.0),
                        "trough": m.get("trough_depth", 0.0)})
            print(f"[{i}/{len(plan)}] T={period:.3f}s   net_Fx {rows[-1]['net_Fx']:+.3f}"
                  f"   net_Fy {rows[-1]['net_Fy']:+.3f}   crest {rows[-1]['crest']:.3f}")
    finally:
        rig.stop()

    if not rows:
        print("nothing measured")
        return 1
    with open(os.path.join(folder, "freq_sweep.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print("\n" + "=" * 74)
    print("PERIOD vs NET THRUST")
    print("=" * 74)
    base = [r for r in rows if r["rep"] == 0]
    base.sort(key=lambda r: -r["period"])
    for r in base:
        print(f"   T={r['period']:.3f}s  f={1/r['period']:.3f}Hz   "
              f"net_Fx {r['net_Fx']:+.3f}   net_Fy {r['net_Fy']:+.3f}")

    if len(base) >= 3:
        T = np.array([r["period"] for r in base])
        F = np.array([r["net_Fx"] for r in base])
        # fit |F| ~ k / T^n  in log space (only where F has a consistent sign)
        pos = F > 1e-4
        if pos.sum() >= 3:
            n_fit = np.polyfit(np.log(T[pos]), np.log(F[pos]), 1)[0]
            print(f"\n   fitted scaling: net_Fx ~ 1/T^{-n_fit:.2f}   "
                  f"(quasi-steady drag predicts ~1/T^2)")
    print(f"\nwrote {folder}/freq_sweep.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
