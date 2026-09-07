#!/usr/bin/env python3
"""Does the feathering offset control the Fx trough?

A targeted falsification test of one specific claim: that the pitch centre
offset C sets the depth of the drag trough with "near-zero impact on the
thrust crest".

The stage-1 Jacobian says the opposite -- C is the most CREST-selective knob
measured (dFx_crest/dC = +0.763 against dFx_trough/dC = +0.097, a selectivity
of 0.13) -- so this is worth running precisely because the two disagree. If C
crushes the trough, the Jacobian is missing something and the whole shaping
approach needs revisiting. If it moves the crest instead, the claim is dead on
this rig and we stop spending evaluations on it.

Two base gaits, because the claim could be true locally without being true
generally:

  NOMINAL   the operating point the Jacobian was measured at -- the fair test
            of the Jacobian's own prediction.
  BEST      the best thrust gait found so far -- the test of whether
            feathering adds anything on top of what the optimiser already has.

C is swept alone on each servo in turn; every other knob is held. That is the
only way to attribute a change to C rather than to a combination.

usage:  feather_test.py <folder> [--levels 7]
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
import amfm_shaper as SH              # noqa: E402
from amfm_waveform import Knobs       # noqa: E402


def base_gaits():
    """(name, {qualified knob: value}) for each starting point."""
    nom = {f"s{s}.{n}": getattr(EX.NOMINAL[s], n)
           for s in (1, 2) for n in list(SH.TUNABLE) + ["n"]}
    out = [("nominal", nom)]

    res = os.path.join(WORKSPACE_ROOT, "amfm_thrust", "result.json")
    if os.path.exists(res):
        r = json.load(open(res))
        kb = r.get("best_knobs") or {}
        if kb:
            best = dict(nom)
            best.update({k: v for k, v in kb.items() if k in nom})
            out.append(("best_thrust", best))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--levels", type=int, default=7)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)

    lo, hi = SH.BOUNDS["C"]
    levels = np.linspace(lo, hi, a.levels)

    rig = SH.Rig()
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    rows = []
    try:
        for bname, base in base_gaits():
            for servo in (1, 2):
                print(f"--- base {bname}, sweeping s{servo}.C")
                for lv in levels:
                    kd = dict(base)
                    kd[f"s{servo}.C"] = float(lv)
                    d1, d2 = SH._split(kd)
                    k1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
                    k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
                    label = f"{bname}_s{servo}C_{lv:+.3f}".replace("+", "p").replace("-", "m")
                    paths, err = rig.measure(k1, k2, os.path.join(folder, "data"), label)
                    if paths is None:
                        print(f"   C={lv:+.3f}: {err}")
                        continue
                    m = SH.evaluate(paths, "Fx", None)
                    if m is None:
                        print(f"   C={lv:+.3f}: no force data")
                        continue
                    crest = m.get("crest_height", 0.0)
                    trough = m.get("trough_depth", 0.0)
                    rows.append({"base": bname, "servo": servo, "C": float(lv),
                                 "crest": crest, "trough": trough,
                                 "ratio": trough / max(crest, 1e-9),
                                 "net_Fx": m.get("bias", 0.0),
                                 "net_Fy": m.get("other_bias", 0.0),
                                 "label": label})
                    print(f"   C={lv:+.3f}   crest {crest:6.3f}  trough {trough:6.3f}"
                          f"  ratio {rows[-1]['ratio']:5.3f}  net {rows[-1]['net_Fx']:+.3f}")
    finally:
        rig.stop()

    if not rows:
        print("nothing measured")
        return 1
    with open(os.path.join(folder, "feather.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    print("\n" + "=" * 74)
    print("VERDICT — does C control the trough, or the crest?")
    print("=" * 74)
    for bname in sorted({r["base"] for r in rows}):
        for servo in (1, 2):
            g = [r for r in rows if r["base"] == bname and r["servo"] == servo]
            if len(g) < 3:
                continue
            C = np.array([r["C"] for r in g])
            dc = np.ptp(C)
            sc = np.polyfit(C, [r["crest"] for r in g], 1)[0]
            st = np.polyfit(C, [r["trough"] for r in g], 1)[0]
            sel = abs(st) / max(abs(sc), 1e-9)
            claim = ("TROUGH-selective (claim supported)" if sel > 2.0 else
                     "CREST-selective (claim refuted)" if sel < 0.5 else
                     "moves both")
            print(f"   {bname:12s} s{servo}.C   d(crest)/dC {sc:+7.3f}   "
                  f"d(trough)/dC {st:+7.3f}   sel {sel:5.2f}   {claim}")
            print(f"                   trough range over the sweep: "
                  f"{min(r['trough'] for r in g):.3f} .. {max(r['trough'] for r in g):.3f}"
                  f"   (ratio {min(r['ratio'] for r in g):.3f} .. "
                  f"{max(r['ratio'] for r in g):.3f})")
    print(f"\nwrote {folder}/feather.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
