#!/usr/bin/env python3
"""Grey-box identification, validation and the gait-C prediction test.

Runs the full system-identification workflow on the stage-1 campaign:

  1. LOAD    aligned kinematics + force, one steady cycle per mission.
  2. SPLIT   A = stage J (estimation)   B = stage S (validation)
             C = stage P (prediction -- never touched until step 5)
  3. SELECT  fit every candidate structure on A, rank on B.
  4. DIAGNOSE residual auto/cross-correlation to say WHERE the model fails.
  5. PREDICT gait C and compare. This is the falsification test: the numbers
             are produced by the model from kinematics alone, and the
             measured forces are only opened afterwards.

The order matters. Ranking on the estimation data would always favour the
higher-order structure, and checking gait C before fixing the structure would
turn the prediction set into just another tuning set.

usage:  greybox_report.py <folder> [--predict-only]
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

import amfm_analyze as AN          # noqa: E402
import greybox_model as GB         # noqa: E402


def load_dataset(folder, row):
    """One mission -> (t, th1, th2, Fx, Fy, Fz) on a common time grid.

    Force is resampled onto the KINEMATICS grid (100 Hz) rather than the
    reverse: the model is driven by joint velocity and acceleration, and
    differentiating a 100 Hz trace onto a 10 kHz grid would manufacture
    derivative noise the measurement does not contain.
    """
    kp = os.path.join(folder, "data", row["kin_csv"])
    fp = os.path.join(folder, "data", row["force_csv"])
    if not (os.path.exists(kp) and os.path.exists(fp)):
        return None
    k = AN.load_kin(kp)
    f = AN.load_force(fp)
    if k is None or f is None:
        return None
    tk, c1, c2, m1, m2 = k
    tf, Fx, Fy, Fz = f
    tk = tk - tk[0]
    tf = tf - tf[0]
    Fxi = np.interp(tk, tf, Fx)
    Fyi = np.interp(tk, tf, Fy)
    Fzi = np.interp(tk, tf, Fz)
    # measured joint angles, not commanded: the model must be identified
    # against the motion that actually happened
    good = np.isfinite(m1) & np.isfinite(m2)
    if good.sum() < 50:
        return None
    return (tk[good], m1[good], m2[good], Fxi[good], Fyi[good], Fzi[good])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--predict-only", action="store_true")
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)

    man = list(csv.DictReader(open(os.path.join(folder, "manifest.csv"))))
    sets = {"J": [], "S": [], "P": [], "R": []}
    labels = {"J": [], "S": [], "P": [], "R": []}
    for r in man:
        d = load_dataset(folder, r)
        if d is None:
            continue
        st = r.get("stage", "J")
        if st in sets:
            sets[st].append(d)
            labels[st].append(r["label"])

    est, val, pred = sets["J"] + sets["R"], sets["S"], sets["P"]
    print(f"estimation (A, stage J+R): {len(est)} missions")
    print(f"validation (B, stage S)  : {len(val)} missions")
    print(f"prediction (C, stage P)  : {len(pred)} missions\n")
    if len(est) < 4 or len(val) < 4:
        print("not enough data to identify — run the stage-1 campaign first")
        return 1

    # ---------------- 3. structure selection
    print("=" * 78)
    print("MODEL SELECTION  (fitted on A, ranked on HELD-OUT B)")
    print("=" * 78)
    results = GB.select(est, val)
    best = results[0]
    print(f"\n   chosen: {best['model']}, sweep servo "
          f"{'2' if best['swap'] else '1'}  (validation mean "
          f"{best['val_mean']:.1f}%)")
    print("   parameters: " + "  ".join(f"{k}={v:+.4f}" for k, v in best["params"].items()))

    # Does the extra tangential term earn its place?
    m1b = next((r for r in results if r["model"] == "M1" and r["swap"] == best["swap"]), None)
    m2b = next((r for r in results if r["model"] == "M2" and r["swap"] == best["swap"]), None)
    if m1b and m2b:
        d = m2b["val_mean"] - m1b["val_mean"]
        print(f"\n   M2 - M1 on validation: {d:+.1f} percentage points -> "
              + ("M2 justified" if d > 2.0 else
                 "M2 NOT justified; the tangential term adds parameters without "
                 "generalising, so M1 stands"))

    # ---------------- 4. residual diagnostics
    print()
    print("=" * 78)
    print("RESIDUAL DIAGNOSTICS  (on validation data)")
    print("=" * 78)
    diag = GB.residual_diagnostics(best["x"], val, best["model"], best["swap"])
    print(GB.verdict(diag))
    print()
    print("   reminder: high cross-correlation means dynamics missing in the")
    print("   input->output path; low cross- but high auto-correlation means the")
    print("   system model is fine and a disturbance model is what is missing.")

    bz = best["params"].get("bz", 0.0)
    print(f"\n   Fz offset bz = {bz:+.4f} N — a constant, kinematics-independent")
    print("   term. A large value here is the mechanical-asymmetry disturbance")
    print("   appearing as a model parameter rather than as unexplained noise.")

    # ---------------- 5. gait C
    print()
    print("=" * 78)
    print("PREDICTION TEST — GAIT C  (model has never seen these)")
    print("=" * 78)
    if not pred:
        print("   no stage-P missions found; run them to complete the test.")
    else:
        rows = []
        for lab, ds in zip(labels["P"], pred):
            t, th1, th2, Fx, Fy, Fz = ds
            sw, pi_ = (th2, th1) if best["swap"] else (th1, th2)
            px, py, pz = GB.predict(best["x"], sw, pi_, t, best["model"])
            rows.append({"label": lab,
                         "fit_Fx": GB.nrmse(px, Fx), "fit_Fy": GB.nrmse(py, Fy),
                         "pred_Fx_peak": float(px.max()), "meas_Fx_peak": float(Fx.max()),
                         "pred_Fy_peak": float(py.max()), "meas_Fy_peak": float(Fy.max())})
        print(f"{'gait':16s}{'fit Fx %':>10}{'fit Fy %':>10}"
              f"{'Fx peak pred/meas':>22}{'Fy peak pred/meas':>22}")
        print("-" * 80)
        for r in rows:
            print(f"{r['label']:16s}{r['fit_Fx']:10.1f}{r['fit_Fy']:10.1f}"
                  f"{r['pred_Fx_peak']:11.3f}/{r['meas_Fx_peak']:<10.3f}"
                  f"{r['pred_Fy_peak']:11.3f}/{r['meas_Fy_peak']:<10.3f}")
        mx = float(np.nanmean([r["fit_Fx"] for r in rows]))
        my = float(np.nanmean([r["fit_Fy"] for r in rows]))
        print(f"\n   mean prediction fit: Fx {mx:.1f}%   Fy {my:.1f}%")
        vx, vy = best["val"]["Fx"], best["val"]["Fy"]
        drop = ((vx + vy) - (mx + my)) / 2.0
        print(f"   versus validation  : Fx {vx:.1f}%   Fy {vy:.1f}%   "
              f"(drop {drop:+.1f} pts)")
        if drop > 15:
            print("   -> large drop on unseen gaits: the model is interpolating,")
            print("      not generalising. Structure needs revisiting.")
        else:
            print("   -> holds up on gaits it was never fitted to: the structure")
            print("      is carrying real physics, not just curve-fitting stage J.")

    # ---------------- the structural claim
    print()
    print("=" * 78)
    print("THE STRUCTURAL CLAIM:  are Fx and Fy one rotated force?")
    print("=" * 78)
    print("   M1 says Fx and Fy are the SAME scalar F_N projected through the")
    print("   blade orientation. If M1 validates well, independent Fx/Fy shaping")
    print("   is structurally impossible -- but arbitrary-direction thrust is a")
    print("   geometry problem (point the normal), which makes stage 4 cheap.")
    if best["model"] == "M1" and best["val_mean"] > 60:
        print(f"\n   -> M1 validates at {best['val_mean']:.1f}%: the claim SURVIVES.")
        print("      Stage 4 should start from the analytic blade orientation for")
        print("      each requested direction rather than searching blind.")
    elif best["model"] == "M2":
        print(f"\n   -> M2 won: a tangential component exists, so the in-plane force")
        print("      has two degrees of freedom, not one. Some independent Fx/Fy")
        print("      control is available -- how much is set by C_t/C_d = "
              f"{abs(best['params'].get('C_t',0))/max(abs(best['params'].get('C_d',1)),1e-9):.3f}.")
    else:
        print(f"\n   -> validation only {best['val_mean']:.1f}%: neither structure is")
        print("      carrying the physics; treat the claim as unproven.")

    json.dump({"model": best["model"], "swap": best["swap"],
               "params": best["params"], "est": best["est"], "val": best["val"],
               "diagnostics": {k: {kk: vv for kk, vv in v.items()
                                   if not isinstance(vv, list)}
                               for k, v in diag.items()}},
              open(os.path.join(folder, "greybox.json"), "w"), indent=2)
    print(f"\nwrote {folder}/greybox.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
