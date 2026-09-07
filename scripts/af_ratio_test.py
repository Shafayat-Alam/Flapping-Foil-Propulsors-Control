#!/usr/bin/env python3
"""Does amplitude-to-frequency RATIO predict thrust better than either alone?

This is the flapping-foil Strouhal-number question (St ~ f*A/U, the classic
dimensionless parameter governing thrust efficiency in real flapping
propulsion) applied to this rig's two axes and their two "frequencies"
(each servo's own crest count n, which stage 4 already showed CAN differ
between axes -- thrust2's winner had s1.n=2, s2.n=1). No existing data varies
amplitude and n together (checked amfm_shaping/metrics.csv: A0 and n were
only ever perturbed independently, never jointly) so this needs new
measurements, not just re-analysis.

  A1 = s1.A0 (pitch amplitude)   f1 = s1.n (pitch frequency)
  A2 = s2.A0 (heave amplitude)   f2 = s2.n (heave frequency)

PHASE 1  ISOLATE the four cross ratios. For each amplitude axis, sweep it at
         BOTH its own frequency and the OTHER axis's frequency, other axis
         held flat:
           A1 vs f1   (own)      A1 vs f2   (cross)
           A2 vs f2   (own)      A2 vs f1   (cross)
         4 sweeps x 5 amplitude levels = 20 missions.

PHASE 2  COMPARE. For each sweep, correlation of Fx_bias against raw
         amplitude vs against amplitude/frequency ratio -- whichever is
         higher tells you whether the RATIO is the thing that actually
         matters, or just amplitude alone (with n along for the ride).

PHASE 3  COMBINE. n is a genuine integer (crest count), not continuously
         tunable, so this treats the 4 (n1,n2) combinations as a discrete
         choice and refines A0 (+ the already-established heave rate
         correction, + phase) continuously within whichever combination
         phase 2 flags as best -- so if the ratio idea is real, this is
         where it gets combined with everything else already found to work,
         not left as an isolated finding.

usage:  af_ratio_test.py <folder> [--period 2.0] [--levels 5] [--refine-evals 20]
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
from amfm_waveform import Knobs, cycle # noqa: E402

A_BOUNDS = {1: (0.20, 0.90), 2: (0.15, 0.70)}   # PITCH_LIMIT/HEAVE_LIMIT margin
HEAVE_CORRECTION = {"s_com": +0.467, "w_diff": 0.0}  # from heave_flip_test.py phase 1
PHASE_WARM_START = 120.0 / 360.0


def measure_phased(rig, k1, k2, phase, out_dir, label):
    import time
    n_s = int(EX.HW_RATE * rig.period)
    _, th1, _ = cycle(k1.to_params(), rig.period, n_s)
    _, th2, _ = cycle(k2.to_params(), rig.period, n_s)
    th2 = np.roll(th2, int(round(phase * n_s)))
    if (np.max(np.abs(th1)) > EX.PITCH_LIMIT or np.max(np.abs(th2)) > EX.HEAVE_LIMIT):
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


def run(rig, k1, k2, out_dir, label, history, phase=0.0):
    paths, err = measure_phased(rig, k1, k2, phase, out_dir, label)
    if paths is None:
        print(f"   {label}: {err}")
        return None
    m = SH.evaluate(paths, "Fx", None)
    if m is None:
        return None
    net = m.get("bias", 0.0)
    rec = {"label": label, "net_Fx": net, "A1": k1.A0, "f1": k1.n,
          "A2": k2.A0, "f2": k2.n, "phase": phase}
    history.append(rec)
    print(f"   {label:22s} A1={k1.A0:.3f} f1={k1.n}  A2={k2.A0:.3f} f2={k2.n}  "
          f"net_Fx={net:+.3f}")
    return rec


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S)
    ap.add_argument("--levels", type=int, default=5)
    ap.add_argument("--refine-evals", type=int, default=20)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    out_dir = os.path.join(folder, "data")
    os.makedirs(out_dir, exist_ok=True)

    rig = SH.Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop(); return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    history = []
    sweeps = {}
    try:
        print("--- PHASE 1: isolate A1 vs f1 (own), A1 vs f2 (cross)")
        for f1 in (1, 2):
            for A1 in np.linspace(*A_BOUNDS[1], a.levels):
                k1 = Knobs(A0=float(A1), n=f1)
                k2 = Knobs(A0=EX.NOMINAL[2].A0, n=1)
                r = run(rig, k1, k2, out_dir, f"A1_f1={f1}_{A1:.3f}", history)
                if r: sweeps.setdefault(("A1", "f1", f1), []).append(r)
        for f2 in (1, 2):
            for A1 in np.linspace(*A_BOUNDS[1], a.levels):
                k1 = Knobs(A0=float(A1), n=1)
                k2 = Knobs(A0=EX.NOMINAL[2].A0, n=f2)
                r = run(rig, k1, k2, out_dir, f"A1_f2={f2}_{A1:.3f}", history)
                if r: sweeps.setdefault(("A1", "f2", f2), []).append(r)

        print("\n--- PHASE 1: isolate A2 vs f2 (own), A2 vs f1 (cross)")
        for f2 in (1, 2):
            for A2 in np.linspace(*A_BOUNDS[2], a.levels):
                k1 = Knobs(A0=EX.NOMINAL[1].A0, n=1)
                k2 = Knobs(A0=float(A2), n=f2)
                r = run(rig, k1, k2, out_dir, f"A2_f2={f2}_{A2:.3f}", history)
                if r: sweeps.setdefault(("A2", "f2", f2), []).append(r)
        for f1 in (1, 2):
            for A2 in np.linspace(*A_BOUNDS[2], a.levels):
                k1 = Knobs(A0=EX.NOMINAL[1].A0, n=f1)
                k2 = Knobs(A0=float(A2), n=1)
                r = run(rig, k1, k2, out_dir, f"A2_f1={f1}_{A2:.3f}", history)
                if r: sweeps.setdefault(("A2", "f1", f1), []).append(r)

        # ---- PHASE 2: does the ratio predict better than raw amplitude?
        print("\n--- PHASE 2: raw amplitude vs A/f ratio, correlation with net thrust")
        combined = {}
        for (ax, fx, _), recs in sweeps.items():
            combined.setdefault((ax, fx), []).extend(recs)
        best_combo, best_net = None, -1e9
        for (ax, fx), recs in combined.items():
            A = np.array([r[ax] for r in recs])
            F = np.array([r[fx] for r in recs])
            net = np.array([r["net_Fx"] for r in recs])
            ratio = A / F
            c_raw = np.corrcoef(A, net)[0, 1] if len(set(F)) == 1 else np.nan
            c_ratio = np.corrcoef(ratio, net)[0, 1]
            print(f"   {ax} vs {fx}:  corr(raw {ax})={c_raw:+.3f}  "
                  f"corr(ratio {ax}/{fx})={c_ratio:+.3f}  "
                  f"{'RATIO WINS' if (not np.isnan(c_raw) and abs(c_ratio) > abs(c_raw)) else ''}")
            i = int(np.argmax(net))
            if net[i] > best_net:
                best_net = net[i]; best_combo = recs[i]
        print(f"\n   best single point from phase 1: {best_combo}")

        # ---- PHASE 3: refine A0 (+ heave correction + phase) within the
        # best (f1,f2) combination found
        print(f"\n--- PHASE 3: refine A0 within f1={best_combo['f1']}, "
              f"f2={best_combo['f2']}, {a.refine_evals} evals")
        f1b, f2b = best_combo["f1"], best_combo["f2"]
        k = {"A1": best_combo["A1"], "A2": best_combo["A2"]}
        step = {"A1": 0.2 * (A_BOUNDS[1][1] - A_BOUNDS[1][0]),
               "A2": 0.2 * (A_BOUNDS[2][1] - A_BOUNDS[2][0]), "phase": 0.15}
        phase = PHASE_WARM_START

        def score(kd, ph, tag):
            k1 = Knobs(A0=kd["A1"], n=f1b)
            k2 = Knobs(A0=kd["A2"], n=f2b, **HEAVE_CORRECTION)
            return run(rig, k1, k2, out_dir, tag, history, phase=ph)

        base = score(k, phase, "refine_start")
        if base is None:
            print("   refine start infeasible")
        else:
            best = (base["net_Fx"], dict(k), phase)
            ev = 0
            while ev < a.refine_evals and max(step.values()) > 0.01:
                improved = False
                for name in ("A1", "A2", "phase"):
                    if ev >= a.refine_evals: break
                    for sgn in (+1, -1):
                        if ev >= a.refine_evals: break
                        ev += 1
                        if name == "phase":
                            trial_ph = (best[2] + sgn * step["phase"]) % 1.0
                            r = score(best[1], trial_ph, f"ref{ev:02d}_phase")
                            if r and r["net_Fx"] > best[0]:
                                best = (r["net_Fx"], dict(best[1]), trial_ph); improved = True; break
                        else:
                            lo, hi = A_BOUNDS[1 if name == "A1" else 2]
                            trial = dict(best[1])
                            trial[name] = float(np.clip(trial[name] + sgn * step[name], lo, hi))
                            r = score(trial, best[2], f"ref{ev:02d}_{name}")
                            if r and r["net_Fx"] > best[0]:
                                best = (r["net_Fx"], trial, best[2]); improved = True; break
                if not improved:
                    for n in step: step[n] *= 0.5
            print(f"\n   PHASE 3 best: net_Fx {best[0]:+.3f}  A1={best[1]['A1']:.3f} "
                  f"A2={best[1]['A2']:.3f}  phase={best[2]:.2f}  (f1={f1b}, f2={f2b})")
    finally:
        rig.stop()

    if history:
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(history[0]))
            w.writeheader(); w.writerows(history)
        best = max(history, key=lambda r: r["net_Fx"])
        print(f"\nBEST OVERALL: {best}")
        json.dump(best, open(os.path.join(folder, "result.json"), "w"), indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
