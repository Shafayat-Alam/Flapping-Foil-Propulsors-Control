#!/usr/bin/env python3
"""Closed-loop SHAPE matching against a described target force curve.

Shape only. Magnitude is deliberately not optimised -- the target is
described in terms of peak count and trough depth RELATIVE to the peak, so
every controlled descriptor here is scale-free and the controller never
trades shape for newtons (the failure mode of the earlier magnitude-driven
runs).

WHAT IS CONTROLLED, AND WITH WHICH KNOB
---------------------------------------
  peak count      freq_ratio, snapped to a rational LOCK.
                  Only low-denominator ratios give a well-defined count:
                  pooled over 284 measured missions the locks yield
                  0.75-1.5 peaks per pitch cycle, while off-lock ratios
                  (denominator 14-20) give 0.05-0.30, because the gait
                  never actually repeats. So this knob is treated as
                  discrete -- snapped, never interpolated.

  trough depth    phase, the pitch-vs-heave offset.
                  This is the decoupling result: at the 3:4 lock, rotating
                  phase moved the +/- ratio from 0.41 to 2.46 (a factor of
                  6) while peaks-per-cycle stayed at 0.62-0.75. Rep-to-rep
                  scatter at those settings was 0.05-0.10, so the effect is
                  ~25x noise. Phase had been pinned at 0.000 rad for all
                  212 missions of the first three campaigns.

  Fy net         amp_ratio.
                  Fy is naturally near zero (median 5% of the Fx peak) and
                  amp_ratio trims it; it is 7.7x more selective for Fy than
                  for Fx, so it can be moved without disturbing the shape.

SEARCH STRATEGY. Phase is circular and its effect is a smooth rotation
(rising to a maximum near 90 deg and falling through a minimum near 270 at
the 3:4 lock), not a spike. A gradient step is therefore unreliable near the
turning points, where the local slope vanishes while the global optimum sits
elsewhere. Instead: one coarse rotation to locate the basin, then successive
local refinements around the best point. That is still closed-loop -- every
decision comes from load-cell measurement -- but it cannot be trapped by a
flat gradient, and it costs about 15 missions.

WHY NOT REUSE force_control.py's LOOP. That controller steers on
waveform_match, a scale-invariant Pearson correlation with no derivative
with respect to any parameter; during its MATCH stage it emitted an all-zero
signal vector and left the parameters frozen at the seed for an entire run.
Here every controlled quantity is a scalar descriptor with a knob attached.

usage: shape_match.py <folder> [--dry-run]
"""
import argparse
import csv
import json
import math
import os
import sys
import time
from fractions import Fraction

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))

import design_harmonic_sweep as dhs          # noqa: E402
import analyze_harmonic_sweep as aha         # noqa: E402

SETTLE_S = 4.0
IDLE_TAIL_S = 2.0
N_CYCLES = 4      # 4 commanded so a whole beat sits clear of both ramps


# --------------------------------------------------------------- targets
TARGETS = {
    "drag": {
        "description": "one positive peak per cycle that dies away, almost no trough",
        "peaks_per_cycle": 1.0,
        "trough_frac": 0.15,      # trough at most ~15% of the peak
        "fy_frac": 0.10,          # |Fy net| under 10% of the Fx peak
        # Seed from the best measured match among 284 missions:
        # P_fr0.667_ph135 gave 1.0 peak/cycle, trough_frac 0.58, Fy 0.5%.
        "seed": {"freq_ratio": 0.667, "amp_ratio": 3.0,
                 "freq_scale": 1.0, "phase_deg": 135.0},
        "lock_options": [0.667, 0.750, 0.500],
    },
    "lift": {
        "description": "two positive peaks per cycle, troughs well below the peaks",
        "peaks_per_cycle": 2.0,
        "trough_frac": 0.40,
        "fy_frac": 0.15,
        # Best measured 2-peak case was A_ar3.00_fs0.56 (freq_ratio 0.400,
        # 2.0 peaks/cycle) but with trough_frac 1.04 -- the trough is the
        # thing the phase search has to bring down.
        "seed": {"freq_ratio": 0.500, "amp_ratio": 3.0,
                 "freq_scale": 1.0, "phase_deg": 0.0},
        "lock_options": [0.500, 0.400, 0.750],
    },
}


def mission_line(p, label, n_cycles=N_CYCLES):
    pa, ha, pf, hf = dhs.kinematics(p["amp_ratio"], p["freq_ratio"], p["freq_scale"])
    return (f"forward_paddle frequency:{pf:.6f} pitch_amp:{pa:.6f} "
            f"heave_amp:{ha:.6f} phase:{math.radians(p['phase_deg']):.6f} "
            f"freq_ratio:{p['freq_ratio']:.6f} pitch_k:0.000000 "
            f"cycles:{n_cycles} label:{label}"), pf


def check(p):
    pa, ha, pf, hf = dhs.kinematics(p["amp_ratio"], p["freq_ratio"], p["freq_scale"])
    pv = dhs.peak_velocity(0, 0, 0, pa, pf)
    hv = dhs.peak_velocity(0, 0, 0, ha, hf)
    if pv > dhs.SLEW_LIMIT or hv > dhs.SLEW_LIMIT:
        return False, f"slew {pv:.2f}/{hv:.2f}"
    if pa > dhs.PITCH_LIMIT or ha > dhs.HEAVE_LIMIT:
        return False, f"amp {pa:.2f}/{ha:.2f}"
    return True, f"slew {pv:.2f}/{hv:.2f}"


def measure(node, p, label, data_dir, np_mod=np):
    """Command one mission and reduce it to shape descriptors."""
    line, pf = mission_line(p, label)
    time.sleep(SETTLE_S)
    timeout = (N_CYCLES / pf) * 1.6 + 8.0
    node.start_recording()
    ok = node.send(line, label, timeout=timeout)
    time.sleep(IDLE_TAIL_S)
    lc, _ = node.stop_recording()
    if not lc:
        return None
    lc.sort(key=lambda x: x[0])
    arr = np_mod.array([[x[0], x[1], x[2], x[3]] for x in lc], float)
    t = arr[:, 0] - arr[0, 0]
    path = os.path.join(data_dir, f"{label}.csv")
    np_mod.savetxt(path, np_mod.column_stack([t, arr[:, 1], arr[:, 2], arr[:, 3]]),
                   delimiter=",", header="t,Fx_raw,Fy_raw,Fz_raw", comments="")

    row = {"pitch_freq_hz": pf, "freq_ratio": p["freq_ratio"]}
    res = aha.analyse_mission(path, row)
    if not res:
        return None
    fx, fy = res["Fx"], res["Fy"]
    return {
        # peaks per PITCH cycle, normalised by the window actually analysed
        "peaks_per_cycle": fx["peaks_per_cycle"],
        "win_pitch_cycles": fx["win_pitch_cycles"],
        "trough_frac": fx["trough_frac"],
        "pos_peak": fx["pos_peak"], "neg_peak": fx["neg_peak"],
        "fy_frac": abs(fy["net"]) / max(abs(fx["pos_peak"]), 1e-9),
        "fy_net": fy["net"], "fz_net": res["Fz"]["net"],
        "csv": os.path.basename(path),
    }


def shape_error(m, tgt):
    """Scale-free shape error. Peak count is weighted hardest because lobe
    depth is not a meaningful thing to tune while the number of lobes is
    wrong -- the same precedence that stopped an earlier controller
    oscillating between two knobs fighting over one parameter."""
    e_pk = abs(m["peaks_per_cycle"] - tgt["peaks_per_cycle"])
    e_tr = max(0.0, m["trough_frac"] - tgt["trough_frac"])
    e_fy = max(0.0, m["fy_frac"] - tgt["fy_frac"])
    return 3.0 * e_pk + 2.0 * e_tr + 1.0 * e_fy, (e_pk, e_tr, e_fy)


def run(name, folder, node, log=print):
    tgt = TARGETS[name]
    data_dir = os.path.join(folder, "data")
    os.makedirs(data_dir, exist_ok=True)
    p = dict(tgt["seed"])
    history = []
    k = 0

    def trial(pp, tag):
        nonlocal k
        k += 1
        label = f"{name}_{k:03d}_{tag}"
        ok, why = check(pp)
        if not ok:
            log(f"  [{k:02d}] {tag:16s} SKIPPED ({why})")
            return None
        m = measure(node, pp, label, data_dir)
        if not m:
            log(f"  [{k:02d}] {tag:16s} no data")
            return None
        err, (epk, etr, efy) = shape_error(m, tgt)
        m.update({"params": dict(pp), "label": label, "err": err})
        history.append(m)
        log(f"  [{k:02d}] {tag:16s} fr={pp['freq_ratio']:.3f} ph={pp['phase_deg']:>5.0f} "
            f"ar={pp['amp_ratio']:.2f} | peaks {m['peaks_per_cycle']:.2f} "
            f"trough {m['trough_frac']:.2f} Fy {m['fy_frac']*100:.0f}% "
            f"| err {err:.3f}")
        return m

    log(f"\n{'='*78}\nSHAPE MATCH: {name} -- {tgt['description']}")
    log(f"target: {tgt['peaks_per_cycle']:.0f} peak(s)/cycle, "
        f"trough <= {tgt['trough_frac']:.2f} of peak, |Fy| <= {tgt['fy_frac']*100:.0f}% of peak")
    log(f"seed (best of 284 measured missions): {tgt['seed']}")
    log("=" * 78)

    # ---- STAGE 1: peak count. Discrete, so try each candidate lock once.
    log("\nSTAGE 1 -- peak count (freq_ratio locks)")
    best_lock, best_pk_err = None, 1e9
    for fr in tgt["lock_options"]:
        pp = dict(p, freq_ratio=fr)
        m = trial(pp, f"lock{fr:.3f}")
        if m is None:
            continue
        e = abs(m["peaks_per_cycle"] - tgt["peaks_per_cycle"])
        if e < best_pk_err:
            best_pk_err, best_lock = e, fr
    if best_lock is None:
        log("  no usable lock -- aborting")
        return history, None
    p["freq_ratio"] = best_lock
    log(f"  -> freq_ratio = {best_lock:.3f} (peak-count error {best_pk_err:.2f})")

    # ---- STAGE 2: coarse phase rotation to find the basin.
    log("\nSTAGE 2 -- phase rotation (trough depth)")
    best = None
    for ph in (0, 45, 90, 135, 180, 225, 270, 315):
        m = trial(dict(p, phase_deg=ph), f"phase{ph:03d}")
        if m and (best is None or m["err"] < best["err"]):
            best = m
    if best is None:
        return history, None
    p["phase_deg"] = best["params"]["phase_deg"]
    log(f"  -> best phase {p['phase_deg']:.0f} deg (err {best['err']:.3f})")

    # ---- STAGE 3: refine phase locally, halving the step.
    log("\nSTAGE 3 -- local phase refinement")
    for step in (20, 10):
        for d in (-step, step):
            m = trial(dict(p, phase_deg=(p["phase_deg"] + d) % 360), f"refine{d:+d}")
            if m and m["err"] < best["err"]:
                best = m
        p["phase_deg"] = best["params"]["phase_deg"]
        log(f"  -> phase {p['phase_deg']:.0f} deg (err {best['err']:.3f})")

    # ---- STAGE 4: Fy trim, only if it is actually out of tolerance.
    if best["fy_frac"] > tgt["fy_frac"]:
        log("\nSTAGE 4 -- Fy trim (amp_ratio)")
        for ar in (3.0, 1.5, 0.67):
            if abs(ar - p["amp_ratio"]) < 1e-6:
                continue
            m = trial(dict(p, amp_ratio=ar), f"fytrim{ar:.2f}")
            if m and m["err"] < best["err"]:
                best = m
        p["amp_ratio"] = best["params"]["amp_ratio"]
    else:
        log(f"\nSTAGE 4 -- skipped, Fy already within tolerance "
            f"({best['fy_frac']*100:.0f}% <= {tgt['fy_frac']*100:.0f}%)")

    log("\n" + "=" * 78)
    log(f"BEST {name}: {best['label']}")
    log(f"  params: freq_ratio={best['params']['freq_ratio']:.3f} "
        f"phase={best['params']['phase_deg']:.0f} deg "
        f"amp_ratio={best['params']['amp_ratio']:.2f} "
        f"freq_scale={best['params']['freq_scale']:.2f}")
    log(f"  shape:  {best['peaks_per_cycle']:.2f} peaks/cycle "
        f"(target {tgt['peaks_per_cycle']:.0f})")
    log(f"          trough {best['trough_frac']:.2f} of peak "
        f"(target <= {tgt['trough_frac']:.2f})")
    log(f"          +peak {best['pos_peak']:+.3f} N  -peak {best['neg_peak']:+.3f} N")
    log(f"          Fy net {best['fy_net']:+.3f} N = {best['fy_frac']*100:.0f}% of peak")
    log(f"          Fz net {best['fz_net']:+.3f} N (unconstrained)")
    log(f"  error:  {best['err']:.3f}  (from {history[0]['err']:.3f} at the seed)")
    log("=" * 78)
    return history, best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("target", choices=list(TARGETS))
    ap.add_argument("folder")
    args = ap.parse_args()
    folder = args.folder if os.path.isabs(args.folder) else \
        os.path.join(WORKSPACE_ROOT, args.folder)
    os.makedirs(folder, exist_ok=True)

    import soft_propulsors_control.motion_command as mc
    node = mc.start_hil_node()
    try:
        node.capture_rest_baseline()
        history, best = run(args.target, folder, node)
    finally:
        mc.stop_hil_node(node)

    with open(os.path.join(folder, "result.json"), "w") as fh:
        json.dump({"target": TARGETS[args.target], "best": best,
                   "history": history}, fh, indent=2, default=str)
    if history:
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            cols = ["label", "err", "peaks_per_cycle", "trough_frac",
                    "pos_peak", "neg_peak", "fy_net", "fy_frac", "fz_net", "csv"]
            w = csv.writer(fh)
            w.writerow(cols + ["freq_ratio", "phase_deg", "amp_ratio", "freq_scale"])
            for h in history:
                w.writerow([h.get(c) for c in cols] +
                           [h["params"][k] for k in
                            ("freq_ratio", "phase_deg", "amp_ratio", "freq_scale")])
    print(f"\nwrote {folder}/result.json and history.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
