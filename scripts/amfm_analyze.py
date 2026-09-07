#!/usr/bin/env python3
"""Analyse the AM/FM shaping campaign.

Produces, in order of what actually decides the outcome:

  1. NOISE FLOOR from the replicate missions. Every sensitivity below is
     judged against this, so "knob X moves metric Y" means "by more than the
     rig moves on its own".

  2. JACOBIAN  d(metric)/d(knob), by central difference from the +/- pairs.
     Rows are the 8 shape metrics on Fx and on Fy; columns are the knobs on
     each servo.

  3. INDEPENDENCE. The singular values of the (noise-normalised) Jacobian
     say how many metric directions are actually steerable. Counting knobs
     is not the same as counting controllable metrics: if two knobs move the
     same combination of metrics, they add a column but no rank.

  4. SELECTIVITY / COUPLING. For each knob: which metric it moves most, and
     what else it disturbs on the way -- including the OTHER force channel,
     which is the X/Y decoupling question.

  5. SWEEP CURVES for the stage-S knobs, so a non-monotone response (an
     effect that reverses past some level) is visible rather than hidden
     inside a single slope.

Force metrics come from the measured load cell; kinematic metrics from the
measured encoder trace. Both are reported, because a knob that fails to move
a force metric has two very different explanations -- the motion did not
change, or it changed and the fluid did not care -- and only the kinematic
column separates them.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np
from scipy.signal import butter, filtfilt

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
from amfm_metrics import metrics, METRIC_NAMES   # noqa: E402
from amfm_waveform import KNOB_NAMES             # noqa: E402

PERIOD_S = 2.0
N_CYCLES = 3          # must match amfm_experiment.N_CYCLES
ANALYSE_CYCLE = 1     # 0-based: the SECOND commanded cycle
N_AXES = 6
CUTOFF_HZ = 12.0        # low-pass; the ~29-30 Hz structural resonance is not force


def load_force(path, period_s=PERIOD_S):
    """Rebuild (t, Fx, Fy, Fz) from the packet log and return ONE steady cycle.

    The load cell sends batched packets; each row here is one packet with its
    arrival time and a flat samples x 6 payload. Sample times are
    reconstructed inside the packet from the sensor rate rather than the
    arrival time, which is quantised to the packet interval.
    """
    rows = list(csv.reader(open(path)))[1:]
    if not rows:
        return None
    t_all, F = [], []
    for wall, payload in rows:
        vals = [float(x) for x in payload.split()]
        n = len(vals) // N_AXES
        if n == 0:
            continue
        w = float(wall)
        for s in range(n):
            F.append(vals[s * N_AXES:s * N_AXES + 3])
        # spread samples backwards from arrival; packets are uniform in time
        t_all.extend(list(np.linspace(w - n / 10000.0, w, n, endpoint=False)))
    if len(F) < 100:
        return None
    t = np.asarray(t_all)
    F = np.asarray(F)
    order = np.argsort(t)
    t, F = t[order], F[order]

    fs = (len(t) - 1) / max(t[-1] - t[0], 1e-9)
    if fs > 3 * CUTOFF_HZ:
        b, a = butter(4, CUTOFF_HZ / (fs / 2.0))
        F = np.column_stack([filtfilt(b, a, F[:, i]) for i in range(3)])

    # Tare on the PRE-MOTION quiet window. Times in this file are relative to
    # the instant motion starts, so everything at t < 0 was recorded with the
    # fin held still in settled water -- the only stretch that is a true zero.
    # The post-motion tail is the fallback, not the default: the fin has just
    # stopped there and its wake is still loading the cell, so taring on it
    # subtracts part of the signal.
    pre = F[t < -0.2]
    if len(pre) > 500:
        base = pre.mean(axis=0)
    else:
        tail = F[t > t[-1] - 1.2]
        base = tail.mean(axis=0) if len(tail) > 50 else F[:200].mean(axis=0)
    F = F - base

    # Analyse the SECOND commanded cycle. Cycle 1 carries the start-up
    # transient (the joint is eased in from wherever it was parked) and the
    # last cycle can catch the ramp-down, so the middle one is the only
    # window unambiguously at the commanded parameters. Anchored to the
    # motion START rather than its midpoint, so the window is the same
    # commanded cycle in every mission regardless of capture length.
    # t == 0 IS the motion start (the runner writes motion-relative times), so
    # the window is anchored absolutely rather than to the first sample --
    # which now sits in the pre-motion baseline and would shift every window.
    t0 = ANALYSE_CYCLE * period_s
    sl = (t >= t0) & (t < t0 + period_s)
    if np.count_nonzero(sl) < 100:                 # capture shorter than hoped
        mid = 0.5 * (t[0] + t[-1])
        sl = (t >= mid - period_s / 2) & (t < mid + period_s / 2)
    if np.count_nonzero(sl) < 100:
        return None
    return t[sl], F[sl, 0], F[sl, 1], F[sl, 2]


def load_kin(path, period_s=PERIOD_S):
    """Same second-cycle window as the force, so the two are directly
    comparable -- a kinematic change and the force it produced must be
    measured over the same commanded cycle."""
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"]
    t0 = ANALYSE_CYCLE * period_s          # motion-relative, as in load_force
    sl = (t >= t0) & (t < t0 + period_s)
    if np.count_nonzero(sl) < 50:
        mid = 0.5 * (t[0] + t[-1])
        sl = (t >= mid - period_s / 2) & (t < mid + period_s / 2)
    if np.count_nonzero(sl) < 50:
        return None
    return (t[sl], d["cmd1"][sl], d["cmd2"][sl], d["meas1"][sl], d["meas2"][sl])


def mission_metrics(folder, row):
    out = {}
    fp = os.path.join(folder, "data", row["force_csv"])
    if os.path.exists(fp):
        r = load_force(fp)
        if r:
            _, fx, fy, fz = r
            out.update(metrics(fx, "Fx"))
            out.update(metrics(fy, "Fy"))
            out["Fz_bias"] = float(np.mean(fz))
    kp = os.path.join(folder, "data", row["kin_csv"])
    if os.path.exists(kp):
        r = load_kin(kp)
        if r:
            _, c1, c2, m1, m2 = r
            out.update(metrics(m1, "k1"))
            out.update(metrics(m2, "k2"))
            out["k1_track_rms"] = float(np.sqrt(np.nanmean((m1 - c1) ** 2)))
            out["k2_track_rms"] = float(np.sqrt(np.nanmean((m2 - c2) ** 2)))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)

    man = list(csv.DictReader(open(os.path.join(folder, "manifest.csv"))))
    recs = []
    for r in man:
        m = mission_metrics(folder, r)
        if m:
            recs.append({**r, **m})
    if not recs:
        print("no analysable missions")
        return 1
    import pandas as pd
    df = pd.DataFrame(recs)
    df.to_csv(os.path.join(folder, "metrics.csv"), index=False)
    print(f"{len(df)} missions analysed -> metrics.csv\n")

    force_cols = [f"{ch}_{m}" for ch in ("Fx", "Fy") for m in METRIC_NAMES
                  if f"{ch}_{m}" in df.columns]
    kin_cols = [f"k{s}_{m}" for s in (1, 2) for m in METRIC_NAMES
                if f"k{s}_{m}" in df.columns]

    # ---------------- 1. noise floor
    rep = df[df.stage == "R"]
    print("=" * 78)
    print(f"1. NOISE FLOOR  ({len(rep)} replicates of the nominal)")
    print("=" * 78)
    noise = {}
    for c in force_cols:
        sd = float(rep[c].std()) if len(rep) > 1 else 0.0
        if not np.isfinite(sd):
            sd = 0.0
        # FLOOR the noise estimate. Replicates can land identical (a perfectly
        # repeatable rig, or too few of them), and dividing a sensitivity by a
        # zero spread produces meaningless 1e10 "significance" that swamps the
        # singular values and makes every knob look decisive. The floor is the
        # larger of a small fraction of the metric's own scale and an absolute
        # minimum, so "moves by N noise units" stays finite and comparable.
        scale = float(np.nanmax(np.abs(df[c]))) if c in df else 1.0
        floor = max(1e-3 * (scale if np.isfinite(scale) else 1.0), 1e-6)
        noise[c] = max(sd, floor)
        flag = "" if sd > floor else "  (floored)"
        print(f"   {c:22s} mean {rep[c].mean():+9.4f}   sd {sd:8.4f}"
              f"   used {noise[c]:8.4f}{flag}")
    print()

    # ---------------- 2. Jacobian
    J = df[df.stage == "J"]
    print("=" * 78)
    print("2. JACOBIAN  d(metric)/d(knob), central difference")
    print("=" * 78)
    jac, cols = {}, []
    # Nominal baseline, for knobs that have no +/- pair. `n` is an integer
    # (1 crest vs 2), so it is perturbed in one direction only; without this
    # it was silently dropped from the Jacobian entirely -- the peak-count
    # knob, one of the metrics the campaign exists to test, had no column.
    nominal = rep[force_cols + kin_cols].mean() if len(rep) else None
    for (servo, knob), g in J.groupby(["servo", "knob"]):
        g = g.sort_values("value")
        if len(g) < 2:
            if nominal is None:
                continue
            hi = g.iloc[0]
            dv = float(hi["value"]) - (1.0 if knob == "n" else 0.0)
            if abs(dv) < 1e-9:
                continue
            col = f"s{servo}.{knob}"
            cols.append(col)
            for c in force_cols + kin_cols:
                jac[(c, col)] = (float(hi[c]) - float(nominal[c])) / dv
            continue
        lo, hi = g.iloc[0], g.iloc[-1]
        dv = float(hi["value"]) - float(lo["value"])
        if abs(dv) < 1e-9:
            continue
        col = f"s{servo}.{knob}"
        cols.append(col)
        for c in force_cols + kin_cols:
            jac[(c, col)] = (float(hi[c]) - float(lo[c])) / dv
    hdr = f"{'metric':22s}" + "".join(f"{c:>13s}" for c in cols)
    print(hdr); print("-" * len(hdr))
    for c in force_cols:
        print(f"{c:22s}" + "".join(f"{jac.get((c,k),float('nan')):13.3f}" for k in cols))
    print()

    # ---------------- 3. independence
    M = np.array([[jac.get((c, k), 0.0) for k in cols] for c in force_cols])
    scale = np.array([max(noise.get(c, 1.0), 1e-9) for c in force_cols])[:, None]
    Mn = M / np.where(scale > 0, scale, 1.0)      # metric changes in units of noise
    sv = np.linalg.svd(Mn, compute_uv=False) if Mn.size else np.array([])
    print("=" * 78)
    print("3. INDEPENDENCE  (singular values of the noise-normalised Jacobian)")
    print("=" * 78)
    if sv.size:
        tot = sv.sum()
        for i, s in enumerate(sv):
            bar = "#" * int(40 * s / max(sv[0], 1e-12))
            print(f"   sigma_{i+1:<2d} {s:9.2f}  ({100*s/tot:4.1f}%)  {bar}")
        rank = int(np.sum(sv > max(sv[0] * 0.05, 1.0)))
        print(f"\n   -> ~{rank} independently steerable force-metric directions "
              f"out of {len(force_cols)} metrics and {len(cols)} knobs")
    print()

    # ---------------- 4. selectivity
    print("=" * 78)
    print("4. SELECTIVITY  (per knob: strongest effect, and the cross-channel cost)")
    print("=" * 78)
    print(f"{'knob':14s}{'moves most':22s}{'/noise':>8}   {'Fx-side':>9}{'Fy-side':>9}  verdict")
    print("-" * 78)
    for k in cols:
        eff = {c: abs(jac.get((c, k), 0.0)) / max(noise.get(c, 1e-9), 1e-9)
               for c in force_cols}
        if not eff:
            continue
        best = max(eff, key=eff.get)
        fx = max((v for c, v in eff.items() if c.startswith("Fx")), default=0.0)
        fy = max((v for c, v in eff.items() if c.startswith("Fy")), default=0.0)
        sel = ("Fx-selective" if fx > 3 * fy else
               "Fy-selective" if fy > 3 * fx else
               "couples both" if max(fx, fy) > 3 else "no effect")
        print(f"{k:14s}{best:22s}{eff[best]:8.1f}   {fx:9.1f}{fy:9.1f}  {sel}")
    print()

    # ---------------- 5. sweep curves
    S = df[df.stage == "S"]
    if len(S):
        print("=" * 78)
        print("5. SWEEP CURVES  (is the response monotone?)")
        print("=" * 78)
        for (servo, knob), g in S.groupby(["servo", "knob"]):
            # sort NUMERICALLY. 'value' arrives from the CSV as text, so a
            # plain sort orders the levels lexicographically -- '-0.225'
            # before '-0.450' -- and the monotonicity test below then runs on
            # a shuffled sequence and calls almost everything non-monotone.
            g = g.assign(_v=g["value"].astype(float)).sort_values("_v")
            tgt = max(force_cols,
                      key=lambda c: abs(jac.get((c, f"s{servo}.{knob}"), 0.0))
                      / max(noise.get(c, 1e-9), 1e-9))
            vals = g[tgt].to_numpy()
            # Judge reversals against the NOISE FLOOR, not against zero: a dip
            # smaller than the rig's own spread is not evidence of a turning
            # point, and flagging it as one would make a usable knob look
            # unusable to the optimiser.
            tol = noise.get(tgt, 0.0)
            mono = (np.all(np.diff(vals) >= -tol) or np.all(np.diff(vals) <= tol))
            pts = "  ".join(f"{v:+.3f}@{x:+.2f}"
                            for x, v in zip(g["_v"], vals))
            print(f"  s{servo}.{knob:9s} -> {tgt:18s} "
                  f"{'monotone' if mono else 'NON-MONOTONE'}")
            print(f"      {pts}")
        print()

    json.dump({"cols": cols, "force_metrics": force_cols,
               "jacobian": {f"{c}|{k}": v for (c, k), v in jac.items()},
               "noise": noise,
               "singular_values": sv.tolist() if sv.size else []},
              open(os.path.join(folder, "jacobian.json"), "w"), indent=2)
    print(f"wrote {folder}/jacobian.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
