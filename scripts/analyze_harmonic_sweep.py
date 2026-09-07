#!/usr/bin/env python3
"""Analyse the harmonic sweep: which kinematic knob moves which force feature.

Reads the raw per-mission captures written by run_harmonic_sweep.py and
reduces each to the shape descriptors under test, then reports each block
against the claim it was designed to test.

Processing per mission, in this order and for this reason:
  1. Find the motion window by rolling standard deviation. Every capture
     has leading dead-time (the mission_input round-trip) and a deliberate
     idle tail, and slicing a "cycle" without locating the motion first is
     what made the first pass of the 3D re-analysis report a 0.02 N
     peak-to-peak and a cycle mean equal to the untared rest offset.
  2. Tare on the idle tail of that same capture. Contemporaneous with the
     motion, so no drift can enter between baseline and measurement.
  3. Low-pass at 10x the pitch frequency to remove the ~29-30 Hz structural
     resonance, which is mount ringing rather than hydrodynamic force and
     otherwise lands directly in every peak and width estimate.
  4. Analyse the SECOND of the three commanded cycles -- cycle 1 holds the
     start-up transient, cycle 3 can catch the ramp-down.

The cycle used is the BEAT period (b pitch cycles when freq_ratio = a/b in
lowest terms), not 1/min(f_pitch, f_heave): at freq_ratio 0.4 the gait only
repeats after 5 pitch cycles, and the naive period puts 2.5 pitch cycles in
the window, reporting several force peaks where the gait produces one.
"""
import argparse
import csv
import math
import os
from fractions import Fraction

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
CUTOFF_MULT = 10.0


def motion_window(y, fs, quiet_mult=4.0):
    w = max(16, int(0.1 * fs))
    n = len(y) // w
    if n < 4:
        return 0, len(y)
    seg = y[:n * w].reshape(n, w).std(axis=1)
    quiet = np.median(np.sort(seg)[:max(2, n // 5)])
    mov = np.where(seg > max(quiet * quiet_mult, quiet + 1e-6))[0]
    if len(mov) < 2:
        return 0, len(y)
    return int(mov[0] * w), int(min(len(y), (mov[-1] + 1) * w))


def lowpass(y, cutoff, fs):
    wn = cutoff / (fs / 2.0)
    if not (1e-3 < wn < 0.99):
        return y
    try:
        b, a = butter(4, wn)
        return filtfilt(b, a, y)
    except Exception:
        return y


def descriptors(F):
    n = len(F)
    p, q = float(F.max()), float(F.min())
    ip, iq = int(np.argmax(F)), int(np.argmin(F))
    span = max(p - q, 1e-9)
    tiled = np.concatenate([F, F, F])
    pk, _ = find_peaks(tiled, prominence=0.2 * span, distance=max(1, int(0.07 * n)))
    npos = len([x for x in pk if n <= x < 2 * n])
    nk, _ = find_peaks(-tiled, prominence=0.2 * span, distance=max(1, int(0.07 * n)))
    nneg = len([x for x in nk if n <= x < 2 * n])
    return {
        "pos_peak": p, "neg_peak": q, "p2p": p - q, "net": float(F.mean()),
        # ratio of the lobes, and the bounded trough measure. Both are
        # reported because ratio is the intuitive one but diverges when the
        # force never crosses zero, which is exactly the target case.
        "ratio": p / abs(q) if abs(q) > 1e-6 else float("inf"),
        "trough_frac": max(0.0, -q) / max(p, 1e-6),
        "skew": ((ip - iq) % n) / n,
        "pos_width": float(np.count_nonzero(F >= 0.5 * p)) / n if p > 0 else 0.0,
        "neg_width": float(np.count_nonzero(F <= 0.5 * q)) / n if q < 0 else 0.0,
        "n_pos": npos, "n_neg": nneg,
    }


def analyse_mission(path, row):
    d = np.genfromtxt(path, delimiter=",", names=True)
    t = d["t"]
    if len(t) < 2000:
        return None
    fs = (len(t) - 1) / (t[-1] - t[0])
    pitch_f = float(row["pitch_freq_hz"])
    fr = float(row["freq_ratio"])
    beat = Fraction(fr).limit_denominator(20).denominator / pitch_f

    i0, i1 = motion_window(d["Fx_raw"], fs)
    out = {}
    for ch in ("Fx", "Fy", "Fz"):
        raw = d[f"{ch}_raw"].astype(float)
        tail = raw[i1:]
        base = float(np.mean(tail[len(tail) // 5:])) if len(tail) > int(0.4 * fs) \
            else float(np.mean(raw[:int(0.2 * fs)]))
        y = lowpass(raw - base, CUTOFF_MULT * pitch_f, fs)
        # Analysis window: the second beat period, but CLAMPED INSIDE the
        # motion window. The mission commands a fixed number of PITCH
        # cycles while the gait repeats only every b of them, so whenever
        # b exceeds the commanded count the "second beat" starts after the
        # servos have already stopped. Unclamped, that slice lands in the
        # recorded idle tail and returns a flat near-zero trace, which then
        # reads as a dead mission: freq_ratio 0.250 and 0.750 (both b=4,
        # against 3 commanded cycles) came back 6/6 "dead" purely from this,
        # and were nearly written off as a rig failure.
        # Analysis window: one beat period CENTRED in the motion.
        #
        # Anchoring to either end of the motion has now produced three
        # separate false results, because the detected motion window is
        # wider than the commanded gait -- typically 4.1-4.4 pitch cycles
        # for 3 commanded, since the rolling-std detector necessarily
        # includes ramp-up and ramp-down. Windows anchored at the start ran
        # off the end into the idle tail (freq_ratio 0.250/0.750 read as
        # 6/6 dead); windows anchored at the end sat in the decaying final
        # cycle, where the negative lobe collapses as the servo stops and
        # the +/- ratio blows up (freq_ratio 0.500 read as 4.41, and phase
        # 315 as 19.46, against true steady values near 1.0).
        #
        # The centre is the only part of the capture that is unambiguously
        # steady at the commanded parameters, so the window is placed there
        # and shrunk to fit rather than allowed to reach either edge.
        motion_t0, motion_t1 = t[i0], t[min(i1, len(t) - 1)]
        span = motion_t1 - motion_t0
        mid = 0.5 * (motion_t0 + motion_t1)
        win = min(beat, 0.6 * span)      # never more than the middle 60%
        t0 = mid - 0.5 * win
        sl = (t >= t0) & (t < t0 + win)
        if np.count_nonzero(sl) < 100:
            sl = slice(i0, i1)
        out[ch] = descriptors(y[sl])
        # Length of the window ACTUALLY analysed, in pitch cycles. Peak
        # counts must be divided by this, never by the beat denominator:
        # the window is capped at 60% of the motion, so whenever the beat
        # is longer than that the window holds fewer pitch cycles than the
        # beat contains. Normalising by the denominator instead reported
        # 1.00 peaks/cycle for a trace that visibly had 1.63 -- the plot
        # disagreed with the number, and the plot was right.
        tsl = t[sl] if not isinstance(sl, slice) else t[sl]
        out[ch]["win_pitch_cycles"] = float((tsl[-1] - tsl[0]) * pitch_f) if len(tsl) > 1 else 1.0
        out[ch]["peaks_per_cycle"] = (out[ch]["n_pos"] /
                                      max(out[ch]["win_pitch_cycles"], 1e-9))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    args = ap.parse_args()
    folder = args.folder if os.path.isabs(args.folder) else \
        os.path.join(WORKSPACE_ROOT, args.folder)

    plan = {r["label"]: r for r in csv.DictReader(
        open(os.path.join(folder, "sweep_plan.csv")))}
    data_dir = os.path.join(folder, "data")

    rows = []
    for label, r in plan.items():
        p = os.path.join(data_dir, f"{label}.csv")
        if not os.path.exists(p):
            continue
        try:
            res = analyse_mission(p, r)
        except Exception as e:
            print(f"  ({label}: {e})")
            continue
        if not res:
            continue
        rec = {"label": label, "block": r["block"],
               "amp_ratio": float(r["amp_ratio"]), "freq_ratio": float(r["freq_ratio"]),
               "freq_scale": float(r["freq_scale"]),
               "p_a2": float(r["p_a2"]), "p_phi2": float(r["p_phi2"]),
               "p_a3": float(r["p_a3"]), "h_a2": float(r["h_a2"]),
               "h_phi2": float(r["h_phi2"]), "bias_frac": float(r.get("bias_frac", 0) or 0)}
        for ch, dd in res.items():
            for k, v in dd.items():
                rec[f"{ch.lower()}_{k}"] = v
        rows.append(rec)

    if not rows:
        print("no analysable missions yet")
        return 0

    import pandas as pd
    df = pd.DataFrame(rows)
    out = os.path.join(folder, "descriptors.csv")
    df.to_csv(out, index=False)
    print(f"{len(df)} missions analysed -> {out}\n")

    def block(name):
        return df[df.block == name].copy()

    # ---- R: the noise floor every other claim is judged against
    r = block("R_replicate")
    if len(r) >= 2:
        print("=" * 74)
        print(f"R  NOISE FLOOR from {len(r)} baseline replicates")
        print("=" * 74)
        for k in ("fx_pos_peak", "fx_trough_frac", "fx_pos_width", "fx_skew"):
            if k in r:
                print(f"   {k:16s} mean {r[k].mean():+.3f}  sd {r[k].std():.3f}  "
                      f"spread {r[k].max()-r[k].min():.3f}")
        print()

    # ---- C: a3 should move WIDTH and nothing else
    c = block("C_width").sort_values("p_a3")
    if len(c):
        print("=" * 74)
        print("C  a3 (odd harmonic)  ->  claim: WIDTH ONLY, never asymmetry")
        print("=" * 74)
        print(f"{'a3':>6} {'+peak':>8} {'-peak':>8} {'ratio':>7} {'+width':>8} {'skew':>7} {'npos':>5}")
        for _, x in c.iterrows():
            print(f"{x.p_a3:>6.3f} {x.fx_pos_peak:>8.3f} {x.fx_neg_peak:>8.3f} "
                  f"{x.fx_ratio:>7.2f} {x.fx_pos_width:>8.3f} {x.fx_skew:>7.3f} {x.fx_n_pos:>5.0f}")
        print()

    # ---- D/E: a2 magnitude and phi2 rotation
    for blk, ch, knob in (("D_Fx_shape", "fx", "p_a2"), ("E_Fy_shape", "fy", "h_a2")):
        b = block(blk)
        if not len(b):
            continue
        phic = "p_phi2" if knob == "p_a2" else "h_phi2"
        print("=" * 74)
        print(f"{blk[0]}  {knob}/{phic} -> claim: a2 sets HOW MUCH asymmetry, "
              f"phi2 sets WHICH lobe  [{ch.upper()}]")
        print("=" * 74)
        print(f"{'a2':>6} {'phi2':>6} {'+peak':>8} {'-peak':>8} {'ratio':>7} "
              f"{'trough':>7} {'skew':>7} {'npos':>5}")
        for _, x in b.sort_values([knob, phic]).iterrows():
            print(f"{x[knob]:>6.2f} {x[phic]:>6.0f} {x[f'{ch}_pos_peak']:>8.3f} "
                  f"{x[f'{ch}_neg_peak']:>8.3f} {x[f'{ch}_ratio']:>7.2f} "
                  f"{x[f'{ch}_trough_frac']:>7.3f} {x[f'{ch}_skew']:>7.3f} "
                  f"{x[f'{ch}_n_pos']:>5.0f}")
        print()

    # ---- H: bias, the remaining route to skew
    h = block("H_bias")
    if len(h):
        print("=" * 74)
        print("H  pitch_bias -> claim: angle-of-attack offset makes the strokes unequal")
        print("=" * 74)
        print(f"{'a2':>5} {'bias':>6} {'+peak':>8} {'-peak':>8} {'ratio':>7} "
              f"{'trough':>7} {'skew':>7} {'+width':>7}")
        for _, x in h.sort_values(["p_a2", "bias_frac"]).iterrows():
            print(f"{x.p_a2:>5.1f} {x.bias_frac:>6.1f} {x.fx_pos_peak:>8.3f} "
                  f"{x.fx_neg_peak:>8.3f} {x.fx_ratio:>7.2f} {x.fx_trough_frac:>7.3f} "
                  f"{x.fx_skew:>7.3f} {x.fx_pos_width:>7.3f}")
        print()

    # ---- B: peak count
    b = block("B_peakcount").sort_values("freq_ratio")
    if len(b):
        print("=" * 74)
        print("B  freq_ratio -> claim: sets PEAK COUNT")
        print("=" * 74)
        print(f"{'freq_ratio':>11} {'npos':>6} {'nneg':>6} {'+peak':>9}")
        for _, x in b.iterrows():
            print(f"{x.freq_ratio:>11.3f} {x.fx_n_pos:>6.0f} {x.fx_n_neg:>6.0f} "
                  f"{x.fx_pos_peak:>9.3f}")
        print()

    # ---- F: can Fy be held near zero while Fx is shaped?
    f = block("F_Fx_null_Fy").sort_values(["p_a2", "amp_ratio"])
    if len(f):
        print("=" * 74)
        print("F  amp_ratio as Fy trim while a2 shapes Fx")
        print("=" * 74)
        print(f"{'a2':>5} {'amp_ratio':>10} {'Fx +peak':>9} {'Fx ratio':>9} "
              f"{'Fy net':>8} {'|Fy net| / Fx peak':>19}")
        for _, x in f.iterrows():
            frac = abs(x.fy_net) / max(abs(x.fx_pos_peak), 1e-9)
            print(f"{x.p_a2:>5.1f} {x.amp_ratio:>10.2f} {x.fx_pos_peak:>9.3f} "
                  f"{x.fx_ratio:>9.2f} {x.fy_net:>+8.3f} {frac*100:>18.0f}%")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
