#!/usr/bin/env python3
"""
analyze_for_control.py — derive force_control.py's control model from the
full in_house_wet_test_3D sweep, in the SAME descriptor definitions
force_control.py itself measures at runtime.

Two things this does that the earlier ad-hoc analysis did not:

  1. Samples the FULL 3D grid (freq_ratio x amp_ratio x phase), not just
     the three 1D slices through the baseline. The 1D slices can't see
     interactions, and the controller moves through the interior of the
     box, not along its axes.

  2. Reports descriptors against BOTH the ratio parameters AND the actual
     commanded physical values (pitch/heave amplitude in rad, pitch/heave
     frequency in Hz) read from each mission's own cmd.* columns -- so the
     model is anchored to what the servos physically did, not to a ratio
     convention that only makes sense relative to a particular center.

    python3 scripts/analyze_for_control.py <split_data_root> [phase_step_deg]
"""
import csv, glob, math, os, sys
import numpy as np
from scipy.stats import skew as scipy_skew

STEP = 20   # loadcell row downsample; 10kHz raw -> 500Hz, plenty for <2Hz motion


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def load_mission(md):
    """(cmd dict, t, Fx, Fy, Fz) for one mission folder, tared to its own
    post-gait rest tail (same convention as prove_claims.load_raw)."""
    lc = glob.glob(os.path.join(md, "*_loadcell.csv"))
    fb = [f for f in glob.glob(os.path.join(md, "*.csv")) if "_loadcell" not in f]
    if not lc or not fb:
        return None
    r0 = next(csv.DictReader(open(fb[0])), {})
    f0 = _num(r0.get("cmd.frequency"))
    if not f0:
        return None
    cmd = {
        "pitch_amp": _num(r0.get("cmd.pitch_amp")) or 0.0,
        "heave_amp": _num(r0.get("cmd.heave_amp")) or 0.0,
        "pitch_freq": f0,
        "freq_ratio_stored": _num(r0.get("cmd.freq_ratio")) or 1.0,
        "phase": _num(r0.get("cmd.phase")) or 0.0,
        "cycles": _num(r0.get("cmd.cycles")) or 4.0,
    }
    cmd["heave_freq"] = cmd["pitch_freq"] * cmd["freq_ratio_stored"]
    gait_end = cmd["cycles"] / f0

    t, F = [], {"Fx": [], "Fy": [], "Fz": []}
    with open(lc[0]) as fh:
        for i, row in enumerate(csv.DictReader(fh)):
            if i % STEP:
                continue
            tv = _num(row.get("time_s"))
            if tv is None:
                continue
            vals = {a: _num(row.get(a)) for a in F}
            if any(v is None for v in vals.values()):
                continue
            t.append(tv)
            for a in F:
                F[a].append(vals[a])
    if len(t) < 40:
        return None
    t = np.asarray(t)
    rest = t > gait_end + 1.0
    out = {}
    for a in F:
        x = np.asarray(F[a])
        if rest.sum() > 10:
            x = x - np.median(x[rest])
        out[a] = x
    return cmd, t, out["Fx"], out["Fy"], out["Fz"]


def cycle2(t, x, pitch_freq):
    period = 1.0 / pitch_freq
    m = (t >= period) & (t < 2 * period)
    return t[m] - period, x[m]


def descriptors(t, fx, pitch_freq):
    """force_control.py's own descriptor set, same definitions."""
    tc, xc = cycle2(t, fx, pitch_freq)
    if len(xc) < 20:
        return None
    p2p = float(np.max(xc) - np.min(xc))
    trough = float(np.min(xc))
    sk = float(scipy_skew(xc)) if np.std(xc) > 1e-6 else 0.0
    mean_fx = float(np.mean(xc))

    # peak count: same filtered method as force_control.count_peaks
    n = len(xc)
    pad = max(1, n // 6)
    xp = np.concatenate([xc[-pad:], xc, xc[:pad]])
    dt = tc[1] - tc[0] if n > 1 else 1.0
    sr = 1.0 / dt if dt > 1e-9 else 1.0
    if sr > 8 * pitch_freq:
        from scipy.signal import butter, filtfilt
        wn = min(4.0 * pitch_freq / (sr / 2.0), 0.99)
        b, a = butter(4, wn, btype="low")
        xs = filtfilt(b, a, xp)
    else:
        w = max(3, len(xp) // 15)
        xs = np.convolve(xp, np.ones(w) / w, mode="same")
    lo, hi = xs.min(), xs.max()
    rng = hi - lo
    npk = 0
    if rng > 1e-9:
        thr = hi - 0.2 * rng
        cand = [i for i in range(1, len(xs) - 1)
                if xs[i] > xs[i - 1] and xs[i] >= xs[i + 1] and xs[i] >= thr]
        core = [i for i in cand if pad <= i < pad + n]
        mind = max(1, int(0.07 * n))
        core.sort(key=lambda i: -xs[i])
        kept = []
        for i in core:
            if all(abs(i - j) >= mind for j in kept):
                kept.append(i)
        npk = len(kept)
    return {"peak_height": p2p, "trough_min": trough, "skew": sk,
            "peak_count": npk, "mean_fx": mean_fx}


def parse_block(name):
    if name == "k1_pf0p500_r1p000":
        return 1.0, 1.0
    import re
    m = re.match(r"^k1_fr(\d+)p(\d+)_ar(\d+)p(\d+)$", name)
    if not m:
        return None, None
    return float(f"{m.group(1)}.{m.group(2)}"), float(f"{m.group(3)}.{m.group(4)}")


def main(root, phase_step=45):
    blocks = sorted(d for d in glob.glob(os.path.join(root, "k1_*")) if os.path.isdir(d))
    want_phases = set(range(0, 360, phase_step))
    rows = []
    for bi, bd in enumerate(blocks):
        fr_stored, ar = parse_block(os.path.basename(bd))
        if fr_stored is None:
            continue
        for md in sorted(glob.glob(os.path.join(bd, "PH_*"))):
            base = os.path.basename(md)
            try:
                ph_deg = int(base.split("_")[1])
            except (IndexError, ValueError):
                continue
            if ph_deg % 360 not in want_phases:
                continue
            got = load_mission(md)
            if got is None:
                continue
            cmd, t, fx, fy, fz = got
            d = descriptors(t, fx, cmd["pitch_freq"])
            if d is None:
                continue
            dy = descriptors(t, fy, cmd["pitch_freq"])
            rows.append({
                # force_control's own parameter convention
                "amp_ratio": ar,
                "freq_ratio_fc": 1.0 / fr_stored,          # pitch/heave
                "delta_phi": -cmd["phase"],                 # phi1(=0) - phi2
                # actual physical commands
                "pitch_amp_rad": cmd["pitch_amp"],
                "heave_amp_rad": cmd["heave_amp"],
                "pitch_freq_hz": cmd["pitch_freq"],
                "heave_freq_hz": cmd["heave_freq"],
                "pitch_tipspeed": 2 * math.pi * cmd["pitch_freq"] * cmd["pitch_amp"],
                "heave_tipspeed": 2 * math.pi * cmd["heave_freq"] * cmd["heave_amp"],
                **d,
                "fy_p2p": dy["peak_height"] if dy else float("nan"),
            })
        print(f"  [{bi+1}/{len(blocks)}] {os.path.basename(bd)}: {len(rows)} rows so far",
              flush=True)

    if not rows:
        print("no data")
        return

    keys = list(rows[0].keys())
    out_csv = os.path.join(os.path.dirname(root), "control_model_data.csv")
    with open(out_csv, "w", newline="") as fo:
        w = csv.DictWriter(fo, fieldnames=keys)
        w.writeheader()
        w.writerows(rows)
    print(f"\nwrote {len(rows)} rows -> {out_csv}\n")

    arr = {k: np.array([r[k] for r in rows], dtype=float) for k in keys}
    params = ["amp_ratio", "freq_ratio_fc", "delta_phi",
              "pitch_amp_rad", "heave_amp_rad", "pitch_freq_hz", "heave_freq_hz",
              "pitch_tipspeed", "heave_tipspeed"]
    descs = ["peak_height", "trough_min", "skew", "peak_count", "mean_fx"]

    print("=== single-variable correlation (slope, R^2) over the FULL 3D grid ===")
    hdr = f"{'descriptor':12s}" + "".join(f"{p:>18s}" for p in params)
    print(hdr)
    for d in descs:
        y = arr[d]
        line = f"{d:12s}"
        for p in params:
            x = arr[p]
            if np.std(x) < 1e-12 or np.std(y) < 1e-12:
                line += f"{'--':>18s}"
                continue
            slope = np.polyfit(x, y, 1)[0]
            r2 = np.corrcoef(x, y)[0, 1] ** 2
            line += f"{slope:+9.3f}/{r2:5.3f}   "[:18]
        print(line)

    print("\n=== multivariate least squares: descriptor ~ a*amp_ratio + b*freq_ratio_fc + c*delta_phi + d ===")
    A = np.vstack([arr["amp_ratio"], arr["freq_ratio_fc"], arr["delta_phi"],
                   np.ones(len(rows))]).T
    for d in descs:
        y = arr[d]
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        pred = A @ coef
        ss_res = np.sum((y - pred) ** 2)
        ss_tot = np.sum((y - np.mean(y)) ** 2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
        print(f"  {d:12s}: amp={coef[0]:+.4f}  freq={coef[1]:+.4f}  "
              f"dphi={coef[2]:+.4f}  const={coef[3]:+.4f}   R2={r2:.3f}")

    print("\n=== best-achievable per descriptor (what the rig can actually do) ===")
    for d in descs:
        y = arr[d]
        i_max, i_min = int(np.argmax(y)), int(np.argmin(y))
        print(f"  {d:12s}: max={y[i_max]:+8.3f} at amp={arr['amp_ratio'][i_max]:.2f} "
              f"freq={arr['freq_ratio_fc'][i_max]:.3f} dphi={arr['delta_phi'][i_max]:+.2f} | "
              f"min={y[i_min]:+8.3f} at amp={arr['amp_ratio'][i_min]:.2f} "
              f"freq={arr['freq_ratio_fc'][i_min]:.3f} dphi={arr['delta_phi'][i_min]:+.2f}")

    print("\n=== peak_height marginal means (grid cell averages) ===")
    for p in ["amp_ratio", "freq_ratio_fc"]:
        print(f"  by {p}:")
        for v in sorted(set(np.round(arr[p], 3))):
            m = np.abs(arr[p] - v) < 1e-6
            print(f"    {v:7.3f}: peak_height={arr['peak_height'][m].mean():6.3f} "
                  f"(n={m.sum()}, sd={arr['peak_height'][m].std():.3f})  "
                  f"peak_count_mode={int(np.median(arr['peak_count'][m]))}")


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "in_house_wet_test_3D/split_data"
    step = int(sys.argv[2]) if len(sys.argv) > 2 else 45
    main(root, step)
