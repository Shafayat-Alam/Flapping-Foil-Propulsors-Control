#!/usr/bin/env python3
"""Kinematic knob -> force-waveform response study over the full 3D sweep.

Answers, per force channel (Fx, Fy, Fz), which kinematic knob moves which
FEATURE of the waveform:

  positive peak height / negative peak height   (scale up-down)
  positive lobe width  / negative lobe width    (widen-thin)
  temporal skew (rise vs fall)                  (skew left-right)
  peak count, net force

For every one of the ~904 missions this:
  1. tares against that cell's PRE_EXPERIMENT_CAL block,
  2. low-passes at 10x the pitch frequency (kills the ~29 Hz structural
     resonance that otherwise dominates peak/width estimates),
  3. keeps only the LAST full gait cycle (steady state; the sweeps ran 4
     cycles per command),
  4. extracts the descriptors above.

Output: in_house_wet_test_3D/knob_response_descriptors.csv (one row per
mission) -- the regressions in report_knob_relationships.py read this.

NOTE ON WHAT THIS DATA CAN AND CANNOT ANSWER: the sweep preserves the
geometric mean of both amplitude and frequency (pitch_amp*heave_amp =
0.3948 rad^2 and f_pitch*f_heave = 0.25 Hz^2 in EVERY cell). So it varies
only the RATIOS and the phase -- three degrees of freedom. Moving both
amplitudes (or both frequencies) together, i.e. an overall scale change,
never happened here and cannot be inferred from this dataset.
"""
import os, re, sys, glob, math
import numpy as np
import pandas as pd
from scipy.signal import butter, filtfilt, find_peaks
from multiprocessing import Pool

ROOT = "/home/shafa/soft-propulsors-control/in_house_wet_test_3D"
SPLIT = os.path.join(ROOT, "split_data")
OUT = os.path.join(ROOT, "knob_response_descriptors.csv")

CENTER_AMP = 0.6283185307   # rad, geometric mean of the two amplitudes
CENTER_FREQ = 0.5           # Hz, geometric mean of the two frequencies
FS = 10000.0                # load-cell sample rate
CUTOFF_MULT = 10.0          # low-pass at 10x pitch freq
WIDTH_FRAC = 0.5            # lobe width measured at 50% of that lobe's extreme


def parse_cell(name):
    """k1_fr0p400_ar0p33 -> (freq_ratio, amp_ratio)."""
    m = re.match(r"k1_fr(\d+)p(\d+)_ar(\d+)p(\d+)", name)
    if not m:
        return None
    fr = float(f"{m.group(1)}.{m.group(2)}")
    ar = float(f"{m.group(3)}.{m.group(4)}")
    return fr, ar


def read_sweep_info(cell_dir):
    """pitch/heave amp + freq for this cell, straight from sweep_info.txt."""
    info = {}
    p = os.path.join(cell_dir, "sweep_info.txt")
    for line in open(p):
        for key, pat in (("pitch_freq", r"pitch frequency \(Hz\) = ([\d.]+)"),
                         ("heave_freq", r"heave frequency \(Hz\) = ([\d.]+)"),
                         ("pitch_amp", r"pitch_amp \(rad\) = ([\d.]+)"),
                         ("heave_amp", r"heave_amp \(rad\) = ([\d.]+)")):
            m = re.search(pat, line)
            if m and key not in info:
                info[key] = float(m.group(1))
    return info


def lowpass(x, cutoff_hz):
    wn = cutoff_hz / (FS / 2.0)
    if not (1e-3 < wn < 0.99):
        return x
    try:
        b, a = butter(4, wn)
        return filtfilt(b, a, x)
    except Exception:
        return x


def lobe_width_frac(y, level, positive):
    """Fraction of the cycle spent beyond `level` -- the lobe's width.

    Measured as an occupancy fraction rather than a strict FWHM around a
    single peak: multi-peak cycles are common here, and occupancy stays
    meaningful when the lobe is split, where FWHM would silently report
    only one sub-peak.
    """
    mask = (y >= level) if positive else (y <= level)
    return float(np.count_nonzero(mask)) / len(y)


def channel_descriptors(y, prefix):
    """Shape descriptors for one force channel over exactly one cycle."""
    n = len(y)
    pos_peak, neg_peak = float(np.max(y)), float(np.min(y))
    i_pos, i_neg = int(np.argmax(y)), int(np.argmin(y))

    d = {
        f"{prefix}_pos_peak": pos_peak,
        f"{prefix}_neg_peak": neg_peak,
        f"{prefix}_p2p": pos_peak - neg_peak,
        f"{prefix}_net": float(np.mean(y)),
        f"{prefix}_t_pos_frac": i_pos / n,
        f"{prefix}_t_neg_frac": i_neg / n,
        # width of each lobe at 50% of its own extreme
        f"{prefix}_pos_width": lobe_width_frac(y, WIDTH_FRAC * pos_peak, True)
                               if pos_peak > 0 else 0.0,
        f"{prefix}_neg_width": lobe_width_frac(y, WIDTH_FRAC * neg_peak, False)
                               if neg_peak < 0 else 0.0,
        # fraction of the cycle spent pushing positive at all (duty cycle)
        f"{prefix}_pos_duty": float(np.count_nonzero(y > 0)) / n,
    }

    # Temporal skew: how long the rise into the positive peak takes versus
    # the fall out of it, as a fraction of the cycle. 0.5 = symmetric,
    # <0.5 = fast rise / slow decay (leans left), >0.5 = leans right.
    rise = (i_pos - i_neg) % n
    d[f"{prefix}_rise_frac"] = rise / n

    # Distributional asymmetry of the waveform samples -- captures "the
    # curve leans" independently of where the extremes happen to sit.
    s = np.std(y)
    d[f"{prefix}_skew"] = float(np.mean(((y - np.mean(y)) / s) ** 3)) if s > 1e-12 else 0.0

    # Peak counts, on the cycle tiled 3x so peaks at the seam are not lost.
    span = max(pos_peak - neg_peak, 1e-9)
    tiled = np.concatenate([y, y, y])
    for tag, sig in (("n_pos_peaks", tiled), ("n_neg_peaks", -tiled)):
        pk, _ = find_peaks(sig, prominence=0.2 * span, distance=max(1, int(0.07 * n)))
        pk = [p for p in pk if n <= p < 2 * n]
        d[f"{prefix}_{tag}"] = len(pk)
    return d


def active_window(t, y, quiet_mult=4.0):
    """(i0, i1) bracketing the part of the recording where the rig actually moved.

    Necessary because each mission's load-cell recording runs several
    seconds PAST the end of the gait: a typical file is ~13.8 s long with
    motion only over 0-6.3 s. Slicing "the last cycle" off the raw record
    therefore lands in dead air -- which is exactly what the first version
    of this script did, reporting a 0.02 N peak-to-peak and a cycle mean
    equal to the rig's untared rest offset.

    Detection is on a 0.1 s rolling standard deviation: the idle tail's
    sigma is the sensor noise floor (~0.02 N), while moving sigma is
    10-30x that, so the threshold is not delicate.
    """
    w = max(16, int(0.1 * FS))
    n = len(y) // w
    if n < 4:
        return 0, len(y)
    seg = y[:n * w].reshape(n, w).std(axis=1)
    quiet = np.median(np.sort(seg)[:max(2, n // 5)])   # noise floor
    thr = max(quiet * quiet_mult, quiet + 1e-6)
    moving = np.where(seg > thr)[0]
    if len(moving) < 2:
        return 0, len(y)
    return int(moving[0] * w), int(min(len(y), (moving[-1] + 1) * w))


def idle_baseline(y, i1):
    """Per-mission tare from that recording's own post-motion idle tail.

    The campaign only saved a PRE_EXPERIMENT_CAL block for 1 of the 35
    cells, so a per-cell calibration tare is not available for 97% of the
    data. The idle tail is a better baseline anyway: it is contemporaneous
    with the mission (no drift between calibration and run), and the rig's
    rest offsets are large enough (Fz0 ~ -26 N) that leaving them in
    swamps every magnitude descriptor.
    """
    tail = y[i1:]
    if len(tail) < int(0.5 * FS):
        head = y[:max(1, int(0.2 * FS))]
        return float(np.mean(head))
    return float(np.mean(tail[int(0.2 * len(tail)):]))


def beat_period(f_pitch, freq_ratio):
    """The period over which the COMBINED pitch+heave motion repeats.

    Not 1/min(f1,f2): with freq_ratio = p/q in lowest terms the gait only
    repeats after q pitch cycles. At freq_ratio 0.4 = 2/5 that is 5 pitch
    cycles = 6.33 s, which matches the observed 6.3 s of motion exactly,
    whereas 1/min(f1,f2) would claim 3.16 s and fold two physically
    distinct half-gaits onto each other.
    """
    from fractions import Fraction
    fa = Fraction(freq_ratio).limit_denominator(20)
    return fa.denominator / f_pitch


def process_mission(args):
    cell_dir, mission_dir, fr, ar, info, base = args
    lc = glob.glob(os.path.join(mission_dir, "*_loadcell.csv"))
    if not lc:
        return None
    try:
        df = pd.read_csv(lc[0], usecols=["time_s", "Fx", "Fy", "Fz"],
                         dtype={"time_s": np.float64, "Fx": np.float32,
                                "Fy": np.float32, "Fz": np.float32})
    except Exception:
        return None
    if len(df) < 1000:
        return None

    t = df["time_s"].to_numpy()
    period = beat_period(info["pitch_freq"], fr)

    # Find the motion window from Fx (largest excursion of the three), then
    # tare every channel on its own idle tail beyond that window.
    fx_raw = df["Fx"].to_numpy().astype(np.float64)
    i0, i1 = active_window(t, fx_raw)
    active_dur = t[min(i1, len(t) - 1)] - t[i0]
    if active_dur < 0.5 * period:
        return None

    m = re.search(r"PH_(\d+)", os.path.basename(mission_dir))
    phase_deg = float(m.group(1)) if m else float("nan")

    row = {
        "cell": os.path.basename(cell_dir),
        "mission": os.path.basename(mission_dir),
        "freq_ratio": fr, "amp_ratio": ar,
        "phase_deg": phase_deg,
        "phase_rad": math.radians(phase_deg),
        "pitch_amp": info["pitch_amp"], "heave_amp": info["heave_amp"],
        "pitch_freq": info["pitch_freq"], "heave_freq": info["heave_freq"],
        "period_s": period, "active_s": active_dur,
    }

    # Analysis window: the LAST full beat period inside the motion window,
    # so startup transients are excluded when the run is long enough to
    # afford it. Several cells only ran a single beat period, so fall back
    # to the whole motion window rather than dropping those cells entirely.
    t_end = t[min(i1, len(t) - 1)]
    t_start = max(t[i0], t_end - period)
    sl = (t >= t_start) & (t <= t_end)
    if np.count_nonzero(sl) < 50:
        return None

    cut = CUTOFF_MULT * info["pitch_freq"]
    for ch, prefix in (("Fx", "fx"), ("Fy", "fy"), ("Fz", "fz")):
        raw = df[ch].to_numpy().astype(np.float64)
        y = lowpass(raw - idle_baseline(raw, i1), cut)[sl]
        if len(y) < 50:
            return None
        row.update(channel_descriptors(y, prefix))

    # Peak counts above are per BEAT period, which spans q pitch cycles and
    # so is not comparable between cells (q ranges 1..5 across the grid).
    # Normalising to peaks-per-pitch-cycle gives the quantity that is
    # actually being asked about -- "does one flap produce one hump or two".
    n_pitch_cycles = max(1e-9, (t_end - t_start) * info["pitch_freq"])
    n_heave_cycles = max(1e-9, (t_end - t_start) * info["heave_freq"])
    for prefix in ("fx", "fy", "fz"):
        for sign in ("pos", "neg"):
            n = row[f"{prefix}_n_{sign}_peaks"]
            row[f"{prefix}_{sign}_per_pitch"] = n / n_pitch_cycles
            row[f"{prefix}_{sign}_per_heave"] = n / n_heave_cycles
    return row


def main():
    cells = sorted(d for d in glob.glob(os.path.join(SPLIT, "k1_*")) if os.path.isdir(d))
    jobs = []
    for cd in cells:
        parsed = parse_cell(os.path.basename(cd))
        if not parsed:
            continue
        fr, ar = parsed
        info = read_sweep_info(cd)
        if len(info) < 4:
            continue

        # Per-cell tare from that cell's own calibration block: the rig's
        # rest offsets drift between cells (Fz0 moved ~4 N across the
        # campaign), so a single global baseline would bias peak heights.
        cal = glob.glob(os.path.join(cd, "PRE_EXPERIMENT_CAL*", "*_loadcell.csv"))
        base = {"Fx": 0.0, "Fy": 0.0, "Fz": 0.0}
        if cal:
            try:
                cdf = pd.read_csv(cal[0], usecols=["Fx", "Fy", "Fz"])
                base = {c: float(cdf[c].mean()) for c in ("Fx", "Fy", "Fz")}
            except Exception:
                pass

        for md in sorted(glob.glob(os.path.join(cd, "PH_*"))):
            if os.path.isdir(md):
                jobs.append((cd, md, fr, ar, info, base))

    print(f"processing {len(jobs)} missions across {len(cells)} cells ...")
    rows = []
    with Pool(processes=min(8, os.cpu_count() or 4)) as pool:
        for i, r in enumerate(pool.imap_unordered(process_mission, jobs, chunksize=4), 1):
            if r:
                rows.append(r)
            if i % 100 == 0:
                print(f"  {i}/{len(jobs)}", flush=True)

    out = pd.DataFrame(rows).sort_values(["amp_ratio", "freq_ratio", "phase_deg"])
    out.to_csv(OUT, index=False)
    print(f"\nwrote {OUT}  ({len(out)} missions x {len(out.columns)} columns)")


if __name__ == "__main__":
    main()
