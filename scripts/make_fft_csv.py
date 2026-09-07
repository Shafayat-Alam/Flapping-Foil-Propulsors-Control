#!/usr/bin/env python3
"""
make_fft_csv.py — same missions as make_combined_csv.py, but the force columns
hold the FFT magnitude spectrum instead of the raw time series.

One row per (mission, frequency bin):
  mission_label, phase_deg, freq_hz,
  Fx_thrust_mag_N, Fy_lateral_mag_N, Fz_heave_mag_N

Each axis is: recorded-axis corrected (thrust<-Fy, lateral<-Fz, heave<-Fx),
tared against its own at-rest tail, windowed to the steady gait cycles
(1st cycle dropped as warm-up), resampled to a uniform grid (the load cell
arrives in non-uniform bursts — an FFT of that would smear), then rFFT'd.

    python3 scripts/make_fft_csv.py <stage_dir> <out.csv>
"""
import csv, glob, os, sys, math
import numpy as np

csv.field_size_limit(2**31 - 1)
STEP = 20                # downsample the 10 kHz load cell before resampling
RESAMPLE_HZ = 200.0
FMAX_REPORT = 5.0        # only write bins up to this frequency (keeps the CSV small)

# analysis axis <- recorded channel (thrust/lateral/heave)
AXES = [("Fx_thrust_mag_N", "Fy"), ("Fy_lateral_mag_N", "Fz"), ("Fz_heave_mag_N", "Fx")]


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _label(md):
    for f in glob.glob(os.path.join(md, "*.csv")):
        b = os.path.basename(f)
        if not b.endswith("_loadcell.csv"):
            return b[:-4]
    return None


def _zero_spectrum(label, phase_deg, gait_end):
    """Synthetic all-zero spectrum for a mission with no load-cell data at all,
    on the same frequency grid a real mission would produce (same RESAMPLE_HZ,
    same gait duration) so it lines up with its neighbors in the CSV/plots.
    Callers mark these with no_loadcell=True so they render distinctly, not
    mistaken for a genuinely quiet (near-zero) real signal."""
    n = max(32, int(gait_end * RESAMPLE_HZ))
    fr = np.fft.rfftfreq(n, d=1.0 / RESAMPLE_HZ)
    zero = np.zeros_like(fr)
    out_spec = {out_col: (fr, zero) for out_col, _ in AXES}
    return {"label": label, "phase_deg": phase_deg, "spec": out_spec,
            "no_loadcell": True}


def fft_mission(md):
    label = _label(md)
    if not label:
        return None
    fbp = os.path.join(md, f"{label}.csv")
    lcp = os.path.join(md, f"{label}_loadcell.csv")
    if not os.path.exists(fbp):
        return None
    r0 = next(csv.DictReader(open(fbp)), {})
    f0 = _num(r0.get("cmd.frequency"))
    ph = _num(r0.get("cmd.phase")) or 0.0
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    if not f0:
        return None
    gait_end = cyc / f0
    phase_deg = round(ph * 180 / math.pi)

    if not os.path.exists(lcp):
        # No load-cell file at all for this mission (e.g. a live dropout) —
        # plot it as zero force rather than silently dropping the phase.
        return _zero_spectrum(label, phase_deg, gait_end)

    with open(lcp) as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        ix = {h: i for i, h in enumerate(hdr)}
        if "time_s" not in ix or not all(c in ix for _, c in AXES):
            return None
        t, raw = [], {c: [] for _, c in AXES}
        for k, row in enumerate(rd):
            if k % STEP:
                continue
            tv = _num(row[ix["time_s"]])
            if tv is None:
                continue
            vals = {c: _num(row[ix[c]]) for _, c in AXES}
            if any(v is None for v in vals.values()):
                continue
            t.append(tv)
            for _, c in AXES:
                raw[c].append(vals[c])
    t = np.asarray(t)
    if t.size < 50 or not (t.max() > gait_end + 2):   # too short / spurious
        return None

    rest = t > gait_end + 1.0
    win = (t >= 1.0 / f0) & (t <= gait_end)
    if win.sum() < 50:
        return None

    out_spec = {}
    freq = None
    for out_col, rec_col in AXES:
        x = np.asarray(raw[rec_col])
        if rest.sum() > 10:
            x = x - np.median(x[rest])          # tare
        tw, xw = t[win], x[win]
        n = int((tw[-1] - tw[0]) * RESAMPLE_HZ)
        if n < 32:
            return None
        tu = np.linspace(tw[0], tw[-1], n)
        xu = np.interp(tu, tw, xw)
        xu = xu - xu.mean()
        mag = np.abs(np.fft.rfft(xu * np.hanning(n))) * (2.0 / n)
        fr = np.fft.rfftfreq(n, d=(tu[1] - tu[0]))
        out_spec[out_col] = (fr, mag)
        freq = fr if freq is None or fr.size < freq.size else freq

    return {"label": label, "phase_deg": phase_deg, "spec": out_spec}


def _fft_one_window(t, x, t_lo, t_hi):
    """Resample [t_lo, t_hi] onto a uniform grid and rFFT it. None if too short."""
    m = (t >= t_lo) & (t <= t_hi)
    if m.sum() < 8:
        return None
    tw, xw = t[m], x[m]
    n = int((tw[-1] - tw[0]) * RESAMPLE_HZ)
    if n < 16:
        return None
    tu = np.linspace(tw[0], tw[-1], n)
    xu = np.interp(tu, tw, xw)
    xu = xu - xu.mean()
    mag = np.abs(np.fft.rfft(xu * np.hanning(n))) * (2.0 / n)
    fr = np.fft.rfftfreq(n, d=(tu[1] - tu[0]))
    return fr, mag


def fft_mission_percycle(md):
    """Like fft_mission, but ALSO breaks the steady window into its individual
    cycles and FFTs each one separately — so cycle-to-cycle consistency (or
    drift) is visible, not just the one spectrum averaged over the whole
    window.  Returns the same dict as fft_mission plus "cycles": a list of
    per-cycle {axis: (freq, mag)} dicts, one per cycle in the steady window
    (cycle 1 dropped as warm-up, same convention as everywhere else)."""
    label = _label(md)
    if not label:
        return None
    fbp = os.path.join(md, f"{label}.csv")
    lcp = os.path.join(md, f"{label}_loadcell.csv")
    if not os.path.exists(fbp):
        return None
    r0 = next(csv.DictReader(open(fbp)), {})
    f0 = _num(r0.get("cmd.frequency"))
    ph = _num(r0.get("cmd.phase")) or 0.0
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    if not f0:
        return None
    gait_end = cyc / f0
    phase_deg = round(ph * 180 / math.pi)
    n_cycles = max(1, int(round(cyc)) - 1)   # cycles in the steady window

    if not os.path.exists(lcp):
        base = _zero_spectrum(label, phase_deg, gait_end)
        base["cycles"] = [base["spec"] for _ in range(n_cycles)]
        base["n_cycles"] = n_cycles
        return base

    with open(lcp) as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        ix = {h: i for i, h in enumerate(hdr)}
        if "time_s" not in ix or not all(c in ix for _, c in AXES):
            return None
        t, raw = [], {c: [] for _, c in AXES}
        for k, row in enumerate(rd):
            if k % STEP:
                continue
            tv = _num(row[ix["time_s"]])
            if tv is None:
                continue
            vals = {c: _num(row[ix[c]]) for _, c in AXES}
            if any(v is None for v in vals.values()):
                continue
            t.append(tv)
            for _, c in AXES:
                raw[c].append(vals[c])
    t = np.asarray(t)
    if t.size < 50 or not (t.max() > gait_end + 2):
        return None

    rest = t > gait_end + 1.0
    win = (t >= 1.0 / f0) & (t <= gait_end)
    if win.sum() < 50:
        return None

    out_spec, cycle_specs = {}, [{} for _ in range(n_cycles)]
    for out_col, rec_col in AXES:
        x = np.asarray(raw[rec_col])
        if rest.sum() > 10:
            x = x - np.median(x[rest])          # tare
        combined = _fft_one_window(t, x, 1.0 / f0, gait_end)
        if combined is None:
            return None
        out_spec[out_col] = combined
        for ci in range(n_cycles):
            lo, hi = (1 + ci) / f0, (2 + ci) / f0
            per = _fft_one_window(t, x, lo, hi)
            cycle_specs[ci][out_col] = per if per is not None else combined

    return {"label": label, "phase_deg": phase_deg, "spec": out_spec,
            "cycles": cycle_specs, "n_cycles": n_cycles}


def main(stage, out_csv):
    mdirs = [d for d in sorted(glob.glob(os.path.join(stage, "*")))
             if os.path.isdir(d) and os.path.basename(d) != "raw"]
    n_written = n_missions = 0
    with open(out_csv, "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["mission_label", "phase_deg", "freq_hz",
                    "Fx_thrust_mag_N", "Fy_lateral_mag_N", "Fz_heave_mag_N"])
        for md in mdirs:
            r = fft_mission(md)
            if r is None:
                lab = _label(md) or os.path.basename(md)
                print(f"  {lab}: skipped (no data / spurious segment)")
                continue
            fr_ref, _ = next(iter(r["spec"].values()))
            m = fr_ref <= FMAX_REPORT
            fr_ref = fr_ref[m]
            for i, fq in enumerate(fr_ref):
                row = [r["label"], r["phase_deg"], round(float(fq), 4)]
                for col, _ in AXES:
                    fr, mag = r["spec"][col]
                    row.append(round(float(mag[i]), 6) if i < mag.size else "")
                w.writerow(row)
                n_written += 1
            n_missions += 1
    print(f"wrote {n_written} rows from {n_missions} missions -> {out_csv}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        sys.exit("usage: make_fft_csv.py <stage_dir> <out.csv>")
    main(sys.argv[1], sys.argv[2])
