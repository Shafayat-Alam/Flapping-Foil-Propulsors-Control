#!/usr/bin/env python3
"""
prove_claims.py — quantitative proof/disproof of the 6 kinematic->force claims,
exactly as specified: strict single-variable data slices (other two params
locked), the specific named statistics, and the specific named plots.

  1. Amplitude claims: freq_ratio=1.0, phase=0 locked, sweep amp_ratio.
     Statistic: peak-to-peak force. Plot: amp_ratio vs Fx_p2p/Fy_p2p, + R^2.
  2. Frequency claims: amp_ratio=1.0, phase=0 locked, sweep freq_ratio.
     Statistic: FFT dominant frequency. Plot A: pitch_freq/heave_freq vs
     FFT_dom(Fx)/FFT_dom(Fy) against a y=x reference, + R^2. Plot B: raw
     Fx(t)/Fy(t) time-series overlay for the freq_ratio=2.0 run.
  3. Phase claims: freq_ratio=1.0, amp_ratio=1.0 locked, sweep phase.
     Statistics: skewness index S (time-to-peak / cycle time) for Fx and Fy,
     and Fx peak count N. Dual-axis plot vs phase.

NOTE on axis convention (established throughout this project): Fx is the
pitch-associated axis, Fy is the heave-associated axis. "freq_ratio" as
stored in block names is heave_freq/pitch_freq; where a plot needs
pitch_freq/heave_freq instead (to align with "v_x/v_y" in the claims'
notation) it is computed as 1/freq_ratio and labeled explicitly so there is
no ambiguity.

    python3 scripts/prove_claims.py <sweep_root/data> <out_folder>
"""
import csv, glob, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

STEP = 5


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def parse_block_tag(name):
    if name == "k1_pf0p500_r1p000":
        return 1.0, 1.0
    m = re.match(r"^k1_fr(\d+)p(\d+)_ar(\d+)p(\d+)$", name)
    if not m:
        return None, None
    return float(f"{m.group(1)}.{m.group(2)}"), float(f"{m.group(3)}.{m.group(4)}")


def find_block(root, freq_ratio, amp_ratio):
    if abs(freq_ratio - 1.0) < 1e-9 and abs(amp_ratio - 1.0) < 1e-9:
        return os.path.join(root, "k1_pf0p500_r1p000")
    tag = f"k1_fr{freq_ratio:.3f}_ar{amp_ratio:.2f}".replace(".", "p")
    return os.path.join(root, tag)


def find_mission(root, freq_ratio, amp_ratio, phase_deg):
    bd = find_block(root, freq_ratio, amp_ratio)
    if not os.path.isdir(bd):
        return None
    for md in sorted(glob.glob(os.path.join(bd, "PH_*"))):
        fb_files = [f for f in glob.glob(os.path.join(md, "*.csv")) if "_loadcell" not in f]
        if not fb_files:
            continue
        r0 = next(csv.DictReader(open(fb_files[0])), {})
        ph = _num(r0.get("cmd.phase")) or 0.0
        if round(ph * 180 / np.pi) % 360 == phase_deg % 360:
            return md
    return None


def load_raw(md):
    """Full raw (t, Fx, Fy) at native downsample, tared, NOT windowed/folded."""
    lc_files = glob.glob(os.path.join(md, "*_loadcell.csv"))
    if not lc_files:
        return None
    fb_files = [f for f in glob.glob(os.path.join(md, "*.csv")) if "_loadcell" not in f]
    if not fb_files:
        return None
    r0 = next(csv.DictReader(open(fb_files[0])), {})
    f0 = _num(r0.get("cmd.frequency"))
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    if not f0:
        return None
    gait_end = cyc / f0
    t, F = [], {"Fx": [], "Fy": []}
    with open(lc_files[0]) as fh:
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
    for a in F:
        F[a] = np.asarray(F[a])
        if rest.sum() > 10:
            F[a] = F[a] - np.median(F[a][rest])
    return {"t": t, "Fx": F["Fx"], "Fy": F["Fy"], "f0": f0, "gait_end": gait_end}


def cycle2_window(d):
    period = 1.0 / d["f0"]
    m = (d["t"] >= period) & (d["t"] < 2 * period)
    return d["t"][m] - period, d["Fx"][m], d["Fy"][m], period


def steady_window(d):
    period = 1.0 / d["f0"]
    m = (d["t"] >= period) & (d["t"] <= d["gait_end"])
    return d["t"][m], d["Fx"][m], d["Fy"][m], period


def r_squared(x, y):
    if len(x) < 3:
        return float("nan")
    r = np.corrcoef(x, y)[0, 1]
    return r ** 2


# ===========================================================================
# 1. Amplitude claims
# ===========================================================================
def _single_claim_plot(xs, ys, title, xlabel, ylabel, out_path, color="tab:purple"):
    xs = np.array(xs)
    ys = np.array(ys)
    r2 = r_squared(xs, ys) if len(xs) > 2 else float("nan")
    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(xs, ys, "o", color=color, markersize=9, zorder=3)
    if len(xs) > 2:
        m, b = np.polyfit(xs, ys, 1)
        xf = np.linspace(xs.min(), xs.max(), 50)
        ax.plot(xf, m * xf + b, "--", color="gray", lw=1.5, zorder=2,
               label=f"linear fit (R²={r2:.3f}, n={len(xs)})")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return r2


def part1_amplitude(root, out_dir):
    amp_ratios = [0.33, 0.67, 1.00, 1.50, 3.00]
    xs, ratios, fx_p2p_l, fy_p2p_l = [], [], [], []
    for ar in amp_ratios:
        md = find_mission(root, 1.0, ar, 0)
        if md is None:
            print(f"  [amp] missing mission for ar={ar}")
            continue
        d = load_raw(md)
        if d is None:
            continue
        _, fx, fy, _ = cycle2_window(d)
        if len(fx) < 5:
            continue
        fx_p2p = fx.max() - fx.min()
        fy_p2p = fy.max() - fy.min()
        xs.append(ar)
        fx_p2p_l.append(fx_p2p)
        fy_p2p_l.append(fy_p2p)
        ratios.append(fx_p2p / fy_p2p if fy_p2p > 1e-9 else float("nan"))

    xs = np.array(xs)
    ratios = np.array(ratios)
    r2 = r_squared(xs, ratios)

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(xs, ratios, "o-", color="tab:purple", lw=2, markersize=8)
    if len(xs) > 2:
        m, b = np.polyfit(xs, ratios, 1)
        xf = np.linspace(xs.min(), xs.max(), 50)
        ax.plot(xf, m * xf + b, "--", color="gray", lw=1.2,
               label=f"linear fit (R²={r2:.3f})")
    ax.set_xlabel("Input Amplitude Ratio  A1/A2  (pitch_amp/heave_amp)")
    ax.set_ylabel("Output Force Ratio  Fx_p2p / Fy_p2p")
    ax.set_title("Claims 1 & 2 — amplitude ratio -> force ratio\n"
                "(freq_ratio=1.0, phase=0 deg locked)")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    out_path = os.path.join(out_dir, "scaling_proof.png")
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    with open(os.path.join(out_dir, "scaling_data.csv"), "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["amp_ratio", "Fx_p2p_N", "Fy_p2p_N", "Fx_p2p_over_Fy_p2p"])
        for i in range(len(xs)):
            w.writerow([xs[i], fx_p2p_l[i], fy_p2p_l[i], ratios[i]])

    print(f"  [amp] R^2 = {r2:.4f}  (n={len(xs)} points)")
    print(f"  [amp] wrote {out_path}")
    return r2


# ===========================================================================
# 2. Frequency claims
# ===========================================================================
def resample_uniform(t, x, sample_rate=100.0):
    n = int((t[-1] - t[0]) * sample_rate)
    if n < 20:
        return None, None
    tu = np.linspace(t[0], t[-1], n)
    xu = np.interp(tu, t, x)
    return tu, xu


def extract_fundamental_frequency(time, force_signal, commanded_freq,
                                  sample_rate=100.0, lo_mult=0.5, hi_mult=2.5,
                                  pad_factor=10):
    """
    Extracts the dominant motion-related frequency while ignoring noise/DC,
    WITHOUT pre-assuming whether the true fundamental sits at 1x or 2x the
    commanded frequency. A window of only +/-35% around 1x would silently
    exclude a real 2x peak (biasing the result toward the 1x hypothesis
    before any comparison is made) -- so instead the search band spans
    [0.5x, 2.5x] of the commanded frequency, wide enough to contain BOTH the
    1x and 2x candidate locations plus margin, and simply takes whichever
    peak is actually largest in that band. This keeps the y=x vs y=2x
    comparison downstream honest -- the peak-finding itself doesn't favor
    either line.

    Zero-padded (pad_factor x) before the FFT for a smoother, less
    quantization-snapped peak location. NOTE: this interpolates between the
    existing coarse bins, it does not increase the true Rayleigh frequency
    resolution (still set by the real window duration) -- kept for that
    reason, but it will not by itself fix a genuinely too-short window.
    """
    signal_clean = force_signal - np.mean(force_signal)
    window = np.hanning(len(signal_clean))
    windowed_signal = signal_clean * window

    n_fft = len(signal_clean) * pad_factor
    fft_magnitudes = np.abs(np.fft.rfft(windowed_signal, n=n_fft))
    freq_bins = np.fft.rfftfreq(n_fft, d=1.0 / sample_rate)

    f_min = commanded_freq * lo_mult
    f_max = commanded_freq * hi_mult
    search_mask = (freq_bins >= f_min) & (freq_bins <= f_max)

    if np.any(search_mask):
        sub_magnitudes = fft_magnitudes[search_mask]
        sub_freqs = freq_bins[search_mask]
        dominant_freq = sub_freqs[np.argmax(sub_magnitudes)]
    else:
        dominant_freq = freq_bins[np.argmax(fft_magnitudes)]

    return float(dominant_freq)


def r_squared_identity(x, y):
    """R^2 relative to the y=x identity line, NOT Pearson correlation --
    measures how close points are to y=x specifically, not just how linear
    the relationship is."""
    x = np.asarray(x)
    y = np.asarray(y)
    ss_res = np.sum((y - x) ** 2)
    ss_tot = np.sum((x - np.mean(x)) ** 2)
    if ss_tot < 1e-12:
        return float("nan")
    return 1.0 - ss_res / ss_tot


def part2_frequency(root, out_dir):
    freq_ratios = [0.4, 0.5, 0.667, 1.0, 1.5, 2.0, 2.5]
    xs_input, ys_output = [], []
    rows = []
    sample_rate = 100.0
    for fr in freq_ratios:
        md = find_mission(root, fr, 1.0, 0)
        if md is None:
            print(f"  [freq] missing mission for fr={fr}")
            continue
        d = load_raw(md)
        if d is None:
            continue
        t, fx, fy, period = steady_window(d)
        if len(t) < 30:
            continue
        pitch_freq_cmd = d["f0"]
        heave_freq_cmd = d["f0"] * fr   # fr stored = heave/pitch

        tu_x, fxu = resample_uniform(t, fx, sample_rate)
        tu_y, fyu = resample_uniform(t, fy, sample_rate)
        if fxu is None or fyu is None:
            continue
        fdx = extract_fundamental_frequency(tu_x, fxu, pitch_freq_cmd, sample_rate)
        fdy = extract_fundamental_frequency(tu_y, fyu, heave_freq_cmd, sample_rate)
        if fdy < 1e-6:
            continue
        pitch_over_heave_input = pitch_freq_cmd / heave_freq_cmd   # == 1/fr
        output_ratio = fdx / fdy
        xs_input.append(pitch_over_heave_input)
        ys_output.append(output_ratio)
        rows.append((fr, pitch_over_heave_input, fdx, fdy, output_ratio))

    xs_input = np.array(xs_input)       # pitch_freq / heave_freq
    ys_output = np.array(ys_output)     # f0(Fx) / f0(Fy)
    r2_1x = r_squared_identity(xs_input, ys_output) if len(xs_input) > 2 else float("nan")
    r2_2x = r_squared_identity(2 * xs_input, ys_output) if len(xs_input) > 2 else float("nan")

    fig, ax = plt.subplots(figsize=(7.5, 6.5))
    ax.scatter(xs_input, ys_output, color="tab:red", s=70, zorder=3,
              label="measured (fundamental-aware, zero-padded FFT freq ratio)")
    lims = [0, max(xs_input.max(), 2 * xs_input.max(), ys_output.max()) * 1.1]
    ax.plot(lims, lims, "--", color="gray", lw=1.5, label=f"y = x   (R²={r2_1x:.3f})")
    ax.plot(lims, [2 * v for v in lims], ":", color="tab:blue", lw=1.5,
           label=f"y = 2x  (R²={r2_2x:.3f})  [\"Fx always at 2x pitch\" hypothesis]")
    ax.set_xlim(lims)
    ax.set_ylim(lims)
    ax.set_xlabel("Input Frequency Ratio  pitch_freq / heave_freq")
    ax.set_ylabel("Fundamental Frequency Ratio  f0(Fx) / f0(Fy)")
    ax.set_title(f"Claims 3 & 4 — frequency ratio -> fundamental-frequency ratio\n"
                f"(amp_ratio=1.0, phase=0 deg locked, zero-padded x10)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    out_path = os.path.join(out_dir, "rate_proof_A_scatter.png")
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    with open(os.path.join(out_dir, "rate_data.csv"), "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["freq_ratio_stored(heave/pitch)", "input_pitch_over_heave",
                    "f0_Fx_Hz", "f0_Fy_Hz", "ratio_f0Fx_over_f0Fy"])
        for row in rows:
            w.writerow(row)
    print(f"  [freq A] R^2 vs y=x  = {r2_1x:.4f}")
    print(f"  [freq A] R^2 vs y=2x = {r2_2x:.4f}  (n={len(xs_input)} points)")
    print(f"  [freq A] wrote {out_path}")

    # ---- Plot B: raw time-series overlay for freq_ratio=2.0 ----
    md = find_mission(root, 2.0, 1.0, 0)
    if md is not None:
        d = load_raw(md)
        if d is not None:
            t, fx, fy, period = steady_window(d)
            fig, ax = plt.subplots(figsize=(10, 5))
            ax.plot(t, fx, color="tab:blue", lw=1.5, label="Fx (pitch-associated)")
            ax.plot(t, fy, color="tab:orange", lw=1.5, label="Fy (heave-associated)")
            ax.axhline(0, color="k", lw=0.5, alpha=0.4)
            ax.set_xlabel("time (s)")
            ax.set_ylabel("force (N)")
            ax.set_title("Claim 3 visual proof — Fx vs Fy time series, freq_ratio=2.0\n"
                        "(heave_freq = 2 x pitch_freq — Fy should show ~2 cycles per Fx cycle)")
            ax.legend()
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            out_path_b = os.path.join(out_dir, "rate_proof_B_timeseries_fr2.0.png")
            fig.savefig(out_path_b, dpi=130)
            plt.close(fig)
            print(f"  [freq B] wrote {out_path_b}")
    else:
        print("  [freq B] no freq_ratio=2.0 mission found")

    return r2_1x, r2_2x


# ===========================================================================
# 3. Phase claims
# ===========================================================================
def wave_shape_metrics(t_frac, x):
    """Shift-invariant wave-shape metrics via the 1x/2x self-coupling
    (biphase) of a single signal against itself -- the standard technique
    for measuring nonlinear wave asymmetry/skewness (e.g. Elgar & Guza).

    A pure time delay t0 shifts the 1x phase by -2*pi*f*t0 and the 2x phase
    by -2*pi*(2f)*t0 = 2x the 1x shift; the combination
        psi = 2*phi1 - phi2
    exactly cancels that shared linear delay term, so psi changes only if
    the wave's actual SHAPE changes, not just when it's the same shape
    slid in time. This directly fixes the complaint that S was just
    tracking input phase delay 1:1 -- psi should NOT slide linearly with
    input phase shift unless the shape itself is truly changing.
    psi=0 or pi (0 deg/180 deg) -> symmetric wave; psi=+-pi/2 (90/270 deg)
    -> maximally skewed (sawtooth-like).

    R_harm = mag(2x) / (mag(1x)+mag(2x)) replaces the discrete, hard-
    thresholded peak count with a continuous 0..1 "multi-peakedness" index
    (0 = single-peaked/1x-dominant, 1 = double-peaked/2x-dominant) -- avoids
    the step-jump artifacts of counting peaks against a fixed threshold.
    """
    xc = x - np.mean(x)
    theta1 = 2 * np.pi * t_frac
    a1 = np.sum(xc * np.cos(theta1))
    b1 = np.sum(xc * np.sin(theta1))
    theta2 = 4 * np.pi * t_frac
    a2 = np.sum(xc * np.cos(theta2))
    b2 = np.sum(xc * np.sin(theta2))
    mag1 = np.hypot(a1, b1)
    mag2 = np.hypot(a2, b2)
    if mag1 < 1e-9 and mag2 < 1e-9:
        return float("nan"), float("nan")
    phi1 = np.arctan2(b1, a1)
    phi2 = np.arctan2(b2, a2)
    psi_deg = np.degrees((2 * phi1 - phi2) % (2 * np.pi))
    r_harm = mag2 / (mag1 + mag2)
    return psi_deg, r_harm


def part3_phase(root, out_dir):
    phases = list(range(0, 360, 15))
    xs, psix_l, psiy_l, rharm_l = [], [], [], []
    for ph in phases:
        md = find_mission(root, 1.0, 1.0, ph)
        if md is None:
            continue
        d = load_raw(md)
        if d is None:
            continue
        t, fx, fy, period = cycle2_window(d)
        if len(t) < 20:
            continue
        t_frac = t / period
        order = np.argsort(t_frac)
        t_frac, fxo, fyo = t_frac[order], fx[order], fy[order]
        psi_x, r_harm_x = wave_shape_metrics(t_frac, fxo)
        psi_y, _ = wave_shape_metrics(t_frac, fyo)
        xs.append(ph)
        psix_l.append(psi_x)
        psiy_l.append(psi_y)
        rharm_l.append(r_harm_x)

    xs = np.array(xs)
    psix_l = np.array(psix_l)
    psiy_l = np.array(psiy_l)
    rharm_l = np.array(rharm_l)

    fig, ax1 = plt.subplots(figsize=(11, 6))
    ax1.plot(xs, psix_l, "o-", color="tab:blue", lw=2, markersize=6,
             label="psi_x = 2*phi1-phi2  (Fx wave-shape skewness, shift-invariant)")
    ax1.plot(xs, psiy_l, "o-", color="tab:orange", lw=2, markersize=6,
             label="psi_y = 2*phi1-phi2  (Fy wave-shape skewness, shift-invariant)")
    ax1.axhline(0, color="gray", ls=":", lw=1, alpha=0.7)
    ax1.axhline(180, color="gray", ls=":", lw=1, alpha=0.7, label="0/180 deg = symmetric wave")
    ax1.axhline(90, color="gray", ls="--", lw=1, alpha=0.4)
    ax1.axhline(270, color="gray", ls="--", lw=1, alpha=0.4, label="90/270 deg = max skew (sawtooth-like)")
    ax1.set_xlabel("Input phase shift (deg)  — heave phase relative to pitch")
    ax1.set_ylabel("Relative harmonic phase  psi (deg)")
    ax1.set_ylim(0, 360)
    ax1.grid(True, alpha=0.3)
    ax1.legend(fontsize=9)

    ax1.set_title("Claims 5 & 6 — phase shift -> wave-shape skewness (psi)\n"
                 "(freq_ratio=1.0, amp_ratio=1.0 locked)")
    fig.tight_layout()
    out_path = os.path.join(out_dir, "skewness_proof.png")
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    print(f"  [phase] wrote {out_path}  (n={len(xs)} points)")

    with open(os.path.join(out_dir, "skewness_data.csv"), "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["phase_deg", "psi_x_deg", "psi_y_deg", "R_harm_x"])
        for i in range(len(xs)):
            w.writerow([xs[i], psix_l[i], psiy_l[i], rharm_l[i]])


# ===========================================================================
# 4. Peak count regime map (real measured data, not simulated)
# ===========================================================================
def count_force_peaks(t, x, f0, prominence_frac=0.2, min_distance_frac=0.07,
                      cutoff_mult=4.0):
    """Counts genuine positive local maxima in the RAW (non-rectified)
    signal. Circularly pads (wraps ~1/6 cycle from each edge) before
    filtering/peak-finding so a real peak straddling the t=0/period
    boundary isn't cut in half and double-counted, then keeps only maxima
    that land back in the true (unpadded) window.

    Smoothing uses a proper 4th-order Butterworth low-pass (zero-phase,
    via filtfilt) at cutoff_mult*f0 -- generous enough to pass the 2nd/3rd
    harmonic of the commanded frequency (real multi-peak content) while
    still attenuating load-cell mechanical ringing/high-frequency noise
    well above any physically-expected harmonic. Replaces a plain moving
    average, which has no defined frequency cutoff and can either blur
    real closely-spaced peaks or let noise through depending on window size.

    prominence_frac (fraction of the full signal range a candidate peak
    must reach) and min_distance_frac (minimum peak separation as a
    fraction of one cycle) were both loosened from stricter defaults
    (0.4 / 0.15) after review -- 0.4 could suppress genuine secondary
    thrust-pulse shoulders, and 0.15 (54 deg of phase) could delete real
    closely-spaced split peaks."""
    from scipy.signal import butter, filtfilt
    n = len(x)
    pad = max(1, n // 6)
    xp = np.concatenate([x[-pad:], x, x[:pad]])

    fs = 1.0 / np.median(np.diff(t)) if len(t) > 1 else None
    if fs and fs > 2 * cutoff_mult * f0:
        nyq = fs / 2.0
        wn = min(cutoff_mult * f0 / nyq, 0.99)
        b, a = butter(4, wn, btype="low")
        xs_ = filtfilt(b, a, xp)
    else:
        win = max(3, len(xp) // 15)
        kernel = np.ones(win) / win
        xs_ = np.convolve(xp, kernel, mode="same")

    lo, hi = xs_.min(), xs_.max()
    rng = hi - lo
    if rng < 1e-9:
        return 0
    thr = hi - prominence_frac * rng
    candidates = [i for i in range(1, len(xs_) - 1)
                  if xs_[i] > xs_[i - 1] and xs_[i] >= xs_[i + 1] and xs_[i] >= thr]
    core = [i for i in candidates if pad <= i < pad + n]
    if not core:
        return 0
    min_dist = max(1, int(min_distance_frac * n))
    core.sort(key=lambda i: -xs_[i])
    kept = []
    for i in core:
        if all(abs(i - j) >= min_dist for j in kept):
            kept.append(i)
    return len(kept)


def _draw_peak_voxels(ax, grid, ph_edges, log_ar_edges, fr_edges, phases,
                      amp_ratios, freq_ratios_locked, label, n_found, n_total):
    color_by_n = {1: "tab:blue", 2: "tab:green", 3: "tab:red"}
    default_color = "0.6"  # 4+

    filled = ~np.isnan(grid)
    facecolors = np.empty(grid.shape, dtype=object)
    for j in range(grid.shape[0]):
        for i in range(grid.shape[1]):
            for k in range(grid.shape[2]):
                if filled[j, i, k]:
                    facecolors[j, i, k] = color_by_n.get(int(grid[j, i, k]), default_color)

    X = np.zeros((len(phases) + 1, len(amp_ratios) + 1, len(freq_ratios_locked) + 1))
    Y = np.zeros_like(X)
    Z = np.zeros_like(X)
    for j in range(len(phases) + 1):
        for i in range(len(amp_ratios) + 1):
            for k in range(len(freq_ratios_locked) + 1):
                X[j, i, k] = ph_edges[j]
                Y[j, i, k] = log_ar_edges[i]
                Z[j, i, k] = fr_edges[k]

    ax.voxels(X, Y, Z, filled, facecolors=facecolors, edgecolors="white", linewidth=0.3, shade=True)

    ax.set_xlabel("Phase Shift  Δφ (deg)")
    ax.set_xticks(range(0, 361, 90))
    ax.set_xlim(0, 360)

    ax.set_ylabel("Amp Ratio  A_r (log scale)")
    ar_ticks = [0.33, 1.0, 3.0]
    ax.set_yticks(np.log10(ar_ticks))
    ax.set_yticklabels([str(v) for v in ar_ticks])

    ax.set_zlabel("Frequency Ratio  f_r")
    ax.set_zticks([0, 1, 2])
    ax.set_zticklabels([str(v) for v in freq_ratios_locked])

    ax.view_init(elev=22, azim=-50)
    ax.set_title(f"{label}  —  {n_found}/{n_total} points populated", fontsize=12)


def part4_peak_count(root, out_dir):
    """3D voxel regime map of Fx and Fy peak counts, computed directly from
    the measured 3D sweep -- NOT simulated. Each voxel is one real mission
    (freq_ratio x amp_ratio x phase), colored by the actual local-maxima
    count on that mission's raw cycle-2 waveform. All 360 sampled grid
    points are drawn at their true measured category; no pattern (stripes,
    corridors, corner clustering) is hand-placed -- whatever regime
    structure appears is exactly what count_force_peaks found."""
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
    from matplotlib.patches import Patch

    freq_ratios_locked = [1.0, 1.5, 2.0]
    amp_ratios = [0.33, 0.67, 1.00, 1.50, 3.00]
    phases = list(range(0, 360, 15))

    n_total, n_found = 0, 0
    grid_fx = np.full((len(phases), len(amp_ratios), len(freq_ratios_locked)), np.nan)
    grid_fy = np.full((len(phases), len(amp_ratios), len(freq_ratios_locked)), np.nan)
    for k, fr in enumerate(freq_ratios_locked):
        for i, ar in enumerate(amp_ratios):
            for j, ph in enumerate(phases):
                n_total += 1
                md = find_mission(root, fr, ar, ph)
                if md is None:
                    continue
                d = load_raw(md)
                if d is None:
                    continue
                t, fx, fy, _ = cycle2_window(d)
                if len(fx) < 20:
                    continue
                grid_fx[j, i, k] = count_force_peaks(t, fx, d["f0"])
                grid_fy[j, i, k] = count_force_peaks(t, fy, d["f0"])
                n_found += 1

    # voxel grid edges: phase (24 wide), amp_ratio (5 deep, log-spaced
    # midpoints between real sampled values), freq_ratio (3 layers, evenly
    # spaced by index since only 3 discrete locked values are sampled)
    ph_edges = np.array(phases + [360]) - 7.5
    ar_arr = np.array(amp_ratios)
    ar_mid = np.sqrt(ar_arr[:-1] * ar_arr[1:])
    ar_edges = np.concatenate([[ar_arr[0] ** 2 / ar_mid[0]], ar_mid, [ar_arr[-1] ** 2 / ar_mid[-1]]])
    log_ar_edges = np.log10(ar_edges)
    fr_edges = np.array([-0.5, 0.5, 1.5, 2.5])  # layer index edges for 3 freq_ratios

    fig = plt.figure(figsize=(22, 11))
    ax_fx = fig.add_subplot(121, projection="3d")
    ax_fy = fig.add_subplot(122, projection="3d")
    _draw_peak_voxels(ax_fx, grid_fx, ph_edges, log_ar_edges, fr_edges, phases,
                      amp_ratios, freq_ratios_locked, "Fx", n_found, n_total)
    _draw_peak_voxels(ax_fy, grid_fy, ph_edges, log_ar_edges, fr_edges, phases,
                      amp_ratios, freq_ratios_locked, "Fy", n_found, n_total)

    legend_elems = [Patch(facecolor="tab:blue", label="1 Peak"),
                    Patch(facecolor="tab:green", label="2 Peaks"),
                    Patch(facecolor="tab:red", label="3 Peaks"),
                    Patch(facecolor="0.6", label="4+ Peaks")]
    ax_fx.legend(handles=legend_elems, loc="upper left", bbox_to_anchor=(0.02, 0.95), fontsize=9)

    fig.suptitle("3D Spatial Volume: Parametric Regime Map of Peak Counts (N_peaks)\n"
                "(measured 3D sweep: f_r x A_r x Δφ)", fontsize=13)
    fig.tight_layout()
    out_path = os.path.join(out_dir, "peak_count_regime_map.png")
    fig.savefig(out_path, dpi=130)
    plt.close(fig)

    with open(os.path.join(out_dir, "peak_count_data.csv"), "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["freq_ratio", "amp_ratio", "phase_deg", "N_peaks_Fx", "N_peaks_Fy"])
        for k, fr in enumerate(freq_ratios_locked):
            for i, ar in enumerate(amp_ratios):
                for j, ph in enumerate(phases):
                    nx = grid_fx[j, i, k]
                    ny = grid_fy[j, i, k]
                    w.writerow([fr, ar, ph,
                               "" if np.isnan(nx) else int(nx),
                               "" if np.isnan(ny) else int(ny)])

    print(f"  [peak_count] wrote {out_path}  ({n_found}/{n_total} voxels populated)")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: prove_claims.py <sweep_root/data> <out_folder>")
    root, out = sys.argv[1], sys.argv[2]

    print("=== Part 1: amplitude claims ===")
    part1_amplitude(root, os.path.join(out, "1_scaling"))

    print("\n=== Part 2: frequency claims ===")
    part2_frequency(root, os.path.join(out, "2_rate"))

    print("\n=== Part 3: phase claims ===")
    part3_phase(root, os.path.join(out, "3_skewness"))

    print("\n=== Part 4: peak count regime map ===")
    part4_peak_count(root, os.path.join(out, "4_peak_count"))

    print(f"\ndone -> {out}")


if __name__ == "__main__":
    main()
