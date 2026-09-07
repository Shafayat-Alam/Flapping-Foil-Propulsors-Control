"""
analysis_common.py — shared analysis for sweep data
===================================================
Reads the per-mission folders written by split_missions.py and turns them into
physical metrics.  Used by propulsion_identifier.py, find_amp.py and find_freq.py.

Two deliberate choices worth knowing:

1. HARMONIC FIT, NOT FFT.  The load cell's per-sample times are reconstructed
   by split_missions.py as `packet_time + i/loadcell_rate`, so samples arrive in
   tight bursts with gaps between packets — badly non-uniform.  An FFT assumes
   uniform sampling and would smear.  Instead we least-squares fit
       y(t) = DC + Σ_k A_k·cos(2πk·f₀·t + φ_k)
   at the COMMANDED frequency f₀.  That is exact for non-uniform samples and
   gives amplitude and phase per harmonic directly.  `dominant_freq()` reports
   the measured fundamental separately, so a wrong loadcell_rate (or a servo not
   tracking the command) shows up as a mismatch instead of silently biasing.

2. CROSS-STREAM PHASE USES bag_time_s.  Feedback and load-cell CSVs each zero
   their own `time_s` at their own first row, so the two streams' origins differ
   by a few ms.  `bag_time_s` is the shared absolute clock — the only sound basis
   for "force phase relative to motion".
"""

import os, csv, glob, math
import numpy as np

# np.trapz was renamed np.trapezoid in numpy 2.0; support both.
_trapz = getattr(np, "trapezoid", None) or np.trapz

IGNORE_CYCLES = 3      # procedure: 10 cycles per command, ignore the first 3

FORCE_AXES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]

# --- classification thresholds (Fx 2nd-harmonic / 1st-harmonic ratio) -------
# Ideal drag-based Fx is one thrust pulse per cycle then ~0 → energy sits at 1f.
# Ideal lift-based Fx is two symmetric peaks per cycle → energy sits at 2f, and
# a perfectly symmetric pair cancels the fundamental entirely (r21 → ∞).
R21_LIFT = 1.5         # r21 above this  → lift-based
R21_DRAG = 0.67        # r21 below this  → drag-based
DEAD_FRAC_DRAG = 0.30  # ≥30% of the cycle spent near zero Fx supports drag
PEAK_ASYM_MAX = 0.35   # |p1-p2|/(p1+p2) above this → peaks not symmetric

THD_SMOOTH = 0.15      # position THD above this = "sharp", not a smooth sine
TRACK_MIN = 0.90       # achieved/commanded amplitude below this = clipping

# --- jagged-force rejection ------------------------------------------------
# "Reject runs where the harmonics carry significant energy vs the fundamental"
# CANNOT be applied to 2f: for lift-based propulsion the 2nd harmonic IS the
# thrust signal (two peaks per cycle), and a symmetric pair cancels the
# fundamental outright — such a rule would reject every good lift-based run.
# Real hydrodynamic thrust is smooth at 1f and/or 2f; mechanical jaggedness
# shows up at 3f and ABOVE, plus broadband (non-harmonic) residual.  So the
# gate compares that high/incoherent energy against the COHERENT signal
# (1f and 2f together).
NOISE_REJECT = 0.50    # sqrt(Σ_{k≥3}A_k² + resid²) / sqrt(A_1²+A_2²) above this = jagged
EFF_BAND = 0.90        # "still near-best efficiency" = ≥90% of the peak


# ===========================================================================
# Loading
# ===========================================================================
def _read(path):
    try:
        with open(path) as f:
            return list(csv.DictReader(f))
    except OSError:
        return []


def _col(rows, key):
    """Column as float array with non-numeric entries dropped (paired mask)."""
    out, keep = [], []
    for r in rows:
        try:
            out.append(float(r.get(key, "")))
            keep.append(True)
        except (TypeError, ValueError):
            keep.append(False)
    return np.asarray(out, float), np.asarray(keep, bool)


def mission_label(mdir):
    for f in glob.glob(os.path.join(mdir, "*.csv")):
        b = os.path.basename(f)
        if not b.endswith("_loadcell.csv"):
            return b[:-4]
    return None


def find_missions(root):
    """[(label, dir)] for a sweep folder, or a single mission folder."""
    if mission_label(root):
        return [(mission_label(root), root)]
    out = []
    for d in sorted(glob.glob(os.path.join(root, "*"))):
        if os.path.isdir(d) and os.path.basename(d) != "raw":
            lab = mission_label(d)
            if lab:
                out.append((lab, d))
    return out


def _cmd(fb, key, default=None):
    if not fb:
        return default
    v = fb[0].get(f"cmd.{key}", "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load_mission(mdir, label):
    """
    Everything one mission needs for analysis, already windowed to the steady
    cycles (IGNORE_CYCLES..cycles).  Returns None if the mission has no feedback.
    """
    fb = _read(os.path.join(mdir, f"{label}.csv"))
    lc = _read(os.path.join(mdir, f"{label}_loadcell.csv"))
    if not fb:
        return None

    freq = _cmd(fb, "frequency")
    if not freq or freq <= 1e-6:
        freq = _cmd(fb, "frequency", 0.0)
    cycles = _cmd(fb, "cycles", 10.0) or 10.0

    m = {
        "label": label, "dir": mdir,
        "frequency": freq,
        "pitch_amp": _cmd(fb, "pitch_amp"),
        "heave_amp": _cmd(fb, "heave_amp"),
        "phase": _cmd(fb, "phase"),
        "cycles": cycles,
        "has_force": False,
    }

    # ---- steady window on each stream, in its own time base ----------------
    period = 1.0 / freq if freq and freq > 1e-6 else 0.0
    t_lo, t_hi = IGNORE_CYCLES * period, cycles * period

    t_fb, keep = _col(fb, "time_s")
    fbw = [r for r, k in zip(fb, keep) if k]
    bag_fb, _ = _col(fbw, "bag_time_s")
    sel = (t_fb >= t_lo) & (t_fb <= t_hi) if period else np.ones(len(t_fb), bool)
    m["t_fb"] = t_fb[sel]
    m["bag_fb"] = bag_fb[sel] if len(bag_fb) == len(t_fb) else m["t_fb"]
    for sid, name in ((1, "pitch"), (2, "heave")):
        for src, dst in (("position_rad", "pos"), ("velocity_rps", "vel"),
                         ("current_a", "cur"), ("voltage_v", "volt")):
            v, k2 = _col(fbw, f"s{sid}_{src}")
            m[f"{name}_{dst}"] = v[sel] if len(v) == len(t_fb) else np.array([])

    # ---- forces ------------------------------------------------------------
    if lc:
        t_lc, keepl = _col(lc, "time_s")
        lcw = [r for r, k in zip(lc, keepl) if k]
        bag_lc, _ = _col(lcw, "bag_time_s")
        force_all = {}
        for ax in FORCE_AXES:
            v, _k = _col(lcw, ax)
            force_all[ax] = v if len(v) == len(t_lc) else np.array([])

        # ---- TARE: subtract the static baseline ----------------------------
        # The load cell reads the flipper's weight/buoyancy + mount bias on top
        # of the hydrodynamic force.  After the gait's integer cycles the flipper
        # returns to centre and holds still through the inter-mission delay, so
        # the median force over that at-rest tail is the static baseline; we
        # subtract it per axis so only the hydrodynamic force is analysed.
        gait_end = cycles / freq if freq and freq > 1e-6 else 0.0
        tare = {ax: 0.0 for ax in FORCE_AXES}
        if gait_end > 0 and t_lc.size:
            rest = t_lc > (gait_end + 1.0)
            if rest.sum() > 20:
                for ax in FORCE_AXES:
                    if force_all[ax].size == t_lc.size:
                        tare[ax] = float(np.median(force_all[ax][rest]))
        m["tare"] = tare

        sell = (t_lc >= t_lo) & (t_lc <= t_hi) if period else np.ones(len(t_lc), bool)
        m["t_lc"] = t_lc[sell]
        m["bag_lc"] = bag_lc[sell] if len(bag_lc) == len(t_lc) else m["t_lc"]
        for ax in FORCE_AXES:
            fa = force_all[ax]
            m[ax] = (fa[sell] - tare[ax]) if fa.size == t_lc.size else np.array([])
        m["has_force"] = len(m["t_lc"]) > 20 and m["Fx"].size > 20
    else:
        m["t_lc"] = np.array([])
        for ax in FORCE_AXES:
            m[ax] = np.array([])
    return m


# ===========================================================================
# Harmonic machinery
# ===========================================================================
def harmonic_fit(t, y, f0, n_harm=5):
    """
    Least-squares fit y(t) = DC + Σ A_k cos(2πk f0 t + φ_k), k=1..n_harm.
    Exact for non-uniformly sampled data.  Returns dict with dc, amp[], phase[],
    resid_rms and thd (Σ_{k≥2} A_k² normalized by A_1).
    """
    t = np.asarray(t, float); y = np.asarray(y, float)
    if t.size < 2 * n_harm + 4 or f0 is None or f0 <= 1e-9 or not np.isfinite(y).all():
        return None
    cols = [np.ones_like(t)]
    for k in range(1, n_harm + 1):
        w = 2 * np.pi * k * f0 * t
        cols += [np.cos(w), np.sin(w)]
    A = np.column_stack(cols)
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    amp, ph = [], []
    for k in range(n_harm):
        a, b = coef[1 + 2 * k], coef[2 + 2 * k]
        amp.append(float(math.hypot(a, b)))
        ph.append(float(math.atan2(-b, a)))       # y ≈ A cos(wt + φ)
    resid = y - A @ coef
    thd = (math.sqrt(sum(a * a for a in amp[1:])) / amp[0]
           if amp[0] > 1e-12 else float("nan"))
    return {"dc": float(coef[0]), "amp": amp, "phase": ph,
            "resid_rms": float(np.sqrt(np.mean(resid ** 2))), "thd": float(thd)}


def dominant_freq(t, y, f_lo=0.05, f_hi=5.0, n=400):
    """
    Measured fundamental, by scanning a 1-harmonic fit over a frequency grid and
    taking the best-explaining frequency.  Independent of the commanded value —
    a mismatch flags a bad loadcell_rate or a servo not following the command.
    """
    t = np.asarray(t, float); y = np.asarray(y, float)
    if t.size < 16:
        return float("nan")
    y = y - y.mean()
    best, best_f = -1.0, float("nan")
    for f in np.linspace(f_lo, f_hi, n):
        w = 2 * np.pi * f * t
        A = np.column_stack([np.ones_like(t), np.cos(w), np.sin(w)])
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
        power = coef[1] ** 2 + coef[2] ** 2
        if power > best:
            best, best_f = power, f
    return float(best_f)


def fold_cycle(t, y, f0, nbins=72):
    """
    Phase-average every steady cycle onto one clean cycle.  This is what makes
    peak counting trustworthy: per-cycle noise averages down, the repeatable
    shape survives.  Returns (phase 0..1, mean, std, n_per_bin).
    """
    t = np.asarray(t, float); y = np.asarray(y, float)
    if t.size == 0 or not f0 or f0 <= 1e-9:
        return None
    ph = np.mod(t * f0, 1.0)
    idx = np.clip((ph * nbins).astype(int), 0, nbins - 1)
    mean = np.full(nbins, np.nan); std = np.zeros(nbins); cnt = np.zeros(nbins, int)
    for b in range(nbins):
        v = y[idx == b]
        cnt[b] = v.size
        if v.size:
            mean[b] = v.mean(); std[b] = v.std()
    # interpolate any empty bin so downstream peak finding sees a closed curve
    if np.isnan(mean).any():
        good = ~np.isnan(mean)
        if good.sum() < 4:
            return None
        centers = (np.arange(nbins) + 0.5) / nbins
        mean = np.interp(centers, centers[good], mean[good], period=1.0)
    return (np.arange(nbins) + 0.5) / nbins, mean, std, cnt


def _wrapped_peaks(y, prominence_frac=0.15):
    """Peaks of a cyclic curve (tile ×3, keep the middle) so a peak sitting on
    the wrap point isn't missed."""
    from scipy.signal import find_peaks
    n = y.size
    span = np.ptp(y)
    if span <= 1e-12:
        return np.array([], int), {}
    tiled = np.tile(y, 3)
    pk, props = find_peaks(tiled, prominence=prominence_frac * span)
    keep = (pk >= n) & (pk < 2 * n)
    return pk[keep] - n, {k: v[keep] for k, v in props.items()}


# ===========================================================================
# Propulsion style
# ===========================================================================
def classify_propulsion(m):
    """
    Drag-based vs lift-based from the Fx signature over one folded cycle.

      drag : one thrust peak per cycle, then ~0 for the rest  → energy at 1f
      lift : two symmetric peaks per cycle (maybe troughs)    → energy at 2f

    r21 = A(2f)/A(1f) is the primary discriminator (a symmetric peak pair
    cancels the fundamental outright).  Peak count on the folded cycle and the
    near-zero "dead" fraction corroborate it, and break ties in the grey band.
    """
    out = {"style": "unknown", "reason": "", "confidence": 0.0}
    if not m["has_force"]:
        out["reason"] = "no load-cell data"
        return out
    f0 = m["frequency"]
    fit = harmonic_fit(m["t_lc"], m["Fx"], f0)
    fold = fold_cycle(m["t_lc"], m["Fx"], f0)
    if fit is None or fold is None:
        out["reason"] = "too few force samples to fit"
        return out

    ph, mean, std, cnt = fold
    a1, a2 = fit["amp"][0], fit["amp"][1]
    r21 = a2 / a1 if a1 > 1e-9 else float("inf")

    peaks, props = _wrapped_peaks(mean)
    pk_vals = np.sort(mean[peaks])[::-1] if peaks.size else np.array([])
    n_peaks = int(peaks.size)

    amax = np.max(np.abs(mean)) if mean.size else 0.0
    dead = float(np.mean(np.abs(mean) < 0.10 * amax)) if amax > 1e-12 else float("nan")

    peak_asym = float("nan")
    if pk_vals.size >= 2 and (pk_vals[0] + pk_vals[1]) != 0:
        peak_asym = abs(pk_vals[0] - pk_vals[1]) / abs(pk_vals[0] + pk_vals[1])

    # --- decide ---
    if r21 >= R21_LIFT and n_peaks >= 2:
        style, conf = "lift", min(1.0, (r21 - R21_LIFT) / R21_LIFT + 0.5)
        reason = f"2f dominates (r21={r21:.2f}) with {n_peaks} peaks/cycle"
    elif r21 <= R21_DRAG and (n_peaks <= 1 or dead >= DEAD_FRAC_DRAG):
        style, conf = "drag", min(1.0, (R21_DRAG - r21) / R21_DRAG + 0.5)
        reason = (f"1f dominates (r21={r21:.2f}), {n_peaks} peak/cycle, "
                  f"{dead*100:.0f}% of cycle near zero")
    else:
        # grey band — corroborating evidence decides, with low confidence
        if n_peaks >= 2 and not (peak_asym > PEAK_ASYM_MAX) and dead < DEAD_FRAC_DRAG:
            style, conf, reason = "lift", 0.35, f"ambiguous r21={r21:.2f}; 2 peaks, low dead fraction"
        elif n_peaks <= 1 and dead >= DEAD_FRAC_DRAG:
            style, conf, reason = "drag", 0.35, f"ambiguous r21={r21:.2f}; single peak, high dead fraction"
        else:
            style, conf, reason = "ambiguous", 0.0, f"r21={r21:.2f} in grey band, peaks={n_peaks}"

    if style == "lift" and peak_asym == peak_asym and peak_asym > PEAK_ASYM_MAX:
        conf *= 0.5
        reason += f" (peaks asymmetric {peak_asym:.2f})"

    out.update({"style": style, "confidence": float(conf), "reason": reason,
                "r21": float(r21), "n_peaks": n_peaks, "dead_frac": dead,
                "peak_asym": peak_asym,
                "fx_a1": a1, "fx_a2": a2, "fx_dc": fit["dc"],
                "fx_phase1": fit["phase"][0], "fx_phase2": fit["phase"][1],
                "fold": (ph, mean, std)})
    return out


def phase_vs_motion(m):
    """
    Phase of each force's sine curve, measured relative to the PITCH motion.

    Cross-stream phase must be computed on `bag_time_s`: the feedback and
    load-cell CSVs zero their own `time_s` independently, so using `time_s`
    would fold their few-ms origin offset straight into the answer.

    Returns radians in (-π, π]: force lag behind pitch, at 1f (and 2f for Fx,
    which is where lift-based thrust lives).
    """
    out = {}
    f0 = m["frequency"]
    pos = m.get("pitch_pos")
    if pos is None or pos.size < 8 or not m["has_force"]:
        return out
    ref = harmonic_fit(m["bag_fb"], pos, f0)
    if ref is None:
        return out
    out["pitch_phase_abs"] = ref["phase"][0]

    def _wrap(a):
        return (a + math.pi) % (2 * math.pi) - math.pi

    for ax in ("Fx", "Fy", "Fz"):
        if m[ax].size < 8:
            continue
        fit = harmonic_fit(m["bag_lc"], m[ax], f0)
        if fit is None:
            continue
        out[f"{ax}_phase1"] = _wrap(fit["phase"][0] - ref["phase"][0])
        out[f"{ax}_amp1"] = fit["amp"][0]
        if ax == "Fx":
            # 2f phase referenced to 2× the pitch phase (the lift-based harmonic)
            out["Fx_phase2"] = _wrap(fit["phase"][1] - 2 * ref["phase"][0])
            out["Fx_amp2"] = fit["amp"][1]
    return out


def force_quality(m):
    """
    The invariants that must hold regardless of style:
      Fz symmetric peak/trough, net Fy ≈ 0.
    fz_asym = |peak+trough| / (peak-trough): 0 = perfectly symmetric.
    """
    q = {}
    if not m["has_force"]:
        return q
    f0 = m["frequency"]
    fz = m["Fz"]
    fold = fold_cycle(m["t_lc"], fz, f0)
    if fold is not None:
        _, mz, _, _ = fold
        pk, tr = float(np.max(mz)), float(np.min(mz))
        q["fz_peak"], q["fz_trough"] = pk, tr
        q["fz_asym"] = abs(pk + tr) / (pk - tr) if (pk - tr) > 1e-12 else float("nan")
    q["fz_net"] = float(np.mean(fz)) if fz.size else float("nan")
    q["fy_net"] = float(np.mean(m["Fy"])) if m["Fy"].size else float("nan")
    q["fy_mean_abs"] = float(np.mean(np.abs(m["Fy"]))) if m["Fy"].size else float("nan")
    q["mean_Fx"] = float(np.mean(m["Fx"]))
    q["peak_Fx"] = float(np.max(m["Fx"]))
    # normalize the lateral leak against the thrust it comes with
    q["fy_ratio"] = (abs(q["fy_net"]) / abs(q["mean_Fx"])
                     if abs(q.get("mean_Fx", 0.0)) > 1e-9 else float("nan"))
    return q


# ===========================================================================
# Servo-side metrics: tracking, smoothness, efficiency
# ===========================================================================
def servo_metrics(m):
    """
    Per-axis achieved amplitude, tracking ratio vs command, and THD of the
    position curve.  THD is the "sharpness" measure: a clean sine tracked well
    has THD ≈ 0; slew clipping or a resonance injects harmonics and drives it up.
    """
    s = {}
    f0 = m["frequency"]
    for name, cmd_key in (("pitch", "pitch_amp"), ("heave", "heave_amp")):
        pos, cur = m.get(f"{name}_pos"), m.get(f"{name}_cur")
        if pos is None or pos.size < 8:
            continue
        achieved = float((np.max(pos) - np.min(pos)) / 2.0)
        s[f"{name}_achieved_amp"] = achieved
        cmd = m.get(cmd_key)
        if cmd and cmd > 1e-9:
            s[f"{name}_track"] = achieved / cmd
        fit = harmonic_fit(m["t_fb"], pos, f0)
        if fit:
            s[f"{name}_thd"] = fit["thd"]
            s[f"{name}_fit_amp"] = fit["amp"][0]
        if cur is not None and cur.size:
            s[f"{name}_mean_current"] = float(np.mean(np.abs(cur)))
            cfit = harmonic_fit(m["t_fb"], np.abs(cur), f0)
            if cfit:
                s[f"{name}_current_thd"] = cfit["thd"]
    # total current drawn by the propulsor: both servos share the supply, so the
    # physically meaningful "current drawn" is their sum, not their average.
    s["total_current"] = sum(s.get(f"{n}_mean_current", 0.0)
                             for n in ("pitch", "heave"))
    s["max_thd"] = max([s[k] for k in s if k.endswith("_thd")], default=float("nan"))
    s["smooth"] = (s["max_thd"] <= THD_SMOOTH) if s["max_thd"] == s["max_thd"] else False
    tracks = [s[k] for k in s if k.endswith("_track")]
    s["min_track"] = min(tracks) if tracks else float("nan")
    return s


def spectral_quality(m):
    """
    Harmonic decomposition of Fx, used to reject JAGGED runs (mechanical
    failure) while preserving lift-based thrust.

    Computed by least-squares harmonic fit rather than a literal FFT: it yields
    the same per-harmonic energies an FFT would, but is valid for the load
    cell's bursty, non-uniform sample times (see the module docstring).  The
    broadband part an FFT would spread across its noise floor appears here as
    the fit residual.

      coherent  = sqrt(A_1² + A_2²)        the real thrust signal (1f drag, 2f lift)
      jagged    = sqrt(Σ_{k≥3}A_k² + resid²)   high harmonics + broadband
      noise_ratio = jagged / coherent      → reject above NOISE_REJECT
    """
    out = {}
    if not m["has_force"]:
        return out
    fit = harmonic_fit(m["t_lc"], m["Fx"], m["frequency"], n_harm=6)
    if fit is None:
        return out
    a = fit["amp"]
    coherent = math.sqrt(a[0] ** 2 + a[1] ** 2)
    high = math.sqrt(sum(v * v for v in a[2:]))
    jagged = math.sqrt(high ** 2 + fit["resid_rms"] ** 2)
    out["fx_coherent"] = coherent
    out["fx_high_harm"] = high
    out["fx_broadband"] = fit["resid_rms"]
    out["noise_ratio"] = jagged / coherent if coherent > 1e-9 else float("nan")
    out["fx_harm_amps"] = a
    return out


def cycle_integrals(m):
    """
    Per-cycle impulse and energy — thrust as an integral, not a peak.

      impulse/cycle = ∫Fx dt over one cycle  [N·s]
      energy/cycle  = ∫(V·I)_pitch + (V·I)_heave dt over one cycle  [J]

    Both are obtained as (mean × period).  The mean comes from the harmonic
    fit's DC term, not np.mean(): with bursty sampling, samples cluster at some
    cycle phases more than others, so a plain average is phase-biased while the
    fit's DC term is not.  `impulse_trapz` integrates the raw samples directly
    as an independent cross-check — if the two disagree, the sample timing is
    suspect.

    NOTE ON RANKING: impulse/cycle = mean_Fx / f.  At FIXED frequency (the
    amplitude sweep) it ranks identically to mean Fx — the 1/f is a constant.
    It only changes the ordering ACROSS frequencies.
    """
    out = {}
    f0 = m["frequency"]
    if not f0 or f0 <= 1e-9:
        return out
    T = 1.0 / f0

    if m["has_force"]:
        fit = harmonic_fit(m["t_lc"], m["Fx"], f0)
        mean_fx = fit["dc"] if fit else float(np.mean(m["Fx"]))
        out["mean_Fx_fit"] = float(mean_fx)
        out["impulse_per_cycle"] = float(mean_fx * T)          # N·s
        # independent check: integrate the raw samples over whole cycles
        t, fx = m["t_lc"], m["Fx"]
        if t.size > 4:
            span = t[-1] - t[0]
            n_cyc = span * f0
            if n_cyc > 0.5:
                out["impulse_trapz"] = float(_trapz(fx, t) / n_cyc)

    # ---- electrical cost ----
    # ∫I dt is CHARGE (coulombs), not energy.  Energy needs ∫V·I dt — and the
    # voltage is recorded, so the true joules are computed here.  The charge
    # form is kept too, since it is what was specified.
    pw = np.zeros(0)
    for name in ("pitch", "heave"):
        cur, volt = m.get(f"{name}_cur"), m.get(f"{name}_volt")
        if cur is None or cur.size == 0:
            continue
        i = np.abs(cur)
        v = volt if (volt is not None and volt.size == i.size) else np.full(i.size, 12.0)
        p = i * v
        out[f"{name}_mean_current"] = float(np.mean(i))
        out[f"{name}_mean_power"] = float(np.mean(p))
        out[f"{name}_charge_per_cycle"] = float(np.mean(i) * T)      # C
        out[f"{name}_energy_per_cycle"] = float(np.mean(p) * T)      # J
        pw = p if pw.size == 0 else (pw + p if pw.size == p.size else pw)
    if pw.size:
        out["mean_power"] = float(np.mean(pw))
        out["energy_per_cycle"] = float(np.mean(pw) * T)             # J, both servos
    return out


def efficiency(m, q=None, s=None, ci=None):
    """
    Propulsion efficiency = net forward impulse / electrical cost.

        efficiency_energy = ∫Fx dt / ∫(V·I) dt   [N·s/J = N/W]   ← primary
        efficiency_charge = ∫Fx dt / ∫I dt       [N·s/C = N/A]

    Returns a dict of variants.  Three things worth knowing:

    * The integration window CANCELS: ∫Fx dt / ∫I dt = (mean_Fx·W)/(mean_I·W)
      = mean_Fx / mean_I.  Integrating per-cycle vs over the whole steady window
      gives the identical number — and it equals the mean-thrust/mean-current
      form.  So this ratio is frequency-independent by construction.
    * BOTH servos are charged for by default.  Pitch draws real current; billing
      only heave would credit the propulsor for energy it actually spent.  The
      heave-only variant is reported alongside.
    * ∫V·I dt is the true joules; ∫I dt is coulombs.  They differ whenever the
      bus voltage sags — which is exactly when the servo is working hardest.
    """
    q = q if q is not None else force_quality(m)
    ci = ci if ci is not None else cycle_integrals(m)
    out = {}
    imp = ci.get("impulse_per_cycle")
    if not m["has_force"] or imp is None:
        return out
    e = ci.get("energy_per_cycle", 0.0)
    if e > 1e-9:
        out["efficiency_energy"] = imp / e                   # N·s/J
    tot_q = sum(ci.get(f"{n}_charge_per_cycle", 0.0) for n in ("pitch", "heave"))
    if tot_q > 1e-9:
        out["efficiency_charge"] = imp / tot_q               # N·s/C
    hq = ci.get("heave_charge_per_cycle", 0.0)
    if hq > 1e-9:
        out["efficiency_charge_heave"] = imp / hq            # as literally specified
    he = ci.get("heave_energy_per_cycle", 0.0)
    if he > 1e-9:
        out["efficiency_energy_heave"] = imp / he
    return out


# ===========================================================================
# Gates — shared by find_amp.py and find_freq.py.
# Each returns (ok, reason).  A sample must pass every gate to be selectable:
# forces that violate the invariants, strokes the servo never reached, and
# jagged/sharp runs are not real datapoints.
# ===========================================================================
def quality_ok(r):
    """The force invariants that must hold regardless of amplitude/frequency."""
    if not r.get("has_force"):
        return False, "no load-cell data"
    if r.get("fz_asym", 0) > 0.25:
        return False, f"Fz asymmetric ({r['fz_asym']:.2f})"
    if abs(r.get("fy_ratio", 0)) > 0.25:
        return False, f"net Fy leak ({r['fy_ratio']:.2f} of Fx)"
    return True, ""


def tracking_ok(r):
    """A stroke the servo never reached belongs to a smaller stroke than commanded."""
    t = r.get("min_track", float("nan"))
    if t != t:
        return True, ""
    if t < TRACK_MIN:
        return False, f"servo tracked only {t:.0%} of commanded stroke"
    return True, ""


def smooth_ok(r):
    """Position curve sharpness — sharp edges suggest resonance or clipping."""
    thd = r.get("max_thd", float("nan"))
    if thd != thd:
        return True, ""
    if thd > THD_SMOOTH:
        return False, f"curve sharp (THD {thd:.2f} > {THD_SMOOTH:.2f})"
    return True, ""


def force_smooth_ok(r):
    """Jagged-force rejection: high harmonics (3f+) and broadband vs the coherent
    1f/2f thrust signal.  Deliberately does NOT penalize 2f — that is lift-based
    thrust, not noise."""
    nr = r.get("noise_ratio", float("nan"))
    if nr != nr:
        return True, ""
    if nr > NOISE_REJECT:
        return False, (f"force jagged (3f+/broadband {nr:.2f} > {NOISE_REJECT:.2f} "
                       f"of coherent signal)")
    return True, ""


def analyze(mdir, label):
    """One mission → every metric, flattened."""
    m = load_mission(mdir, label)
    if m is None:
        return None
    q, s = force_quality(m), servo_metrics(m)
    ci = cycle_integrals(m)
    sq = spectral_quality(m)
    cls = classify_propulsion(m)
    r = {"label": label, "dir": mdir, "frequency": m["frequency"],
         "pitch_amp": m["pitch_amp"], "heave_amp": m["heave_amp"],
         "phase": m["phase"], "has_force": m["has_force"]}
    r.update(q); r.update(s); r.update(ci); r.update(sq)
    r.update(efficiency(m, q, s, ci))
    r["style"] = cls["style"]; r["confidence"] = cls["confidence"]
    r["reason"] = cls["reason"]
    for k in ("r21", "n_peaks", "dead_frac", "peak_asym", "fx_a1", "fx_a2",
              "fx_phase1", "fx_phase2"):
        if k in cls:
            r[k] = cls[k]
    # primary efficiency figure of merit: impulse per joule, both servos billed
    r["efficiency"] = r.get("efficiency_energy", float("nan"))
    r["jagged"] = (r.get("noise_ratio", 0.0) > NOISE_REJECT
                   if r.get("noise_ratio") == r.get("noise_ratio") else False)
    r["_mission"] = m
    r["_cls"] = cls
    return r
