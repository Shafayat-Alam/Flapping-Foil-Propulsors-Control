#!/usr/bin/env python3
"""Harmonic run-to-run force controller: SEED -> MATCH -> OPTIMISE.

WHY THIS REPLACES THE OLD CONTROLLER
------------------------------------
The previous version tuned five scalars (amp_ratio, freq_ratio, delta_phi,
scale, freq_scale) against a whole-waveform correlation score. That failed
for a structural reason, not a tuning one:

  * waveform_match is a Pearson correlation over the resampled cycle. It
    has no derivative with respect to any single parameter, so it could not
    carry a gain. During the MATCH stage every other objective was deferred,
    which left the controller with no gain pointing anywhere -- it emitted
    an all-zero signal vector and the parameters sat frozen at the seed for
    an entire run while the score rattled on measurement noise alone.
  * Being scale-invariant, that same score reported 0.72 "match" on a run
    producing 0.71 N against a 9 N target. Shape was being optimised while
    thrust collapsed, and nothing in the objective set noticed.

This version fixes both by controlling in DESCRIPTOR SPACE. The target curve
is reduced to five numbers with physical meaning, each of which has its own
knob and its own gain. Shape error is then a vector of scalar errors, every
one of which is directly actionable -- so MATCH is a decoupled controller,
not a search, and magnitude is one of the matched quantities rather than an
afterthought.

  descriptor        knob          basis
  ----------------- ------------- --------------------------------------
  peak_count        freq_ratio    measured, R^2 0.67, sharp thresholds
  peak magnitude    freq_scale    F ~ v^2, analytic
  +peak/-peak       a2            predicted (even harmonic breaks the
                                  half-wave symmetry that locks the lobes)
  lobe dominance    phi2          predicted, ratio inverts across phase
  skew              phi2          predicted
  lobe width        a3            predicted (odd harmonic, width only)
  off-channel null  amp_ratio     measured, 7.7:1 selectivity for Fy

STATUS OF THE RELATIONSHIPS. peak_count, magnitude-vs-amp_ratio and the
Fy/Fz trims are measured over 860 missions. The harmonic entries (a2, phi2,
a3) are PREDICTED -- no mission has ever commanded an even harmonic -- from
a symmetry argument that holds under linear, quadratic and cubic force laws
alike. They are encoded here as seedable gains so the controller starts
informed, and every one of them is re-identified online from the run's own
measurements (see GainAdapter), so a wrong prediction is corrected by data
rather than believed indefinitely.

usage: force_control_harmonic.py <folder> [--hardware] [--max-cycles=N]
"""
from __future__ import annotations

import glob
import json
import math
import os
import sys
import time
from dataclasses import dataclass, replace, asdict

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"

# ----------------------------------------------------------------- limits
SLEW_LIMIT = 5.5           # rad/s per servo
PITCH_LIMIT = math.pi      # rad
HEAVE_LIMIT = math.pi / 2  # rad

CENTER_AMP = 0.6283185307  # rad, geometric mean (matches the 3D campaign)
CENTER_FREQ = 0.5          # Hz

N_CYCLES = 3               # commanded per measurement; cycle 2 is analysed
SETTLE_S = 4.0             # quiet delay between 3-cycle sets
IDLE_TAIL_S = 2.0
FS_LOADCELL = 10000.0
CUTOFF_MULT = 10.0         # low-pass at 10x pitch freq (kills ~29 Hz resonance)

MAX_ITERS = 60
MATCH_PATIENCE = 8
OPT_PATIENCE = 10

# Parameter bounds. a2/a3 are RELATIVE harmonic amplitudes (fractions of the
# fundamental); a2 is capped at 0.35 because the predicted +/- ratio peaks
# near a2=0.3 and then REVERSES (3.92 at 0.3, 3.16 at 0.5), so letting the
# controller push past the turning point would make the gain sign a lie.
BOUNDS = {
    "amp_ratio":  (0.33, 3.00),
    "freq_ratio": (0.35, 2.50),
    "freq_scale": (0.50, 3.00),
    "a2":         (0.00, 0.35),
    "phi2":       (0.0, 360.0),   # circular
    "a3":         (0.00, 0.30),
}


# =========================================================================
# 1. PARAMETERS AND KINEMATICS
# =========================================================================
@dataclass
class Params:
    """The tunable gait. `channel` selects which servo carries the harmonics:
    'Fx' puts them on pitch, 'Fy' on heave (the rig is underactuated, so
    only one channel is shaped at a time -- see the module docstring)."""
    amp_ratio: float = 1.0
    freq_ratio: float = 0.40
    freq_scale: float = 1.0
    a2: float = 0.0
    phi2: float = 0.0
    a3: float = 0.0
    channel: str = "Fx"

    def clipped(self) -> "Params":
        d = asdict(self)
        for k, (lo, hi) in BOUNDS.items():
            if k == "phi2":
                d[k] = d[k] % 360.0
            else:
                d[k] = float(np.clip(d[k], lo, hi))
        return Params(**d)

    def vector(self):
        return np.array([self.amp_ratio, self.freq_ratio, self.freq_scale,
                         self.a2, self.phi2, self.a3], dtype=float)


TUNABLE = ("amp_ratio", "freq_ratio", "freq_scale", "a2", "phi2", "a3")


def beat_period(p: Params):
    """Period over which the COMBINED pitch+heave gait actually repeats.

    NOT 1/min(f_pitch, f_heave): with freq_ratio = a/b in lowest terms the
    motion only comes back to the same state after b pitch cycles. At
    freq_ratio 0.4 = 2/5 that is 5 pitch cycles, and using the naive value
    puts ~2.5 pitch cycles inside the "one cycle" analysis window -- which
    reports 3 force peaks where the gait produces 1, and then no amount of
    tuning can drive peak_count to its target because the target is being
    measured wrong. (This is the same error that corrupted the first pass
    of the 3D re-analysis.)
    """
    from fractions import Fraction
    _, _, pf, _ = kinematics(p)
    b = Fraction(p.freq_ratio).limit_denominator(20).denominator
    return b / pf


def kinematics(p: Params):
    """Geometric-mean-preserving decode -> (pitch_amp, heave_amp, f_pitch, f_heave)."""
    s_a, s_f = math.sqrt(p.amp_ratio), math.sqrt(p.freq_ratio)
    return (CENTER_AMP * s_a, CENTER_AMP / s_a,
            CENTER_FREQ * p.freq_scale / s_f, CENTER_FREQ * p.freq_scale * s_f)


def composite_cycle(a2, phi2, a3, n=2000):
    """One period of fundamental + 2nd + 3rd harmonic, normalised to peak 1.

    Normalisation is what keeps SHAPE independent of SIZE: without it,
    raising a2 would also raise the stroke amplitude, and the controller
    could not tell a shape correction from a magnitude one.
    """
    t = np.arange(n) / n
    w = 2 * np.pi
    th = (np.sin(w * t)
          + a2 * np.sin(2 * w * t + math.radians(phi2))
          + a3 * np.sin(3 * w * t))
    pk = float(np.max(np.abs(th)))
    return t, th / (pk if pk > 1e-9 else 1.0)


def peak_velocity(a2, phi2, a3, amp_rad, freq_hz):
    t, th = composite_cycle(a2, phi2, a3)
    return float(np.max(np.abs(np.gradient(th, t) * amp_rad * freq_hz)))


def slew_headroom(p: Params):
    """(pitch_vpk, heave_vpk, max_freq_scale_allowed)."""
    pa, ha, pf, hf = kinematics(p)
    on_pitch = (p.channel == "Fx")
    pv = peak_velocity(p.a2 if on_pitch else 0.0, p.phi2, p.a3 if on_pitch else 0.0, pa, pf)
    hv = peak_velocity(0.0 if on_pitch else p.a2, p.phi2, 0.0 if on_pitch else p.a3, ha, hf)
    worst = max(pv, hv)
    fs_max = p.freq_scale * SLEW_LIMIT / worst if worst > 1e-9 else BOUNDS["freq_scale"][1]
    return pv, hv, fs_max


def enforce_limits(p: Params, log=None) -> Params:
    """Pull freq_scale down until both servos are inside the slew limit.

    Frequency is the right thing to give up because it is the magnitude
    knob: reducing it costs force but leaves every shape descriptor exactly
    where the controller put it. Clipping the waveform instead would deform
    the shape the MATCH stage is working to establish.
    """
    p = p.clipped()
    pv, hv, fs_max = slew_headroom(p)
    if max(pv, hv) > SLEW_LIMIT:
        new_fs = max(BOUNDS["freq_scale"][0], min(p.freq_scale, fs_max * 0.98))
        if log:
            log(f"    [slew] pitch {pv:.2f} heave {hv:.2f} rad/s > {SLEW_LIMIT} "
                f"-> freq_scale {p.freq_scale:.3f} -> {new_fs:.3f}")
        p = replace(p, freq_scale=new_fs).clipped()
    pa, ha, _, _ = kinematics(p)
    if pa > PITCH_LIMIT or ha > HEAVE_LIMIT:
        # amp_ratio is the only knob that moves the two amplitudes in
        # opposite directions, so it is what resolves an amplitude breach.
        while ha > HEAVE_LIMIT and p.amp_ratio < BOUNDS["amp_ratio"][1]:
            p = replace(p, amp_ratio=min(p.amp_ratio * 1.05, BOUNDS["amp_ratio"][1]))
            pa, ha, _, _ = kinematics(p)
        while pa > PITCH_LIMIT and p.amp_ratio > BOUNDS["amp_ratio"][0]:
            p = replace(p, amp_ratio=max(p.amp_ratio / 1.05, BOUNDS["amp_ratio"][0]))
            pa, ha, _, _ = kinematics(p)
    return p.clipped()


# =========================================================================
# 2. THE RELATIONSHIP TABLES  (seeding + gains come from these)
# =========================================================================
# peak_count -> freq_ratio. Measured: 0.4 gave 1 peak in 98% of 125
# missions, 0.5/0.667 gave 2 in 80-88%, >=1.0 gave 4+ in 67-94%.
PEAKCOUNT_TO_FREQ_RATIO = {1: 0.40, 2: 0.55, 3: 0.80, 4: 1.00}

# a2 -> +peak/-peak ratio, at phi2 = 0. Predicted; note the TURNING POINT
# near a2=0.3 -- beyond it the ratio falls again, which is why BOUNDS caps
# a2 at 0.35 rather than letting the controller walk past the maximum.
A2_TO_RATIO = [(0.00, 1.00), (0.10, 2.25), (0.20, 3.86), (0.30, 3.92)]

# phi2 -> (+peak/-peak ratio, skew). Predicted. The ratio INVERTS across
# phase (3.92 at 0 deg, 1.00 at 90, 0.26 at 180) -- that inversion is the
# signature the sweep is designed to confirm.
PHI2_TABLE = [(0, 3.92, 0.68), (45, 1.85, 0.32), (90, 1.00, 0.32),
              (135, 0.54, 0.32), (180, 0.26, 0.32), (225, 0.54, 0.68),
              (270, 1.00, 0.68), (315, 1.85, 0.68)]

# a3 -> positive-lobe width (fraction of cycle at half-max). Predicted;
# odd harmonic, so width only -- asymmetry stays exactly 0.
A3_TO_WIDTH = [(0.00, 0.250), (0.10, 0.154), (0.20, 0.128), (0.30, 0.117)]

# Reference operating point for the magnitude law: the best measured 1-peak
# mission (freq_ratio 0.4) reached 1.99 N at freq_scale 1.0.
REF_PEAK_N = 1.99
REF_FREQ_SCALE = 1.0


def _invert(table, y_target, lo, hi):
    """Interpolate x from a monotone-ish (x, y) table, clamped to [lo, hi]."""
    xs = np.array([r[0] for r in table], dtype=float)
    ys = np.array([r[1] for r in table], dtype=float)
    order = np.argsort(ys)
    x = float(np.interp(y_target, ys[order], xs[order]))
    return float(np.clip(x, lo, hi))


def _slope(table, x, col=1, h=0.02):
    """d(y)/d(x) at x, by central difference on the table."""
    xs = np.array([r[0] for r in table], dtype=float)
    ys = np.array([r[col] for r in table], dtype=float)
    y1 = float(np.interp(x + h, xs, ys))
    y0 = float(np.interp(max(0.0, x - h), xs, ys))
    return (y1 - y0) / (h + min(h, x))


# =========================================================================
# 3. DESCRIPTORS  (the space the controller actually works in)
# =========================================================================
def lowpass(y, cutoff_hz, fs=FS_LOADCELL):
    wn = cutoff_hz / (fs / 2.0)
    if not (1e-3 < wn < 0.99):
        return y
    try:
        b, a = butter(4, wn)
        return filtfilt(b, a, y)
    except Exception:
        return y


def descriptors(t, F):
    """Reduce one cycle to the five numbers the controller steers.

    All are scale-free EXCEPT pos_peak, which is deliberately absolute:
    making magnitude one of the matched descriptors is what stops the
    controller trading thrust away for shape, the failure mode of the
    previous version.
    """
    n = len(F)
    p, q = float(np.max(F)), float(np.min(F))
    ip, iq = int(np.argmax(F)), int(np.argmin(F))
    span = max(p - q, 1e-9)

    ratio = p / abs(q) if abs(q) > 1e-6 else 10.0
    pos_w = float(np.count_nonzero(F >= 0.5 * p)) / n if p > 0 else 0.0
    neg_w = float(np.count_nonzero(F <= 0.5 * q)) / n if q < 0 else 0.0

    tiled = np.concatenate([F, F, F])
    pk, _ = find_peaks(tiled, prominence=0.2 * span,
                       distance=max(1, int(0.07 * n)))
    n_pos = len([x for x in pk if n <= x < 2 * n])

    return {
        "pos_peak": p,
        "neg_peak": q,
        "ratio": ratio,                      # +peak / |-peak|
        # trough_frac: how deep the negative lobe is RELATIVE to the peak.
        # 0 = no trough at all ("one hump then zero thrust"), 1 = trough as
        # deep as the peak is tall. Well conditioned wherever the peak is
        # positive, which it always is for a thrust curve.
        # It replaces asym = |p+q|/(p-q), which is only bounded when the
        # waveform crosses zero: once the force stays positive (q > 0) the
        # denominator collapses and the metric ran to 51 and then 626 in
        # tolerance units, swamping every other term in the cost.
        "trough_frac": max(0.0, -q) / max(p, 1e-6),
        "asym": abs(p + q) / span,
        "skew": ((ip - iq) % n) / n,
        "pos_width": pos_w,
        "neg_width": neg_w,
        "peak_count": float(max(1, n_pos)),
        "net": float(np.mean(F)),
    }


def target_descriptors(spec):
    """Descriptors of the JSON target curve itself -- what is being asked for."""
    ch = spec["channel_definitions"]
    key = "Fx" if "Fx" in ch else list(ch)[0]
    pts = ch[key]["target_points"]
    per = float(spec.get("period_s", 1.0))
    tt = np.array([p["t"] for p in pts], float)
    ff = np.array([p["F"] for p in pts], float)
    t = np.linspace(0, per, 2000, endpoint=False)
    F = np.interp(t, tt, ff)
    d = descriptors(t, F)
    d["channel"] = key
    return d, t, F


# =========================================================================
# 4. SEEDING  -- invert the relationships instead of guessing
# =========================================================================
def seed_from_target(td, channel="Fx", log=print):
    """Choose a full parameter set directly from the target's descriptors.

    Each knob is set from the relationship that governs its own descriptor,
    in dependency order: peak count first (it fixes freq_ratio, which sets
    the velocity budget), then shape, then magnitude last -- because
    magnitude is the only one that can be traded against the slew limit.
    """
    want_count = int(round(td["peak_count"]))
    fr = PEAKCOUNT_TO_FREQ_RATIO.get(want_count, 0.40)

    # Shape: how one-sided does the target want the lobes, and which side?
    # asym -> the equivalent +/- ratio -> a2
    # trough_frac -> equivalent +/- ratio -> a2. A shallow trough needs a
    # large ratio, hence a large a2.
    want_tf = float(np.clip(td["trough_frac"], 0.0, 2.0))
    want_ratio = 1.0 / max(want_tf, 0.02)
    a2 = _invert(A2_TO_RATIO, min(want_ratio, 3.92), *BOUNDS["a2"])

    # phi2 picks WHICH lobe dominates; interpolate on the ratio column.
    phis = np.array([r[0] for r in PHI2_TABLE], float)
    ratios = np.array([r[1] for r in PHI2_TABLE], float)
    half = phis <= 180
    phi2 = float(np.interp(np.clip(want_ratio, ratios[half].min(), ratios[half].max()),
                           ratios[half][::-1], phis[half][::-1]))

    a3 = _invert(A3_TO_WIDTH, td["pos_width"], *BOUNDS["a3"])

    # Magnitude last: F ~ v^2 -> freq_scale = sqrt(F_target / F_ref).
    want_peak = max(td["pos_peak"], 1e-6)
    fs = REF_FREQ_SCALE * math.sqrt(want_peak / REF_PEAK_N)

    p = Params(amp_ratio=1.0, freq_ratio=fr, freq_scale=fs,
               a2=a2, phi2=phi2, a3=a3, channel=channel)
    p_before = p.freq_scale
    p = enforce_limits(p)

    log(f"SEED from target descriptors:")
    log(f"   target: peak={td['pos_peak']:.2f} N  trough={want_tf:.2f}  "
        f"width={td['pos_width']:.3f}  peaks={want_count}")
    log(f"   -> freq_ratio={p.freq_ratio:.3f} (peak count {want_count})")
    log(f"   -> a2={p.a2:.3f} phi2={p.phi2:.0f} deg (+/- ratio {want_ratio:.2f})")
    log(f"   -> a3={p.a3:.3f} (lobe width {td['pos_width']:.3f})")
    log(f"   -> freq_scale={p.freq_scale:.3f}"
        + (f"  (wanted {p_before:.3f}, cut by slew limit)"
           if p.freq_scale < p_before - 1e-6 else ""))
    if p.freq_scale < p_before - 1e-6:
        reach = REF_PEAK_N * (p.freq_scale / REF_FREQ_SCALE) ** 2
        log(f"   NOTE: slew caps the reachable peak near {reach:.1f} N, "
            f"below the {want_peak:.1f} N asked for.")
    return p


# =========================================================================
# 5. SHAPE GAINS  -- d(descriptor) / d(knob)
# =========================================================================
def shape_gains(p: Params, measured_peak=None):
    """Local gains at the current operating point, from the tables above.

    Returned as {descriptor: (knob, gain)}. These are what make MATCH a
    controller rather than a search: every shape error maps to a knob with
    a signed, sized correction.
    """
    g = {}
    # a2 -> asym. Converted from the ratio table: asym = (r-1)/(r+1).
    r_now = float(np.interp(p.a2, [x for x, _ in A2_TO_RATIO],
                            [y for _, y in A2_TO_RATIO]))
    r_up = float(np.interp(p.a2 + 0.02, [x for x, _ in A2_TO_RATIO],
                           [y for _, y in A2_TO_RATIO]))
    d_asym = ((r_up - 1) / (r_up + 1)) - ((r_now - 1) / (r_now + 1))
    # a2 deepens/shallows the trough; sign is negative because raising a2
    # at phi2 ~ 0 grows the positive lobe at the negative lobe's expense.
    g["trough_frac"] = ("a2", -max(d_asym / 0.02, 0.3))
    g["pos_width"] = ("a3", min(_slope(A3_TO_WIDTH, p.a3), -0.05))
    # magnitude: F ~ freq_scale^2, so d(peak)/d(fs) = 2*peak/fs. Uses the
    # peak MEASURED this cycle rather than the bench reference -- with the
    # reference the correction was sized for a 1.99 N rig while the plant
    # was delivering 5-20 N, which drove freq_scale oscillations of 3x per
    # cycle (0.5 <-> 1.6) instead of converging.
    meas_peak = max(abs(measured_peak), 0.05) if measured_peak else REF_PEAK_N
    g["pos_peak"] = ("freq_scale", 2.0 * meas_peak / max(p.freq_scale, 1e-6))
    # dominance/skew via phi2: slope of the ratio column near current phase
    phis = np.array([r[0] for r in PHI2_TABLE], float)
    ratios = np.array([r[1] for r in PHI2_TABLE], float)
    d_ratio_d_phi = float(np.gradient(ratios, phis)[
        int(np.argmin(np.abs(phis - (p.phi2 % 360))))])
    # expressed in asym units so it is comparable with the a2 gain above
    g["phi2_trough"] = ("phi2", (-d_ratio_d_phi / 30.0) if abs(d_ratio_d_phi) > 1e-4 else 0.01)
    return g


# Which descriptors MATCH tries to fit, their tolerances, and their weight.
# Tolerances are what "close enough" means physically, and they also
# normalise the cost so a 0.5 N miss and a 0.05 width miss are comparable.
# `asym`, not `ratio`. The +peak/-peak RATIO diverges exactly where the
# interesting targets live: a "one hump then zero thrust" curve has
# neg_peak ~ 0, so the ratio runs to infinity and its error swamps every
# other term (measured: a single ratio error of -22 contributed 968 of a
# 1594 total cost, burying the magnitude and width terms entirely).
# asym = |p+q|/(p-q) encodes the same thing bounded on [0, 1]: 0 = lobes
# are mirror images, 1 = fully one-sided.
MATCH_SPEC = {
    "pos_peak":   {"tol": 0.5,  "w": 3.0},
    "trough_frac": {"tol": 0.08, "w": 2.0},
    "pos_width":  {"tol": 0.04, "w": 1.5},
    "peak_count": {"tol": 0.5,  "w": 2.0},
}


def match_cost(meas, td):
    """Tolerance-normalised squared error over the matched descriptors."""
    c, terms = 0.0, {}
    for k, s in MATCH_SPEC.items():
        if k not in meas or k not in td:
            continue
        e = (meas[k] - td[k]) / s["tol"]
        terms[k] = e
        c += s["w"] * e * e
    return c, terms


class GainAdapter:
    """Re-identifies each gain from the run's own measurements.

    The harmonic gains are PREDICTIONS. Rather than trusting them for a
    whole run, every (parameter, descriptor) pair observed is regressed to
    get the realised slope, and the seed gain is replaced once there is
    enough spread in that knob to fit one. A prediction that is wrong in
    magnitude -- or in sign -- is therefore corrected by the rig, while a
    knob that has not moved keeps its seeded value instead of fitting noise.
    """
    MIN_SPREAD = {"a2": 0.04, "a3": 0.04, "phi2": 15.0, "freq_scale": 0.08}

    def __init__(self):
        self.hist = []

    def observe(self, p: Params, meas):
        self.hist.append((p, dict(meas)))

    def effective(self, seeded):
        out = dict(seeded)
        if len(self.hist) < 4:
            return out
        for desc, (knob, g0) in seeded.items():
            key = "ratio" if desc == "phi2_ratio" else desc
            xs = np.array([getattr(p, knob) for p, _ in self.hist], float)
            ys = np.array([m.get(key, np.nan) for _, m in self.hist], float)
            ok = np.isfinite(ys)
            if ok.sum() < 4 or np.ptp(xs[ok]) < self.MIN_SPREAD.get(knob, 0.05):
                continue
            slope = float(np.polyfit(xs[ok], ys[ok], 1)[0])
            # Keep the seed if the fit is degenerate; a near-zero slope
            # would otherwise divide into an enormous correction.
            if abs(slope) > 1e-6:
                out[desc] = (knob, slope)
        return out


def match_step(p: Params, meas, td, gains, step_frac=0.6, log=print,
               saturated=None):
    """One decoupled correction: each descriptor error drives its own knob.

    step_frac damps every correction because the gains are local slopes on
    curves that bend (the a2 -> ratio relation actually turns over near
    a2=0.3); a full-step Newton correction would overshoot into the region
    where the gain changes sign.
    """
    new = p
    moves = []
    saturated = saturated if saturated is not None else set()

    # peak_count is categorical -- it snaps freq_ratio to the band that is
    # known to produce the requested count rather than nudging toward it.
    want_count = int(round(td.get("peak_count", 1)))
    got_count = int(round(meas.get("peak_count", want_count)))
    count_wrong = (got_count != want_count)
    if count_wrong:
        fr = PEAKCOUNT_TO_FREQ_RATIO.get(want_count)
        if fr and abs(fr - p.freq_ratio) > 1e-3:
            new = replace(new, freq_ratio=fr)
            moves.append(f"peak_count {got_count}->{want_count}: freq_ratio->{fr:.3f}")
        elif got_count > want_count:
            # freq_ratio is ALREADY in the band for the requested count, so
            # the surplus peaks are coming from the harmonics themselves --
            # every added harmonic puts extra turning points in the velocity
            # and therefore extra lobes in the force. Offline this pinned
            # peak_count at 3 against a target of 1 while freq_ratio sat
            # correctly at 0.400 and the categorical snap did nothing at all.
            # Back the harmonics off, highest order first: a3 contributes
            # more turning points per unit amplitude than a2, and a2 is the
            # one carrying the asymmetry that is actually wanted.
            if new.a3 > 0.01:
                cut = max(0.0, new.a3 - 0.06)
                moves.append(f"peak_count {got_count}>{want_count}: a3 "
                             f"{new.a3:.3f}->{cut:.3f} (harmonics add lobes)")
                new = replace(new, a3=cut)
            elif new.a2 > 0.06:
                cut = max(0.0, new.a2 - 0.05)
                moves.append(f"peak_count {got_count}>{want_count}: a2 "
                             f"{new.a2:.3f}->{cut:.3f} (harmonics add lobes)")
                new = replace(new, a2=cut)

    # PEAK COUNT IS A PRECONDITION, NOT A PEER.
    # Width and asymmetry describe the shape of a lobe; they are not
    # meaningful while the cycle has the wrong NUMBER of lobes, and worse,
    # they fight for the same knob. Offline, peak_count drove a3 down to 0
    # and pos_width drove it straight back to 0.06 within the same step,
    # producing a 12-cycle limit cycle in which nothing else could settle.
    # So when the count is wrong, that is the only correction applied.
    if count_wrong:
        new = enforce_limits(new, log=log)
        for m in moves:
            log(f"    {m}")
        log("    (peak count wrong -- deferring shape corrections until it is right)")
        return new

    for desc, spec in MATCH_SPEC.items():
        if desc == "peak_count" or desc not in meas or desc not in td:
            continue
        err = td[desc] - meas[desc]
        if abs(err) <= spec["tol"]:
            continue
        knob, gain = gains.get(desc, (None, None))
        if knob is None or abs(gain) < 1e-9 or knob in saturated:
            continue
        delta = step_frac * err / gain
        cur = getattr(new, knob)
        lo, hi = BOUNDS[knob]
        # cap any single move to a fifth of the knob's range, so one noisy
        # measurement cannot fling a parameter across its whole span
        delta = float(np.clip(delta, -(hi - lo) / 5.0, (hi - lo) / 5.0))
        new = replace(new, **{knob: cur + delta})
        moves.append(f"{desc} err={err:+.3f} -> {knob} {cur:.3f}->{getattr(new, knob):.3f}")

    # Dominance is corrected by phi2 only once a2 is big enough to have an
    # effect to rotate -- at a2 ~ 0 the phase does nothing at all.
    if new.a2 > 0.05 and "trough_frac" in meas and "trough_frac" in td:
        err = td["trough_frac"] - meas["trough_frac"]
        if abs(err) > MATCH_SPEC["trough_frac"]["tol"]:
            knob, gain = gains.get("phi2_trough", ("phi2", 0.01))
            d = float(np.clip(step_frac * err / gain, -30.0, 30.0))
            new = replace(new, phi2=(new.phi2 + d) % 360.0)
            moves.append(f"dominance err={err:+.3f} -> phi2 {p.phi2:.0f}->{new.phi2:.0f}")

    requested = {k: getattr(new, k) for k in TUNABLE}
    new = enforce_limits(new, log=log)

    # SATURATION DETECTION. enforce_limits can undo a correction entirely --
    # freq_scale in particular gets pulled straight back to the slew
    # ceiling. Without noticing, the controller re-issues the identical
    # rejected move every cycle and deadlocks: an offline run sat at
    # freq_scale 1.310 for 6 consecutive cycles, each time asking for 1.810
    # and each time being refused, with every other knob frozen because
    # their errors were already inside tolerance.
    for k in TUNABLE:
        if abs(requested[k] - getattr(new, k)) > 1e-6:
            saturated.add(k)
            log(f"    [saturated] {k} cannot go to {requested[k]:.3f} "
                f"(limit holds it at {getattr(new, k):.3f}) -- "
                f"its descriptor is out of reach, not untuned")

    for m in moves:
        log(f"    {m}")
    if not moves:
        log("    (all matched descriptors inside tolerance)")
    return new


# =========================================================================
# 6. OPTIMISE  -- push thrust once the shape is right
# =========================================================================
def optimise_step(p: Params, meas, td, best_shape_cost, log=print):
    """Raise the positive peak while holding the matched shape.

    Runs only after MATCH converges. Thrust is bought with freq_scale (the
    magnitude knob), and every step is conditional: if the shape cost has
    drifted more than 50% above what MATCH achieved, the step is refused
    and the shape is repaired first. That gate is the thing that stops this
    from repeating the old controller's failure of maximising a number
    while the waveform walked away from the target.
    """
    cost, _ = match_cost(meas, td)
    if cost > 1.5 * max(best_shape_cost, 1e-6):
        log(f"    [opt] shape cost {cost:.2f} drifted from {best_shape_cost:.2f} "
            f"-- repairing shape before pushing thrust")
        return match_step(p, meas, td, shape_gains(p, meas.get("pos_peak")),
                          step_frac=0.6, log=log)

    _, _, fs_max = slew_headroom(p)
    if p.freq_scale >= min(fs_max, BOUNDS["freq_scale"][1]) - 1e-3:
        log(f"    [opt] freq_scale at the slew ceiling ({p.freq_scale:.3f}) -- "
            f"thrust cannot be raised further by frequency")
        return p
    new_fs = min(p.freq_scale * 1.08, fs_max * 0.98, BOUNDS["freq_scale"][1])
    log(f"    [opt] freq_scale {p.freq_scale:.3f} -> {new_fs:.3f} "
        f"(predicted peak {meas['pos_peak'] * (new_fs / p.freq_scale) ** 2:.2f} N)")
    return enforce_limits(replace(p, freq_scale=new_fs), log=log)


# =========================================================================
# 7. PLANTS
# =========================================================================
def run_plant_SIMULATED(p: Params, n_cycles=N_CYCLES):
    """Stand-in plant. NOT a validated fluid model -- quadratic drag on the
    commanded velocities, enough to exercise the loop offline. Absolute
    newtons from this are meaningless; only the qualitative knob responses
    are (deliberately) reproduced."""
    pa, ha, pf, hf = kinematics(p)
    period = beat_period(p)
    n = 1200 * n_cycles
    t = np.linspace(0, n_cycles * period, n, endpoint=False)
    on_pitch = (p.channel == "Fx")

    def ang(amp, f, a2, phi2, a3, ph=0.0):
        w = 2 * np.pi * f
        th = (np.sin(w * t + ph)
              + a2 * np.sin(2 * (w * t + ph) + math.radians(phi2))
              + a3 * np.sin(3 * (w * t + ph)))
        return amp * th / max(np.max(np.abs(th)), 1e-9)

    th1 = ang(pa, pf, p.a2 if on_pitch else 0.0, p.phi2, p.a3 if on_pitch else 0.0)
    th2 = ang(ha, hf, 0.0 if on_pitch else p.a2, p.phi2, 0.0 if on_pitch else p.a3)
    v1, v2 = np.gradient(th1, t), np.gradient(th2, t)
    Fx = 2.0 * v1 * np.abs(v1) + 0.4 * v1 * v2
    Fy = 2.0 * v2 * np.abs(v2) + 0.4 * v1 * v2
    Fz = 0.05 * (v1 ** 2 - v2 ** 2)
    for arr in (Fx, Fy, Fz):
        arr += np.random.normal(0, 0.02 * max(np.ptp(arr), 1e-6), arr.shape)
    return t, Fx, Fy, Fz, th1, th2


def run_plant_HYPOTHESIS(p: Params, n_cycles=N_CYCLES):
    """Plant that OBEYS the hypothesised relationships, by construction.

    This exists to answer one question and only one: IF the harmonic
    relationships hold, does this controller converge to the target? It is
    the model the relationships were derived from -- quadratic drag on the
    normalised composite command -- so agreement here is a consistency
    check on the CONTROLLER's logic (seeding, gains, staging, limits), not
    evidence about the rig. The sweep is what tests the physics; this tests
    the code that assumes it.
    """
    pa, ha, pf, hf = kinematics(p)
    period = beat_period(p)
    n = 2000 * n_cycles
    t = np.linspace(0, n_cycles * period, n, endpoint=False)
    on_pitch = (p.channel == "Fx")

    def ang(amp, f, a2, phi2, a3):
        w = 2 * np.pi * f
        th = (np.sin(w * t) + a2 * np.sin(2 * w * t + math.radians(phi2))
              + a3 * np.sin(3 * w * t))
        return amp * th / max(np.max(np.abs(th)), 1e-9)

    th1 = ang(pa, pf, p.a2 if on_pitch else 0.0, p.phi2, p.a3 if on_pitch else 0.0)
    th2 = ang(ha, hf, 0.0 if on_pitch else p.a2, p.phi2, 0.0 if on_pitch else p.a3)
    v1, v2 = np.gradient(th1, t), np.gradient(th2, t)

    # Calibrated so the reference point (freq_ratio 0.4, freq_scale 1.0,
    # plain sine) lands on the measured 1.99 N, making absolute newtons
    # here comparable with the bench numbers the seeding uses.
    drive = v1 if on_pitch else v2
    F_on = drive * np.abs(drive)
    ref = Params(freq_ratio=0.40, freq_scale=1.0, channel=p.channel)
    rpa, rha, rpf, rhf = kinematics(ref)
    rv = (rpa * 2 * np.pi * rpf) if on_pitch else (rha * 2 * np.pi * rhf)
    F_on = F_on * (REF_PEAK_N / max(rv ** 2, 1e-9))

    other = v2 if on_pitch else v1
    F_off = other * np.abs(other) * (REF_PEAK_N / max(rv ** 2, 1e-9)) * 0.3
    Fz = 0.05 * (v1 ** 2 - v2 ** 2)
    Fx, Fy = (F_on, F_off) if on_pitch else (F_off, F_on)
    for arr in (Fx, Fy, Fz):
        arr += np.random.normal(0, 0.01 * max(np.ptp(arr), 1e-6), arr.shape)
    return t, Fx, Fy, Fz, th1, th2


def make_hardware_plant(node):
    import soft_propulsors_control.motion_command as mc

    def run(p: Params, n_cycles=N_CYCLES):
        return mc.run_plant_HARMONIC(p, n_cycles=n_cycles, node=node)
    return run


def measure(p: Params, run_plant, log=print):
    """One measurement: settle, run 3 cycles, analyse ONLY the second.

    Cycle 1 holds the start-up transient and cycle 3 can catch the
    ramp-down, so the middle cycle is the only unambiguously steady one.
    """
    time.sleep(SETTLE_S)
    t, Fx, Fy, Fz, th1, th2 = run_plant(p, n_cycles=N_CYCLES)
    per = (t[-1] - t[0]) / N_CYCLES
    sl = (t >= t[0] + per) & (t < t[0] + 2 * per)
    if np.count_nonzero(sl) < 50:
        sl = slice(None)
    _, _, pf, _ = kinematics(p)
    # Sample rate from the data itself, NOT the 10 kHz load-cell constant:
    # offline plants sample at a few hundred Hz, and assuming 10 kHz put the
    # cutoff far below the signal band -- filtfilt then flattened Fx to
    # near-DC (peak 0.53 N against a mean of 0.52 N) and every shape
    # descriptor measured a straight line.
    fs_actual = (len(t) - 1) / max(t[-1] - t[0], 1e-9)
    out = {}
    for name, F in (("Fx", Fx), ("Fy", Fy), ("Fz", Fz)):
        y = lowpass(np.asarray(F, float), CUTOFF_MULT * pf, fs=fs_actual)[sl]
        out[name] = descriptors(np.asarray(t)[sl], y)
    return out, (np.asarray(t)[sl], np.asarray(Fx)[sl],
                 np.asarray(Fy)[sl], np.asarray(Fz)[sl])


# =========================================================================
# 8. THE LOOP
# =========================================================================
def run(spec, run_plant, channel="Fx", max_cycles=MAX_ITERS, log=print):
    td, _, _ = target_descriptors(spec)
    p = seed_from_target(td, channel=channel, log=log)

    adapter = GainAdapter()
    stage = "MATCH"
    best_cost, best = float("inf"), None
    stall = 0
    saturated = set()
    history = []

    log("\n" + "=" * 72)
    log(f"target channel: {channel}   |   stage 1: MATCH")
    log("=" * 72)

    for k in range(1, max_cycles + 1):
        meas_all, waves = measure(p, run_plant, log=log)
        meas = meas_all[channel]
        adapter.observe(p, meas)
        cost, terms = match_cost(meas, td)

        log(f"\n[cycle {k:02d}] {stage}  amp_ratio={p.amp_ratio:.3f} "
            f"freq_ratio={p.freq_ratio:.3f} freq_scale={p.freq_scale:.3f} "
            f"a2={p.a2:.3f} phi2={p.phi2:.0f} a3={p.a3:.3f}")
        log(f"   {channel}: peak={meas['pos_peak']:+.2f} N  trough={meas['trough_frac']:.2f}  "
            f"width={meas['pos_width']:.3f}  peaks={meas['peak_count']:.0f}  "
            f"net={meas['net']:+.3f}")
        off = "Fy" if channel == "Fx" else "Fx"
        log(f"   off-channel {off}: net={meas_all[off]['net']:+.3f} N  "
            f"({abs(meas_all[off]['net']) / max(abs(meas['pos_peak']), 1e-9) * 100:.0f}% "
            f"of on-channel peak)")
        log(f"   shape cost={cost:.3f}  " +
            "  ".join(f"{d}:{e:+.2f}" for d, e in terms.items()))

        history.append({"cycle": k, "stage": stage, "params": asdict(p),
                        "meas": meas, "off": meas_all[off], "cost": cost})

        if cost < best_cost - 1e-3:
            best_cost, best, stall = cost, (p, meas, k), 0
        else:
            stall += 1

        if stage == "MATCH":
            if all(abs(e) <= 1.0 for e in terms.values()):
                log(f"\n--- MATCH achieved at cycle {k} (every descriptor inside "
                    f"tolerance). Switching to OPTIMISE.\n")
                stage = "OPTIMISE"
                stall = 0
                continue
            if stall >= MATCH_PATIENCE:
                log(f"\n--- MATCH plateaued at cost {best_cost:.3f} after {k} cycles. "
                    f"Switching to OPTIMISE from the best-matched parameters.\n")
                stage, p, stall = "OPTIMISE", best[0], 0
                continue
            p = match_step(p, meas, td,
                           adapter.effective(shape_gains(p, meas.get("pos_peak"))),
                           log=log, saturated=saturated)
        else:
            if stall >= OPT_PATIENCE:
                log(f"\nStopped at cycle {k}: {OPT_PATIENCE} cycles without "
                    f"improving on cost {best_cost:.3f} (cycle {best[2]}).")
                break
            p = optimise_step(p, meas, td, best_cost, log=log)

    log("\n" + "=" * 72)
    if best:
        bp, bm, bc = best
        log(f"BEST (cycle {bc}): peak={bm['pos_peak']:.2f} N  trough={bm['trough_frac']:.2f}  "
            f"width={bm['pos_width']:.3f}  peaks={bm['peak_count']:.0f}")
        log(f"  params: {asdict(bp)}")
        log(f"  target: peak={td['pos_peak']:.2f} N  ratio={td['ratio']:.2f}  "
            f"width={td['pos_width']:.3f}  peaks={td['peak_count']:.0f}")
    log("=" * 72)
    return best, history


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    folder = args[0] if args else "drag_dominant"
    if not os.path.isabs(folder):
        folder = os.path.join(WORKSPACE_ROOT, folder)

    cands = [p for p in glob.glob(os.path.join(folder, "target_curve_*.json"))
             if "_with_result" not in p]
    if not cands:
        print(f"no target_curve_*.json in {folder}")
        return 2
    spec = json.load(open(cands[0]))
    channel = "Fy" if "--fy" in flags else "Fx"
    max_cycles = MAX_ITERS
    for f in flags:
        if f.startswith("--max-cycles="):
            max_cycles = int(f.split("=", 1)[1])

    print(f"experiment folder: {folder}")
    print(f"target curve:      {cands[0]}")

    if "--hardware" in flags:
        import soft_propulsors_control.motion_command as mc
        node = mc.start_hil_node()
        try:
            node.capture_rest_baseline()
            best, hist = run(spec, make_hardware_plant(node), channel, max_cycles)
        finally:
            mc.stop_hil_node(node)
    else:
        plant = (run_plant_SIMULATED if "--toy" in flags else run_plant_HYPOTHESIS)
        print(f"OFFLINE plant = {plant.__name__} "
              f"(pass --hardware to drive the rig)\n")
        best, hist = run(spec, plant, channel, max_cycles)

    out = os.path.join(folder, "harmonic_result.json")
    with open(out, "w") as fh:
        json.dump({"best_params": asdict(best[0]) if best else None,
                   "best_cycle": best[2] if best else None,
                   "history": [{**h, "meas": {k: float(v) for k, v in h["meas"].items()},
                                "off": {k: float(v) for k, v in h["off"].items()}}
                               for h in hist]}, fh, indent=2, default=str)
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
