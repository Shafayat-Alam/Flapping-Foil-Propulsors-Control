#!/usr/bin/env python3
"""Grey-box plant model: kinematics -> force, structure from physics.

STRUCTURE (what we know) -- quasi-steady blade element in STILL water
---------------------------------------------------------------------
    v(t)   = d(theta_sweep)/dt                 blade speed through the water
    F_N(t) = C_d*|v|*v  +  C_a*dv/dt           form drag + added mass
    psi(t) = theta_pitch(t) + phi0             blade normal orientation

    Fx = F_N*cos(psi) [- F_T*sin(psi)] + bx
    Fy = F_N*sin(psi) [+ F_T*cos(psi)] + by
    Fz = bz

Still water is what makes this tractable: with no free stream there is no
advance ratio and no oncoming-flow convection term, so the blade's own motion
is the entire relative velocity.

PARAMETERS (what we fit): C_d, C_a, phi0, bx, by, bz  (+ C_t for M2).
The effective radius and blade area are not separately identifiable from
force alone -- they scale C_d and C_a -- so they are deliberately absorbed
rather than pretending to estimate them.

WHY THIS AND NOT THE JACOBIAN
-----------------------------
The stage-1 Jacobian is a local linearisation of a static map at one
operating point. It describes the neighbourhood it was measured in and has no
mechanism to extrapolate: by the distinction in the system-identification
literature it is closer to curve fitting than to a dynamic model. This model
has physical structure, so it can be asked to predict a gait it has never
seen -- which is the only test that can actually falsify it.

THE FALSIFIABLE PREDICTION
--------------------------
Fx and Fy here are ONE scalar force rotated by the blade orientation, not two
independent channels. If that holds it explains, from first principles, why
pitch harmonics failed to shape Fx while heave harmonics shaped Fy, and why
amp_ratio gave only 7.7:1 selectivity instead of decoupling. It also makes
arbitrary-direction thrust a geometry problem (point the blade normal) rather
than a blind search. If the data refuses this structure, that is a real
result and the residual diagnostics will say where it broke.

MODEL ORDER
-----------
  M1  normal force only            6 parameters
  M2  M1 + tangential/chordwise    7 parameters
Selected on HELD-OUT data, never on the estimation fit -- a higher-order
model always fits the training set better and that is exactly how overfitting
gets mistaken for a better model.

Which servo sweeps and which pitches is a STRUCTURAL choice, not a parameter,
so both assignments are fitted and compared the same way.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

PARAM_NAMES_M1 = ["C_d", "C_a", "phi0", "bx", "by", "bz"]
PARAM_NAMES_M2 = ["C_d", "C_a", "C_t", "phi0", "bx", "by", "bz"]
# M3 tests a KINEMATIC hypothesis, not a fluid one: M1/M2 assume the blade's
# angle relative to the LOCAL flow is theta_pitch + phi0 -- a constant offset,
# correct only if the pitch/roll servo is mounted ON the sweeping arm (so its
# own encoder already reads angle relative to the arm's rotating frame, and
# the local flow direction -- tangent to the sweep -- is fixed in that frame
# regardless of how the sweep speeds up or slows down through the stroke).
# If instead the effective angle of attack depends on WHERE in the sweep the
# blade currently is (not just on commanded pitch), a constant offset cannot
# capture it, and psi should depend on theta_sweep itself -- not merely on its
# derivative, which already enters through v. k is exactly that dependence,
# left free rather than assumed zero.
PARAM_NAMES_M3 = ["C_d", "C_a", "k", "phi0", "bx", "by", "bz"]


def _derivs(theta, t):
    """Velocity and acceleration of a joint trace."""
    v = np.gradient(theta, t)
    a = np.gradient(v, t)
    return v, a


def predict(params, th_sweep, th_pitch, t, model="M1"):
    """Forward model: kinematics -> (Fx, Fy, Fz)."""
    v, dv = _derivs(th_sweep, t)
    if model == "M1":
        C_d, C_a, phi0, bx, by, bz = params
        C_t, k = 0.0, 0.0
    elif model == "M2":
        C_d, C_a, C_t, phi0, bx, by, bz = params
        k = 0.0
    else:   # M3
        C_d, C_a, k, phi0, bx, by, bz = params
        C_t = 0.0
    F_N = C_d * np.abs(v) * v + C_a * dv
    F_T = C_t * np.abs(v) * v
    psi = th_pitch + k * th_sweep + phi0
    Fx = F_N * np.cos(psi) - F_T * np.sin(psi) + bx
    Fy = F_N * np.sin(psi) + F_T * np.cos(psi) + by
    Fz = np.full_like(Fx, bz)
    return Fx, Fy, Fz


def fit(datasets, model="M1", swap=False, log=print):
    """Least-squares fit over many missions at once.

    datasets: list of (t, th1, th2, Fx, Fy, Fz), each one steady cycle.
    swap:     False -> servo1 sweeps / servo2 pitches;  True -> the reverse.

    All missions are fitted JOINTLY. Fitting each mission separately would
    give a different C_d per gait, which is precisely the curve-fitting
    failure mode -- the coefficients are properties of the blade and fluid,
    not of the gait, so they must be shared across every gait at once.
    """
    def residual(p):
        out = []
        for (t, th1, th2, Fx, Fy, Fz) in datasets:
            sw, pi_ = (th2, th1) if swap else (th1, th2)
            px, py, pz = predict(p, sw, pi_, t, model)
            out.append(np.concatenate([px - Fx, py - Fy, pz - Fz]))
        return np.concatenate(out)

    p0 = {"M1": [1.0, 0.1, 0.0, 0.0, 0.0, 0.0],
          "M2": [1.0, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0],
          "M3": [1.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0]}[model]
    r = least_squares(residual, p0, method="trf", max_nfev=4000)
    names = {"M1": PARAM_NAMES_M1, "M2": PARAM_NAMES_M2,
            "M3": PARAM_NAMES_M3}[model]
    p = dict(zip(names, r.x))
    log(f"   {model}{' (swapped)' if swap else ''}: "
        + "  ".join(f"{k}={v:+.4f}" for k, v in p.items()))
    return r.x, p


def nrmse(pred, meas):
    """Normalised RMSE as a percentage fit, the convention in the
    system-identification literature: 100*(1 - |meas-pred| / |meas-mean|)."""
    denom = np.linalg.norm(meas - np.mean(meas))
    if denom < 1e-12:
        return float("nan")
    return 100.0 * (1.0 - np.linalg.norm(meas - pred) / denom)


def score(params, datasets, model="M1", swap=False):
    """Per-channel fit over a set of missions."""
    acc = {"Fx": [], "Fy": [], "Fz": []}
    for (t, th1, th2, Fx, Fy, Fz) in datasets:
        sw, pi_ = (th2, th1) if swap else (th1, th2)
        px, py, pz = predict(params, sw, pi_, t, model)
        for k, (p, m) in zip(("Fx", "Fy", "Fz"),
                             ((px, Fx), (py, Fy), (pz, Fz))):
            acc[k].append(nrmse(p, m))
    return {k: float(np.nanmean(v)) for k, v in acc.items()}


# =========================================================================
# Residual diagnostics
# =========================================================================
def _norm_corr(a, b, max_lag):
    a = a - a.mean()
    b = b - b.mean()
    d = np.sqrt(np.sum(a * a) * np.sum(b * b))
    if d < 1e-12:
        return np.zeros(2 * max_lag + 1)
    full = np.correlate(a, b, mode="full") / d
    mid = len(full) // 2
    return full[mid - max_lag: mid + max_lag + 1]


def residual_diagnostics(params, datasets, model="M1", swap=False, max_lag=40):
    """Autocorrelation of residuals, and cross-correlation with the input.

    Interpretation, which is the whole point of running these:
      cross-correlation with input HIGH  -> dynamics missing in the
                                            input->output path itself
      cross-corr LOW but autocorr HIGH   -> the system model is adequate and
                                            what is missing is a DISTURBANCE
                                            model
      both low                           -> residual is white; the systematic
                                            part has been captured

    A residual that correlates with its own past is by definition partly
    predictable from past data, so it cannot be dismissed as noise -- some
    dynamics are unmodelled.
    """
    out = {}
    for ch, idx in (("Fx", 3), ("Fy", 4), ("Fz", 5)):
        auto, cross = [], []
        for ds in datasets:
            t, th1, th2 = ds[0], ds[1], ds[2]
            meas = ds[idx]
            sw, pi_ = (th2, th1) if swap else (th1, th2)
            pred_ = predict(params, sw, pi_, t, model)[{"Fx": 0, "Fy": 1, "Fz": 2}[ch]]
            res = meas - pred_
            v, _ = _derivs(sw, t)
            auto.append(_norm_corr(res, res, max_lag))
            cross.append(_norm_corr(res, v, max_lag))
        A = np.mean(auto, axis=0)
        C = np.mean(cross, axis=0)
        # exclude zero lag from the autocorrelation summary: it is 1 by
        # construction and says nothing about whether the residual is white
        nz = np.concatenate([A[:max_lag], A[max_lag + 1:]])
        out[ch] = {
            "autocorr_max_nonzero_lag": float(np.max(np.abs(nz))),
            "crosscorr_max": float(np.max(np.abs(C))),
            "autocorr": A.tolist(), "crosscorr": C.tolist(),
        }
    return out


def verdict(diag, auto_thresh=0.30, cross_thresh=0.20):
    """Turn the two correlation numbers into the diagnosis they imply."""
    lines = []
    for ch, d in diag.items():
        a, c = d["autocorr_max_nonzero_lag"], d["crosscorr_max"]
        if c > cross_thresh:
            v = "MISSING SYSTEM DYNAMICS (residual tracks the input)"
        elif a > auto_thresh:
            v = "system model OK; missing a DISTURBANCE model"
        else:
            v = "residual ~white; systematic part captured"
        lines.append(f"   {ch}: autocorr {a:.3f}  crosscorr {c:.3f}   -> {v}")
    return "\n".join(lines)


def select(est, val, log=print):
    """Fit every candidate structure, rank on HELD-OUT data."""
    results = []
    for model in ("M1", "M2"):
        for swap in (False, True):
            try:
                x, p = fit(est, model, swap, log=lambda *a: None)
            except Exception as e:
                log(f"   {model} swap={swap}: fit failed ({e})")
                continue
            s_est = score(x, est, model, swap)
            s_val = score(x, val, model, swap)
            results.append({"model": model, "swap": swap, "x": x, "params": p,
                            "est": s_est, "val": s_val,
                            "val_mean": float(np.nanmean([s_val["Fx"], s_val["Fy"]]))})
    results.sort(key=lambda r: -r["val_mean"])
    log(f"{'model':10s}{'sweep servo':13s}{'est Fx':>8}{'est Fy':>8}"
        f"{'VAL Fx':>8}{'VAL Fy':>8}{'VAL mean':>10}")
    log("-" * 65)
    for r in results:
        log(f"{r['model']:10s}{'2' if r['swap'] else '1':13s}"
            f"{r['est']['Fx']:8.1f}{r['est']['Fy']:8.1f}"
            f"{r['val']['Fx']:8.1f}{r['val']['Fy']:8.1f}{r['val_mean']:10.1f}")
    return results
