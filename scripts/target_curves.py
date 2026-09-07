#!/usr/bin/env python3
"""Target force curves defined as MATH, not as sampled points.

Replaces the target_curve_*.json point tables. A target is now a periodic
cubic B-spline whose control points are GENERATED from a handful of shape
parameters, so the curve is a function S(t; theta) rather than an
interpolation through hand-typed coordinates.

    S(t) = sum_i  N_{i,3}(t) * P_i(theta)

Why a B-spline rather than an analytic bump (a Gaussian, a raised cosine,
a sum of harmonics):

  * NON-NEGATIVITY IS STRUCTURAL. A B-spline lies inside the convex hull of
    its control points, so P_i >= 0 for all i implies S(t) >= 0 everywhere
    -- exactly, not approximately, and without evaluating the curve. The
    drag requirement "no troughs" therefore becomes a property of the
    parameterisation instead of a penalty term the optimiser has to be
    talked into respecting.
  * LOCAL CONTROL. Each control point influences only the 4 spans around
    it, so "make the crest narrower" and "make the tail decay sooner" are
    independent edits. With a harmonic series every coefficient changes the
    whole cycle, which is what made the earlier descriptor targets fight
    each other.
  * DIFFERENTIABLE IN THE PARAMETERS. Piecewise polynomial in t and linear
    in P, so dS/dtheta is available in closed form for any downstream fit.

Periodic knots throughout: the gait repeats, so the curve must join itself
with C2 continuity at t=0/1 rather than merely starting and ending at the
same value.

CONVENTION. t is the normalised cycle, t in [0, 1). Amplitudes are in
newtons but the shapes are scale-free -- `height` simply scales the whole
curve, and the controller is not asked to chase it.
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import BSpline

DEGREE = 3


# =========================================================================
# Core: periodic cubic B-spline from a control-point vector
# =========================================================================
def periodic_spline(P, degree=DEGREE):
    """Periodic B-spline over t in [0,1] from control points P.

    Built by wrapping `degree` control points around each end and using a
    uniform knot vector, which is the standard construction for a closed
    curve; the wrap is what makes S and its first two derivatives continuous
    across t=0 rather than only S itself.
    """
    P = np.asarray(P, dtype=float)
    n = len(P)
    Pw = np.concatenate([P, P[:degree]])
    knots = (np.arange(-degree, n + degree + 1, dtype=float)) / n
    return BSpline(knots, Pw, degree, extrapolate=False)


def evaluate(spline, n_samples=1000):
    t = np.linspace(0.0, 1.0, n_samples, endpoint=False)
    return t, spline(t)


# =========================================================================
# Control-point generators: theta -> P
# =========================================================================
def _crest_envelope(n, centre, rise, fall, height):
    """Non-negative control points forming ONE crest that dies away.

    Asymmetric by construction: `rise` and `fall` are separate widths, so
    the crest can climb quickly and decay slowly ("dies down") without any
    even-harmonic trickery. Everything outside the crest is exactly 0, so
    the curve is flat-zero there rather than merely small.

    Distances are measured on the CIRCLE (wrapped to +/-0.5) because the
    cycle is periodic -- a crest near t=0 must decay into the end of the
    cycle, not fall off an edge.
    """
    u = np.arange(n) / n
    d = (u - centre + 0.5) % 1.0 - 0.5          # signed circular distance
    w = np.where(d < 0, rise, fall)             # rising side vs falling side
    x = d / np.maximum(w, 1e-9)
    # Raised-cosine lobe on |x| <= 1, exactly zero beyond it. C1 at the
    # join, and non-negative everywhere by construction.
    env = np.where(np.abs(x) <= 1.0, 0.5 * (1.0 + np.cos(np.pi * x)), 0.0)
    return height * env


def drag_control_points(n=48, centre=0.30, rise=0.16, fall=0.30, height=1.0):
    """DRAG: one positive crest that dies away, and never goes negative.

    All control points are >= 0, so S(t) >= 0 is guaranteed exactly -- the
    hard no-trough requirement is satisfied by the parameterisation itself.
    `fall` > `rise` by default gives the asymmetric "dies down" tail.
    """
    return _crest_envelope(n, centre, rise, fall, height)


def lift_control_points(n=48, centres=(0.25, 0.75), width=0.15,
                        height=1.0, trough=0.0):
    """LIFT: two crests of EQUAL AREA, with an optional dip between them.

    `trough` is the depth of the dip as a fraction of `height`, and is the
    one parameter allowed to make the curve negative. It is kept explicit
    (rather than emerging from the crest shapes) precisely because the goal
    is to drive it toward zero: with trough=0 the curve is non-negative by
    the same convex-hull argument as the drag target, so "no trough" and
    "some trough" are the same family at different parameter values, not
    two different curves.

    Crests are symmetric to each other by construction (same width, same
    height, antipodal centres), which makes their integrals equal -- the
    "net of each the same as the other" requirement.
    """
    P = np.zeros(n)
    for c in centres:
        P = np.maximum(P, _crest_envelope(n, c, width, width, height))
    if trough > 0:
        # Dip centred between the crests, on both inter-crest gaps, so the
        # two crests stay symmetric.
        mids = [((centres[i] + centres[(i + 1) % len(centres)]) / 2.0
                 + (0.5 if i == len(centres) - 1 else 0.0)) % 1.0
                for i in range(len(centres))]
        for m in mids:
            P = P - _crest_envelope(n, m, width * 0.8, width * 0.8,
                                    trough * height)
    return P


# =========================================================================
# The two targets
# =========================================================================
def drag_target(**kw):
    P = drag_control_points(**kw)
    return periodic_spline(P), P


def lift_target(**kw):
    P = lift_control_points(**kw)
    return periodic_spline(P), P


# =========================================================================
# Properties -- what the controller actually scores against
# =========================================================================
def curve_properties(t, S):
    """Shape properties of a curve, all computed the same way for the
    target and for a measured trace so the two are directly comparable."""
    from scipy.signal import find_peaks
    pos, neg = float(S.max()), float(S.min())
    span = max(pos - neg, 1e-12)
    tiled = np.concatenate([S, S, S])
    n = len(S)
    pk, _ = find_peaks(tiled, prominence=0.15 * span,
                       distance=max(1, int(0.05 * n)))
    crests = [p - n for p in pk if n <= p < 2 * n]

    # Area under each crest, split at the minima between them -- this is the
    # "net of each crest" the symmetry requirement is stated in terms of.
    areas = []
    if len(crests) == 1:
        # A single crest owns the whole cycle; splitting at "the minimum
        # between crests" is undefined with only one, and the earlier
        # version silently returned 0.0 for the drag target because of it.
        areas = [float(np.trapz(np.maximum(S, 0.0), dx=1.0 / n))]
    elif len(crests) > 1:
        # Split the cycle at the lowest point between consecutive crests,
        # so each crest's area is measured over its own basin.
        bounds = []
        for i, c in enumerate(crests):
            nxt = crests[(i + 1) % len(crests)] + (n if i == len(crests) - 1 else 0)
            seg = np.arange(c, nxt) % n
            bounds.append(seg[int(np.argmin(S[seg]))] if len(seg) else c)
        for i in range(len(crests)):
            lo, hi = bounds[i - 1], bounds[i]
            idx = np.arange(lo, hi + (n if hi <= lo else 0)) % n
            areas.append(float(np.trapz(np.maximum(S[idx], 0.0), dx=1.0 / n)))
    return {
        "n_crests": len(crests),
        "crest_positions": [float(c) / n for c in crests],
        "pos_peak": pos,
        "neg_peak": neg,
        "net": float(np.trapz(S, dx=1.0 / n)),
        "trough_frac": max(0.0, -neg) / max(pos, 1e-12),
        "min_value": neg,
        "non_negative": bool(neg >= -1e-9),
        "crest_areas": areas,
        "crest_area_imbalance": (abs(areas[0] - areas[1]) /
                                 max(abs(areas[0]) + abs(areas[1]), 1e-12)
                                 if len(areas) == 2 else float("nan")),
        "crest_minus_trough": pos - neg,
    }


def describe(name):
    """Print the mathematical definition and the resulting properties."""
    if name == "drag":
        sp, P = drag_target()
        spec = ("DRAG: one positive crest that dies away, S(t) >= 0 for all t.\n"
                "  P_i = height * raisedcos(circdist(i/n, centre) / w),  "
                "w = rise before the crest, fall after\n"
                "  all P_i >= 0  =>  S(t) >= 0 exactly (convex-hull property)\n"
                "  objective: maximise net = integral S dt, subject to S >= 0")
    else:
        sp, P = lift_target()
        spec = ("LIFT: two equal-area positive crests, dip between them "
                "allowed but minimised.\n"
                "  P_i = max over crests of height*raisedcos(...)  "
                "- trough*raisedcos(... at the midpoints)\n"
                "  crests share width and height and sit antipodally  =>  "
                "equal integrals by construction\n"
                "  objective: maximise net, and maximise (crest - trough); "
                "trough -> 0 recovers S >= 0")
    t, S = evaluate(sp)
    props = curve_properties(t, S)
    print("=" * 74)
    print(spec)
    print("-" * 74)
    for k, v in props.items():
        if isinstance(v, float):
            print(f"  {k:22s} {v:+.4f}")
        else:
            print(f"  {k:22s} {v}")
    print()
    return sp, P, t, S, props


if __name__ == "__main__":
    for nm in ("drag", "lift"):
        describe(nm)


# =========================================================================
# SOLVING for the family's parameters -- no shape numbers are chosen
# =========================================================================
# The compact-support raised-cosine family above is what GUARANTEES the
# shape: the envelope is exactly zero outside the crest, so "returns to zero
# and stays there" is a property of the construction rather than something
# an optimiser has to be persuaded into. What the optimiser chooses is only
# WHERE within that family to sit -- the crest's centre, its rise, its fall,
# its width.
#
# This is the split that the free-control-point version got wrong. Optimising
# 48 unconstrained control points against "maximise net" produced plateaus
# and collapsed slivers, because maximum net genuinely IS a wide dome; the
# peaked, zero-returning shape came from the envelope, not from the search.
# Constrain the family, then optimise inside it.
#
# The only literals below are relative PRIORITIES (w1, w2, w3) -- statements
# about what is worth more, not about the shape.
from scipy.optimize import minimize, NonlinearConstraint   # noqa: E402


def solve_drag_params(w_net=1.0, w_skew=1.0, w_quiet=1.0, n=48, seed=0):
    """Choose (centre, rise, fall) for the drag family.

    Constraints
        rise, fall > 0
        rise + fall < 1        strictly inside the cycle, so a quiet stretch
                               always exists and the crest really does die
                               away rather than wrapping into itself
        fall > rise            right skew
    Objective
        max  w_net*net + w_skew*(fall-rise) + w_quiet*quiet_fraction
    """
    def build(v):
        centre, rise, fall = v
        P = _crest_envelope(n, centre, rise, fall, 1.0)
        return periodic_spline(P), P

    def props(v):
        sp, P = build(v)
        t, S = evaluate(sp)
        pk = max(S.max(), 1e-12)
        Sn = S / pk
        return (float(np.trapz(Sn, dx=1.0 / len(Sn))),      # net
                float(np.mean(Sn < 0.02)))                   # quiet fraction

    def neg(v):
        net, quiet = props(v)
        _, rise, fall = v
        return -(w_net * net + w_skew * (fall - rise) + w_quiet * quiet)

    cons = [NonlinearConstraint(lambda v: np.array(
        [v[1], v[2], 1.0 - (v[1] + v[2]), v[2] - v[1]]), 1e-3, np.inf)]
    rng = np.random.default_rng(seed)
    best, bf = None, np.inf
    for _ in range(12):
        x0 = np.array([rng.uniform(.2, .5), rng.uniform(.05, .25),
                       rng.uniform(.2, .45)])
        r = minimize(neg, x0, method="SLSQP",
                     bounds=[(0, 1), (.01, .5), (.01, .6)],
                     constraints=cons, options={"maxiter": 300, "ftol": 1e-10})
        if r.success and r.fun < bf:
            bf, best = r.fun, r.x
    return dict(centre=best[0], rise=best[1], fall=best[2])


def solve_lift_params(w_net=1.0, w_contrast=1.0, w_quiet=1.0, n=48, seed=0):
    """Choose (width, trough) for the lift family.

    The two crest centres are NOT free: two equal crests spaced evenly around
    a periodic cycle must sit half a cycle apart, and the absolute position
    is pure phase, so (1/4, 3/4) is the canonical representative rather than
    a chosen coordinate. That placement is what makes the crest areas
    identical.

    Constraints
        0 < width < 1/4        crests cannot overlap
        trough >= 0            depth of the dip, as a fraction of the crest
    Objective
        max  w_net*net + w_contrast*(crest - trough) + w_quiet*quiet_fraction
    """
    def props(v):
        width, trough = v
        P = lift_control_points(n, (0.25, 0.75), width, 1.0, trough)
        t, S = evaluate(periodic_spline(P))
        pk = max(S.max(), 1e-12)
        Sn = S / pk
        return (float(np.trapz(Sn, dx=1.0 / len(Sn))),
                float(Sn.max() - Sn.min()),
                float(np.mean(Sn < 0.02)))

    def neg(v):
        net, contrast, quiet = props(v)
        # quiet_fraction is what keeps the two crests SEPARATE: without it,
        # maximising net simply widens both until their bases meet at the
        # crest-spacing bound (width -> 0.249 of a 0.25 ceiling) and the
        # flat zero between them disappears.
        return -(w_net * net + w_contrast * contrast + w_quiet * quiet)

    cons = [NonlinearConstraint(lambda v: np.array(
        [v[0], 0.25 - v[0], v[1]]), 1e-3, np.inf)]
    rng = np.random.default_rng(seed)
    best, bf = None, np.inf
    for _ in range(12):
        x0 = np.array([rng.uniform(.06, .22), rng.uniform(.0, .5)])
        r = minimize(neg, x0, method="SLSQP", bounds=[(.01, .25), (0, 1)],
                     constraints=cons, options={"maxiter": 300, "ftol": 1e-10})
        if r.success and r.fun < bf:
            bf, best = r.fun, r.x
    return dict(width=best[0], trough=best[1])
