#!/usr/bin/env python3
"""The reference formulation, implemented as given.

    C(t) = sum_i P_i N_{i,k}(t),     P_i = (x_i, y_i)

Objectives are written on the CONTROL POINTS, as in the reference -- y_2 and
(y_1 - y_2), not quantities measured off the evaluated curve.

The width term -(x_3 - x_1) is KEPT, alongside net. Dropping it (net alone)
collapses both shapes: net is maximised by a crest spanning the whole cycle,
so drag became a broad symmetric dome touching zero only at its endpoints
and lift a shallow double-hump on a raised shelf, with the weight w2 having
almost no effect on either. The width penalty is what makes the crest tall
and THIN, and it is the term that has to oppose net for the shape to survive.
w3 sets how hard it pushes.

------------------------------------------------------------------------
DRAG -- one crest, tallest, no troughs
------------------------------------------------------------------------
  P_0 .. P_4
    y_0 = 0,  y_4 = 0                  decays to zero at both ends
    0 <= y_1 <= y_2                    strict rise
    y_2 >= y_3 >= 0                    strict fall, no troughs allowed
    x_0 < x_1 < x_2 < x_3 < x_4        sequential order
  max   w1 * net  +  w2 * y_2  -  w3 * (x_4 - x_0)   [support width]

  The monotone chain y_1 <= y_2 >= y_3 with both ends pinned at zero is what
  forbids an interior minimum: y can only climb to y_2 and then descend.

------------------------------------------------------------------------
LIFT -- two symmetric crests, one trough
------------------------------------------------------------------------
  P_0 .. P_4
    y_0 = 0,  y_4 = 0
    y_1 = y_3                          crests equal
    y_1 > y_2 >= 0                     the middle point is the trough
    x_0 < x_1 < x_2 < x_3 < x_4
    x_1 = 1 - x_3,  x_2 = 1/2          full mirror symmetry
  max   w1 * net  +  w2 * (y_1 - y_2)  -  w3 * (x_3 - x_1)

  The mirror symmetry is what makes the two crests carry the SAME net --
  stated as a constraint on the layout rather than approached by a penalty.
  The trough sits between the crests by the ordering constraint.
"""
from __future__ import annotations

import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import minimize, NonlinearConstraint

DEGREE = 3
N = 500
EPS = 1e-3


def curve_from(X, Y, degree=DEGREE, n=N):
    """Clamped cubic B-spline on [x_0, x_4], and exactly ZERO outside it.

    COMPACT SUPPORT is the piece the plain formulation was missing. With x_0
    and x_4 pinned to the ends of the cycle, the crest's support is the whole
    cycle by construction: there is nowhere for the curve to be flat-zero, so
    "dies down and stays there" is unreachable no matter how the objective is
    weighted -- pushing the width penalty just collapsed the interior points
    to a 0.002-wide sliver while the curve still spanned 0 to 1.

    Letting x_0 > 0 and x_4 < 1 float, and defining the curve as zero outside
    [x_0, x_4], makes the crest a genuinely local event with quiet cycle on
    either side. The width penalty then does what it was meant to: it narrows
    the crest rather than just crowding its interior knots.
    """
    X, Y = np.asarray(X, float), np.asarray(Y, float)
    n_int = len(X) - degree - 1
    knots = np.concatenate([np.zeros(degree + 1),
                            np.linspace(0, 1, n_int + 2)[1:-1],
                            np.ones(degree + 1)])
    u = np.linspace(0, 1, n)
    xs, ys = BSpline(knots, X, degree)(u), BSpline(knots, Y, degree)(u)
    t = np.linspace(0.0, 1.0, n)                 # always the full cycle
    F = np.interp(t, xs, ys, left=0.0, right=0.0)
    F[(t < xs[0]) | (t > xs[-1])] = 0.0
    return t, F


def net_of(t, F):
    return float(np.trapz(F, t))


def report(t, F):
    from scipy.signal import find_peaks
    span = max(F.max() - F.min(), 1e-12)
    pk, _ = find_peaks(F, prominence=0.05 * span)
    areas = []
    if len(pk) >= 2:
        lo = pk[0] + int(np.argmin(F[pk[0]:pk[-1] + 1]))
        areas = [float(np.trapz(F[:lo + 1], t[:lo + 1])),
                 float(np.trapz(F[lo:], t[lo:]))]
    return {
        "n_crests": len(pk),
        "crest": float(F.max()),
        "trough": float(F.min()),
        "net": net_of(t, F),
        "crest_areas": areas,
        "imbalance": (abs(areas[0] - areas[1]) / max(areas[0] + areas[1], 1e-12)
                      if len(areas) == 2 else 0.0),
        "no_troughs": bool(F.min() >= -1e-9),
    }


# =========================================================================
# DRAG
# =========================================================================
def solve_drag(w1=1.0, w2=1.0, w3=1.0, restarts=14, seed=0):
    def poly(v):
        xs, x1, x2, x3, xe, y1, y2, y3 = v
        return (np.array([xs, x1, x2, x3, xe]),
                np.array([0.0, y1, y2, y3, 0.0]))

    def neg(v):
        X, Y = poly(v)
        t, F = curve_from(X, Y)
        # width penalised on the SUPPORT (x_4 - x_0), not on the interior
        # spread (x_3 - x_1). Penalising the interior only crowded the middle
        # control points together while x_0 and x_4 stayed at the cycle ends,
        # so the crest still occupied the whole cycle. The support is the
        # quantity that actually decides how much quiet time surrounds it.
        return -(w1 * net_of(t, F) + w2 * v[6]
                 - w3 * (v[4] - v[0]))

    cons = [
        # x_0 < x_1 < x_2 < x_3 < x_4, all inside the cycle
        NonlinearConstraint(lambda v: np.array(
            [v[0], v[1] - v[0], v[2] - v[1], v[3] - v[2], v[4] - v[3],
             1.0 - v[4]]), EPS, np.inf),
        NonlinearConstraint(lambda v: np.array(
            [v[6] - v[5], v[6] - v[7], v[5], v[7]]), 0.0, np.inf),
    ]
    rng = np.random.default_rng(seed)
    best, bf = None, np.inf
    for _ in range(restarts):
        xx = np.sort(rng.uniform(.05, .95, 5))
        x0 = np.concatenate([xx, np.sort(rng.uniform(.2, 1, 3))[[0, 2, 1]]])
        r = minimize(neg, x0, method="SLSQP", bounds=[(0, 1)] * 8,
                     constraints=cons, options={"maxiter": 400, "ftol": 1e-11})
        if r.success and r.fun < bf:
            bf, best = r.fun, r.x
    X, Y = poly(best)
    return (X, Y), curve_from(X, Y)


# =========================================================================
# LIFT
# =========================================================================
def solve_lift(w1=1.0, w2=1.0, w3=1.0, restarts=14, seed=0):
    def poly(v):
        x1, y1, y2 = v
        return (np.array([0, x1, 0.5, 1 - x1, 1.0]),
                np.array([0, y1, y2, y1, 0]))

    def neg(v):
        X, Y = poly(v)
        t, F = curve_from(X, Y)
        # support width under the mirror layout is (1 - x1) - x1
        return -(w1 * net_of(t, F) + w2 * (v[1] - v[2])
                 - w3 * (1.0 - 2.0 * v[0]))

    cons = [
        NonlinearConstraint(lambda v: np.array([v[0], 0.5 - v[0]]), EPS, np.inf),
        NonlinearConstraint(lambda v: np.array([v[1] - v[2], v[2]]), EPS, np.inf),
    ]
    rng = np.random.default_rng(seed)
    best, bf = None, np.inf
    for _ in range(restarts):
        x0 = np.array([rng.uniform(.05, .45), rng.uniform(.4, 1),
                       rng.uniform(.0, .4)])
        r = minimize(neg, x0, method="SLSQP",
                     bounds=[(0, .5), (0, 1), (0, 1)],
                     constraints=cons, options={"maxiter": 400, "ftol": 1e-11})
        if r.success and r.fun < bf:
            bf, best = r.fun, r.x
    X, Y = poly(best)
    return (X, Y), curve_from(X, Y)


if __name__ == "__main__":
    for nm, fn in (("DRAG", solve_drag), ("LIFT", solve_lift)):
        (X, Y), (t, F) = fn()
        r = report(t, F)
        print("=" * 66)
        print(f"{nm}")
        print("  x_i = " + "  ".join(f"{v:.4f}" for v in X))
        print("  y_i = " + "  ".join(f"{v:+.4f}" for v in Y))
        for k, v in r.items():
            print(f"  {k:14s} {v}")
        print()
