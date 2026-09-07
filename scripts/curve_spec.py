#!/usr/bin/env python3
"""Target force curves as CONSTRAINED B-SPLINES. The input format, from here on.

A target is a parametric cubic B-spline

    C(t) = sum_i P_i N_{i,k}(t),      P_i = (x_i, y_i)

whose control points are found by solving a constrained optimisation. The
SHAPE is fixed by topological constraints on the control polygon; the
OBJECTIVE only chooses among shapes that already satisfy them. Nothing about
the curve is typed in as a coordinate.

Both x and y are splined, so the control points move in time as well as in
force -- the x_i are optimisation variables under a strict ordering
constraint, not a fixed grid.

------------------------------------------------------------------------
DRAG -- one crest, no troughs
------------------------------------------------------------------------
  control points   P_0 .. P_4
  constraints      y_0 = 0,  y_4 = 0            decays to zero at both ends
                   0 <= y_1 <= y_2              strict rise
                   y_2 >= y_3 >= 0              strict fall, no troughs
                   x_0 < x_1 < x_2 < x_3 < x_4  sequential order
  objective        max  w1 * net  +  w2 * (crest - trough)

  The monotone rise/fall chain is what forbids troughs: y can only climb to
  y_2 and then descend, so no interior minimum can form. Pinning
  y_0 = y_4 = 0 is what makes it "die down" -- and because that is a
  CONSTRAINT rather than an objective, widening the crest to chase net can
  never stop the curve returning to zero. Width is deliberately not
  penalised.

------------------------------------------------------------------------
LIFT -- two symmetric crests, trough between them
------------------------------------------------------------------------
  control points   P_0 .. P_4
  constraints      y_0 = 0,  y_4 = 0
                   y_1 = y_3                    the two crests are equal
                   y_1 > y_2                    the middle point is a trough
                   x_2 - x_1 = x_3 - x_2        trough centred between crests
                   x_0 < x_1 < x_2 < x_3 < x_4
  objective        max  w1 * net  +  w2 * (crest - trough)

  y_1 = y_3 together with the centred trough makes the two crests mirror
  images, so their individual nets are equal BY CONSTRUCTION -- the "net the
  same for each crest" requirement, met exactly rather than approached by a
  penalty. The trough's depth is left free (it may go negative); only its
  POSITION is pinned, by the ordering constraint.

The weights w1, w2 are positive and only their ratio matters: they state
what is worth more, not what value to hit.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import minimize, NonlinearConstraint
from scipy.signal import find_peaks

DEGREE = 3
N_SAMPLES = 400          # numerical resolution only
EPS = 1e-3               # strictness margin for the "<" ordering constraints


# =========================================================================
# Parametric B-spline through a control polygon
# =========================================================================
def spline_xy(X, Y, degree=DEGREE, n_samples=N_SAMPLES):
    """Evaluate the parametric curve C(u) = (x(u), y(u)).

    Clamped knot vector, so the curve starts exactly at P_0 and ends exactly
    at P_4. That is what makes y_0 = y_4 = 0 bite: with a clamped spline the
    endpoint values are attained, not merely approached.
    """
    X, Y = np.asarray(X, float), np.asarray(Y, float)
    n = len(X)
    n_int = n - degree - 1
    knots = np.concatenate([np.zeros(degree + 1),
                            np.linspace(0, 1, n_int + 2)[1:-1],
                            np.ones(degree + 1)])
    u = np.linspace(0.0, 1.0, n_samples)
    return BSpline(knots, X, degree)(u), BSpline(knots, Y, degree)(u)


def resample_uniform(x, y, n=N_SAMPLES):
    """Put the curve on a uniform time grid so it can be compared with a
    measured trace. x(u) is monotone under the ordering constraint, so the
    interpolation is well defined."""
    t = np.linspace(x[0], x[-1], n)
    return t, np.interp(t, x, y)


# =========================================================================
# Properties -- the vocabulary the objectives are written in
# =========================================================================
def properties(t, F):
    crest = float(F.max())
    trough = float(F.min())
    span = max(crest - trough, 1e-12)
    net = float(np.trapz(F, t))

    pk, _ = find_peaks(F, prominence=0.05 * span)
    crests = list(pk)

    # Net under each crest, split at the interior minimum between them.
    # This is the quantity the lift spec requires to be equal.
    areas = []
    if len(crests) >= 2:
        lo = crests[0] + int(np.argmin(F[crests[0]:crests[-1] + 1]))
        areas = [float(np.trapz(F[:lo + 1], t[:lo + 1])),
                 float(np.trapz(F[lo:], t[lo:]))]
    elif len(crests) == 1:
        areas = [net]

    # INTERIOR trough: the lowest point strictly between the first and last
    # crest. This, not the global minimum, is what "the trough between the
    # crests" means -- the global minimum is the pinned zero at the
    # endpoints, so an objective written against it is indifferent to
    # whether an interior notch exists at all. Written that way, the lift
    # solve drove the middle control point to 0.999 against crests of 1.000,
    # merging the two crests into a single plateau while still scoring a
    # maximal "crest - trough".
    interior = crest
    if len(crests) >= 2:
        interior = float(F[crests[0]:crests[-1] + 1].min())

    return {
        "crest": crest,
        "trough": trough,
        "interior_trough": interior,
        "interior_prominence": crest - interior,
        "prominence": crest - trough,
        "net": net,
        "n_crests": len(crests),
        "crest_areas": areas,
        "crest_net_imbalance": (abs(areas[0] - areas[1]) /
                                max(abs(areas[0]) + abs(areas[1]), 1e-12)
                                if len(areas) == 2 else 0.0),
        "min_value": trough,
        "no_troughs": bool(trough >= -1e-9),
    }


# =========================================================================
# Specs
# =========================================================================
@dataclass
class DragSpec:
    """One crest, no troughs, skewed right.

    w1 weights net, w2 crest prominence, w3 the right-skew (how far the
    centre of area trails the peak).
    """
    w1: float = 1.0
    w2: float = 1.0
    w3: float = 1.0
    name: str = "drag"

    def describe(self):
        return ("drag:\n"
                "  P_0..P_4,  C(t) = sum P_i N_{i,3}(t)\n"
                "  y_0 = y_4 = 0                 decays to zero at both ends\n"
                "  0 <= y_1 <= y_2               strict rise\n"
                "  y_2 >= y_3 >= 0               strict fall, no troughs\n"
                "  x_0 < x_1 < x_2 < x_3 < x_4   sequential\n"
                "  x_4 - x_2 > x_2 - x_0         skewed right\n"
                f"  max  {self.w1:g}*net + {self.w2:g}*(crest - trough)"
                f" + {self.w3:g}*skew")


@dataclass
class LiftSpec:
    """Two symmetric crests with a trough between them."""
    w1: float = 1.0
    w2: float = 1.0
    name: str = "lift"

    def describe(self):
        return ("lift:\n"
                "  P_0..P_6,  C(t) = sum P_i N_{i,3}(t)\n"
                "  y_0 = y_6 = 0\n"
                "  y_1 = y_5, y_2 = y_4                     crests equal\n"
                "  valley <= 0                   crests separate to baseline\n"
                "  polygon mirror-symmetric      => crest nets identical\n"
                "  x_0 < x_1 < x_2 < x_3 < x_4   sequential\n"
                f"  max  {self.w1:g}*net + {self.w2:g}*(crest - trough)")


# =========================================================================
# Solvers
# =========================================================================
def _peak_time(tf):
    """Time of the curve's maximum, normalised to the cycle."""
    t, F = tf
    return float((t[int(np.argmax(F))] - t[0]) / max(t[-1] - t[0], 1e-12))


def solve_drag(spec: DragSpec, n_restarts=8, seed=0):
    """Variables: x1, x2, x3, y1, y2, y3.  x0=0, x4=1, y0=y4=0 fixed.

    y is bounded to [0, 1]: the curve is scale-free, so the ceiling merely
    fixes the unit and stops `max crest` running away. The rise/fall
    ORDERING is imposed as inequality constraints on the variables, not as a
    penalty, so no choice of w1/w2 can produce a trough.
    """
    def unpack(v):
        x1, x2, x3, y1, y2, y3 = v
        return np.array([0.0, x1, x2, x3, 1.0]), np.array([0.0, y1, y2, y3, 0.0])

    def curve(v):
        X, Y = unpack(v)
        return resample_uniform(*spline_xy(X, Y))

    def neg_obj(v):
        t, F = curve(v)
        pr = properties(t, F)
        # skew reward: how much later the curve's centre of area sits than
        # its peak, normalised by the cycle. Positive = long tail to the
        # right. Written on the CURVE rather than the polygon so it measures
        # the shape actually produced.
        centroid = float(np.trapz(t * np.maximum(F, 0), t) /
                         max(np.trapz(np.maximum(F, 0), t), 1e-12))
        skew = centroid - t[int(np.argmax(F))]
        return -(spec.w1 * pr["net"] + spec.w2 * pr["prominence"]
                 + spec.w3 * skew)

    cons = [
        NonlinearConstraint(lambda v: np.array(
            [v[0], v[1] - v[0], v[2] - v[1], 1.0 - v[2]]), EPS, np.inf),
        NonlinearConstraint(lambda v: np.array([v[4] - v[3], v[4] - v[5]]),
                            0.0, np.inf),
        # RIGHT SKEW, measured on the CURVE: the peak must fall in the first
        # half of the cycle, so the decay occupies more of it than the rise.
        # Stated on the polygon (x_2 < 0.5) it did not bite -- the curve's
        # own peak need not sit at x_2 -- and as a weighted objective it was
        # outbid by net, which is maximised by a symmetric dome (measured
        # skew wandered between -0.038 and +0.112 with no monotone response
        # to its weight). On the curve it is unambiguous.
        NonlinearConstraint(lambda v: 0.5 - _peak_time(curve(v)), EPS, np.inf),
    ]
    bounds = [(0, 1)] * 6
    rng = np.random.default_rng(seed)
    best, best_f = None, np.inf
    for _ in range(n_restarts):
        x0 = np.concatenate([np.sort(rng.uniform(.1, .9, 3)),
                             rng.uniform(.2, 1.0, 3)])
        r = minimize(neg_obj, x0, method="SLSQP", bounds=bounds,
                     constraints=cons, options={"maxiter": 250, "ftol": 1e-9})
        if r.success and r.fun < best_f:
            best_f, best = r.fun, r.x
    if best is None:
        raise RuntimeError("drag solve failed")
    return unpack(best), curve(best)


def _interior_min(tf):
    """Lowest point of the curve strictly between its two outermost crests.

    Falls back to the global minimum when fewer than two crests are found,
    so a degenerate single-hump candidate is scored as having no separation
    at all rather than silently passing."""
    t, F = tf
    span = max(F.max() - F.min(), 1e-12)
    pk, _ = find_peaks(F, prominence=0.05 * span)
    if len(pk) < 2:
        return float(F.max())
    return float(F[pk[0]:pk[-1] + 1].min())


def solve_lift(spec: LiftSpec, n_restarts=10, seed=0):
    """Seven MIRROR-SYMMETRIC control points.

        X = [0, x1, x2, 1/2, 1-x2, 1-x1, 1]
        Y = [0, h1, h2, tr,  h2,   h1,   0]

    Two changes from the five-point layout, both forced by what it could not
    express:

      * SEVEN points, not five. A cubic B-spline over five control points has
        a single interior knot -- two spans -- and simply cannot bend into
        two separated humps with a valley that reaches the baseline. Asked
        to, it collapsed the crests into a 0.002-wide sliver (the only way to
        make the curve dip to zero between them). Seven points give four
        spans, which is enough for rise-crest-fall-valley-rise-crest-fall.

      * EXACT MIRROR SYMMETRY of the polygon about t = 1/2. The clamped knot
        vector for seven points is itself symmetric, so a symmetric polygon
        produces a symmetric curve, and the two crests then have IDENTICAL
        nets -- not merely similar. The five-point version left a residual
        crest-net imbalance of 0.0426 because its knot vector was not
        symmetric about the middle control point.

    tr <= 0 pins the valley to the baseline or below, so the crests are
    separate excursions rather than two bumps on a raised plateau.
    """
    def unpack(v):
        x1, x2, h1, h2, tr = v
        X = np.array([0.0, x1, x2, 0.5, 1.0 - x2, 1.0 - x1, 1.0])
        Y = np.array([0.0, h1, h2, tr, h2, h1, 0.0])
        return X, Y

    def curve(v):
        X, Y = unpack(v)
        return resample_uniform(*spline_xy(X, Y))

    def neg_obj(v):
        t, F = curve(v)
        pr = properties(t, F)
        if pr["n_crests"] < 2:
            return 1e3 - pr["interior_prominence"]
        return -(spec.w1 * pr["net"] + spec.w2 * pr["interior_prominence"])

    cons = [
        # 0 < x1 < x2 < 1/2  (strict ordering; the mirror supplies the rest)
        NonlinearConstraint(lambda v: np.array(
            [v[0], v[1] - v[0], 0.5 - v[1]]), EPS, np.inf),
        # the valley reaches the baseline, measured on the CURVE
        NonlinearConstraint(lambda v: -_interior_min(curve(v)), 0.0, np.inf),
    ]
    bounds = [(0, .5), (0, .5), (0, 1), (0, 1), (-1, 0)]
    rng = np.random.default_rng(seed)
    best, best_f = None, np.inf
    for _ in range(n_restarts):
        x0 = np.array([rng.uniform(.03, .18), rng.uniform(.20, .42),
                       rng.uniform(.3, 1.0), rng.uniform(.3, 1.0),
                       rng.uniform(-.8, -.05)])
        r = minimize(neg_obj, x0, method="SLSQP", bounds=bounds,
                     constraints=cons, options={"maxiter": 300, "ftol": 1e-9})
        if r.success and r.fun < best_f:
            best_f, best = r.fun, r.x
    if best is None:
        raise RuntimeError("lift solve failed")
    return unpack(best), curve(best)


SOLVERS = {"drag": (DragSpec, solve_drag), "lift": (LiftSpec, solve_lift)}


def solve(name, **kw):
    cls, fn = SOLVERS[name]
    spec = cls(**kw)
    (X, Y), (t, F) = fn(spec)
    return spec, (X, Y), (t, F), properties(t, F)


if __name__ == "__main__":
    for nm in ("drag", "lift"):
        spec, (X, Y), (t, F), pr = solve(nm)
        print("=" * 72)
        print(spec.describe())
        print("  ---- solved control polygon ----")
        print("    x_i = " + "  ".join(f"{v:.4f}" for v in X))
        print("    y_i = " + "  ".join(f"{v:+.4f}" for v in Y))
        print("  ---- resulting curve ----")
        for k in ("n_crests", "crest", "trough", "prominence", "net",
                  "crest_net_imbalance", "no_troughs"):
            v = pr[k]
            print(f"    {k:22s} {v if isinstance(v, bool) else f'{v:+.4f}'}")
        print()
