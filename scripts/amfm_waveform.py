#!/usr/bin/env python3
"""AM/FM sine waveform: the input parameterisation for the shaping experiment.

    theta(t) = C + A(t) * sin(Phi(t))
    Phi(t)   = 2*pi * ( n*t + w(t) )

with A (amplitude envelope) and w (phase warp) both smooth and periodic over
one gait cycle, t in [0,1).

WHY THIS FORM RATHER THAN A FOURIER SERIES
------------------------------------------
A plain harmonic sum  sum a_k sin(k*wt + phi_k)  gives every coefficient a
GLOBAL effect: change one to widen the crest and the trough, the skew and the
peak count all move with it. That is what limited the earlier harmonic
campaign -- a2 on pitch moved Fx asymmetry over a span of only 0.35 while
disturbing everything else.

AM/FM separates the two things the metric list actually asks for:

    A(t)          how TALL the waveform is at this point of the cycle
    dPhi/dt       how FAST the cycle passes through this point

Height and time-distribution are independent in the input, so crest height,
trough depth, crest width, trough width and both skews get distinct knobs
instead of competing for the same coefficient.

Skew in particular is only reachable this way. Odd harmonics preserve
half-wave symmetry (they cannot skew at all) and even harmonics change the
crest/trough amplitude balance at the same time as the shape. A phase warp
changes ONLY the time distribution, leaving amplitude untouched -- so it
skews without side effects.

STRUCTURAL GUARANTEES (properties of the parameterisation, not checks)
---------------------------------------------------------------------
  A(t) > 0        A = A0 * (1 + sum alpha_k cos(2 pi k t + beta_k)) with
                  sum |alpha_k| < 1. Positive for every parameter value.
  dPhi/dt > 0     time never runs backwards. Written as
                  dPhi/dt = 2 pi n (1 + sum c_k cos(2 pi k t + gamma_k))
                  with sum |c_k| < 1.
  smooth          everything is a finite trig sum, so C-infinity, and both
                  theta and theta_dot are available in closed form -- the
                  slew check is exact rather than a finite difference.

The warp is integrated analytically:
    w(t) = sum (c_k / (2 pi k)) sin(2 pi k t + gamma_k)
so Phi is continuous and periodic. Substituting a time-varying frequency
directly into sin(2 pi f(t) t) does NOT do this -- it silently doubles the
frequency sweep, which was verified earlier on the chirp work.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, asdict

import numpy as np

# Safety factors keeping the structural guarantees strictly inside their bounds
POS_SUM_CAP = 0.90     # sum |alpha_k| ceiling  -> A(t) >= 0.10 * A0
WARP_SUM_CAP = 0.85    # sum |c_k|     ceiling  -> dPhi/dt >= 0.15 * nominal


@dataclass
class Knobs:
    """ORTHOGONAL named knobs -- the form the experiment actually sweeps.

    The polar form (alpha_k, beta_k) hides what a parameter does: the SAME
    alpha_1 produces differential height at beta=-pi/2 and a skew/width mix
    at beta=0. Splitting each modulation into its sin and cos components
    instead gives each physical effect its own coordinate, verified above:

        A(t)      sin(2 pi t)  -> differential height (crest up, trough down)
                  cos(4 pi t)  -> common fullness (both lobes together)
        dPhi/dt   cos(2 pi t)  -> differential skew  (crest one way,
                                                      trough the other)
                  sin(2 pi t)  -> differential width (crest narrow,
                                                      trough wide)
                  sin(4 pi t)  -> common skew  (both lobes the same way)
                  cos(4 pi t)  -> common width (both lobes together)

    A_0 is common height and C is bias, giving nine knobs per servo. Both a
    DIFFERENTIAL and a COMMON mode exist for height, width and skew, which is
    what allows crest and trough to be addressed independently rather than
    only in opposition.
    """
    A0: float = 0.4        # common height
    C: float = 0.0         # bias
    n: int = 1             # crest count per cycle
    h_diff: float = 0.0    # differential height
    h_com: float = 0.0     # common fullness
    s_diff: float = 0.0    # differential skew
    s_com: float = 0.0     # common skew
    w_diff: float = 0.0    # differential width
    w_com: float = 0.0     # common width

    def to_params(self) -> "AMFMParams":
        """Convert to the (alpha, beta, c, gamma) form the synthesis uses.

        cos(x + b) = cos b cos x - sin b sin x, so a sin-component S and a
        cos-component K combine to amplitude hypot(K, S) at phase
        atan2(-S, K).
        """
        def polar(cos_comp, sin_comp):
            return math.hypot(cos_comp, sin_comp), math.atan2(-sin_comp, cos_comp)
        a1, b1 = polar(0.0, self.h_diff)          # sin(2 pi t)
        a2, b2 = polar(self.h_com, 0.0)           # cos(4 pi t)
        c1, g1 = polar(self.s_diff, self.w_diff)  # cos & sin of 2 pi t
        c2, g2 = polar(self.w_com, self.s_com)    # cos & sin of 4 pi t
        return AMFMParams(C=self.C, A0=self.A0, n=int(self.n),
                          alpha=[a1, a2], beta=[b1, b2],
                          c=[c1, c2], gamma=[g1, g2]).clipped()

    def as_dict(self):
        return {"A0": self.A0, "C": self.C, "n": self.n,
                "h_diff": self.h_diff, "h_com": self.h_com,
                "s_diff": self.s_diff, "s_com": self.s_com,
                "w_diff": self.w_diff, "w_com": self.w_com}


KNOB_NAMES = ["A0", "C", "n", "h_diff", "h_com", "s_diff", "s_com",
              "w_diff", "w_com"]


@dataclass
class AMFMParams:
    """One servo's waveform.

    C        bias (rad)
    A0       base amplitude (rad)
    n        crest count per gait cycle (integer; sets peak count)
    alpha    amplitude-modulation depths, index k = 1..K
    beta     amplitude-modulation phases (rad)
    c        phase-warp depths, index k = 1..K
    gamma    phase-warp phases (rad)

    Nominal roles at n = 1 (crest at t=1/4, trough at t=3/4):
      alpha_1  differential  crest vs trough height (raises one, lowers other)
      alpha_2  common        both lobes together (overall fullness)
      c_1      skew          shifts time from one half-cycle to the other
      c_2      width         crest wide / trough narrow, or the reverse
    beta_k and gamma_k rotate WHERE each effect lands, which is what turns
    "skew" into "skew left" versus "skew right".
    """
    C: float = 0.0
    A0: float = 0.4
    n: int = 1
    alpha: list = field(default_factory=lambda: [0.0, 0.0])
    beta: list = field(default_factory=lambda: [0.0, 0.0])
    c: list = field(default_factory=lambda: [0.0, 0.0])
    gamma: list = field(default_factory=lambda: [0.0, 0.0])

    def clipped(self) -> "AMFMParams":
        """Scale the modulation vectors back inside their guarantee bounds."""
        a = list(self.alpha)
        cc = list(self.c)
        sa = sum(abs(x) for x in a)
        sc = sum(abs(x) for x in cc)
        if sa > POS_SUM_CAP:
            a = [x * POS_SUM_CAP / sa for x in a]
        if sc > WARP_SUM_CAP:
            cc = [x * WARP_SUM_CAP / sc for x in cc]
        return AMFMParams(C=self.C, A0=self.A0, n=int(self.n),
                          alpha=a, beta=list(self.beta),
                          c=cc, gamma=list(self.gamma))

    def as_dict(self):
        d = asdict(self)
        out = {"C": d["C"], "A0": d["A0"], "n": d["n"]}
        for i, v in enumerate(d["alpha"], 1):
            out[f"alpha{i}"] = v
        for i, v in enumerate(d["beta"], 1):
            out[f"beta{i}"] = v
        for i, v in enumerate(d["c"], 1):
            out[f"c{i}"] = v
        for i, v in enumerate(d["gamma"], 1):
            out[f"gamma{i}"] = v
        return out


def envelope(p: AMFMParams, t):
    """A(t) -- strictly positive by construction."""
    s = np.zeros_like(t)
    for k, (a, b) in enumerate(zip(p.alpha, p.beta), start=1):
        s = s + a * np.cos(2 * np.pi * k * t + b)
    return p.A0 * (1.0 + s)


def phase(p: AMFMParams, t):
    """Phi(t), with the warp integrated analytically so Phi is continuous."""
    w = np.zeros_like(t)
    for k, (c, g) in enumerate(zip(p.c, p.gamma), start=1):
        w = w + (c / (2 * np.pi * k)) * np.sin(2 * np.pi * k * t + g)
    return 2 * np.pi * (p.n * t + w)


def phase_rate(p: AMFMParams, t):
    """dPhi/dt -- strictly positive by construction."""
    s = np.zeros_like(t)
    for k, (c, g) in enumerate(zip(p.c, p.gamma), start=1):
        s = s + c * np.cos(2 * np.pi * k * t + g)
    return 2 * np.pi * (p.n + s)


def envelope_rate(p: AMFMParams, t):
    s = np.zeros_like(t)
    for k, (a, b) in enumerate(zip(p.alpha, p.beta), start=1):
        s = s - a * 2 * np.pi * k * np.sin(2 * np.pi * k * t + b)
    return p.A0 * s


def position(p: AMFMParams, t):
    return p.C + envelope(p, t) * np.sin(phase(p, t))


def velocity(p: AMFMParams, t, period_s):
    """dtheta/dt in rad/s. Exact (product rule), not a finite difference."""
    A, dA = envelope(p, t), envelope_rate(p, t)
    Ph, dPh = phase(p, t), phase_rate(p, t)
    return (dA * np.sin(Ph) + A * dPh * np.cos(Ph)) / period_s


def cycle(p: AMFMParams, period_s, n_samples=720):
    """One precomputed gait cycle: (t_seconds, theta, theta_dot).

    Precomputing the whole period is what lets every limit be verified
    against the ACTUAL commanded array before anything moves, instead of
    discovering a violation part-way through a stroke.
    """
    u = np.linspace(0.0, 1.0, n_samples, endpoint=False)
    return u * period_s, position(p, u), velocity(p, u, period_s)


# =========================================================================
# Feasibility
# =========================================================================
def check(p: AMFMParams, period_s, pos_limit, slew_limit, n_samples=2000):
    """Verify the commanded cycle against position and slew limits.

    Returns (ok, info). Checked on the sampled cycle rather than on bounds,
    because the AM/FM product can peak somewhere neither factor does.
    """
    u = np.linspace(0.0, 1.0, n_samples, endpoint=False)
    th = position(p, u)
    dth = velocity(p, u, period_s)
    A = envelope(p, u)
    dPh = phase_rate(p, u)
    info = {
        "pos_min": float(th.min()), "pos_max": float(th.max()),
        "vel_peak": float(np.max(np.abs(dth))),
        "A_min": float(A.min()), "phase_rate_min": float(dPh.min()),
        "pos_limit": pos_limit, "slew_limit": slew_limit,
    }
    problems = []
    if max(abs(th.min()), abs(th.max())) > pos_limit:
        problems.append(f"position {th.min():+.3f}..{th.max():+.3f} exceeds "
                        f"+/-{pos_limit:.3f} rad")
    if info["vel_peak"] > slew_limit:
        problems.append(f"peak velocity {info['vel_peak']:.2f} > "
                        f"{slew_limit:.2f} rad/s")
    if info["A_min"] <= 0:
        problems.append(f"amplitude envelope went non-positive ({info['A_min']:.3f})")
    if info["phase_rate_min"] <= 0:
        problems.append(f"phase rate went non-positive ({info['phase_rate_min']:.3f}) "
                        f"-- time would run backwards")
    info["problems"] = problems
    return (len(problems) == 0), info


def max_feasible_period(p: AMFMParams, pos_limit, slew_limit):
    """Shortest period (fastest gait) this shape can run inside the slew limit.

    Peak velocity scales as 1/period, so the bound is exact: measure the peak
    at a reference period and scale. Reported because narrowing a crest raises
    phase rate and therefore velocity -- width and achievable speed are
    coupled, and that coupling is physics, not a tuning artefact.
    """
    ref = 1.0
    _, _, dth = cycle(p, ref, 2000)
    peak = float(np.max(np.abs(dth)))
    if peak <= 1e-9:
        return 0.0
    return peak / slew_limit          # minimum period in seconds


if __name__ == "__main__":
    base = AMFMParams(A0=0.4, n=1)
    for label, pp in [
        ("nominal",            base),
        ("crest taller",       AMFMParams(A0=.4, n=1, alpha=[+.30, 0], beta=[-math.pi/2, 0])),
        ("trough deeper",      AMFMParams(A0=.4, n=1, alpha=[-.30, 0], beta=[-math.pi/2, 0])),
        ("skew (c1)",          AMFMParams(A0=.4, n=1, c=[+.45, 0], gamma=[0, 0])),
        ("crest/trough width", AMFMParams(A0=.4, n=1, c=[0, +.45], gamma=[0, 0])),
        ("2 crests",           AMFMParams(A0=.4, n=2)),
        ("biased",             AMFMParams(C=.2, A0=.4, n=1)),
    ]:
        q = pp.clipped()
        ok, info = check(q, 2.0, math.pi/2, 5.5)
        print(f"{label:20s} ok={ok!s:5s} pos {info['pos_min']:+.3f}..{info['pos_max']:+.3f} "
              f"vpk {info['vel_peak']:.2f}  A_min {info['A_min']:.3f}  "
              f"dPhi_min {info['phase_rate_min']:.2f}  Tmin {max_feasible_period(q, math.pi/2, 5.5):.2f}s")
