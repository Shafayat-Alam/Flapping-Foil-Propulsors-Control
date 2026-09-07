#!/usr/bin/env python3
"""The ten shape metrics, computed identically for any periodic waveform.

Used on the commanded kinematics, the measured kinematics AND the measured
forces, so a change can be traced through the chain: did the SHAPE of the
motion change as intended, and did the FORCE follow? Applying different
definitions at each stage would make that comparison meaningless.

  crest_height    tallest positive excursion
  trough_depth    deepest negative excursion (reported positive)
  crest_width     fraction of the cycle above half the crest height
  trough_width    fraction of the cycle below half the trough depth
  crest_skew      where the peak sits inside its own lobe, 0..1
                  0.5 = symmetric,  <0.5 = fast rise / slow decay
  trough_skew     same, for the negative lobe
  crest_count     number of distinct positive lobes
  bias            cycle mean

Everything except crest_height/trough_depth/bias is scale-free, so shape can
be compared between runs of different magnitude.

LOBE BOUNDARIES are the zero crossings, not fixed fractions of the cycle:
with a warped or biased waveform the crest no longer sits at t=1/4, and
measuring "the crest" over a fixed window would mix parts of both lobes.
"""
from __future__ import annotations

import numpy as np
from scipy.signal import find_peaks

# A lobe smaller than this fraction of the largest is noise, not a feature.
LOBE_REL_FLOOR = 0.15


def _lobes(x, sign):
    """Contiguous index runs where sign*x > 0, joined across the periodic seam."""
    m = (x > 0) if sign > 0 else (x < 0)
    if not m.any():
        return []
    idx = np.flatnonzero(np.diff(m.astype(int)) != 0) + 1
    runs, start = [], 0
    for i in idx:
        runs.append((start, i))
        start = i
    runs.append((start, len(x)))
    runs = [(a, b) for a, b in runs if m[a]]
    # join the seam if the waveform is in the same lobe at both ends
    if len(runs) > 1 and m[0] and m[-1]:
        a0, b0 = runs[0]
        aN, bN = runs[-1]
        runs = runs[1:-1] + [(aN, bN + b0)]      # wrapped run, indices mod n
    return runs


def _lobe_stats(x, runs, sign):
    """(extreme value, extreme index, start, end) for the dominant lobe, and
    the count of significant lobes."""
    n = len(x)
    if not runs:
        return None, 0
    vals = []
    for a, b in runs:
        idx = np.arange(a, b) % n
        seg = x[idx]
        ext = seg.max() if sign > 0 else seg.min()
        vals.append((abs(ext), a, b, idx[int(np.argmax(seg) if sign > 0
                                             else np.argmin(seg))]))
    vals.sort(reverse=True)
    biggest = vals[0][0]
    count = sum(1 for v in vals if v[0] >= LOBE_REL_FLOOR * biggest)
    _, a, b, ipk = vals[0]
    return (a, b, ipk), count


def lobe_peaks(x, sign, floor=LOBE_REL_FLOOR):
    """Peak magnitude of every significant lobe, sorted largest first.

    Used to score TWO crests against each other (are they equal?) rather than
    only ever reporting the dominant one, which is all `metrics()` below
    keeps -- a second, smaller lobe is invisible to it by design.
    """
    runs = _lobes(x, sign)
    if not runs:
        return []
    n = len(x)
    vals = []
    for a, b in runs:
        idx = np.arange(a, b) % n
        seg = x[idx]
        vals.append(abs(float(seg.max() if sign > 0 else seg.min())))
    vals.sort(reverse=True)
    biggest = vals[0]
    return [v for v in vals if v >= floor * biggest]


def metrics(x, prefix=""):
    """Compute all ten metrics for one cycle of a periodic signal."""
    x = np.asarray(x, dtype=float)
    n = len(x)
    out = {}
    mx, mn = float(x.max()), float(x.min())

    out["crest_height"] = max(mx, 0.0)
    out["trough_depth"] = max(-mn, 0.0)
    out["bias"] = float(x.mean())
    out["p2p"] = mx - mn

    # widths, at half of each lobe's own extreme
    out["crest_width"] = (float(np.count_nonzero(x >= 0.5 * mx)) / n
                          if mx > 0 else 0.0)
    out["trough_width"] = (float(np.count_nonzero(x <= 0.5 * mn)) / n
                           if mn < 0 else 0.0)

    for sign, name in ((+1, "crest"), (-1, "trough")):
        runs = _lobes(x, sign)
        stats, count = _lobe_stats(x, runs, sign)
        if stats is None:
            out[f"{name}_skew"] = 0.5
            out[f"{name}_count"] = 0
            continue
        a, b, ipk = stats
        span = b - a
        # position of the extreme WITHIN its own lobe: 0.5 symmetric,
        # <0.5 fast rise then slow decay, >0.5 slow rise then fast decay
        rel = ((ipk - a) % n) / span if span > 0 else 0.5
        out[f"{name}_skew"] = float(np.clip(rel, 0.0, 1.0))
        out[f"{name}_count"] = int(count)

    if prefix:
        out = {f"{prefix}_{k}": v for k, v in out.items()}
    return out


METRIC_NAMES = ["crest_height", "trough_depth", "crest_width", "trough_width",
                "crest_skew", "trough_skew", "crest_count", "bias"]


if __name__ == "__main__":
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from amfm_waveform import AMFMParams, position
    import math
    u = np.linspace(0, 1, 720, endpoint=False)
    cases = [
        ("nominal",       AMFMParams(A0=.4, n=1)),
        ("crest taller",  AMFMParams(A0=.4, n=1, alpha=[+.35, 0], beta=[-math.pi/2, 0])),
        ("trough deeper", AMFMParams(A0=.4, n=1, alpha=[-.35, 0], beta=[-math.pi/2, 0])),
        ("skew +c1",      AMFMParams(A0=.4, n=1, c=[+.55, 0], gamma=[0, 0])),
        ("skew -c1",      AMFMParams(A0=.4, n=1, c=[-.55, 0], gamma=[0, 0])),
        ("width +c2",     AMFMParams(A0=.4, n=1, c=[0, +.55], gamma=[0, 0])),
        ("width -c2",     AMFMParams(A0=.4, n=1, c=[0, -.55], gamma=[0, 0])),
        ("2 crests",      AMFMParams(A0=.4, n=2)),
        ("bias +0.15",    AMFMParams(C=.15, A0=.4, n=1)),
    ]
    hdr = f"{'case':16s}{'crestH':>8}{'trghD':>8}{'crestW':>8}{'trghW':>8}" \
          f"{'crestSk':>9}{'trghSk':>8}{'cnt':>5}{'bias':>8}"
    print(hdr); print("-" * len(hdr))
    for lab, p in cases:
        m = metrics(position(p.clipped(), u))
        print(f"{lab:16s}{m['crest_height']:8.3f}{m['trough_depth']:8.3f}"
              f"{m['crest_width']:8.3f}{m['trough_width']:8.3f}"
              f"{m['crest_skew']:9.3f}{m['trough_skew']:8.3f}"
              f"{m['crest_count']:5d}{m['bias']:8.3f}")
