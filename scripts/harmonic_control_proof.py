#!/usr/bin/env python3
"""Does a 2nd-harmonic term in the COMMAND survive through to the FORCE?

Status of what this script does and does not establish
-----------------------------------------------------
It is a PREDICTION, not a measurement. No mission in the 3D study ever
commanded an even harmonic (every one of the 904 ran a plain sine with
pitch_k=1), so the rig has never been asked this question and no amount of
re-analysis of that data can answer it.

What makes the prediction worth acting on is that it does not depend on a
tuned fluid model. It assumes only that the blade force is an ODD, faster-
than-linear function of blade velocity -- F = C*v*|v| (quadratic drag) is
the standard form, and the script re-runs everything under F ~ v and
F ~ v^3 as well to show the conclusion is not an artefact of that choice.
Absolute newtons here are meaningless; only the RELATIVE descriptors
(asymmetry, skew, width, count) are being claimed.

The logic being tested: a plain sine has half-wave symmetry
v(t+T/2) = -v(t). Any odd force law then gives F(t+T/2) = -F(t), so the
positive and negative force lobes are forced to be mirror images -- which
is exactly the anti-symmetry seen in the real data (raising the +peak
always deepened the -peak, R^2 0.50/0.47 with opposite signs). Adding an
even harmonic destroys that symmetry in v, hence in F. This checks that it
survives, and by how much.
"""
import numpy as np

N = 20000
t = np.arange(N) / N          # one fundamental period, normalised
W = 2 * np.pi


def theta(a1=1.0, a2=0.0, phi2=0.0, a3=0.0, phi3=0.0):
    """Commanded servo angle: fundamental + 2nd (even) + 3rd (odd) harmonic."""
    return (a1 * np.sin(W * t)
            + a2 * np.sin(2 * W * t + np.radians(phi2))
            + a3 * np.sin(3 * W * t + np.radians(phi3)))


def force(th, law="quadratic"):
    """Blade force from commanded angle, via an odd power of velocity."""
    v = np.gradient(th, t)
    if law == "quadratic":
        return v * np.abs(v)
    if law == "linear":
        return v
    if law == "cubic":
        return v ** 3
    raise ValueError(law)


def descriptors(F):
    """The four features under test, all scale-free."""
    p, n = float(F.max()), float(F.min())
    ip, ino = int(np.argmax(F)), int(np.argmin(F))
    span = p - n
    # asymmetry: 0 = +lobe and -lobe are mirror images, 1 = fully one-sided
    asym = abs(p + n) / span if span > 1e-12 else 0.0
    # skew: fraction of the cycle spent rising into the peak (0.5 = symmetric)
    skew = ((ip - ino) % N) / N
    # width of each lobe at 50% of its own extreme
    pw = float(np.count_nonzero(F >= 0.5 * p)) / N if p > 0 else 0.0
    nw = float(np.count_nonzero(F <= 0.5 * n)) / N if n < 0 else 0.0
    return p, n, asym, skew, pw, nw


def table(title, rows, header):
    print("=" * 86)
    print(title)
    print("=" * 86)
    print(header)
    print("-" * 86)
    for r in rows:
        print(r)
    print()


def main():
    fmt = (f"{'knob':>14} {'+peak':>8} {'-peak':>8} {'+/-ratio':>9} "
           f"{'asym':>6} {'skew':>6} {'+width':>7} {'-width':>7}")

    # ---- 1. a2 magnitude, at phi2=0 (the setting that grows the + lobe)
    rows = []
    for a2 in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
        p, n, asym, skew, pw, nw = descriptors(force(theta(a2=a2, phi2=0)))
        rows.append(f"{'a2=%.1f' % a2:>14} {p:>8.3f} {n:>8.3f} {p/abs(n):>9.2f} "
                    f"{asym:>6.2f} {skew:>6.2f} {pw:>7.3f} {nw:>7.3f}")
    table("1. a2 (2nd-harmonic amplitude) at phi2=0  ->  +PEAK vs -PEAK SCALING",
          rows, fmt)

    # ---- 2. phi2 rotation, at fixed a2
    rows = []
    for phi in range(0, 360, 45):
        p, n, asym, skew, pw, nw = descriptors(force(theta(a2=0.3, phi2=phi)))
        rows.append(f"{'phi2=%d' % phi:>14} {p:>8.3f} {n:>8.3f} {p/abs(n):>9.2f} "
                    f"{asym:>6.2f} {skew:>6.2f} {pw:>7.3f} {nw:>7.3f}")
    table("2. phi2 (2nd-harmonic phase) at a2=0.3  ->  SKEW / which lobe dominates",
          rows, fmt)

    # ---- 3. a3: the odd harmonic pitch_k already produces
    rows = []
    for a3 in (0.0, 0.1, 0.2, 0.3):
        p, n, asym, skew, pw, nw = descriptors(force(theta(a3=a3)))
        rows.append(f"{'a3=%.1f' % a3:>14} {p:>8.3f} {n:>8.3f} {p/abs(n):>9.2f} "
                    f"{asym:>6.2f} {skew:>6.2f} {pw:>7.3f} {nw:>7.3f}")
    table("3. a3 (3rd harmonic, = what pitch_k gives)  ->  WIDTH only, never asymmetry",
          rows, fmt)

    # ---- 4. robustness: does the conclusion depend on the force law?
    print("=" * 86)
    print("4. ROBUSTNESS -- asymmetry produced by a2=0.3,phi2=0 under three force laws")
    print("=" * 86)
    print(f"{'force law':>14} {'+peak/-peak':>13} {'asym':>8}")
    print("-" * 86)
    for law in ("linear", "quadratic", "cubic"):
        p, n, asym, *_ = descriptors(force(theta(a2=0.3, phi2=0), law=law))
        print(f"{law:>14} {p/abs(n):>13.2f} {asym:>8.2f}")
    print()

    # ---- 5. the baseline claim: a pure sine CANNOT be asymmetric
    print("=" * 86)
    print("5. CONTROL CASE -- plain sine, and odd-harmonic-only, under every law")
    print("=" * 86)
    print(f"{'command':>24} {'law':>10} {'asym':>8}   (0.00 => lobes locked as mirrors)")
    print("-" * 86)
    for name, th in (("pure sine", theta()),
                     ("sine + a3 (odd only)", theta(a3=0.3)),
                     ("sine + a2 (even)", theta(a2=0.3, phi2=0))):
        for law in ("linear", "quadratic", "cubic"):
            _, _, asym, *_ = descriptors(force(th, law=law))
            print(f"{name:>24} {law:>10} {asym:>8.2f}")
    print()


if __name__ == "__main__":
    main()
