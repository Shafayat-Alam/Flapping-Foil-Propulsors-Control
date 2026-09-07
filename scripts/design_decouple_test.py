#!/usr/bin/env python3
"""Can peak count and +/- asymmetry be decoupled?

THE PROBLEM. freq_ratio is currently the only knob that moves either one,
and they want different values from it. freq_ratio 0.500 gives 1 peak per
pitch cycle with a +/- ratio of 4.41 (replicated, n=6) -- ideal for the drag
curve. Every setting that yields MORE peaks sits at ratio 0.59-1.06, i.e. no
asymmetry at all. So the lift curve can have its 2 peaks or its asymmetry,
not both.

THE CANDIDATE. `phase` -- the heave-vs-pitch phase offset -- has been pinned
at exactly 0.000 rad for all 212 missions run so far, across three
campaigns. It is the obvious free parameter, and at a rational lock it is
free in a specific and useful way:

  * peak count at a lock is set by the RATIO's denominator (the gait repeats
    after b pitch cycles and the peak pattern with it), so phase cannot
    change it;
  * but phase does change WHERE in that repeating pattern pitch and heave
    coincide, which is precisely what sets whether one stroke loads more
    than the other.

If that holds, phase moves asymmetry at constant peak count -- which is the
decoupling. If it does not, phase will move both together, or neither, and
the lift curve needs a different mechanism entirely.

DESIGN. Three locks with DIFFERENT peak counts (1:2, 2:3, 3:4 -> beats of
2, 3 and 4 pitch cycles), crossed with a full 360 deg phase rotation. Run at
amp_ratio 3.0 because that is the value which zeroes Fz (R^2 = 0.695, the
best-determined relationship measured) -- so every point in this test also
satisfies the Fz constraint, and a winning setting is directly usable rather
than needing to be re-trimmed afterwards.

A second arm sweeps h_a2 (heave 2nd harmonic) at fixed phase, as the backup
candidate: the 104-mission campaign showed heave harmonics move Fy strongly
(ratio span 2.32) while pitch harmonics moved Fx barely at all (span 0.35),
which hints the shaping authority for a given force channel sits on the
OTHER servo from the one assumed.
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design_harmonic_sweep as dhs

# Locks chosen for DISTINCT peak counts: beat period = denominator.
LOCKS = {0.500: "1:2", 0.667: "2:3", 0.750: "3:4"}
PHASES = (0, 45, 90, 135, 180, 225, 270, 315)
AMP_RATIO = 3.00          # the Fz-nulling value
H_A2 = (0.15, 0.30)       # backup arm


def build():
    pts = []
    # --- arm 1: phase rotation at each lock
    for rep in (1, 2):
        for fr in LOCKS:
            for ph in PHASES:
                p = dhs.make_point("P_phase", f"P_fr{fr:.3f}_ph{ph:03d}_r{rep}",
                                   amp_ratio=AMP_RATIO, freq_ratio=fr,
                                   freq_scale=1.0, target="Fx")
                p["phase_deg"] = ph
                p["lock"] = LOCKS[fr]
                pts.append(p)

    # --- arm 2: heave 2nd harmonic at each lock, phase fixed at 0
    for fr in LOCKS:
        for a2 in H_A2:
            for ph2 in (0, 90, 180, 270):
                p = dhs.derate_to_slew(
                    {"h_a2": a2, "h_phi2": ph2, "amp_ratio": AMP_RATIO,
                     "freq_ratio": fr},
                    "Q_heave_harm", f"Q_fr{fr:.3f}_a2{a2:.2f}_ph{ph2:03d}", "Fx")
                p["phase_deg"] = 0
                p["lock"] = LOCKS[fr]
                pts.append(p)
    return pts


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else "decouple_test/sweep_plan.csv"
    pts = build()
    for p in pts:
        p.setdefault("phase_deg", 0)
        p.setdefault("lock", "")
    rng = np.random.default_rng(31)
    for rank, idx in enumerate(rng.permutation(len(pts)), 1):
        pts[idx]["run_order"] = rank
    df = pd.DataFrame(pts).sort_values("run_order")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    df.to_csv(out, index=False)

    print(f"{len(df)} missions -> {out}")
    print(f"  arm 1 (phase):  locks {list(LOCKS)} x phase {list(PHASES)} x 2 reps "
          f"= {2*len(LOCKS)*len(PHASES)}")
    print(f"  arm 2 (h_a2):   locks x a2 {list(H_A2)} x phi2 4 = "
          f"{len(LOCKS)*len(H_A2)*4}")
    print(f"  amp_ratio fixed at {AMP_RATIO} (the Fz-nulling value)")
    print(f"  limit breaches: {int((~df.slew_ok).sum()) + int((~df.amp_ok).sum())}")
    print(f"  peak vel max: pitch {df.pitch_vpk.max():.2f} heave {df.heave_vpk.max():.2f} "
          f"(limit {dhs.SLEW_LIMIT})")
    print(f"  est {len(df)*10.4/60:.1f} min")


if __name__ == "__main__":
    main()
