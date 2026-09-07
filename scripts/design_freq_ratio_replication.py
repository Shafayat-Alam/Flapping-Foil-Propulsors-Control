#!/usr/bin/env python3
"""Replication sweep around freq_ratio = 0.5.

WHY: the 104-mission harmonic sweep found its most one-sided Fx waveform at
freq_ratio 0.500 -- +peak 1.014 N against a -peak of only -0.215 N, a
+/- ratio of 4.71 where every other live mission sat between 0.45 and 2.43.
That is the single most useful point in the whole campaign for a "one hump,
little trough" target, and it rests on ONE unreplicated mission.

Two things have to be separated before it can be trusted or used:

  1. Is it reproducible at all, or was it a one-off (a dropped sample, a
     transient, a wake artefact from whatever ran before it in the
     randomised order)?
  2. Is it a NARROW resonance at exactly 0.5, or a broad shelf? The
     surrounding grid points were 0.400 (ratio 0.76) and 0.667 (dead), so
     the campaign cannot distinguish a spike from a plateau -- and that
     distinction decides whether the controller can hold the operating
     point or would fall off it under the slightest drift.

DESIGN. freq_ratio is sampled finely from 0.40 to 0.60 to resolve the shape
of the feature, crossed with three amp_ratio values to check whether it is a
property of the frequency ratio alone or of a particular gait, and repeated
so every point carries its own error bar rather than borrowing the previous
campaign's. Plain sine throughout (no harmonics, no bias): the point of this
run is to characterise ONE knob cleanly, and the 104-mission sweep already
showed the harmonics contribute little to Fx asymmetry (span 0.35 versus
4.13 for freq_ratio).

Randomised order and a 4 s quiet delay, as before, so drift and wake cannot
align with the swept variable.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import design_harmonic_sweep as dhs

FREQ_RATIOS = (0.40, 0.45, 0.475, 0.50, 0.525, 0.55, 0.60)
AMP_RATIOS = (0.67, 1.00, 1.50)
N_REPS = 2


def build():
    pts = []
    for rep in range(1, N_REPS + 1):
        for ar in AMP_RATIOS:
            for fr in FREQ_RATIOS:
                # Plain sine, so freq_scale is limited only by the base gait;
                # keep it at 1.0 for comparability with the campaign's B block,
                # which is the measurement being replicated.
                p = dhs.make_point("FR_replication",
                                   f"FR_fr{fr:.3f}_ar{ar:.2f}_r{rep}",
                                   amp_ratio=ar, freq_ratio=fr, freq_scale=1.0,
                                   target="Fx")
                pts.append(p)
    return pts


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="freq_ratio_replication/sweep_plan.csv")
    ap.add_argument("--seed", type=int, default=11)
    args = ap.parse_args()

    pts = build()
    rng = np.random.default_rng(args.seed)
    for rank, idx in enumerate(rng.permutation(len(pts)), 1):
        pts[idx]["run_order"] = rank

    import pandas as pd
    df = pd.DataFrame(pts).sort_values("run_order")
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    df.to_csv(args.out, index=False)

    bad = int((~df.slew_ok).sum()) + int((~df.amp_ok).sum())
    print(f"{len(df)} missions -> {args.out}")
    print(f"  freq_ratio: {list(FREQ_RATIOS)}")
    print(f"  amp_ratio:  {list(AMP_RATIOS)}   reps: {N_REPS}")
    print(f"  limit breaches: {bad}")
    print(f"  peak velocity max: pitch {df.pitch_vpk.max():.2f} "
          f"heave {df.heave_vpk.max():.2f} (limit {dhs.SLEW_LIMIT})")
    est = len(df) * 10.4
    print(f"  estimated {est/60:.1f} min at the campaign's measured 10.4 s/mission")


if __name__ == "__main__":
    main()
