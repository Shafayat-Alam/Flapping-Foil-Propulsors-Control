#!/usr/bin/env python3
"""Did the freq_ratio = 0.5 one-sided-Fx result replicate, and is it usable?

Reports three things, in the order they decide whether the point can be used:

  1. REPRODUCIBILITY -- the ratio at each (freq_ratio, amp_ratio) cell with
     its own spread across replicates. A high mean with a spread as large
     as the effect is not a usable operating point.
  2. SHAPE -- ratio versus freq_ratio, to tell a narrow spike from a broad
     shelf. A spike that only exists at exactly 0.500 cannot be held by a
     controller against drift; a shelf can.
  3. GENERALITY -- whether the feature survives at more than one amp_ratio,
     i.e. whether it is a property of the frequency ratio or of one gait.
"""
import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from analyze_harmonic_sweep import analyse_mission  # noqa: E402

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
FOLDER = os.path.join(WORKSPACE_ROOT, "freq_ratio_replication")


def main():
    import csv
    plan = {r["label"]: r for r in csv.DictReader(
        open(os.path.join(FOLDER, "sweep_plan.csv")))}
    rows = []
    for label, r in plan.items():
        p = os.path.join(FOLDER, "data", f"{label}.csv")
        if not os.path.exists(p):
            continue
        try:
            res = analyse_mission(p, r)
        except Exception as e:
            print(f"  ({label}: {e})")
            continue
        if not res:
            continue
        rec = {"label": label, "freq_ratio": float(r["freq_ratio"]),
               "amp_ratio": float(r["amp_ratio"])}
        for ch, dd in res.items():
            for k, v in dd.items():
                rec[f"{ch.lower()}_{k}"] = v
        rows.append(rec)

    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(FOLDER, "descriptors.csv"), index=False)
    print(f"{len(df)} missions analysed\n")

    # Missions where the rig produced essentially no force are excluded from
    # the statistics rather than averaged in: a stalled or dropped mission
    # has a meaningless +/- ratio (two tiny numbers divided), and letting it
    # into a cell mean would corrupt exactly the quantity under test.
    live = df[df.fx_pos_peak.abs() >= 0.05].copy()
    dead = len(df) - len(live)
    if dead:
        print(f"excluded {dead} mission(s) with |Fx peak| < 0.05 N "
              f"(no usable force -- stalled or dropped, not a measurement)\n")

    print("=" * 78)
    print("1. RATIO (+peak / |-peak|) BY CELL, with replicate spread")
    print("=" * 78)
    print(f"{'freq_ratio':>10} {'amp_ratio':>10} {'n':>3} {'ratio mean':>11} "
          f"{'spread':>8} {'+peak':>8} {'-peak':>8}")
    for (fr, ar), s in live.groupby(["freq_ratio", "amp_ratio"]):
        spread = s.fx_ratio.max() - s.fx_ratio.min() if len(s) > 1 else float("nan")
        print(f"{fr:>10.3f} {ar:>10.2f} {len(s):>3} {s.fx_ratio.mean():>11.2f} "
              f"{spread:>8.2f} {s.fx_pos_peak.mean():>8.3f} {s.fx_neg_peak.mean():>8.3f}")

    print()
    print("=" * 78)
    print("2. SHAPE -- ratio vs freq_ratio (pooled over amp_ratio): spike or shelf?")
    print("=" * 78)
    print(f"{'freq_ratio':>10} {'n':>3} {'ratio mean':>11} {'sd':>7} "
          f"{'min':>7} {'max':>7}")
    for fr, s in live.groupby("freq_ratio"):
        print(f"{fr:>10.3f} {len(s):>3} {s.fx_ratio.mean():>11.2f} "
              f"{s.fx_ratio.std():>7.2f} {s.fx_ratio.min():>7.2f} {s.fx_ratio.max():>7.2f}")

    print()
    print("=" * 78)
    print("3. VERDICT")
    print("=" * 78)
    by_fr = live.groupby("freq_ratio").fx_ratio.mean()
    best_fr = by_fr.idxmax()
    baseline = by_fr.get(0.40, float("nan"))
    print(f"   best freq_ratio = {best_fr:.3f}  (mean ratio {by_fr.max():.2f})")
    print(f"   freq_ratio 0.400 baseline    = {baseline:.2f}")
    print(f"   original single-shot claim   = 4.71 at freq_ratio 0.500")
    got_05 = by_fr.get(0.50, float("nan"))
    print(f"   replicated value at 0.500    = {got_05:.2f}"
          f"  ({'REPRODUCED' if got_05 > 2.0 else 'NOT reproduced'})")
    # Is it a shelf? Compare the best cell against its immediate neighbours.
    frs = sorted(by_fr.index)
    i = frs.index(best_fr)
    nbrs = [by_fr[frs[j]] for j in (i - 1, i + 1) if 0 <= j < len(frs)]
    if nbrs:
        drop = (by_fr.max() - np.mean(nbrs)) / max(by_fr.max(), 1e-9)
        print(f"   neighbours of the best point avg {np.mean(nbrs):.2f} "
              f"-> falls {drop*100:.0f}% off-peak")
        print(f"   -> {'NARROW SPIKE (hard to hold)' if drop > 0.5 else 'BROAD SHELF (usable)'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
