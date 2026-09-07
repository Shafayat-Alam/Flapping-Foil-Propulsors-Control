#!/usr/bin/env python3
"""The best gait found ALL SESSION, pooled across every campaign.

Every previous "best" reported was scoped to whatever folder produced it.
This scans every raw capture on disk -- stage 1's 100-mission survey, both
drag shape campaigns, both thrust campaigns and their replicates, the
feathering test, the frequency sweep, and the lift-shape attempts -- and
ranks them on Fx by TWO separate criteria, since "drag" and "lift" turned out
to be two different waveform families rather than two different force
channels:

  DRAG family   crest_count == 1.  One crest, dying off, minimal trough.
                Ranked by net Fx (the time-integral of crest minus trough --
                the direct measure of "big crest, small trough" together).

  LIFT family   crest_count == 2.  Two crests with troughs between them.
                Ranked by net Fx first, split second by IMBALANCE between the
                two crests (smaller is better -- "symmetric crests" was the
                explicit ask), with Fy net-zero reported alongside rather
                than filtered, so the trade is visible instead of hidden.

usage:  best_overall.py [folder ...]     (default: every known campaign)
"""
from __future__ import annotations

import glob
import os
import sys

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_analyze as AN                        # noqa: E402
from amfm_metrics import metrics, lobe_peaks      # noqa: E402

DEFAULT_FOLDERS = [
    "amfm_shaping", "amfm_stage2", "amfm_stage3", "amfm_stage3_both",
    "amfm_thrust", "amfm_thrust2", "feather_test", "amfm_freq",
    "amfm_lift_stage2", "amfm_lift_stage3",
]


def scan_one(fp, folder):
    r = AN.load_force(fp)
    if r is None:
        return None
    t, fx, fy, fz = r
    m = metrics(fx)
    peaks = lobe_peaks(fx, +1)
    troughs = lobe_peaks(fx, -1)
    return {
        "folder": folder, "label": os.path.basename(fp)[:-10],
        "crest_count": m["crest_count"],
        "crest_1": peaks[0] if len(peaks) > 0 else 0.0,
        "crest_2": peaks[1] if len(peaks) > 1 else 0.0,
        "trough_1": troughs[0] if len(troughs) > 0 else 0.0,
        "trough_2": troughs[1] if len(troughs) > 1 else 0.0,
        "net_Fx": float(np.mean(fx)),
        "net_Fy": float(np.mean(fy)),
        "path": fp,
    }


def scan_all(folders):
    rows = []
    for folder in folders:
        fpath = os.path.join(WORKSPACE_ROOT, folder)
        for sub in ("data", "data_rep"):
            for fp in sorted(glob.glob(os.path.join(fpath, sub, "*_force.csv"))):
                try:
                    row = scan_one(fp, folder)
                except Exception:
                    continue
                if row is not None:
                    rows.append(row)
    return rows


def main():
    folders = sys.argv[1:] or DEFAULT_FOLDERS
    print(f"scanning: {', '.join(folders)}\n")
    rows = scan_all(folders)
    print(f"{len(rows)} total captures scanned\n")

    drag = [r for r in rows if r["crest_count"] == 1]
    lift = [r for r in rows if r["crest_count"] == 2]
    print(f"   crest_count==1 (drag family): {len(drag)}")
    print(f"   crest_count==2 (lift family): {len(lift)}")
    print(f"   other (3+ or 0 crests)      : {len(rows) - len(drag) - len(lift)}\n")

    NULL_TOL = 0.05   # N -- the tolerance used throughout the thrust campaigns

    def report(name, group, key_extra=None):
        print("=" * 78)
        print(name)
        print("=" * 78)
        if not group:
            print("   none found")
            return None, None
        uncon = max(group, key=lambda r: r["net_Fx"])
        nulled = [r for r in group if abs(r["net_Fy"]) <= NULL_TOL]
        best_n = max(nulled, key=lambda r: r["net_Fx"]) if nulled else None
        for tag, r in (("UNCONSTRAINED best thrust", uncon),
                       (f"BEST with |Fy| <= {NULL_TOL} N", best_n)):
            print(f"\n   {tag}:")
            if r is None:
                print("      none satisfy the null tolerance")
                continue
            extra = key_extra(r) if key_extra else ""
            print(f"      {r['folder']}/{r['label']}")
            print(f"      net Fx {r['net_Fx']:+.3f} N   net Fy {r['net_Fy']:+.3f} N"
                  f"{extra}")
            print(f"      file: {r['path']}")
        return uncon, best_n

    _, drag_best = report(
        "BEST DRAG-FAMILY GAIT  (1 crest, dying off, trough minimised)", drag,
        lambda r: (f"\n      crest {r['crest_1']:.3f} N   trough {r['trough_1']:.3f} N"
                  f"   ratio {r['trough_1']/max(r['crest_1'],1e-9):.3f}"))

    print()
    for r in lift:
        r["imbalance"] = abs(r["crest_1"] - r["crest_2"]) / max(r["crest_1"], 1e-9)
    _, lift_best = report(
        "BEST LIFT-FAMILY GAIT  (2 crests, troughs minimised, Fy nulled)", lift,
        lambda r: (f"\n      crest_1 {r['crest_1']:.3f} N   crest_2 {r['crest_2']:.3f} N"
                  f"   imbalance {r['imbalance']:.1%}"
                  f"\n      trough_1 {r['trough_1']:.3f} N   trough_2 {r['trough_2']:.3f} N"))

    if lift:
        # among the Fy-nulled candidates, separately show the most SYMMETRIC
        # one that still carries reasonable thrust -- "crest symmetric is
        # good" was explicit, and the pure-thrust winner need not be it.
        nulled = [r for r in lift if abs(r["net_Fy"]) <= NULL_TOL]
        if nulled:
            top_net = max(r["net_Fx"] for r in nulled)
            near_top = [r for r in nulled if r["net_Fx"] >= 0.7 * top_net]
            sym = min(near_top, key=lambda r: r["imbalance"])
            print(f"\n   MOST SYMMETRIC among Fy-nulled, within 70% of best thrust "
                  f"({top_net:.3f} N):")
            print(f"      {sym['folder']}/{sym['label']}")
            print(f"      net Fx {sym['net_Fx']:+.3f} N   net Fy {sym['net_Fy']:+.3f} N"
                  f"   imbalance {sym['imbalance']:.1%}")
            print(f"      crest_1 {sym['crest_1']:.3f} N   crest_2 {sym['crest_2']:.3f} N")
            print(f"      file: {sym['path']}")

    return drag_best, lift_best


if __name__ == "__main__":
    main()
