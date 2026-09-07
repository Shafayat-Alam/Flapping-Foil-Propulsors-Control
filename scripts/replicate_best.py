#!/usr/bin/env python3
"""Replicate a campaign's best gait, to tell a real result from rig noise.

Every headline number reported this session (+0.399 N, +0.226 N, gait C 09's
+0.152 N reproducing to +0.163 N) has been a SINGLE capture. Stage 1's own
noise floor put replicate spread on Fx_bias at several times smaller than the
gains being chased, but never directly on the winning gait itself -- this
closes that gap before the number gets used for anything.

usage:  replicate_best.py <folder> [--reps 6]
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_experiment as EX      # noqa: E402
from amfm_waveform import Knobs   # noqa: E402
import amfm_shaper as SH          # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--reps", type=int, default=6)
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)

    rp = os.path.join(folder, "result.json")
    r = json.load(open(rp))
    kb = r["best_knobs"]
    d1 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s1.")}
    d2 = {k.split(".", 1)[1]: v for k, v in kb.items() if k.startswith("s2.")}
    d1["n"] = int(d1.get("n", EX.NOMINAL[1].n))
    d2["n"] = int(d2.get("n", EX.NOMINAL[2].n))
    k1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
    k2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
    print(f"replicating best gait from {rp}  ({a.reps} repeats)")

    rig = SH.Rig()
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop()
        return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    nets, verts = [], []
    try:
        for i in range(a.reps):
            paths, err = rig.measure(k1, k2, os.path.join(folder, "data_rep"), f"rep_{i:02d}")
            if paths is None:
                print(f"[{i+1}/{a.reps}]: {err}")
                continue
            m = SH.evaluate(paths, "Fx", None)
            if m is None:
                print(f"[{i+1}/{a.reps}]: no force data")
                continue
            nets.append(m.get("bias", 0.0))
            verts.append(m.get("other_bias", 0.0))
            print(f"[{i+1}/{a.reps}]  net_Fx {nets[-1]:+.3f}   net_Fy {verts[-1]:+.3f}")
    finally:
        rig.stop()

    if not nets:
        print("nothing measured")
        return 1
    nets, verts = np.array(nets), np.array(verts)
    print("\n" + "=" * 60)
    print(f"net_Fx : mean {nets.mean():+.3f}  sd {nets.std():.3f}  "
          f"(cv {100*nets.std()/max(abs(nets.mean()),1e-9):.0f}%)")
    print(f"net_Fy : mean {verts.mean():+.3f}  sd {verts.std():.3f}")
    print(f"reported single-run value: {r['err_best']:+.3f} (objective, not N -- "
          f"see result.json best_knobs)")
    json.dump({"n": len(nets), "net_Fx_mean": float(nets.mean()),
               "net_Fx_sd": float(nets.std()), "net_Fy_mean": float(verts.mean()),
               "net_Fy_sd": float(verts.std())},
              open(os.path.join(folder, "replicate.json"), "w"), indent=2)
    print(f"\nwrote {folder}/replicate.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
