#!/usr/bin/env python3
"""Plot Fx(t) for each available k value at the pf=0.500 Hz, r=1.000 block of the
k x freq-ratio x phase exploratory sweep -- the only block with a loadcell CSV
present for every one of k=1,2,4,8. Phase differs per k (whatever survived), since
that is what the sparse loadcell recording actually left behind; this is a
best-available-data snapshot, not a controlled matched-phase comparison.

usage: python3 scripts/plot_k_sweep_force_snapshot.py <out_dir>
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

ROOT = "/mnt/usb/Fin_Dynmaics/3D_parametric_k_freq-ratio_phase-shift/raw_data"

# (k, block folder, mission folder) -- the only pf=0.500/r=1.000 loadcell files found
POINTS = [
    (1, "k1_pf0p500_r1p000", "PH_0060_150701", "PH_0060"),
    (2, "k2_pf0p500_r1p000", "PH_0240_160400", "PH_0240"),
    (4, "k4_pf0p500_r1p000", "PH_0360_164320", "PH_0360"),
    (8, "k8_pf0p500_r1p000", "PH_0360_171717", "PH_0360"),
]


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)

    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=False)
    for ax, (k, block, mission_dir, label) in zip(axes, POINTS):
        f = os.path.join(ROOT, block, mission_dir, f"{label}_loadcell.csv")
        df = pd.read_csv(f)
        ax.plot(df["time_s"], df["Fx"], lw=0.6, color="#c2410c")
        ax.axhline(df["Fx"].mean(), color="0.75", lw=0.8)
        ax.set_ylabel("Fx (N)")
        ax.set_title(f"k={k}   pf=0.500 Hz   {label}   ({block})", fontsize=10)
        ax.margins(x=0)
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    out = os.path.join(out_dir, "k_sweep_force_snapshot.png")
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
