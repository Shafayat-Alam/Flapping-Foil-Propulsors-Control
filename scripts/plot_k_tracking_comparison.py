#!/usr/bin/env python3
"""Compare COMMANDED vs MEASURED pitch position across k=1,2,4,8, at a matched
pf=0.500 Hz, phase=0 deg point (PH_0000 in each k1..k8_pf0p500_r1p000 block).

Commanded trajectory is reconstructed exactly from the shaped_sine formula used
by the actual controller (motion_command.shaped_sine):
    y = amp * sign(sin theta) * |sin theta|^(1/(k+1)),  theta = 2*pi*freq*t + phase
This is NOT the k-averaged/estimated shape -- it is the literal formula the
hardware was commanded with, evaluated at the same amp/freq/phase logged in
each mission's own CSV.

usage: python3 scripts/plot_k_tracking_comparison.py <out_dir>
"""
import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

ROOT = "/mnt/usb/Fin_Dynmaics/3D_parametric_k_freq-ratio_phase-shift/raw_data"

POINTS = [
    (1, "k1_pf0p500_r1p000", "PH_0000_150703", "PH_0000"),
    (2, "k2_pf0p500_r1p000", "PH_0000_160402", "PH_0000"),
    (4, "k4_pf0p500_r1p000", "PH_0000_164322", "PH_0000"),
    (8, "k8_pf0p500_r1p000", "PH_0000_171718", "PH_0000"),
]


def shaped_sine(t, freq, amp, phase, k):
    s = math.sin(2 * math.pi * freq * t + phase)
    if s == 0.0:
        return 0.0
    exponent = 1.0 / (k + 1.0)
    return amp * math.copysign(abs(s) ** exponent, s)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)

    fig, axes = plt.subplots(4, 1, figsize=(9, 10), sharex=False)
    for ax, (k, block, mission_dir, label) in zip(axes, POINTS):
        f = os.path.join(ROOT, block, mission_dir, f"{label}.csv")
        df = pd.read_csv(f)
        freq = df["cmd.frequency"].iloc[0]
        amp = df["cmd.pitch_amp"].iloc[0]
        phase = df["cmd.phase"].iloc[0]
        cycles = df["cmd.cycles"].iloc[0]
        active_duration = cycles / freq  # mission's actual commanded duration
        t = df["time_s"].values
        cmd = [shaped_sine(ti, freq, amp, phase, k) if ti <= active_duration else float("nan")
               for ti in t]

        ax.plot(t, cmd, lw=1.2, color="0.3", ls="--", label="commanded (shaped_sine)")
        ax.plot(t, df["s1_position_rad"], lw=0.8, color="#2563eb", label="measured (s1_position_rad)")
        ax.axvline(active_duration, color="0.8", lw=0.8, ls=":")
        ax.set_xlim(0, active_duration * 1.15)
        ax.set_ylabel("pitch (rad)")
        ax.set_title(f"k={k}   pf={freq:.3f} Hz   phase={phase:.3f} rad   amp={amp:.3f} rad", fontsize=10)
        ax.margins(x=0)
        if k == 1:
            ax.legend(fontsize=8, loc="upper right")
    axes[-1].set_xlabel("time (s)")
    fig.tight_layout()
    out = os.path.join(out_dir, "k_tracking_comparison.png")
    fig.savefig(out, dpi=150)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
