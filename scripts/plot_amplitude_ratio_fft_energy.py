#!/usr/bin/env python3
"""FFT / spectral-energy comparison of Fx, Fy across amplitude ratio, at the
matched fr=1.0, phase=0 deg operating point used in the amplitude-ratio scaling
check. Tests whether more per-cycle energy shifts into Fx (relative to Fy) as
A1/A2 increases.

Input: pre-filtered slice of data/3D_parametric_study.csv
       (freq_ratio==1.0 and phase_deg==0), columns:
       freq_ratio,amp_ratio,phase_deg,mission_label,time_s,
       s1_pitch_cmd_rad,s1_pitch_fb_rad,s2_heave_cmd_rad,s2_heave_fb_rad,Fx,Fy,Fz

usage: python3 scripts/plot_amplitude_ratio_fft_energy.py <slice_csv> <out_dir>
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

COLS = ["freq_ratio", "amp_ratio", "phase_deg", "mission_label", "time_s",
        "s1_pitch_cmd_rad", "s1_pitch_fb_rad", "s2_heave_cmd_rad", "s2_heave_fb_rad",
        "Fx", "Fy", "Fz"]


def band_energy(sig, dt):
    sig = sig - np.mean(sig)
    n = len(sig)
    spec = np.fft.rfft(sig)
    power = np.abs(spec) ** 2
    return float(np.sum(power))  # Parseval-equivalent total AC energy


def main():
    slice_csv = sys.argv[1]
    out_dir = sys.argv[2] if len(sys.argv) > 2 else "."
    os.makedirs(out_dir, exist_ok=True)

    df = pd.read_csv(slice_csv, header=None, names=COLS)
    amp_ratios = sorted(df["amp_ratio"].unique())

    rows = []
    for ar in amp_ratios:
        sub = df[df["amp_ratio"] == ar].sort_values("time_s")
        t = sub["time_s"].values
        dt = np.median(np.diff(t))
        ex = band_energy(sub["Fx"].values, dt)
        ey = band_energy(sub["Fy"].values, dt)
        rows.append((ar, ex, ey, ex / (ex + ey)))
        print(f"amp_ratio={ar}  n={len(sub)}  E_Fx={ex:.3e}  E_Fy={ey:.3e}  Fx_frac={ex/(ex+ey):.4f}")

    res = pd.DataFrame(rows, columns=["amp_ratio", "E_Fx", "E_Fy", "Fx_energy_fraction"])

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    axes[0].plot(res["amp_ratio"], res["E_Fx"], "o-", color="#c2410c", label="$E(F_x)$")
    axes[0].plot(res["amp_ratio"], res["E_Fy"], "o-", color="#2563eb", label="$E(F_y)$")
    axes[0].set_xlabel("$A_1/A_2$")
    axes[0].set_ylabel("Spectral energy (a.u.)")
    axes[0].set_yscale("log")
    axes[0].legend()
    axes[0].grid(alpha=0.3)
    axes[0].set_title("Total AC spectral energy")

    axes[1].plot(res["amp_ratio"], res["Fx_energy_fraction"], "o-", color="black")
    axes[1].axhline(0.5, color="0.7", lw=0.8, ls="--")
    axes[1].set_xlabel("$A_1/A_2$")
    axes[1].set_ylabel("$E(F_x) / (E(F_x)+E(F_y))$")
    axes[1].set_title("Fx energy fraction")
    axes[1].grid(alpha=0.3)

    fig.suptitle("Fixed: $f_1/f_2=1.0$, $\\Delta\\phi=0^\\circ$   |   Swept: $A_1/A_2$")
    fig.tight_layout()
    out = os.path.join(out_dir, "amplitude_ratio_fft_energy.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


if __name__ == "__main__":
    main()
