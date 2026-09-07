#!/usr/bin/env python3
"""Kinematic-to-force plots for claims 1-7 (Structured 3D Parametric Study:
amplitude ratio, frequency ratio, phase). Reads the existing coupling CSVs
(already verified against the report's stated numbers) and produces one plot
per claim group. Kinematic parameter -> force metric only, no velocity plots.

usage: python3 scripts/plot_3d_claims.py <out_dir>
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = "/mnt/usb/Fin_Dynmaics/3D_parametric_amp-ratio_freq-ratio_phase-shift/coupling"


def plot_amplitude_ratio(out_dir):
    df = pd.read_csv(os.path.join(ROOT, "1_scaling", "scaling_data.csv")).sort_values("amp_ratio")
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(df["amp_ratio"], df["Fx_p2p_N"], "o-", color="#c2410c", label="$F_x$ peak-to-peak")
    ax.plot(df["amp_ratio"], df["Fy_p2p_N"], "o-", color="#2563eb", label="$F_y$ peak-to-peak")
    ax.set_xlabel("$A_1/A_2$ (amplitude ratio)")
    ax.set_ylabel("Force peak-to-peak (N)")
    ax.set_title("Fixed: $f_1/f_2=1.0$, $\\Delta\\phi=0^\\circ$   |   Swept: $A_1/A_2$")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(out_dir, "amplitude_ratio_fx_fy.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)



def plot_claims_3_4(out_dir):
    df = pd.read_csv(os.path.join(ROOT, "2_rate", "rate_data.csv")).sort_values("input_pitch_over_heave")
    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(df["input_pitch_over_heave"], df["f0_Fx_Hz"], "o-", color="#c2410c", label="$f_0(F_x)$")
    ax.plot(df["input_pitch_over_heave"], df["f0_Fy_Hz"], "o-", color="#2563eb", label="$f_0(F_y)$")
    ax.set_xlabel("$f_1/f_2$ (frequency ratio)")
    ax.set_ylabel("Dominant frequency of force signal (Hz)")
    ax.set_title("Fixed: $A_1/A_2=1.0$, $\\Delta\\phi=0^\\circ$   |   Swept: $f_1/f_2$")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(out_dir, "claims_3_4_frequency_ratio.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


def plot_claims_5_6(out_dir):
    df = pd.read_csv(os.path.join(ROOT, "3_skewness", "skewness_data.csv")).sort_values("phase_deg")
    lag = ((df["psi_x_deg"] - df["psi_y_deg"] + 180) % 360 - 180).values
    phase = df["phase_deg"].values

    fig, ax = plt.subplots(figsize=(7, 4.5))
    ax.plot(phase, lag, "o-", color="black")
    ax.axhline(0, color="0.7", lw=0.8, ls="--")
    ax.axvline(180, color="0.8", lw=0.8, ls=":")
    ax.set_xlabel("$\\Delta\\phi$ (deg)")
    ax.set_ylabel("$\\psi_x-\\psi_y$ (deg, wrapped)")
    ax.set_title("Fixed: $A_1/A_2=1.0$, $f_1/f_2=1.0$   |   Swept: $\\Delta\\phi$")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(out_dir, "claims_5_6_phase_skew.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


def plot_claim_7(out_dir):
    df = pd.read_csv(os.path.join(ROOT, "4_peak_count", "peak_count_data.csv"))
    amp_ratios = sorted(df["amp_ratio"].unique())
    fig, axes = plt.subplots(2, len(amp_ratios), figsize=(3.2 * len(amp_ratios), 6), sharex=True, sharey=True)
    for col_i, ar in enumerate(amp_ratios):
        sub = df[df["amp_ratio"] == ar]
        for row_i, (metric, label) in enumerate([("N_peaks_Fx", "$N(F_x)$"), ("N_peaks_Fy", "$N(F_y)$")]):
            ax = axes[row_i, col_i]
            piv = sub.pivot_table(index="freq_ratio", columns="phase_deg", values=metric)
            im = ax.pcolormesh(piv.columns, piv.index, piv.values, cmap="viridis", vmin=1, vmax=4)
            if row_i == 0:
                ax.set_title(f"$A_1/A_2$={ar}", fontsize=9)
            if row_i == 1:
                ax.set_xlabel("$\\Delta\\phi$ (deg)")
            if col_i == 0:
                ax.set_ylabel(f"{label}\n$f_1/f_2$")
    fig.suptitle("Claim 7: $N_{\\mathrm{peaks}}(F_x), N_{\\mathrm{peaks}}(F_y) = g(A_1/A_2,\\ f_1/f_2,\\ \\Delta\\phi)$")
    fig.colorbar(im, ax=axes, shrink=0.6, label="peak count")
    out = os.path.join(out_dir, "claim_7_peak_count_coupling.png")
    fig.savefig(out, dpi=150)
    print("wrote", out)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)
    plot_amplitude_ratio(out_dir)
    plot_claims_3_4(out_dir)
    plot_claims_5_6(out_dir)
    plot_claim_7(out_dir)


if __name__ == "__main__":
    main()
