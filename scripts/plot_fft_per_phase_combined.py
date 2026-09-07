#!/usr/bin/env python3
"""
plot_fft_per_phase_combined.py — same data as plot_fft_per_phase.py, but each
phase's three force FFTs (Fx thrust, Fy lateral, Fz heave) are three separate
subplot PANELS stacked into ONE image file per phase (not one overlaid plot,
not three separate files).

Reads the FFT CSV written by make_fft_csv.py.

    python3 scripts/plot_fft_per_phase_combined.py <fft.csv> <out_folder>
"""
import csv, os, sys
from collections import defaultdict
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FMAX_PLOT = 3.0
AXES = [("Fx_thrust_mag_N", "Fx — thrust", "tab:blue"),
        ("Fy_lateral_mag_N", "Fy — lateral", "tab:orange"),
        ("Fz_heave_mag_N", "Fz — heave", "tab:green")]


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def load(csv_path):
    by_phase = defaultdict(lambda: {"freq": [], "Fx_thrust_mag_N": [],
                                    "Fy_lateral_mag_N": [], "Fz_heave_mag_N": []})
    labels = {}
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            ph = int(row["phase_deg"])
            labels[ph] = row["mission_label"]
            d = by_phase[ph]
            d["freq"].append(_num(row["freq_hz"]))
            for col, _, _ in AXES:
                d[col].append(_num(row[col]))
    return by_phase, labels


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_fft_per_phase_combined.py <fft.csv> <out_folder>")
    csv_path, out_dir = sys.argv[1], sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    by_phase, labels = load(csv_path)
    phases = sorted(by_phase)
    print(f"{len(phases)} phases loaded from {csv_path}")

    for ph in phases:
        d = by_phase[ph]
        freq = d["freq"]
        fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        for ax, (col, title, color) in zip(axes, AXES):
            pts = [(f, m) for f, m in zip(freq, d[col]) if f is not None
                   and m is not None and f <= FMAX_PLOT]
            pts.sort()
            fx = [p[0] for p in pts]; my = [p[1] for p in pts]
            ax.plot(fx, my, lw=1.6, color=color)
            ax.set_title(title, fontsize=11)
            ax.set_ylabel("FFT magnitude (N)")
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("frequency (Hz)")
        fig.suptitle(f"phase {ph}°  ({labels.get(ph,'')}) — force FFT", fontsize=13)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        out_png = os.path.join(out_dir, f"phase_{ph:03d}.png")
        fig.savefig(out_png, dpi=110)
        plt.close(fig)
        print(f"  phase {ph}° -> {out_png}")
    print(f"\nall plots written -> {out_dir}")


if __name__ == "__main__":
    main()
