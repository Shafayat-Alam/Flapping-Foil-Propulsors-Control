#!/usr/bin/env python3
"""
plot_fft_per_phase.py — for every phase, plot the FFT of all three forces
(Fx thrust, Fy lateral, Fz heave) as THREE SEPARATE windows shown together
(not combined into one figure/subplot).  Close all three to advance to the
next phase.  Every window is also saved as a PNG in the output folder.

Reads the FFT CSV written by make_fft_csv.py.

    python3 scripts/plot_fft_per_phase.py <fft.csv> <out_folder> [--no-show]
"""
import csv, os, sys, math
from collections import defaultdict
import matplotlib
import matplotlib.pyplot as plt

FMAX_PLOT = 3.0
AXES = [("Fx_thrust_mag_N", "Fx — thrust"),
        ("Fy_lateral_mag_N", "Fy — lateral"),
        ("Fz_heave_mag_N", "Fz — heave")]


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
            for col, _ in AXES:
                d[col].append(_num(row[col]))
    return by_phase, labels


def main():
    if len(sys.argv) < 3:
        sys.exit("usage: plot_fft_per_phase.py <fft.csv> <out_folder> [--no-show]")
    csv_path, out_dir = sys.argv[1], sys.argv[2]
    show = "--no-show" not in sys.argv[3:]
    if not show:
        matplotlib.use("Agg")
    os.makedirs(out_dir, exist_ok=True)

    by_phase, labels = load(csv_path)
    phases = sorted(by_phase)
    print(f"{len(phases)} phases loaded from {csv_path}")

    for ph in phases:
        d = by_phase[ph]
        freq = d["freq"]
        figs = []
        for col, title in AXES:
            fig, ax = plt.subplots(figsize=(8, 4.5))
            mag = d[col]
            pts = [(f, m) for f, m in zip(freq, mag) if f is not None
                   and m is not None and f <= FMAX_PLOT]
            pts.sort()
            fx = [p[0] for p in pts]; my = [p[1] for p in pts]
            ax.plot(fx, my, lw=1.6, color={"Fx_thrust_mag_N": "tab:blue",
                                           "Fy_lateral_mag_N": "tab:orange",
                                           "Fz_heave_mag_N": "tab:green"}[col])
            ax.set_title(f"phase {ph}°  ({labels.get(ph,'')})  —  {title} FFT")
            ax.set_xlabel("frequency (Hz)")
            ax.set_ylabel("FFT magnitude (N)")
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            out_png = os.path.join(out_dir, f"phase_{ph:03d}_{col.split('_')[0]}.png")
            fig.savefig(out_png, dpi=110)
            figs.append(fig)
        print(f"  phase {ph}°: 3 plots saved (Fx/Fy/Fz)"
              + ("" if show else ""))
        if show:
            plt.show()          # blocks until ALL open windows (all 3) are closed
        else:
            for f in figs:
                plt.close(f)
    print(f"\nall plots written -> {out_dir}")


if __name__ == "__main__":
    main()
