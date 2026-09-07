#!/usr/bin/env python3
"""
make_fft_plots_pitch_k.py — the same 3-panel (Fx/Fy/Fz) FFT-per-phase image as
plot_fft_per_phase_combined.py, but run across EVERY (k, pitch_freq) block of a
pitch_k x freq x phase sweep, with k and pitch frequency marked on every image
and in the folder layout.

For each block folder (k<K>_pf<F>_r<R>) under the sweep root:
  * FFTs every mission's Fx/Fy/Fz (axis-corrected + tared, same as before)
  * writes one 3-panel image per phase: <out>/k<K>_pf<F>/phase_<deg>.png
  * title reads "k=<K>  pitch_freq=<F> Hz  phase=<deg>°"
Also writes each block's FFT CSV alongside (k<K>_pf<F>_fft.csv), same columns
as make_fft_csv.py, in case you want the numbers too.

    python3 scripts/make_fft_plots_pitch_k.py <sweep_root> <out_folder>
"""
import csv, glob, os, re, sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_fft_csv import fft_mission_percycle, _label, FMAX_REPORT, AXES

BLOCK_RE = re.compile(r"^k([\d.]+|inf)_pf([\d.]+)p(\d+)_r")
FMAX_PLOT = 3.0
PANELS = [("Fx_thrust_mag_N", "Fx — thrust", "tab:blue"),
          ("Fy_lateral_mag_N", "Fy — lateral", "tab:orange"),
          ("Fz_heave_mag_N", "Fz — heave", "tab:green")]
# per-cycle overlay colors (cycle 2, 3, 4 of the mission; cycle 1 dropped as warm-up)
CYCLE_COLORS = ["#bbbbbb", "#888888", "#555555"]


def parse_block_name(name):
    """'k8_pf1p250_r0p400' -> ('8', '1.250')."""
    m = re.match(r"^k(inf|[\d.]+)_pf(\d+)p(\d+)_r", name)
    if not m:
        return None, None
    k_str = m.group(1)
    pf_str = f"{m.group(2)}.{m.group(3)}"
    return k_str, pf_str


def process_block(block_dir, out_root):
    name = os.path.basename(block_dir)
    k_str, pf_str = parse_block_name(name)
    if k_str is None:
        print(f"  {name}: unrecognized block folder name — skipped")
        return
    label_tag = f"k{k_str}_pf{pf_str}"
    out_dir = out_root   # flat: every block's images land directly in out_root,
                         # filenames carry the k/freq/phase tag (not subfolders)
    os.makedirs(out_dir, exist_ok=True)

    mdirs = [d for d in sorted(glob.glob(os.path.join(block_dir, "*")))
             if os.path.isdir(d) and os.path.basename(d) != "raw"]

    csv_path = os.path.join(out_root, f"{label_tag}_fft.csv")
    n_img = 0
    with open(csv_path, "w", newline="") as fo:
        w = csv.writer(fo)
        w.writerow(["mission_label", "phase_deg", "freq_hz",
                    "Fx_thrust_mag_N", "Fy_lateral_mag_N", "Fz_heave_mag_N"])
        for md in mdirs:
            r = fft_mission_percycle(md)
            if r is None:
                continue
            fr_ref, _ = next(iter(r["spec"].values()))
            m = fr_ref <= FMAX_REPORT
            fr_ref = fr_ref[m]
            for i, fq in enumerate(fr_ref):
                row = [r["label"], r["phase_deg"], round(float(fq), 4)]
                for col, _ in AXES:
                    fr, mag = r["spec"][col]
                    row.append(round(float(mag[i]), 6) if i < mag.size else "")
                w.writerow(row)

            # ---- the 3-panel image: each cycle's own FFT (thin grey) overlaid
            # with the combined 3-cycle FFT (bold color) — shows whether the
            # spectrum is consistent cycle-to-cycle or drifting/inconsistent.
            ph = r["phase_deg"]
            fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
            for ax, (col, title, color) in zip(axes, PANELS):
                for ci, cyc_spec in enumerate(r["cycles"]):
                    fr_c, mag_c = cyc_spec[col]
                    keep_c = fr_c <= FMAX_PLOT
                    ax.plot(fr_c[keep_c], mag_c[keep_c], lw=0.9,
                           color=CYCLE_COLORS[ci % len(CYCLE_COLORS)],
                           alpha=0.8, label=f"cycle {ci+1}" if ax is axes[0] else None)
                fr, mag = r["spec"][col]
                keep = fr <= FMAX_PLOT
                ax.plot(fr[keep], mag[keep], lw=1.8, color=color,
                       label="combined (all cycles)" if ax is axes[0] else None)
                ax.set_title(title, fontsize=11)
                ax.set_ylabel("FFT magnitude (N)")
                ax.grid(True, alpha=0.3)
            axes[0].legend(fontsize=7, loc="upper right")
            axes[-1].set_xlabel("frequency (Hz)")
            no_lc = r.get("no_loadcell", False)
            title = (f"k={k_str}   pitch_freq={pf_str} Hz   phase={ph}°   "
                    f"({r['label']})")
            if no_lc:
                title += "   [NO LOAD-CELL DATA — plotted as 0]"
            fig.suptitle(title, fontsize=13,
                        color="firebrick" if no_lc else "black")
            if no_lc:
                for ax in axes:
                    ax.set_facecolor("#fff0f0")
            fig.tight_layout(rect=[0, 0, 1, 0.96])
            out_png = os.path.join(out_dir, f"{label_tag}_phase_{ph:03d}.png")
            fig.savefig(out_png, dpi=110)
            plt.close(fig)
            n_img += 1
    print(f"  {name}: {n_img} images -> {out_dir}/   ({os.path.basename(csv_path)})")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: make_fft_plots_pitch_k.py <sweep_root> <out_folder>")
    root, out_root = sys.argv[1], sys.argv[2]
    os.makedirs(out_root, exist_ok=True)
    blocks = sorted(d for d in glob.glob(os.path.join(root, "k*_pf*_r*"))
                    if os.path.isdir(d))
    if not blocks:
        sys.exit(f"no block folders (k*_pf*_r*) found under {root}")
    print(f"{len(blocks)} blocks found under {root}")
    for b in blocks:
        process_block(b, out_root)
    print(f"\ndone -> {out_root}")


if __name__ == "__main__":
    main()
