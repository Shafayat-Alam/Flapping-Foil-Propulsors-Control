#!/usr/bin/env python3
"""
make_timeseries_plots_pitch_k.py — the "without FFT" counterpart to
make_fft_plots_pitch_k.py: same 3-panel (Fx/Fy/Fz) layout, one image per phase,
but plotting the RAW tared force vs TIME instead of its FFT spectrum. Run
across every (k, pitch_freq) block of a pitch_k x freq x phase sweep, with k
and pitch frequency marked on every image and in the folder layout.

For each block folder (k<K>_pf<F>_r<R>) under the sweep root:
  * axis-corrects + tares every mission's force (thrust<-Fy, lateral<-Fz,
    heave<-Fx; tared against the at-rest tail), windowed to the steady gait
    cycles (1st cycle dropped as warm-up) — same convention as the FFT scripts
  * writes one 3-panel image per phase: <out>/k<K>_pf<F>/phase_<deg>.png
  * title reads "k=<K>  pitch_freq=<F> Hz  phase=<deg>°"

    python3 scripts/make_timeseries_plots_pitch_k.py <sweep_root> <out_folder>
"""
import csv, glob, os, re, sys, math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_fft_plots_pitch_k import parse_block_name

csv.field_size_limit(2**31 - 1)
STEP = 20   # downsample the 10 kHz load cell for plotting (matches FFT scripts)

# analysis axis <- recorded channel (thrust/lateral/heave), same as elsewhere
PANELS = [("Fx", "Fy", "Fx — thrust", "tab:blue"),
          ("Fy", "Fz", "Fy — lateral", "tab:orange"),
          ("Fz", "Fx", "Fz — heave", "tab:green")]


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def _label(md):
    for f in glob.glob(os.path.join(md, "*.csv")):
        b = os.path.basename(f)
        if not b.endswith("_loadcell.csv"):
            return b[:-4]
    return None


def _zero_series(label, phase_deg, f0, gait_end):
    """Synthetic all-zero time series for a mission with no load-cell data at
    all, spanning the same steady-cycle window a real mission would use.
    Callers mark these with no_loadcell=True so they render distinctly, not
    mistaken for a genuinely quiet (near-zero) real signal."""
    n = 500
    t = np.linspace(1.0 / f0, gait_end, n)
    zero = np.zeros(n)
    out = {"label": label, "phase_deg": phase_deg, "t": t - (1.0 / f0),
          "f0": f0, "no_loadcell": True}
    for out_axis, _rec, _title, _color in PANELS:
        out[out_axis] = zero
    return out


def load_tared(md):
    """Axis-corrected + tared Fx/Fy/Fz vs time for one mission, windowed to the
    steady gait cycles.  Returns dict, or None if the mission itself is too
    short/spurious (a genuinely missing load-cell FILE instead plots as zero
    — see _zero_series — so a phase is never silently dropped)."""
    label = _label(md)
    if not label:
        return None
    fbp = os.path.join(md, f"{label}.csv")
    lcp = os.path.join(md, f"{label}_loadcell.csv")
    if not os.path.exists(fbp):
        return None
    r0 = next(csv.DictReader(open(fbp)), {})
    f0 = _num(r0.get("cmd.frequency"))
    ph = _num(r0.get("cmd.phase")) or 0.0
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    if not f0:
        return None
    gait_end = cyc / f0
    phase_deg = round(ph * 180 / math.pi)

    if not os.path.exists(lcp):
        return _zero_series(label, phase_deg, f0, gait_end)

    with open(lcp) as fh:
        rd = csv.reader(fh)
        hdr = next(rd)
        ix = {h: i for i, h in enumerate(hdr)}
        need = {"time_s", "Fx", "Fy", "Fz"}
        if not need.issubset(ix):
            return None
        t, raw = [], {"Fx": [], "Fy": [], "Fz": []}
        for k, row in enumerate(rd):
            if k % STEP:
                continue
            tv = _num(row[ix["time_s"]])
            if tv is None:
                continue
            vals = {ax: _num(row[ix[ax]]) for ax in ("Fx", "Fy", "Fz")}
            if any(v is None for v in vals.values()):
                continue
            t.append(tv)
            for ax in ("Fx", "Fy", "Fz"):
                raw[ax].append(vals[ax])
    t = np.asarray(t)
    if t.size < 20 or not (t.max() > gait_end + 2):   # too short / spurious
        return None

    rest = t > gait_end + 1.0
    win = (t >= 1.0 / f0) & (t <= gait_end)
    if win.sum() < 20:
        return None

    out = {"label": label, "phase_deg": phase_deg, "t": t[win] - (1.0 / f0),
          "f0": f0}
    for out_axis, rec_axis, _title, _color in PANELS:
        x = np.asarray(raw[rec_axis])
        if rest.sum() > 10:
            x = x - np.median(x[rest])
        out[out_axis] = x[win]
    return out


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

    n_img = 0
    for md in mdirs:
        r = load_tared(md)
        if r is None:
            continue
        ph = r["phase_deg"]
        period = 1.0 / r["f0"]     # one commanded pitch cycle, in seconds
        t_end = r["t"][-1] if len(r["t"]) else 0.0
        n_cycles = max(1, round(t_end / period)) if period > 0 else 0
        cycle_stops = [k * period for k in range(1, n_cycles + 1)]
        half_cycle_stops = [(k + 0.5) * period for k in range(n_cycles + 1)
                            if (k + 0.5) * period <= t_end]

        fig, axes = plt.subplots(3, 1, figsize=(8, 10), sharex=True)
        for pi, (ax, (out_axis, _rec, title, color)) in enumerate(zip(axes, PANELS)):
            ax.plot(r["t"], r[out_axis], lw=1.0, color=color)
            ax.axhline(0, color="k", lw=0.6, alpha=0.5)
            for ci, cs in enumerate(cycle_stops):
                ax.axvline(cs, color="red", ls="--", lw=1.1, alpha=0.8,
                          label="cycle stop" if (pi == 0 and ci == 0) else None)
            for hi, hs in enumerate(half_cycle_stops):
                ax.axvline(hs, color="gold", ls=(0, (3, 2)), lw=1.6, alpha=1.0,
                          label="half cycle" if (pi == 0 and hi == 0) else None)
            ax.set_title(title, fontsize=11)
            ax.set_ylabel("force (N)")
            ax.grid(True, alpha=0.3)
        axes[0].legend(fontsize=8, loc="upper right")
        axes[-1].set_xlabel(f"time (s), steady cycles (1st cycle dropped) — "
                            f"period={period:.3f}s")
        no_lc = r.get("no_loadcell", False)
        title = (f"k={k_str}   pitch_freq={pf_str} Hz   phase={ph}°   "
                f"({r['label']})")
        if no_lc:
            title += "   [NO LOAD-CELL DATA — plotted as 0]"
        fig.suptitle(title, fontsize=13, color="firebrick" if no_lc else "black")
        if no_lc:
            for ax in axes:
                ax.set_facecolor("#fff0f0")
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        out_png = os.path.join(out_dir, f"{label_tag}_phase_{ph:03d}.png")
        fig.savefig(out_png, dpi=110)
        plt.close(fig)
        n_img += 1
    print(f"  {name}: {n_img} images -> {out_dir}/")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: make_timeseries_plots_pitch_k.py <sweep_root> <out_folder>")
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
