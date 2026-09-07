#!/usr/bin/env python3
"""
plot_3d_sweep_curves.py — shows the actual force CURVE shape changing as each
sweep parameter varies (not a summary statistic like RMS/mean). For each of
freq_ratio, amp_ratio, phase: hold the other two parameters at baseline
(freq_ratio=1.0, amp_ratio=1.00, phase=0deg) and overlay the folded one-cycle
Fx/Fy/Fz waveform for every value of the parameter being varied, color-coded
by that value (sequential colormap, light->dark = low->high value; phase uses
a cyclic colormap since it wraps 0-360).

    python3 scripts/plot_3d_sweep_curves.py <sweep_root/data> <out_folder>
"""
import csv, glob, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.cm as cm

PANELS = [("Fx", "Fx — thrust"), ("Fy", "Fy — lateral"), ("Fz", "Fz — heave")]
NB = 24


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def parse_block_tag(name):
    if name == "k1_pf0p500_r1p000":
        return 1.0, 1.0
    m = re.match(r"^k1_fr(\d+)p(\d+)_ar(\d+)p(\d+)$", name)
    if not m:
        return None, None
    return float(f"{m.group(1)}.{m.group(2)}"), float(f"{m.group(3)}.{m.group(4)}")


def load_folded(md):
    lc_files = glob.glob(os.path.join(md, "*_loadcell.csv"))
    if not lc_files:
        return None
    fb_files = [f for f in glob.glob(os.path.join(md, "*.csv")) if "_loadcell" not in f]
    if not fb_files:
        return None
    r0 = next(csv.DictReader(open(fb_files[0])), {})
    f0 = _num(r0.get("cmd.frequency"))
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    ph = _num(r0.get("cmd.phase")) or 0.0
    if not f0:
        return None
    gait_end = cyc / f0
    period = 1.0 / f0
    t, F = [], {"Fx": [], "Fy": [], "Fz": []}
    with open(lc_files[0]) as fh:
        for i, row in enumerate(csv.DictReader(fh)):
            if i % 5:
                continue
            tv = _num(row.get("time_s"))
            if tv is None:
                continue
            vals = {a: _num(row.get(a)) for a in F}
            if any(v is None for v in vals.values()):
                continue
            t.append(tv)
            for a in F:
                F[a].append(vals[a])
    if len(t) < 40:
        return None
    t = np.asarray(t)
    rest = t > gait_end + 1.0
    # Cycle 1 is warmup (dropped, as everywhere else in this project). But
    # unlike other scripts here, this one does NOT average cycles 2-4 together
    # -- averaging can blend cycles that don't actually match into a
    # misleading composite shape (seen earlier this project). Use ONLY cycle
    # 2 ([period, 2*period)), a single real cycle, not a blend.
    win = (t >= period) & (t < 2 * period)
    if win.sum() < 30:
        return None

    out = {"phase_deg": round(ph * 180 / np.pi) % 360}
    tw = t[win] % period
    bins = np.linspace(0, period, NB + 1)
    bin_centers = (bins[:-1] + bins[1:]) / 2 / period  # fraction of cycle 0..1
    out["t_frac"] = bin_centers
    for a in F:
        x = np.asarray(F[a])
        if rest.sum() > 10:
            x = x - np.median(x[rest])
        xw = x[win]
        folded = np.full(NB, np.nan)
        for i in range(NB):
            m = (tw >= bins[i]) & (tw < bins[i + 1])
            if m.sum():
                folded[i] = np.mean(xw[m])
        if np.isnan(folded).any():
            idx = np.arange(NB)
            good = ~np.isnan(folded)
            if good.sum() < 2:
                return None
            folded[~good] = np.interp(idx[~good], idx[good], folded[good])
        out[a] = folded
    return out


def find_mission_for(root, freq_ratio, amp_ratio, phase_deg):
    if abs(freq_ratio - 1.0) < 1e-9 and abs(amp_ratio - 1.0) < 1e-9:
        tag = "k1_pf0p500_r1p000"
    else:
        tag = f"k1_fr{freq_ratio:.3f}_ar{amp_ratio:.2f}".replace(".", "p")
    bd = os.path.join(root, tag)
    if not os.path.isdir(bd):
        return None
    for md in sorted(glob.glob(os.path.join(bd, "PH_*"))):
        d = load_folded(md)
        if d is not None and d["phase_deg"] == phase_deg:
            return d
    return None


def plot_param_sweep(root, values, get_mission, value_label, cmap_name, out_path,
                     title, cyclic=False):
    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    cmap = cm.get_cmap(cmap_name)
    norm_vals = np.linspace(0, 1, len(values))
    for pi, (a, title_a) in enumerate(PANELS):
        ax = axes[pi]
        for v, nv in zip(values, norm_vals):
            d = get_mission(v)
            if d is None:
                continue
            ax.plot(d["t_frac"], d[a], color=cmap(nv), lw=1.8,
                   label=f"{value_label}={v:g}")
        ax.axhline(0, color="k", lw=0.5, alpha=0.4)
        ax.set_title(title_a, fontsize=11)
        ax.set_ylabel("force (N)")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("fraction of cycle (0=cycle start, 1=cycle end)")
    axes[0].legend(fontsize=7, ncol=2, loc="upper right")
    fig.suptitle(title, fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {out_path}")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_3d_sweep_curves.py <sweep_root/data> <out_folder>")
    root, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)

    freq_ratios = [0.4, 0.5, 0.667, 1.0, 1.5, 2.0, 2.5]
    amp_ratios = [0.33, 0.67, 1.00, 1.50, 3.00]
    phases = list(range(0, 360, 15))

    print("freq_ratio curve sweep (amp_ratio=1.0, phase=0 fixed)...")
    plot_param_sweep(
        root, freq_ratios,
        lambda v: find_mission_for(root, v, 1.0, 0),
        "freq_ratio", "viridis",
        os.path.join(out, "curve_freq_ratio.png"),
        "Force CURVE shape vs freq_ratio (amp_ratio=1.0, phase=0 deg held fixed)")

    print("amp_ratio curve sweep (freq_ratio=1.0, phase=0 fixed)...")
    plot_param_sweep(
        root, amp_ratios,
        lambda v: find_mission_for(root, 1.0, v, 0),
        "amp_ratio", "viridis",
        os.path.join(out, "curve_amp_ratio.png"),
        "Force CURVE shape vs amp_ratio (freq_ratio=1.0, phase=0 deg held fixed)")

    print("phase curve sweep (freq_ratio=1.0, amp_ratio=1.0 fixed)...")
    plot_param_sweep(
        root, phases,
        lambda v: find_mission_for(root, 1.0, 1.0, v),
        "phase", "twilight",
        os.path.join(out, "curve_phase.png"),
        "Force CURVE shape vs phase shift (freq_ratio=1.0, amp_ratio=1.0 held fixed)",
        cyclic=True)

    print(f"\ndone -> {out}")


if __name__ == "__main__":
    main()
