#!/usr/bin/env python3
"""
plot_3d_sweep_relationships.py — the 6 freq_ratio/amp_ratio/phase -> Fx/Fy/Fz
relationship plots for the in_house_wet_test_3D sweep.

Loads every mission once, computes per-mission RMS force per axis (tared,
windowed to steady cycles same as every other analysis this project), then
produces 6 figures (one per relationship), each with 3 stacked panels
(Fx/Fy/Fz) using the same blue/orange/green axis colors used everywhere else
in this project:

  1. freq_ratio_vs_force.png   — freq_ratio on x, marginalized over amp_ratio+phase
  2. amp_ratio_vs_force.png    — amp_ratio on x, marginalized over freq_ratio+phase
  3. phase_vs_force.png        — phase on x, marginalized over freq_ratio+amp_ratio
  4. freqratio_phase_vs_force.png — 2D heatmap (phase x freq_ratio), marginalized over amp_ratio
  5. ampratio_phase_vs_force.png  — 2D heatmap (phase x amp_ratio), marginalized over freq_ratio
  6. freqratio_ampratio_vs_force.png — 2D heatmap (amp_ratio x freq_ratio), marginalized over phase

1D plots show mean RMS with a shaded +-1 std band (the "relevant statistic" is
RMS force per axis per mission, matching every other force-magnitude analysis
in this project; the std band reflects real spread across the marginalized
dimensions, not measurement noise).

    python3 scripts/plot_3d_sweep_relationships.py <sweep_root> <out_folder>
"""
import csv, glob, os, re, sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

AXES = [("Fx", "tab:blue"), ("Fy", "tab:orange"), ("Fz", "tab:green")]
AXIS_TITLES = {"Fx": "Fx", "Fy": "Fy", "Fz": "Fz"}
STEP = 10  # downsample raw loadcell rows for speed, matches other scripts


def _num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def parse_block_tag(name):
    """'k1_fr0p400_ar0p33' -> (0.400, 0.33); 'k1_pf0p500_r1p000' -> (1.0, 1.0)"""
    if name == "k1_pf0p500_r1p000":
        return 1.0, 1.0
    # names use 'p' for '.', e.g. fr0p400 -> 0.400 ; ar0p33 -> 0.33
    m = re.match(r"^k1_fr(\d+)p(\d+)_ar(\d+)p(\d+)$", name)
    if not m:
        return None, None
    fr = float(f"{m.group(1)}.{m.group(2)}")
    ar = float(f"{m.group(3)}.{m.group(4)}")
    return fr, ar


def load_mission_rms(md):
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
    t, F = [], {"Fx": [], "Fy": [], "Fz": []}
    with open(lc_files[0]) as fh:
        rd = csv.DictReader(fh)
        for i, row in enumerate(rd):
            if i % STEP:
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
    if len(t) < 30:
        return None
    t = np.asarray(t)
    rest = t > gait_end + 1.0
    win = (t >= 1.0 / f0) & (t <= gait_end)
    if win.sum() < 20:
        return None
    out = {"phase_deg": round(ph * 180 / np.pi) % 360}
    for a in F:
        x = np.asarray(F[a])
        if rest.sum() > 10:
            x = x - np.median(x[rest])
        out[f"{a}_rms"] = float(np.sqrt(np.mean(x[win] ** 2)))
    return out


def load_all(root):
    rows = []
    blocks = sorted(d for d in glob.glob(os.path.join(root, "k1_*")) if os.path.isdir(d))
    for bd in blocks:
        bname = os.path.basename(bd)
        fr, ar = parse_block_tag(bname)
        if fr is None:
            print(f"  skip unrecognized block name: {bname}")
            continue
        mdirs = sorted(glob.glob(os.path.join(bd, "PH_*")))
        for md in mdirs:
            r = load_mission_rms(md)
            if r is None:
                continue
            r["freq_ratio"] = fr
            r["amp_ratio"] = ar
            r["block"] = bname
            rows.append(r)
    return rows


def _grouped_mean_std(rows, key, val):
    groups = {}
    for r in rows:
        groups.setdefault(r[key], []).append(r[val])
    xs = sorted(groups)
    means = [np.mean(groups[x]) for x in xs]
    stds = [np.std(groups[x]) for x in xs]
    return np.array(xs), np.array(means), np.array(stds)


def plot_1d(rows, key, xlabel, out_path, title_prefix):
    fig, axes = plt.subplots(3, 1, figsize=(8, 9), sharex=True)
    for ax, (a, color) in zip(axes, AXES):
        xs, means, stds = _grouped_mean_std(rows, key, f"{a}_rms")
        ax.plot(xs, means, "o-", color=color, lw=2, markersize=6)
        ax.fill_between(xs, means - stds, means + stds, color=color, alpha=0.2)
        ax.set_ylabel(f"{AXIS_TITLES[a]} RMS force (N)")
        ax.set_title(f"{AXIS_TITLES[a]}", fontsize=11)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel(xlabel)
    fig.suptitle(f"{title_prefix} -> Fx/Fy/Fz RMS force\n"
                f"(mean +-1 std, marginalized over the other 2 dimensions)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {out_path}")


def plot_2d(rows, xkey, ykey, xlabel, ylabel, out_path, title_prefix):
    xs_all = sorted(set(r[xkey] for r in rows))
    ys_all = sorted(set(r[ykey] for r in rows))
    fig, axes = plt.subplots(1, 3, figsize=(16, 5))
    for ax, (a, color) in zip(axes, AXES):
        grid = np.full((len(ys_all), len(xs_all)), np.nan)
        counts = np.zeros_like(grid)
        for r in rows:
            xi = xs_all.index(r[xkey])
            yi = ys_all.index(r[ykey])
            v = r[f"{a}_rms"]
            if np.isnan(grid[yi, xi]):
                grid[yi, xi] = 0.0
            grid[yi, xi] += v
            counts[yi, xi] += 1
        with np.errstate(invalid="ignore"):
            grid = grid / counts
        im = ax.imshow(grid, aspect="auto", origin="lower", cmap="viridis",
                       extent=[0, len(xs_all), 0, len(ys_all)])
        ax.set_xticks(np.arange(len(xs_all)) + 0.5)
        ax.set_xticklabels([f"{x:g}" for x in xs_all], rotation=45, ha="right", fontsize=8)
        ax.set_yticks(np.arange(len(ys_all)) + 0.5)
        ax.set_yticklabels([f"{y:g}" for y in ys_all], fontsize=8)
        ax.set_xlabel(xlabel)
        if ax is axes[0]:
            ax.set_ylabel(ylabel)
        ax.set_title(f"{AXIS_TITLES[a]}", fontsize=11)
        cbar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
        cbar.set_label(f"{AXIS_TITLES[a]} RMS (N)", fontsize=8)
    fig.suptitle(f"{title_prefix} -> Fx/Fy/Fz RMS force "
                f"(mean, marginalized over the 3rd dimension)", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.92])
    fig.savefig(out_path, dpi=120)
    plt.close(fig)
    print(f"  wrote {out_path}")


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_3d_sweep_relationships.py <sweep_root> <out_folder>")
    root, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)

    print("loading all missions...")
    rows = load_all(root)
    print(f"loaded {len(rows)} missions")

    plot_1d(rows, "freq_ratio", "freq_ratio (heave/pitch)",
           os.path.join(out, "1_freq_ratio_vs_force.png"), "freq_ratio")
    plot_1d(rows, "amp_ratio", "amp_ratio (pitch/heave)",
           os.path.join(out, "2_amp_ratio_vs_force.png"), "amp_ratio")
    plot_1d(rows, "phase_deg", "phase shift (deg)",
           os.path.join(out, "3_phase_vs_force.png"), "phase")

    plot_2d(rows, "phase_deg", "freq_ratio", "phase shift (deg)", "freq_ratio",
           os.path.join(out, "4_freqratio_phase_vs_force.png"), "freq_ratio x phase")
    plot_2d(rows, "phase_deg", "amp_ratio", "phase shift (deg)", "amp_ratio",
           os.path.join(out, "5_ampratio_phase_vs_force.png"), "amp_ratio x phase")
    plot_2d(rows, "amp_ratio", "freq_ratio", "amp_ratio", "freq_ratio",
           os.path.join(out, "6_freqratio_ampratio_vs_force.png"), "freq_ratio x amp_ratio")

    print(f"\ndone -> {out}")


if __name__ == "__main__":
    main()
