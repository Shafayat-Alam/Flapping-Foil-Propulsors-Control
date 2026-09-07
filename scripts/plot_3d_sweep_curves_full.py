#!/usr/bin/env python3
"""
plot_3d_sweep_curves_full.py — comprehensive version of plot_3d_sweep_curves.py:
instead of one curve-overlay plot per parameter (at a single baseline combo of
the other two), generates one for EVERY combination of the other two
parameters, so every slice of the 3D sweep is covered, not just a baseline
example.

Loads every mission's cycle-2-only folded Fx/Fy/Fz curve ONCE into memory,
then writes three folders of plots:

  plots/freq_ratio/<ar>_<phase>.png   — 125 plots, each overlays all 7
                                         freq_ratio curves at that (amp_ratio,
                                         phase), titled with the fixed values
  plots/amp_ratio/<fr>_<phase>.png    — 175 plots, overlays all 5 amp_ratio
                                         curves at that (freq_ratio, phase)
  plots/phase_shift/<fr>_<ar>.png     — 35 plots, overlays all 25 phase
                                         curves at that (freq_ratio, amp_ratio)

    python3 scripts/plot_3d_sweep_curves_full.py <sweep_root/data> <out_folder>
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


def load_folded_cycle2(md):
    lc_files = glob.glob(os.path.join(md, "*_loadcell.csv"))
    if not lc_files:
        return None
    fb_files = [f for f in glob.glob(os.path.join(md, "*.csv")) if "_loadcell" not in f]
    if not fb_files:
        return None
    r0 = next(csv.DictReader(open(fb_files[0])), {})
    f0 = _num(r0.get("cmd.frequency"))
    ph = _num(r0.get("cmd.phase")) or 0.0
    if not f0:
        return None
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
    cyc = _num(r0.get("cmd.cycles")) or 4.0
    gait_end = cyc / f0
    rest = t > gait_end + 1.0
    win = (t >= period) & (t < 2 * period)   # cycle 2 ONLY, no blending
    if win.sum() < 30:
        return None
    tw = t[win] % period
    bins = np.linspace(0, period, NB + 1)
    out = {"phase_deg": round(ph * 180 / np.pi) % 360}
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


def load_all(root):
    """cache[(freq_ratio, amp_ratio, phase_deg)] = {Fx:[...], Fy:[...], Fz:[...]}"""
    cache = {}
    blocks = sorted(d for d in glob.glob(os.path.join(root, "k1_*")) if os.path.isdir(d))
    print(f"{len(blocks)} blocks to load")
    for bi, bd in enumerate(blocks):
        fr, ar = parse_block_tag(os.path.basename(bd))
        if fr is None:
            continue
        mdirs = sorted(glob.glob(os.path.join(bd, "PH_*")))
        for md in mdirs:
            d = load_folded_cycle2(md)
            if d is None:
                continue
            key = (fr, ar, d["phase_deg"])
            if key not in cache:   # keep first occurrence, skip PH_0360 dup etc
                cache[key] = d
        print(f"  [{bi+1}/{len(blocks)}] {os.path.basename(bd)} loaded")
    return cache


def make_overlay(cache, keys_and_vals, panel_axis_key_index, value_label, cmap_name,
                 title, out_path):
    """keys_and_vals: list of (full_key_tuple, value_for_color) pairs, all sharing
    the same fixed dims except the one being swept."""
    fig, axes = plt.subplots(3, 1, figsize=(9, 10), sharex=True)
    cmap = cm.get_cmap(cmap_name)
    n = len(keys_and_vals)
    t_frac = (np.arange(NB) + 0.5) / NB
    for pi, (a, title_a) in enumerate(PANELS):
        ax = axes[pi]
        for i, (key, v) in enumerate(keys_and_vals):
            d = cache.get(key)
            if d is None:
                continue
            color = cmap(i / max(1, n - 1))
            ax.plot(t_frac, d[a], color=color, lw=1.6, label=f"{value_label}={v:g}")
        ax.axhline(0, color="k", lw=0.5, alpha=0.4)
        ax.set_title(title_a, fontsize=11)
        ax.set_ylabel("force (N)")
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("fraction of cycle 2 (0=start, 1=end)")
    if n <= 8:
        axes[0].legend(fontsize=7, ncol=2, loc="upper right")
    fig.suptitle(title, fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(out_path, dpi=110)
    plt.close(fig)


def main():
    if len(sys.argv) != 3:
        sys.exit("usage: plot_3d_sweep_curves_full.py <sweep_root/data> <out_folder>")
    root, out = sys.argv[1], sys.argv[2]

    freq_ratios = [0.4, 0.5, 0.667, 1.0, 1.5, 2.0, 2.5]
    amp_ratios = [0.33, 0.67, 1.00, 1.50, 3.00]
    phases = list(range(0, 360, 15))

    print("loading all missions (cycle 2 only)...")
    cache = load_all(root)
    print(f"loaded {len(cache)} (freq_ratio, amp_ratio, phase) entries")

    # ---- plots/freq_ratio/ : one per (amp_ratio, phase), overlay freq_ratio ----
    d1 = os.path.join(out, "freq_ratio")
    os.makedirs(d1, exist_ok=True)
    n_done = 0
    for ar in amp_ratios:
        for ph in phases:
            keys_and_vals = [((fr, ar, ph), fr) for fr in freq_ratios]
            if not any(k in cache for k, _ in keys_and_vals):
                continue
            fname = f"ar{ar:.2f}_phase{ph:03d}".replace(".", "p") + ".png"
            make_overlay(cache, keys_and_vals, 0, "freq_ratio", "viridis",
                        f"Fx/Fy/Fz vs freq_ratio  (amp_ratio={ar:g}, phase={ph} deg fixed)",
                        os.path.join(d1, fname))
            n_done += 1
    print(f"freq_ratio folder: {n_done} plots")

    # ---- plots/amp_ratio/ : one per (freq_ratio, phase), overlay amp_ratio ----
    d2 = os.path.join(out, "amp_ratio")
    os.makedirs(d2, exist_ok=True)
    n_done = 0
    for fr in freq_ratios:
        for ph in phases:
            keys_and_vals = [((fr, ar, ph), ar) for ar in amp_ratios]
            if not any(k in cache for k, _ in keys_and_vals):
                continue
            fname = f"fr{fr:.3f}_phase{ph:03d}".replace(".", "p") + ".png"
            make_overlay(cache, keys_and_vals, 1, "amp_ratio", "viridis",
                        f"Fx/Fy/Fz vs amp_ratio  (freq_ratio={fr:g}, phase={ph} deg fixed)",
                        os.path.join(d2, fname))
            n_done += 1
    print(f"amp_ratio folder: {n_done} plots")

    # ---- plots/phase_shift/ : one per (freq_ratio, amp_ratio), overlay phase ----
    d3 = os.path.join(out, "phase_shift")
    os.makedirs(d3, exist_ok=True)
    n_done = 0
    for fr in freq_ratios:
        for ar in amp_ratios:
            keys_and_vals = [((fr, ar, ph), ph) for ph in phases]
            if not any(k in cache for k, _ in keys_and_vals):
                continue
            fname = f"fr{fr:.3f}_ar{ar:.2f}".replace(".", "p") + ".png"
            make_overlay(cache, keys_and_vals, 2, "phase", "twilight",
                        f"Fx/Fy/Fz vs phase shift  (freq_ratio={fr:g}, amp_ratio={ar:g} fixed)",
                        os.path.join(d3, fname))
            n_done += 1
    print(f"phase_shift folder: {n_done} plots")

    print(f"\ndone -> {out}")


if __name__ == "__main__":
    main()
