"""
experiment_plots.py — matplotlib plots + README for run_experiment.py
====================================================================
All plots are written as PNG files next to the data.  Imported lazily by
run_experiment.py so matplotlib is only needed for the final reporting.
"""

import os, csv, glob, math, json
import matplotlib
matplotlib.use("Agg")            # file output, no display needed
import matplotlib.pyplot as plt

D2R = math.pi / 180.0


def _f(row, key):
    try:
        return float(row.get(key, ""))
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# Per-stage sweep plot: the swept variable vs score/metric (coarse + refine).
# ---------------------------------------------------------------------------
def plot_sweep(stage_dir, name, res_coarse, res_refine, xkey, scorer, xlabel,
               ykey="score"):
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
    for res, lbl, mk in ((res_coarse, "coarse", "o"), (res_refine, "refine", "s")):
        if not res:
            continue
        xs = [m[xkey] for m in res]
        if ykey == "efficiency":
            from run_experiment import efficiency
            ys = [efficiency(m) for m in res]
            ylab = "efficiency  (peak Fx / mean current)"
        else:
            ys = [scorer(m)[0] for m in res]
            ylab = "score"
        order = sorted(range(len(xs)), key=lambda i: xs[i])
        xs = [xs[i] for i in order]; ys = [ys[i] for i in order]
        ax[0].plot(xs, ys, mk + "-", label=lbl)
        # thrust panel
        fx = [m.get("mean_Fx", m.get("peak_Fx", float("nan"))) for m in res]
        fx = [fx[i] for i in order]
        ax[1].plot(xs, fx, mk + "-", label=lbl)
    ax[0].set_xlabel(xlabel); ax[0].set_ylabel(ylab); ax[0].set_title(f"{name}: objective")
    ax[0].grid(True, alpha=0.3); ax[0].legend()
    ax[1].set_xlabel(xlabel); ax[1].set_ylabel("Fx (thrust)"); ax[1].set_title(f"{name}: thrust")
    ax[1].grid(True, alpha=0.3); ax[1].legend()
    fig.tight_layout()
    out = os.path.join(stage_dir, f"{name}_sweep.png")
    fig.savefig(out, dpi=110); plt.close(fig)
    print(f"     plot -> {out}")


# ---------------------------------------------------------------------------
# Final optimal-curve plot: position, forces (x,y,z), current — vs time,
# both servos.
# ---------------------------------------------------------------------------
def plot_optimal_curve(folder, label, frequency, optimal):
    hits = glob.glob(os.path.join(folder, f"{label}_*"))
    if not hits:
        print("     (no optimal mission folder to plot)"); return
    mdir = hits[0]
    fb_csv = glob.glob(os.path.join(mdir, f"{label}.csv"))
    lc_csv = glob.glob(os.path.join(mdir, f"{label}_loadcell.csv"))
    if not fb_csv:
        return
    fb = list(csv.DictReader(open(fb_csv[0])))
    lc = list(csv.DictReader(open(lc_csv[0]))) if lc_csv else []

    t  = [_f(r, "time_s") for r in fb]
    p1 = [_f(r, "s1_position_rad") for r in fb]
    p2 = [_f(r, "s2_position_rad") for r in fb]
    c1 = [_f(r, "s1_current_a") for r in fb]
    c2 = [_f(r, "s2_current_a") for r in fb]
    lt = [_f(r, "time_s") for r in lc]
    fx = [_f(r, "Fx") for r in lc]; fy = [_f(r, "Fy") for r in lc]; fz = [_f(r, "Fz") for r in lc]

    fig, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(t, p1, label="pitch (servo 1)")
    ax[0].plot(t, p2, label="heave (servo 2)")
    ax[0].set_ylabel("position (rad)"); ax[0].legend(); ax[0].grid(True, alpha=0.3)
    ax[0].set_title(f"OPTIMAL sine curve — f={optimal['frequency_hz']:.3f} Hz, "
                    f"pitch={optimal['pitch_amp_rad']:.4f}, heave={optimal['heave_amp_rad']:.4f}, "
                    f"phase={optimal['phase_rad']:.4f} rad")
    if lt and any(v is not None for v in fx):
        ax[1].plot(lt, fx, label="Fx (thrust)")
        ax[1].plot(lt, fy, label="Fy (lateral)", alpha=0.7)
        ax[1].plot(lt, fz, label="Fz (heave)", alpha=0.7)
    else:
        ax[1].text(0.5, 0.5, "no load-cell data", ha="center", transform=ax[1].transAxes)
    ax[1].set_ylabel("force (N)"); ax[1].legend(); ax[1].grid(True, alpha=0.3)
    ax[2].plot(t, c1, label="pitch current (servo 1)")
    ax[2].plot(t, c2, label="heave current (servo 2)")
    ax[2].set_ylabel("current (A)"); ax[2].set_xlabel("time (s)")
    ax[2].legend(); ax[2].grid(True, alpha=0.3)
    fig.tight_layout()
    out = os.path.join(folder, "optimal_sine_curve.png")
    fig.savefig(out, dpi=120); plt.close(fig)
    print(f"     plot -> {out}")


# ---------------------------------------------------------------------------
# Evolution graph: the optimum value chosen at each stage, in sequence.
# ---------------------------------------------------------------------------
def plot_evolution(root, evolution):
    stages = ["phase", "pitch_amp", "heave_amp", "frequency"]
    labels, vals, units = [], [], []
    for s in stages:
        e = evolution.get(s)
        if not e:
            continue
        if s == "frequency":
            labels.append("frequency"); vals.append(e["value_hz"]); units.append("Hz")
        else:
            labels.append(s); vals.append(e["value_deg"]); units.append("deg")
    fig, ax = plt.subplots(figsize=(8, 4.5))
    xs = range(len(labels))
    ax.plot(list(xs), vals, "o-", markersize=9)
    for i, (v, u) in enumerate(zip(vals, units)):
        ax.annotate(f"{v:.2f} {u}", (i, vals[i]), textcoords="offset points",
                    xytext=(0, 10), ha="center")
    ax.set_xticks(list(xs)); ax.set_xticklabels(labels)
    ax.set_ylabel("chosen optimum (deg, or Hz for frequency)")
    ax.set_title("Optimum evolution across stages")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    out = os.path.join(root, "evolution.png")
    fig.savefig(out, dpi=110); plt.close(fig)
    print(f"     plot -> {out}")


# ---------------------------------------------------------------------------
# README with numeric optimums + how each stage's curve evolved.
# ---------------------------------------------------------------------------
def write_readme(root, optimal, evolution, warnings):
    lines = ["# Flapping-foil sine-curve optimization — results", ""]
    lines += ["## Optimal sine curve", "",
              f"- **Frequency:** {optimal['frequency_hz']:.4f} Hz",
              f"- **Pitch amplitude:** {optimal['pitch_amp_rad']:.4f} rad "
              f"({optimal['pitch_amp_rad']/D2R:.2f}°)",
              f"- **Heave amplitude:** {optimal['heave_amp_rad']:.4f} rad "
              f"({optimal['heave_amp_rad']/D2R:.2f}°)",
              f"- **Phase (heave vs pitch):** {optimal['phase_rad']:.4f} rad "
              f"({optimal['phase_rad']/D2R:.2f}°)", "",
              "Command to reproduce:", "",
              "```",
              f"forward_paddle frequency:{optimal['frequency_hz']:.4f} "
              f"pitch_amp:{optimal['pitch_amp_rad']:.4f} "
              f"heave_amp:{optimal['heave_amp_rad']:.4f} "
              f"phase:{optimal['phase_rad']:.4f} cycles:10 label:OPTIMAL",
              "```", "",
              "![optimal](optimal_sine_curve/optimal_sine_curve.png)", ""]
    lines += ["## How the optimum evolved", "",
              "![evolution](evolution.png)", ""]
    stage_files = {"phase": "01_phase/phase_sweep.png",
                   "pitch_amp": "02_pitch_amp/pitch_amp_sweep.png",
                   "heave_amp": "03_heave_amp/heave_amp_sweep.png",
                   "frequency": "04_frequency/frequency_sweep.png"}
    for s, e in evolution.items():
        val = (f"{e['value_hz']:.4f} Hz" if s == "frequency"
               else f"{e['value_rad']:.4f} rad ({e['value_deg']:.2f}°)")
        lines += [f"### {s} → **{val}**", "", f"![{s}]({stage_files.get(s,'')})", ""]
    if warnings:
        lines += ["## ⚠ Servo tracking warnings",
                  "(points where the servo under-followed the commanded amplitude — "
                  "forces there reflect a smaller stroke than commanded)", ""]
        lines += [f"- {w}" for w in warnings]
        lines += [""]
    out = os.path.join(root, "README.md")
    open(out, "w").write("\n".join(lines))
    print(f"     README -> {out}")
