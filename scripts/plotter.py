#!/usr/bin/env python3
"""
plotter.py — interactive python plotter for sweep results
=========================================================
Opens LIVE matplotlib windows (zoom/pan with the toolbar) — not static images.

    python3 plotter.py <sweep_folder>              # overlay: all missions per metric
    python3 plotter.py <sweep_folder> --per-mission  # one window per mission
    python3 plotter.py <mission_folder>            # a single mission

Metrics: frequency, position, velocity, current, voltage (pitch=servo1,
heave=servo2) + Fx, Fy, Fz, Tx, Ty, Tz.

Each window is one metric vs time, titled with the curve's properties.
Close a window to advance to the next.  Use --save to also drop PNGs alongside.
"""

import os, sys, csv, glob, argparse

import matplotlib
import matplotlib.pyplot as plt

from sweep_common import ALL_METRICS, _read, _num, _title


def _mission_label(mdir):
    """Infer the mission label from a split folder (<LABEL>_<ts>/<LABEL>.csv)."""
    for f in glob.glob(os.path.join(mdir, "*.csv")):
        b = os.path.basename(f)
        if not b.endswith("_loadcell.csv"):
            return b[:-4]
    return None


def _find_missions(root):
    """Return [(label, mission_dir), ...] for a sweep folder or a single mission."""
    if _mission_label(root):
        return [(_mission_label(root), root)]
    out = []
    for d in sorted(glob.glob(os.path.join(root, "*"))):
        if not os.path.isdir(d) or os.path.basename(d) in ("raw",):
            continue
        lab = _mission_label(d)
        if lab:
            out.append((lab, d))
    return out


def plot_overlay(missions, save=False, outdir=None):
    """One window per metric, every mission overlaid."""
    for col, src, ylab in ALL_METRICS:
        fig, ax = plt.subplots(figsize=(11, 5))
        any_data = False
        for label, mdir in missions:
            rows = _read(os.path.join(
                mdir, f"{label}.csv" if src == "feedback"
                else f"{label}_loadcell.csv"))
            if not rows or col not in rows[0]:
                continue
            t, y = _num(rows, "time_s"), _num(rows, col)
            if not any(v is not None for v in y):
                continue
            ax.plot(t, y, lw=0.9, label=label)
            any_data = True
        if not any_data:
            plt.close(fig); continue
        ax.set_xlabel("time (s)"); ax.set_ylabel(ylab)
        ax.set_title(f"{ylab} — all missions")
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7, ncol=2)
        fig.tight_layout()
        if save and outdir:
            os.makedirs(outdir, exist_ok=True)
            fig.savefig(os.path.join(outdir, f"{col}.png"), dpi=110)
    plt.show()


def plot_per_mission(missions, save=False):
    """One window per metric, per mission (titled with the curve's properties)."""
    for label, mdir in missions:
        for col, src, ylab in ALL_METRICS:
            rows = _read(os.path.join(
                mdir, f"{label}.csv" if src == "feedback"
                else f"{label}_loadcell.csv"))
            if not rows or col not in rows[0]:
                continue
            t, y = _num(rows, "time_s"), _num(rows, col)
            if not any(v is not None for v in y):
                continue
            fb = _read(os.path.join(mdir, f"{label}.csv"))
            title = _title(label, fb[0] if fb else {})
            fig, ax = plt.subplots(figsize=(11, 5))
            ax.plot(t, y, lw=0.9)
            ax.set_xlabel("time (s)"); ax.set_ylabel(ylab)
            ax.set_title(f"{ylab}\n{title}", fontsize=10)
            ax.grid(True, alpha=0.3)
            fig.tight_layout()
            if save:
                pdir = os.path.join(mdir, "plots")
                os.makedirs(pdir, exist_ok=True)
                fig.savefig(os.path.join(pdir, f"{col}.png"), dpi=110)
        print(f"  {label}: close the windows to advance")
        plt.show()
        # Free this mission's figures before the next one; a long sweep would
        # otherwise keep every window of every mission open at once.
        plt.close("all")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder", help="sweep folder, or a single mission folder")
    ap.add_argument("--per-mission", action="store_true",
                    help="one window per metric per mission (default: overlay all)")
    ap.add_argument("--save", action="store_true",
                    help="also save PNGs next to the data")
    args = ap.parse_args()

    root = os.path.abspath(os.path.expanduser(args.folder))
    missions = _find_missions(root)
    if not missions:
        print(f"No mission CSVs found under {root}"); sys.exit(1)
    print(f"Found {len(missions)} mission(s): {', '.join(l for l,_ in missions)}")
    print("Close each window to advance.  Use the toolbar to zoom/pan.")

    if args.per_mission:
        plot_per_mission(missions, save=args.save)
    else:
        plot_overlay(missions, save=args.save,
                     outdir=os.path.join(root, "sweep_overlay") if args.save else None)


if __name__ == "__main__":
    main()
