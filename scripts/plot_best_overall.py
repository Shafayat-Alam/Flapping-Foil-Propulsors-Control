#!/usr/bin/env python3
"""The best drag-family and lift-family gaits, pooled across the whole
session -- plotted the same way regardless of which campaign produced them.

usage:  plot_best_overall.py <out.png> <drag_force.csv> <lift_force.csv>
"""
from __future__ import annotations

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))

import amfm_analyze as AN   # noqa: E402


def trace(path):
    r = AN.load_force(path)
    if r is None:
        return None
    t, fx, fy, fz = r
    return np.linspace(0, 1, len(fx), endpoint=False), fx, fy


def panel(ax, x, y, title, color):
    ax.axhline(0, color="0.8", lw=0.8, zorder=1)
    ax.plot(x, y, color=color, lw=1.7, zorder=3)
    ax.axhline(np.mean(y), color=color, lw=1.3, ls=":", zorder=2,
              label=f"mean {np.mean(y):+.3f} N")
    ax.fill_between(x, np.minimum(y, 0), 0, color=color, alpha=0.12, lw=0, zorder=1)
    ax.set_title(title, fontsize=10.5)
    ax.set_xlabel("cycle phase")
    ax.set_ylabel("N")
    ax.legend(fontsize=8.5, loc="upper right", framealpha=0.9)
    ax.margins(x=0)


def main():
    out, drag_fp, lift_fp = sys.argv[1], sys.argv[2], sys.argv[3]
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 6.6))
    fig.suptitle("Best gait overall, pooled across every campaign this session",
                 fontsize=13.5, y=0.975)

    for col, (fp, label) in enumerate(((drag_fp, "DRAG family (1 crest)"),
                                       (lift_fp, "LIFT family (2 crests)"))):
        tr = trace(fp)
        if tr is None:
            for row in range(2):
                axes[row, col].text(0.5, 0.5, "no data", ha="center", va="center",
                                    transform=axes[row, col].transAxes)
            continue
        x, fx, fy = tr
        panel(axes[0, col], x, fx, f"{label}\nFx — thrust", "#c2410c")
        panel(axes[1, col], x, fy, f"Fy — vertical", "#2563eb")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out, dpi=150, bbox_inches="tight")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
