#!/usr/bin/env python3
"""Generate one plot per target curve for the Propulsion Profile Definition
section: PCHIP (points + shape-preserving cubic Hermite interpolation) and
B-spline (structural periodic cubic B-spline), drag and lift each.

usage: python3 scripts/plot_profile_definition_curves.py <out_dir>
"""
import json
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import PchipInterpolator

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
import target_curves as TC

DRAG_JSON = "/mnt/usb/Fin_Dynmaics/Grey-Box_Parametric_Studies/Target_Curve_PCHIP_Attempts/drag_dominant/target_curve_drag_dominant.json"
LIFT_JSON = "/mnt/usb/Fin_Dynmaics/Grey-Box_Parametric_Studies/Target_Curve_PCHIP_Attempts/target_curve_lift_dominant.json"


def plot_pchip(json_path, title, color, out_path):
    spec = json.load(open(json_path))
    pts = spec["channel_definitions"]["Fx"]["target_points"]
    t_pts = np.array([p["t"] for p in pts])
    F_pts = np.array([p["F"] for p in pts])
    pchip = PchipInterpolator(t_pts, F_pts)
    t_dense = np.linspace(t_pts[0], t_pts[-1], 500)
    F_dense = pchip(t_dense)

    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.axhline(0, color="0.75", lw=0.8)
    ax.plot(t_dense, F_dense, color=color, lw=2.2, label="PCHIP interpolation")
    ax.plot(t_pts, F_pts, "o", color="black", ms=5, label="typed points")
    ax.set_title(title)
    ax.set_xlabel("t (normalized cycle)")
    ax.set_ylabel("F (N)")
    ax.legend(fontsize=9)
    ax.margins(x=0)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print("wrote", out_path)


def plot_bspline(spline_fn, title, color, out_path):
    sp, P = spline_fn()
    t, S = TC.evaluate(sp)
    fig, ax = plt.subplots(figsize=(7, 4.2))
    ax.axhline(0, color="0.75", lw=0.8)
    ax.plot(t, S, color=color, lw=2.2)
    ax.set_title(title)
    ax.set_xlabel("t (normalized cycle)")
    ax.set_ylabel("S(t) (scale-free)")
    ax.margins(x=0)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    print("wrote", out_path)


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else "."
    os.makedirs(out_dir, exist_ok=True)

    plot_pchip(DRAG_JSON, "PCHIP target: drag-dominant", "#c2410c",
              os.path.join(out_dir, "pchip_drag.png"))
    plot_pchip(LIFT_JSON, "PCHIP target: lift-dominant", "#2563eb",
              os.path.join(out_dir, "pchip_lift.png"))
    plot_bspline(TC.drag_target, "B-spline target: drag-dominant", "#c2410c",
                os.path.join(out_dir, "bspline_drag.png"))
    plot_bspline(TC.lift_target, "B-spline target: lift-dominant", "#2563eb",
                os.path.join(out_dir, "bspline_lift.png"))


if __name__ == "__main__":
    main()
