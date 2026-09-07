#!/usr/bin/env python3
"""
sweep_phase.py — phase-shift sweep
==================================
Holds pitch amplitude, heave amplitude and frequency constant; sweeps the
phase (heave relative to pitch).

Run in a second terminal while the launch is up and OPERATIONAL, loadcell
streaming:

    python3 sweep_phase.py

Prompts for the output path (created fresh — everything lands there), the
constants, and the sweep (cycles / start / increment / number of samples).
Results: one folder per mission command with its CSVs; view with plotter.py.
No analysis, no optimum — just the sweep.
"""

import math
import sweep_common as sc

D2R = math.pi / 180.0


def main():
    sc.banner("PHASE SHIFT")

    outdir = sc.ask_outdir()
    print("\n  --- constants ---")
    pitch_amp = sc.ask_float("pitch amplitude (rad)", 0.235619449)
    heave_amp = sc.ask_float("heave amplitude (rad)", 0.235619449)
    frequency = sc.ask_float("frequency (Hz)", 0.75)

    print("\n  --- sweep (phase) ---")
    cycles    = sc.ask_int("cycles per mission command", 10)
    delay     = sc.ask_delay()
    start     = sc.ask_float("phase start (rad)", 0.0)
    increment = sc.ask_float("increment between samples (rad)", 0.174533)
    n         = sc.ask_int("number of samples", 19)

    phases = sc.build_points(start, increment, n)
    points = [{"label": f"PH_{int(round(ph / D2R)):04d}",
               "frequency": frequency, "pitch_amp": pitch_amp,
               "heave_amp": heave_amp, "phase": ph} for ph in phases]

    print(f"\n  {n} missions, phase {phases[0]:.4f} .. {phases[-1]:.4f} rad "
          f"({phases[0]/D2R:.1f}° .. {phases[-1]/D2R:.1f}°)")
    est = n * (cycles / frequency + delay + 5) / 60.0
    print(f"  estimated runtime: ~{est:.0f} min")
    if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        return

    node = sc.start_ros()
    try:
        sc.run_missions(node, outdir, points, cycles, delay)
    finally:
        sc.stop_ros(node)

    sc.write_info(outdir, "phase shift",
                  {"pitch_amp (rad)": pitch_amp, "heave_amp (rad)": heave_amp,
                   "frequency (Hz)": frequency},
                  f"phase: {phases[0]:.4f}..{phases[-1]:.4f} rad, step {increment}",
                  points, cycles)

    print(f"\n=== done -> {outdir}")
    print(f"  view plots:  python3 plotter.py {outdir}")
    print(f"  per mission: python3 plotter.py {outdir} --per-mission")


if __name__ == "__main__":
    main()
