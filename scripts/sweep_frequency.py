#!/usr/bin/env python3
"""
sweep_frequency.py — frequency sweep
====================================
Holds pitch amplitude, heave amplitude and phase constant; sweeps frequency.

Run in a second terminal while the launch is up and OPERATIONAL, loadcell
streaming:

    python3 sweep_frequency.py

Prompts for the output path (created fresh — everything lands there), the
constants, and the sweep (cycles / start / increment / number of samples).
Warns if a sample exceeds the servo's ~5.5 rad/s slew limit for the given
amplitudes (the servo would clip the stroke rather than follow it).
Results: one folder per mission command with its CSVs; view with plotter.py.
No analysis, no optimum — just the sweep.
"""

import math
import sweep_common as sc

D2R = math.pi / 180.0
SLEW_LIMIT = 5.5   # rad/s — measured servo slew ceiling


def main():
    sc.banner("FREQUENCY")

    outdir = sc.ask_outdir()
    print("\n  --- constants ---")
    pitch_amp = sc.ask_float("pitch amplitude (rad)", 0.235619449)
    heave_amp = sc.ask_float("heave amplitude (rad)", 0.235619449)
    phase     = sc.ask_float("phase (rad)", math.pi / 2)

    largest = max(pitch_amp, heave_amp)
    f_max = SLEW_LIMIT / (2 * math.pi * largest) if largest > 1e-6 else float("inf")
    print(f"\n  slew note: largest amplitude {largest:.4f} rad -> the servo can "
          f"follow up to ~{f_max:.3f} Hz (>{f_max:.3f} Hz will clip the stroke)")

    print("\n  --- sweep (frequency) ---")
    cycles    = sc.ask_int("cycles per mission command", 10)
    delay     = sc.ask_delay()
    start     = sc.ask_float("frequency start (Hz)", round(max(0.25, f_max * 0.4), 3))
    increment = sc.ask_float("increment between samples (Hz)",
                             round(max(0.05, (f_max - max(0.25, f_max * 0.4)) / 5), 3))
    n         = sc.ask_int("number of samples", 6)

    freqs = sc.build_points(start, increment, n)
    points = [{"label": f"FQ_{int(round(f * 1000)):05d}",
               "frequency": f, "pitch_amp": pitch_amp,
               "heave_amp": heave_amp, "phase": phase} for f in freqs]

    print(f"\n  {n} missions, frequency {freqs[0]:.4f} .. {freqs[-1]:.4f} Hz")
    over = [f for f in freqs if 2 * math.pi * f * largest > SLEW_LIMIT]
    if over:
        print(f"  ! WARNING: {len(over)} sample(s) exceed the slew limit "
              f"({', '.join(f'{f:.3f}Hz' for f in over)}) — the servo will clip "
              f"the commanded amplitude at those points.")
    est = sum(cycles / f for f in freqs) / 60.0 + n * (delay + 5) / 60.0
    print(f"  estimated runtime: ~{est:.0f} min")
    if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        return

    node = sc.start_ros()
    try:
        sc.run_missions(node, outdir, points, cycles, delay)
    finally:
        sc.stop_ros(node)

    sc.write_info(outdir, "frequency",
                  {"pitch_amp (rad)": pitch_amp, "heave_amp (rad)": heave_amp,
                   "phase (rad)": phase},
                  f"frequency: {freqs[0]:.4f}..{freqs[-1]:.4f} Hz, step {increment}",
                  points, cycles)

    print(f"\n=== done -> {outdir}")
    print(f"  view plots:  python3 plotter.py {outdir}")
    print(f"  per mission: python3 plotter.py {outdir} --per-mission")


if __name__ == "__main__":
    main()
