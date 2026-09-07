#!/usr/bin/env python3
"""
sweep_amplitude.py — amplitude sweep (pitch or heave)
=====================================================
Sweeps ONE amplitude axis (pitch or heave) while holding the other amplitude,
the frequency and the phase constant.

Run in a second terminal while the launch is up and OPERATIONAL, loadcell
streaming:

    python3 sweep_amplitude.py

Prompts for the output path (created fresh — everything lands there), which
axis to sweep, the constants, and the sweep (cycles / start / increment /
number of samples).  Results: one folder per mission command with its CSVs;
view with plotter.py.  No analysis, no optimum — just the sweep.
"""

import math
import sweep_common as sc

D2R = math.pi / 180.0


def main():
    sc.banner("AMPLITUDE")

    outdir = sc.ask_outdir()
    axis = sc.ask_choice("sweep which axis", ["pitch", "heave"])

    print("\n  --- constants ---")
    if axis == "pitch":
        heave_amp = sc.ask_float("heave amplitude (rad) [held]", 0.235619449)
        other = {"heave_amp": heave_amp}
    else:
        pitch_amp = sc.ask_float("pitch amplitude (rad) [held]", 0.235619449)
        other = {"pitch_amp": pitch_amp}
    frequency = sc.ask_float("frequency (Hz)", 0.75)
    phase     = sc.ask_float("phase (rad)", math.pi / 2)

    print(f"\n  --- sweep ({axis} amplitude) ---")
    cycles    = sc.ask_int("cycles per mission command", 10)
    delay     = sc.ask_delay()
    start     = sc.ask_float(f"{axis} amplitude start (rad)",
                             0.261799 if axis == "pitch" else 0.785398)
    increment = sc.ask_float("increment between samples (rad)", 5 * D2R)
    n         = sc.ask_int("number of samples", 7 if axis == "pitch" else 10)

    amps = sc.build_points(start, increment, n)
    tag = "PA" if axis == "pitch" else "HA"
    points = []
    for a in amps:
        p = {"label": f"{tag}_{int(round(a / D2R)):04d}",
             "frequency": frequency, "phase": phase}
        p["pitch_amp"] = a if axis == "pitch" else other["pitch_amp"]
        p["heave_amp"] = a if axis == "heave" else other["heave_amp"]
        points.append(p)

    print(f"\n  {n} missions, {axis} amplitude {amps[0]:.4f} .. {amps[-1]:.4f} rad "
          f"({amps[0]/D2R:.1f}° .. {amps[-1]/D2R:.1f}°)")
    # slew sanity: peak axis speed = 2*pi*f*A must stay under ~5.5 rad/s
    peak = 2 * math.pi * frequency * amps[-1]
    if peak > 5.5:
        print(f"  ! WARNING: largest sample needs {peak:.2f} rad/s peak "
              f"(> ~5.5 rad/s slew limit) — the servo will not reach that "
              f"amplitude; it will be clipped.")
    est = n * (cycles / frequency + delay + 5) / 60.0
    print(f"  estimated runtime: ~{est:.0f} min")
    if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        return

    node = sc.start_ros()
    try:
        sc.run_missions(node, outdir, points, cycles, delay)
    finally:
        sc.stop_ros(node)

    consts = {"frequency (Hz)": frequency, "phase (rad)": phase}
    consts.update({k + " (rad)": v for k, v in other.items()})
    sc.write_info(outdir, f"{axis} amplitude", consts,
                  f"{axis}_amp: {amps[0]:.4f}..{amps[-1]:.4f} rad, step {increment}",
                  points, cycles)

    print(f"\n=== done -> {outdir}")
    print(f"  view plots:  python3 plotter.py {outdir}")
    print(f"  per mission: python3 plotter.py {outdir} --per-mission")


if __name__ == "__main__":
    main()
