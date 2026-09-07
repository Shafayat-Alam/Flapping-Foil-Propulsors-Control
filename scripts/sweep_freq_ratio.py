#!/usr/bin/env python3
"""
sweep_freq_ratio.py — phase sweep repeated at several pitch:heave frequency
ratios (leaves lift/drag classification alone; that's a later analysis step).

HEAVE frequency is held constant; PITCH frequency is the swept outer variable
(0.5 Hz up to the slew-limited max, in fixed increments — auto-derived from
the pitch amplitude and the ~5.5 rad/s slew ceiling, same as find_freq.py).
freq_ratio = heave_freq / pitch_freq is computed per step and passed straight
through crab -> controller -> mc.paddle (heave_freq_ratio).  At each pitch
frequency, a full phase sweep (0..360° by the given increment) is run — that
is the INNER loop.

    python3 scripts/sweep_freq_ratio.py                 # interactive prompts
    python3 scripts/sweep_freq_ratio.py --config x.json # unattended

Output layout:
  <root>/pf<pitch_freq>_r<ratio>/  — one phase sweep's mission folders, per step
"""
import os, sys, json, math, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sweep_common as sc

D2R = math.pi / 180.0
SLEW_LIMIT = 5.5   # rad/s — measured servo slew ceiling


def _pitch_freqs(pitch_amp, start, inc, explicit_max=None):
    """Pitch-frequency steps from start, by inc, up to the slew-safe max implied
    by pitch_amp (2*pi*f*amp <= SLEW_LIMIT), or explicit_max if given (whichever
    is lower)."""
    f_slew = SLEW_LIMIT / (2 * math.pi * pitch_amp) if pitch_amp > 1e-9 else float("inf")
    f_max = min(f_slew, explicit_max) if explicit_max else f_slew
    freqs = []
    f = start
    while f <= f_max + 1e-9:
        freqs.append(round(f, 6))
        f += inc
    return freqs, f_slew


def ask_root_folder():
    while True:
        s = input("  output path (folder created; all steps land here): ").strip()
        if not s:
            print("    -> enter a path"); continue
        p = os.path.abspath(os.path.expanduser(s))
        if os.path.exists(p) and os.listdir(p):
            if input(f"    '{p}' exists and is not empty — use anyway? (y/n): "
                     ).strip().lower() not in ("y", "yes"):
                continue
        os.makedirs(p, exist_ok=True)
        return p


def collect_interactive():
    sc.banner("PITCH-FREQUENCY x PHASE SWEEP (heave frequency held constant)")
    root = ask_root_folder()
    print("\n  --- held constants ---")
    pitch_amp = sc.ask_float("pitch amplitude (rad)", 0.392699082)   # 25%
    heave_amp = sc.ask_float("heave amplitude (rad)", 0.392699082)
    heave_freq = sc.ask_float("heave frequency (Hz) [held]", 0.5)
    cycles = sc.ask_int("cycles per mission command (counted at pitch frequency)", 4)
    delay = sc.ask_delay()

    print("\n  --- phase sweep (inner loop, per pitch-frequency step) ---")
    ph_start = sc.ask_float("phase start (rad)", 0.0)
    ph_inc = sc.ask_float("phase increment (rad)", 0.261799)   # 15 deg
    ph_n = sc.ask_int("phase samples", 25)

    print("\n  --- pitch frequency sweep (outer loop) ---")
    f_slew = SLEW_LIMIT / (2 * math.pi * pitch_amp) if pitch_amp > 1e-9 else float("inf")
    print(f"  slew-safe pitch frequency ceiling for amp={pitch_amp:.4f} rad: "
          f"{f_slew:.3f} Hz")
    pf_start = sc.ask_float("pitch frequency start (Hz)", 0.5)
    pf_inc = sc.ask_float("pitch frequency increment (Hz)", 0.25)
    pitch_freqs, f_slew = _pitch_freqs(pitch_amp, pf_start, pf_inc)

    n_missions = ph_n * len(pitch_freqs)
    print(f"\n  pitch frequencies: {pitch_freqs}")
    print(f"  ratios (heave/pitch): {[round(heave_freq/f, 3) for f in pitch_freqs]}")
    est = sum(cycles / f + delay + 5 for f in pitch_freqs) * ph_n / 60.0
    print(f"  {len(pitch_freqs)} pitch-freq steps x {ph_n} phases = {n_missions} missions")
    print(f"  estimated runtime: ~{est:.0f} min")
    if input("\n  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        sys.exit(0)

    return root, {"pitch_amp": pitch_amp, "heave_amp": heave_amp,
                  "heave_freq": heave_freq, "cycles": cycles, "delay": delay,
                  "ph_start": ph_start, "ph_inc": ph_inc, "ph_n": ph_n,
                  "pitch_freqs": pitch_freqs}


def collect_config(path):
    with open(os.path.expanduser(path)) as f:
        c = json.load(f)
    g = c.get
    name = c["folder"]
    if os.path.sep in name or name in (".", ".."):
        sys.exit("  config 'folder' must be a plain name, not a path")
    root = os.path.join(os.getcwd(), name)
    os.makedirs(root, exist_ok=True)
    pitch_amp = float(g("pitch_amp", 0.392699082))
    cfg = {
        "pitch_amp": pitch_amp,
        "heave_amp": float(g("heave_amp", 0.392699082)),
        "heave_freq": float(g("heave_freq", 0.5)),
        "cycles": int(g("cycles", 4)),
        "delay": float(g("delay", sc.INTER_MISSION_DELAY)),
        "ph_start": float(g("ph_start", 0.0)),
        "ph_inc": float(g("ph_inc", 0.261799)),
        "ph_n": int(g("ph_n", 25)),
    }
    if "pitch_freqs" in c:
        cfg["pitch_freqs"] = [float(x) for x in c["pitch_freqs"]]
        f_slew = SLEW_LIMIT / (2 * math.pi * pitch_amp) if pitch_amp > 1e-9 else float("inf")
    else:
        cfg["pitch_freqs"], f_slew = _pitch_freqs(
            pitch_amp, float(g("pitch_freq_start", 0.5)),
            float(g("pitch_freq_inc", 0.25)), g("pitch_freq_max"))
    n = cfg["ph_n"] * len(cfg["pitch_freqs"])
    print(f"\n=== PITCH-FREQUENCY x PHASE SWEEP, unattended ===")
    print(f"  slew-safe pitch frequency ceiling for amp={pitch_amp:.4f} rad: "
          f"{f_slew:.3f} Hz")
    print(f"  pitch frequencies: {cfg['pitch_freqs']}")
    print(f"  ratios (heave/pitch): "
          f"{[round(cfg['heave_freq']/f, 3) for f in cfg['pitch_freqs']]}")
    print(f"  {len(cfg['pitch_freqs'])} steps x {cfg['ph_n']} phases = {n} missions")
    print(f"  everything lands in: {root}")
    return root, cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None,
                    help="JSON of all values -> run unattended (no prompts)")
    args = ap.parse_args()

    root, cfg = collect_config(args.config) if args.config else collect_interactive()
    phases = sc.build_points(cfg["ph_start"], cfg["ph_inc"], cfg["ph_n"])
    heave_freq = cfg["heave_freq"]

    node = sc.start_ros()
    try:
        for pf in cfg["pitch_freqs"]:
            ratio = heave_freq / pf if pf > 1e-9 else 1.0
            tag = f"pf{pf:.3f}_r{ratio:.3f}".replace(".", "p")
            outdir = os.path.join(root, tag)
            os.makedirs(outdir, exist_ok=True)
            points = [{"label": f"PH_{int(round(p/D2R)):04d}",
                       "frequency": pf, "pitch_amp": cfg["pitch_amp"],
                       "heave_amp": cfg["heave_amp"], "phase": p,
                       "freq_ratio": ratio} for p in phases]
            print(f"\n{'='*70}")
            print(f"PITCH FREQ {pf:.3f} Hz  (heave = {heave_freq:.3f} Hz const, "
                  f"ratio = {ratio:.3f})  — {len(points)} missions")
            print("=" * 70)
            sc.run_missions(node, outdir, points, cfg["cycles"], cfg["delay"])
            sc.write_info(outdir, f"phase sweep @ pitch_freq={pf}, heave_freq={heave_freq}",
                          {"pitch_amp (rad)": cfg["pitch_amp"],
                           "heave_amp (rad)": cfg["heave_amp"],
                           "pitch frequency (Hz)": pf,
                           "heave frequency (Hz)": heave_freq,
                           "freq_ratio (heave/pitch)": ratio},
                          f"phase {phases[0]:.4f}..{phases[-1]:.4f} rad, "
                          f"step {cfg['ph_inc']}", points, cfg["cycles"])
    finally:
        sc.stop_ros(node)

    print(f"\n=== done -> {root}")
    for pf in cfg["pitch_freqs"]:
        ratio = heave_freq / pf if pf > 1e-9 else 1.0
        tag = f"pf{pf:.3f}_r{ratio:.3f}".replace(".", "p")
        print(f"  pitch_freq={pf:.3f}Hz (ratio={ratio:.3f}): "
              f"python3 scripts/propulsion_identifier.py --from {os.path.join(root, tag)}")


if __name__ == "__main__":
    main()
