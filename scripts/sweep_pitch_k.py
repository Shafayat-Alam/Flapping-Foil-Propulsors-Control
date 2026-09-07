#!/usr/bin/env python3
"""
sweep_pitch_k.py — pitch-curve shape (k) x pitch-frequency x phase sweep.

Three nested loops:
  outermost : pitch_k        — the pitch (servo 1) curve's shape exponent
                               (see mc.shaped_sine): k=0 is a plain sine,
                               increasing k squares off the curve, k=inf is a
                               true square wave.  Heave (servo 2) always stays
                               a plain sine — only pitch is shaped.
  middle    : pitch_freq     — 0.5 Hz up to the slew-limited max, in fixed
                               steps.  HEAVE frequency is held constant.
  innermost : phase          — 0..360°, the usual phase sweep.

After EVERY phase sweep (one (k, pitch_freq) block), the script STOPS and
prompts before starting the next block — whether that's the next pitch
frequency at the same k, or the first frequency of the next k.  Answering
anything other than y/yes ends the run early (data already collected stays).

    python3 scripts/sweep_pitch_k.py                 # interactive prompts
    python3 scripts/sweep_pitch_k.py --config x.json # unattended EXCEPT the
                                                       # per-block pause, which
                                                       # always happens
"""
import os, sys, json, math, argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sweep_common as sc
from sweep_freq_ratio import _pitch_freqs, SLEW_LIMIT

D2R = math.pi / 180.0


def _fmt_k(k):
    return "inf" if math.isinf(k) else f"{k:g}"


def _parse_k(s):
    return float("inf") if str(s).strip().lower() in ("inf", "infinity") else float(s)


def ask_root_folder():
    while True:
        s = input("  output path (folder created; all blocks land here): ").strip()
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
    sc.banner("PITCH-k x PITCH-FREQUENCY x PHASE SWEEP (stops after every phase sweep)")
    root = ask_root_folder()
    print("\n  --- held constants ---")
    pitch_amp = sc.ask_float("pitch amplitude (rad)", 0.392699082)
    heave_amp = sc.ask_float("heave amplitude (rad)", 0.392699082)
    heave_freq = sc.ask_float("heave frequency (Hz) [held]", 0.5)
    cycles = sc.ask_int("cycles per mission command (counted at pitch frequency)", 4)
    delay = sc.ask_delay()

    print("\n  --- phase sweep (innermost loop) ---")
    ph_start = sc.ask_float("phase start (rad)", 0.0)
    ph_inc = sc.ask_float("phase increment (rad)", 0.261799)
    ph_n = sc.ask_int("phase samples", 25)

    print("\n  --- pitch frequency sweep (middle loop) ---")
    f_slew = SLEW_LIMIT / (2 * math.pi * pitch_amp) if pitch_amp > 1e-9 else float("inf")
    print(f"  slew-safe pitch frequency ceiling for amp={pitch_amp:.4f} rad: "
          f"{f_slew:.3f} Hz")
    pf_start = sc.ask_float("pitch frequency start (Hz)", 0.5)
    pf_inc = sc.ask_float("pitch frequency increment (Hz)", 0.25)
    pitch_freqs, _ = _pitch_freqs(pitch_amp, pf_start, pf_inc)

    print("\n  --- pitch-k sweep (outermost loop) ---")
    k_raw = input("  k values, comma-separated [1,2,4,8,inf]: ").strip() or "1,2,4,8,inf"
    pitch_ks = [_parse_k(x) for x in k_raw.split(",")]

    n_blocks = len(pitch_ks) * len(pitch_freqs)
    print(f"\n  pitch_k values: {[_fmt_k(k) for k in pitch_ks]}")
    print(f"  pitch frequencies: {pitch_freqs}")
    print(f"  {n_blocks} phase-sweep blocks total ({len(pitch_ks)} k x "
          f"{len(pitch_freqs)} freq), {ph_n} missions each = "
          f"{n_blocks*ph_n} missions if run to completion")
    print("  you will be prompted before EVERY block.")
    if input("\n  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        sys.exit(0)

    return root, {"pitch_amp": pitch_amp, "heave_amp": heave_amp,
                  "heave_freq": heave_freq, "cycles": cycles, "delay": delay,
                  "ph_start": ph_start, "ph_inc": ph_inc, "ph_n": ph_n,
                  "pitch_freqs": pitch_freqs, "pitch_ks": pitch_ks}


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
    else:
        cfg["pitch_freqs"], _ = _pitch_freqs(
            pitch_amp, float(g("pitch_freq_start", 0.5)),
            float(g("pitch_freq_inc", 0.25)), g("pitch_freq_max"))
    cfg["pitch_ks"] = [_parse_k(x) for x in g("pitch_ks", [1, 2, 4, 8, "inf"])]

    n_blocks = len(cfg["pitch_ks"]) * len(cfg["pitch_freqs"])
    print(f"\n=== PITCH-k x PITCH-FREQUENCY x PHASE SWEEP ===")
    print(f"  pitch_k values: {[_fmt_k(k) for k in cfg['pitch_ks']]}")
    print(f"  pitch frequencies: {cfg['pitch_freqs']}")
    print(f"  {n_blocks} phase-sweep blocks ({len(cfg['pitch_ks'])} k x "
          f"{len(cfg['pitch_freqs'])} freq), {cfg['ph_n']} missions each = "
          f"{n_blocks*cfg['ph_n']} missions if run to completion")
    print("  you will be prompted before EVERY block (this is not unattended).")
    print(f"  everything lands in: {root}")
    return root, cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None,
                    help="JSON of all values (the per-block pause still happens)")
    ap.add_argument("--start-block", type=int, default=0,
                    help="skip this many blocks at the start (0-indexed) — for "
                         "resuming a later batch; blocks before this are assumed "
                         "already done and are NOT re-run")
    ap.add_argument("--max-blocks", type=int, default=None,
                    help="run at most this many blocks, then stop automatically "
                         "(no prompt) — for running N blocks at a time. Without "
                         "this, every block pauses for a y/n prompt as usual.")
    args = ap.parse_args()

    root, cfg = collect_config(args.config) if args.config else collect_interactive()
    phases = sc.build_points(cfg["ph_start"], cfg["ph_inc"], cfg["ph_n"])
    heave_freq = cfg["heave_freq"]

    all_blocks = [(k, pf) for k in cfg["pitch_ks"] for pf in cfg["pitch_freqs"]]
    blocks = all_blocks[args.start_block:]
    base_i = args.start_block   # so printed block numbers match the full plan
    if not blocks:
        print(f"\n  --start-block {args.start_block} >= {len(all_blocks)} total "
              f"blocks — nothing to do."); return

    batch_mode = args.max_blocks is not None
    if batch_mode:
        n_this_batch = min(args.max_blocks, len(blocks))
        print(f"\n  batch mode: running blocks {base_i+1}-{base_i+n_this_batch} "
              f"of {len(all_blocks)} (no per-block prompts), then stopping.")
    else:
        print(f"\n  {len(blocks)} blocks queued (starting at block {base_i+1}). "
              f"First: k={_fmt_k(blocks[0][0])}, pitch_freq={blocks[0][1]} Hz\n")
        if input("  start the first block? (y/n): ").strip().lower() not in ("y", "yes"):
            print("  stopped before starting."); return

    node = sc.start_ros()
    done = []
    try:
        for j, (k, pf) in enumerate(blocks):
            i = base_i + j
            if batch_mode and j >= args.max_blocks:
                print(f"\n  BATCH LIMIT ({args.max_blocks}) reached — stopping.")
                left = all_blocks[i:]
                print(f"  {len(left)} block(s) not run. Resume with "
                      f"--start-block {i} --max-blocks {args.max_blocks}")
                for k2, pf2 in left:
                    print(f"    k={_fmt_k(k2)}  pitch_freq={pf2:.3f} Hz")
                break
            ratio = heave_freq / pf if pf > 1e-9 else 1.0
            tag = f"k{_fmt_k(k)}_pf{pf:.3f}_r{ratio:.3f}".replace(".", "p")
            outdir = os.path.join(root, tag)
            os.makedirs(outdir, exist_ok=True)
            points = [{"label": f"PH_{int(round(p/D2R)):04d}",
                       "frequency": pf, "pitch_amp": cfg["pitch_amp"],
                       "heave_amp": cfg["heave_amp"], "phase": p,
                       "freq_ratio": ratio, "pitch_k": k} for p in phases]

            print(f"\n{'='*70}")
            print(f"BLOCK {i+1}/{len(all_blocks)}:  pitch_k={_fmt_k(k)}  "
                  f"pitch_freq={pf:.3f} Hz  (heave={heave_freq:.3f} Hz const, "
                  f"ratio={ratio:.3f})  — {len(points)} missions")
            print("=" * 70)
            sc.run_missions(node, outdir, points, cfg["cycles"], cfg["delay"])
            sc.write_info(outdir,
                          f"phase sweep @ pitch_k={_fmt_k(k)}, pitch_freq={pf}, "
                          f"heave_freq={heave_freq}",
                          {"pitch_amp (rad)": cfg["pitch_amp"],
                           "heave_amp (rad)": cfg["heave_amp"],
                           "pitch frequency (Hz)": pf,
                           "heave frequency (Hz)": heave_freq,
                           "freq_ratio (heave/pitch)": ratio,
                           "pitch_k": _fmt_k(k)},
                          f"phase {phases[0]:.4f}..{phases[-1]:.4f} rad, "
                          f"step {cfg['ph_inc']}", points, cfg["cycles"])
            done.append((k, pf, outdir))

            print(f"\n  block {i+1}/{len(all_blocks)} done -> {outdir}")
            remaining_total = len(all_blocks) - i - 1
            if remaining_total == 0:
                print("\n  ALL BLOCKS COMPLETE.")
                break
            if batch_mode:
                # in batch mode the loop's own limit check (top of loop) handles
                # stopping — just continue straight to the next block, no prompt.
                continue
            nk, npf = all_blocks[i + 1]
            print(f"  next block ({remaining_total} left): "
                  f"k={_fmt_k(nk)}  pitch_freq={npf:.3f} Hz")
            ans = input("  proceed to next block? (y/n): ").strip().lower()
            if ans not in ("y", "yes"):
                print(f"\n  STOPPED by request after block {i+1}/{len(all_blocks)}.")
                left = all_blocks[i + 1:]
                print(f"  {len(left)} block(s) not run. Resume with "
                      f"--start-block {i+1}")
                for k2, pf2 in left:
                    print(f"    k={_fmt_k(k2)}  pitch_freq={pf2:.3f} Hz")
                break
    finally:
        sc.stop_ros(node)

    print(f"\n=== summary: {len(done)}/{len(blocks)} blocks completed -> {root}")
    for k, pf, outdir in done:
        print(f"  k={_fmt_k(k)} pf={pf:.3f}Hz: "
              f"python3 scripts/propulsion_identifier.py --from {outdir}")


if __name__ == "__main__":
    main()
