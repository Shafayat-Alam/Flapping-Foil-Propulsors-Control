#!/usr/bin/env python3
"""
sweep_amp_freq_phase.py — freq-ratio x amp-ratio x phase sweep, k=1 only.

Three nested loops, all built around ONE fixed baseline point
(center_freq, center_amp) — pitch_freq=heave_freq=center_freq and
pitch_amp=heave_amp=center_amp at ratio=1.0 on both dimensions, which is
exactly the original in_house_wet_test_pitch_k baseline (0.5 Hz, 0.6283185307
rad / 40%):

  outermost : freq_ratio — Rf = heave_freq / pitch_freq. Moving Rf away from
                            1.0 moves BOTH frequencies oppositely around
                            center_freq, geometric-mean-preserving:
                              pitch_freq = center_freq / sqrt(Rf)
                              heave_freq = center_freq * sqrt(Rf)
                            (NOT the earlier design, which held one frequency
                            fixed at a baseline and only moved the other.)
  middle    : amp_ratio  — R_A = pitch_amp / heave_amp. Same symmetric split
                            around center_amp:
                              pitch_amp = center_amp * sqrt(R_A)
                              heave_amp = center_amp / sqrt(R_A)
  innermost : phase       — 0..360°, the usual phase sweep.

center_freq=0.5 Hz and center_amp=0.6283185307 rad were picked specifically
because the full 7x5 ratio grid stays under the ~5.5 rad/s slew ceiling on
BOTH servos everywhere (worst case: Rf=2.5 x R_A=0.33, at 5.43 rad/s) — this
is checked again here per-combo rather than assumed, so any future edit to
the ratio lists or the center point gets re-validated automatically.
Combinations that fail are SKIPPED (reported, not silently dropped).

freq_ratio==1.0 and amp_ratio==1.0 together is exactly the original baseline
block (in_house_wet_test_pitch_k/k1_pf0p500_r1p000) — if that folder already
exists and is non-empty, it's reused instead of re-run.

Output layout (flat, one folder per (freq_ratio, amp_ratio) combo, 25 phase
missions each):
  <root>/k1_fr<freq_ratio>_ar<amp_ratio>/
  (the freq_ratio=1.0, amp_ratio=1.0 combo instead reuses/writes
   <root>/k1_pf0p500_r1p000/ to match the pre-existing baseline folder name)

    python3 scripts/sweep_amp_freq_phase.py --config x.json --plan-only
    python3 scripts/sweep_amp_freq_phase.py --config x.json --start-block N --max-blocks M
"""
import os, sys, json, math, argparse, glob

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import sweep_common as sc

D2R = math.pi / 180.0
SLEW_LIMIT = 5.5   # rad/s — measured servo slew ceiling (same as the other sweeps)


def _block_recorded(outdir):
    """True if this block's data has already been captured — either fully
    split (PH_* mission folders present) or recorded-but-split-deferred
    (raw/RECORDING_COMPLETE marker present, see sweep_common.run_missions).
    Deliberately NOT just "does outdir have anything in it": a block killed
    mid-recording has a raw/ folder (partial bag) but no marker and no PH_*
    folders, so it correctly reports False here and gets re-run."""
    if not os.path.isdir(outdir):
        return False
    if glob.glob(os.path.join(outdir, "PH_*")):
        return True
    return os.path.exists(os.path.join(outdir, "raw", sc.RECORDING_COMPLETE_MARKER))


def collect_config(path):
    with open(os.path.expanduser(path)) as f:
        c = json.load(f)
    g = c.get
    name = c["folder"]
    if os.path.sep in name or name in (".", ".."):
        sys.exit("  config 'folder' must be a plain name, not a path")
    root = os.path.join(os.getcwd(), name)
    os.makedirs(root, exist_ok=True)
    cfg = {
        "k": float(g("k", 1)),
        "center_freq": float(g("center_freq", 0.5)),
        "center_amp": float(g("center_amp", 0.6283185307)),
        "cycles": int(g("cycles", 4)),
        "delay": float(g("delay", sc.INTER_MISSION_DELAY)),
        "ph_start": float(g("ph_start", 0.0)),
        "ph_inc": float(g("ph_inc", 0.261799)),
        "ph_n": int(g("ph_n", 25)),
        "freq_ratios": [float(x) for x in c["freq_ratios"]],
        "amp_ratios": [float(x) for x in c["amp_ratios"]],
    }
    return root, cfg


def build_plan(root, cfg):
    """Every (freq_ratio, amp_ratio) combo, classified as
    RUN / SKIP-UNSAFE / SKIP-EXISTS (phase is innermost, handled per-block)."""
    cf, ca = cfg["center_freq"], cfg["center_amp"]
    plan = []
    for fr in cfg["freq_ratios"]:
        pitch_freq = cf / math.sqrt(fr)
        heave_freq = cf * math.sqrt(fr)
        for ar in cfg["amp_ratios"]:
            pitch_amp = ca * math.sqrt(ar)
            heave_amp = ca / math.sqrt(ar)
            pitch_req = 2 * math.pi * pitch_freq * pitch_amp
            heave_req = 2 * math.pi * heave_freq * heave_amp

            if abs(fr - 1.0) < 1e-9 and abs(ar - 1.0) < 1e-9:
                tag = "k1_pf0p500_r1p000"
            else:
                tag = f"k1_fr{fr:.3f}_ar{ar:.2f}".replace(".", "p")
            outdir = os.path.join(root, tag)

            if pitch_req > SLEW_LIMIT or heave_req > SLEW_LIMIT:
                status = "SKIP-UNSAFE"
            elif _block_recorded(outdir):
                status = "SKIP-EXISTS"
            else:
                status = "RUN"
            plan.append({
                "freq_ratio": fr, "amp_ratio": ar,
                "pitch_freq": pitch_freq, "heave_freq": heave_freq,
                "pitch_amp": pitch_amp, "heave_amp": heave_amp,
                "pitch_req": pitch_req, "heave_req": heave_req,
                "outdir": outdir, "tag": tag, "status": status,
            })
    return plan


def print_plan(plan, cfg):
    print(f"\n=== FREQ-RATIO x AMP-RATIO x PHASE SWEEP (k={cfg['k']:g} only) ===")
    print(f"  center_freq = {cfg['center_freq']:.4f} Hz  |  center_amp = "
          f"{cfg['center_amp']:.6f} rad  |  ph_n = {cfg['ph_n']}  |  "
          f"cycles = {cfg['cycles']}\n")
    print(f"  {'freq_ratio':>10} {'amp_ratio':>9} {'pf':>7} {'hf':>7} {'p_amp':>7} "
          f"{'h_amp':>7} {'p_req':>6} {'h_req':>6}  {'status':<12} outdir")
    n_run = n_unsafe = n_exists = 0
    for p in plan:
        print(f"  {p['freq_ratio']:>10.3f} {p['amp_ratio']:>9.2f} {p['pitch_freq']:>7.4f} "
              f"{p['heave_freq']:>7.4f} {p['pitch_amp']:>7.4f} {p['heave_amp']:>7.4f} "
              f"{p['pitch_req']:>6.2f} {p['heave_req']:>6.2f}  {p['status']:<12} {p['outdir']}")
        if p["status"] == "RUN":
            n_run += 1
        elif p["status"] == "SKIP-UNSAFE":
            n_unsafe += 1
        else:
            n_exists += 1
    n_missions = n_run * cfg["ph_n"]
    print(f"\n  {len(plan)} total combos: {n_run} to run ({n_missions} missions "
          f"@ {cfg['ph_n']} phases each), {n_exists} already exist (reused), "
          f"{n_unsafe} unsafe (skipped, over {SLEW_LIMIT} rad/s slew ceiling)")
    if n_unsafe:
        print(f"\n  *** {n_unsafe} combo(s) exceed the slew ceiling at this center "
              f"point/ratio range — see SKIP-UNSAFE rows above. ***")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--start-block", type=int, default=0,
                    help="skip this many RUN-status blocks at the start (0-indexed)")
    ap.add_argument("--max-blocks", type=int, default=None,
                    help="run at most this many blocks, then stop automatically")
    ap.add_argument("--plan-only", action="store_true",
                    help="just print the plan (safety check + dedup) and exit, "
                         "no ROS, no hardware — for prep/review before a run")
    ap.add_argument("--no-split", action="store_true",
                    help="record each block but skip splitting into per-mission "
                         "folders — faster back-to-back blocks (no split delay "
                         "between them). Run scripts/split_pending.py afterward "
                         "to split everything that was deferred. A block is "
                         "still correctly marked done (raw/RECORDING_COMPLETE) "
                         "so resume/dedup works the same either way.")
    args = ap.parse_args()

    root, cfg = collect_config(args.config)
    plan = build_plan(root, cfg)
    print_plan(plan, cfg)

    if args.plan_only:
        return

    phases = sc.build_points(cfg["ph_start"], cfg["ph_inc"], cfg["ph_n"])
    run_blocks = [p for p in plan if p["status"] == "RUN"]
    blocks = run_blocks[args.start_block:]
    base_i = args.start_block
    if not blocks:
        print(f"\n  --start-block {args.start_block} >= {len(run_blocks)} runnable "
              f"blocks — nothing to do."); return

    batch_mode = args.max_blocks is not None
    if batch_mode:
        n_this_batch = min(args.max_blocks, len(blocks))
        print(f"\n  batch mode: running blocks {base_i+1}-{base_i+n_this_batch} "
              f"of {len(run_blocks)} runnable, then stopping.")
    else:
        print(f"\n  {len(blocks)} runnable blocks queued (starting at {base_i+1}).")
        if input("  start the first block? (y/n): ").strip().lower() not in ("y", "yes"):
            print("  stopped before starting."); return

    node = sc.start_ros()
    done = []
    try:
        for j, p in enumerate(blocks):
            i = base_i + j
            if batch_mode and j >= args.max_blocks:
                print(f"\n  BATCH LIMIT ({args.max_blocks}) reached — stopping.")
                left = run_blocks[i:]
                print(f"  {len(left)} block(s) not run. Resume with "
                      f"--start-block {i} --max-blocks {args.max_blocks}")
                for lp in left:
                    print(f"    freq_ratio={lp['freq_ratio']:.3f} amp_ratio={lp['amp_ratio']:.2f}")
                break

            os.makedirs(p["outdir"], exist_ok=True)
            points = [{"label": f"PH_{int(round(ph/D2R)):04d}",
                       "frequency": p["pitch_freq"], "pitch_amp": p["pitch_amp"],
                       "heave_amp": p["heave_amp"], "phase": ph,
                       "freq_ratio": p["freq_ratio"], "pitch_k": cfg["k"]}
                      for ph in phases]

            print(f"\n{'='*70}")
            print(f"BLOCK {i+1}/{len(run_blocks)}:  freq_ratio={p['freq_ratio']:.3f}  "
                  f"amp_ratio={p['amp_ratio']:.2f}  (pf={p['pitch_freq']:.4f}Hz "
                  f"hf={p['heave_freq']:.4f}Hz  p_amp={p['pitch_amp']:.4f} "
                  f"h_amp={p['heave_amp']:.4f})  — {len(points)} missions")
            print("=" * 70)
            sc.run_missions(node, p["outdir"], points, cfg["cycles"], cfg["delay"],
                            split=not args.no_split)
            sc.write_info(p["outdir"],
                          f"freq_ratio x amp_ratio x phase sweep block "
                          f"(center_freq={cfg['center_freq']}, "
                          f"center_amp={cfg['center_amp']}), pitch_k={cfg['k']:g}",
                          {"freq_ratio (heave/pitch)": p["freq_ratio"],
                           "amp_ratio (pitch/heave)": p["amp_ratio"],
                           "pitch frequency (Hz)": p["pitch_freq"],
                           "heave frequency (Hz)": p["heave_freq"],
                           "pitch_amp (rad)": p["pitch_amp"],
                           "heave_amp (rad)": p["heave_amp"],
                           "pitch_k": cfg["k"]},
                          f"phase {phases[0]:.4f}..{phases[-1]:.4f} rad, "
                          f"step {cfg['ph_inc']}", points, cfg["cycles"])
            done.append(p)

            print(f"\n  block {i+1}/{len(run_blocks)} done -> {p['outdir']}")
            remaining_total = len(run_blocks) - i - 1
            if remaining_total == 0:
                print("\n  ALL BLOCKS COMPLETE.")
                break
            if batch_mode:
                continue
            np_ = run_blocks[i + 1]
            print(f"  next block ({remaining_total} left): "
                  f"freq_ratio={np_['freq_ratio']:.3f} amp_ratio={np_['amp_ratio']:.2f}")
            ans = input("  proceed to next block? (y/n): ").strip().lower()
            if ans not in ("y", "yes"):
                print(f"\n  STOPPED by request after block {i+1}/{len(run_blocks)}.")
                left = run_blocks[i + 1:]
                print(f"  {len(left)} block(s) not run. Resume with --start-block {i+1}")
                for lp in left:
                    print(f"    freq_ratio={lp['freq_ratio']:.3f} amp_ratio={lp['amp_ratio']:.2f}")
                break
    finally:
        sc.stop_ros(node)

    print(f"\n=== summary: {len(done)}/{len(blocks)} blocks completed -> {root}")


if __name__ == "__main__":
    main()
