#!/usr/bin/env python3
"""
find_optimal_curve.py — full optimal-sine-curve experiment, lift or drag
========================================================================
Chains every stage into one run and carries each stage's winner into the next:

  01_phase       phase sweep      → propulsion_identifier picks the best phase
                                    FOR THE REQUESTED STYLE (lift or drag)
  02_pitch_amp   pitch amp sweep  @ that phase        → best pitch amplitude
  03_heave_amp   heave amp sweep  @ that phase + best pitch amp
                                                      → best heave amplitude
  04_frequency   frequency sweep  @ phase + both amps → [1] max f
                                                        [2] efficiency-optimal f
                                                        [3] max f keeping ≥{band:.0%} eff
  05_optimal_curve  one confirmation mission at the chosen curve

    python3 find_optimal_curve.py                 # prompts for everything
    python3 find_optimal_curve.py --style lift    # skip the style prompt

Lift vs drag is NOT a different experiment — it is the same sweep chain.  The
only difference is which phase is selected off the identifier's table, so a
single script covers both.

Everything lands in a NEW folder you name, created in this repo's root.
The launch must already be up and OPERATIONAL, with the load cell streaming.
"""

import os, sys, math, json, argparse, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "scripts"))

# The stage scripts are imported, not shelled out to: they are interactive, so
# subprocess would mean piping synthetic stdin and then scraping each stage's
# winner back out of stdout to feed the next.  Importing returns real values.
import sweep_common as sc
import analysis_common as ac
import find_amp
import find_freq
import propulsion_identifier as pid

D2R = math.pi / 180.0
SLEW_LIMIT = 5.5      # rad/s — measured servo slew ceiling
__doc__ = __doc__.format(band=ac.EFF_BAND)


# ===========================================================================
# Folder
# ===========================================================================
def ask_root_folder():
    """A NEW folder, created in the repo root, holding the whole experiment."""
    while True:
        name = input("  experiment folder name (created in repo root): ").strip()
        if not name:
            print("    -> enter a name"); continue
        if os.path.sep in name or name in (".", ".."):
            print("    -> a plain folder name, not a path"); continue
        p = os.path.join(HERE, name)
        if os.path.exists(p) and os.listdir(p):
            if input(f"    '{p}' exists and is not empty — use anyway? (y/n): "
                     ).strip().lower() not in ("y", "yes"):
                continue
        os.makedirs(p, exist_ok=True)
        return p


# ===========================================================================
# Stage 1 selection — the only place lift and drag differ
# ===========================================================================
def select_phase(results, style):
    """
    Best phase among those the identifier classified as the REQUESTED style.

    Ranked by thrust impulse per cycle, but only among samples that pass the
    force-quality, tracking and jaggedness gates — a phase whose forces violate
    the invariants is not a candidate no matter how much thrust it shows.
    """
    matching = [r for r in results if r.get("style") == style]
    if not matching:
        found = {}
        for r in results:
            found[r.get("style", "unknown")] = found.get(r.get("style", "unknown"), 0) + 1
        print(f"\n  !! No phase produced {style}-based propulsion.")
        print(f"     Styles found across the sweep: "
              + ", ".join(f"{k}×{v}" for k, v in sorted(found.items())))
        if not any(r.get("has_force") for r in results):
            print("     Every mission is missing load-cell data — check LabVIEW is "
                  "streaming UDP to 192.168.137.1:5005.")
        else:
            print(f"     Widen the phase sweep, or run --style "
                  f"{'drag' if style == 'lift' else 'lift'} instead.")
        return None

    usable = [r for r in matching
              if ac.quality_ok(r)[0] and ac.tracking_ok(r)[0] and ac.force_smooth_ok(r)[0]]
    if not usable:
        print(f"\n  !! {len(matching)} phase(s) were {style}-based, but none passed the "
              f"quality/tracking/jaggedness gates:")
        for r in matching:
            why = (ac.quality_ok(r)[1] or ac.tracking_ok(r)[1]
                   or ac.force_smooth_ok(r)[1])
            print(f"       {r['phase']/D2R:6.1f}° — {why}")
        return None

    best = max(usable, key=lambda r: r.get("impulse_per_cycle", float("-inf")))
    print(f"\n  >> SELECTED PHASE for {style}-based: {best['phase']:.4f} rad "
          f"({best['phase']/D2R:.2f}°)")
    print(f"     impulse {best.get('impulse_per_cycle',float('nan')):.4f} N·s/cycle, "
          f"confidence {best.get('confidence',0):.2f}, {best.get('reason','')}")
    if len(usable) > 1:
        others = ", ".join(f"{r['phase']/D2R:.0f}°" for r in usable if r is not best)
        print(f"     (other usable {style} phases: {others})")
    return best


# ===========================================================================
# Stages
# ===========================================================================
def stage_phase(node, root, cfg):
    outdir = os.path.join(root, "01_phase")
    os.makedirs(outdir, exist_ok=True)
    phases = sc.build_points(cfg["ph_start"], cfg["ph_inc"], cfg["ph_n"])
    points = [{"label": f"PH_{int(round(p/D2R)):04d}", "frequency": cfg["frequency"],
               "pitch_amp": cfg["pitch_amp"], "heave_amp": cfg["heave_amp"],
               "phase": p} for p in phases]
    print(f"\n{'='*70}\nSTAGE 1/4 — phase sweep ({len(points)} missions)\n{'='*70}")
    sc.run_missions(node, outdir, points, cfg["cycles"], cfg["delay"])
    sc.write_info(outdir, "phase shift",
                  {"pitch_amp (rad)": cfg["pitch_amp"], "heave_amp (rad)": cfg["heave_amp"],
                   "frequency (Hz)": cfg["frequency"]},
                  f"phase {phases[0]:.4f}..{phases[-1]:.4f} rad, step {cfg['ph_inc']}",
                  points, cfg["cycles"])
    results = pid.analyze_folder(outdir)
    pid.report(results)
    return results, select_phase(results, cfg["style"])


def stage_amp(node, root, cfg, axis, phase, held_pitch, held_heave, stage_no):
    outdir = os.path.join(root, f"0{stage_no}_{axis}_amp")
    os.makedirs(outdir, exist_ok=True)
    amps = sc.build_points(cfg[f"{axis}_start"], cfg[f"{axis}_inc"], cfg[f"{axis}_n"])
    tag = "PA" if axis == "pitch" else "HA"
    points = []
    for a in amps:
        points.append({"label": f"{tag}_{int(round(a/D2R)):04d}",
                       "frequency": cfg["frequency"], "phase": phase,
                       "pitch_amp": a if axis == "pitch" else held_pitch,
                       "heave_amp": a if axis == "heave" else held_heave})
    print(f"\n{'='*70}\nSTAGE {stage_no}/4 — {axis} amplitude sweep "
          f"({len(points)} missions) @ phase {phase/D2R:.1f}°\n{'='*70}")
    peak = 2 * math.pi * cfg["frequency"] * amps[-1]
    if peak > SLEW_LIMIT:
        print(f"  ! largest sample needs {peak:.2f} rad/s (> {SLEW_LIMIT} slew limit) "
              f"— it will clip and be excluded by the tracking gate.")
    sc.run_missions(node, outdir, points, cfg["cycles"], cfg["delay"])
    held = {"frequency (Hz)": cfg["frequency"], "phase (rad)": phase}
    held["heave_amp (rad)" if axis == "pitch" else "pitch_amp (rad)"] = (
        held_heave if axis == "pitch" else held_pitch)
    sc.write_info(outdir, f"{axis} amplitude (max thrust impulse)", held,
                  f"{axis}_amp {amps[0]:.4f}..{amps[-1]:.4f} rad, step {cfg[f'{axis}_inc']}",
                  points, cfg["cycles"])
    results = find_amp.analyze_folder(outdir)
    results.sort(key=lambda r: (r["pitch_amp"] if axis == "pitch" else r["heave_amp"]) or 0)
    best = find_amp.report_amp(results, axis)
    return results, best


def stage_freq(node, root, cfg, phase, pitch_amp, heave_amp):
    outdir = os.path.join(root, "04_frequency")
    os.makedirs(outdir, exist_ok=True)
    largest = max(pitch_amp, heave_amp)
    f_slew = SLEW_LIMIT / (2 * math.pi * largest) if largest > 1e-6 else 2.0
    if cfg.get("freq_fixed_step"):
        # Fixed increment (e.g. 0.25 Hz); the number of samples is derived here,
        # at runtime, to span from the start up to the slew-limited max implied
        # by the amplitudes just found — so "up to what the amps/slew allow".
        start, inc = cfg["freq_start"], cfg["freq_inc"]
        n = max(1, int((f_slew - start) / inc) + 1) if f_slew > start else 1
        print(f"\n  frequency: fixed {inc} Hz steps from {start} Hz up to the slew "
              f"limit (~{f_slew:.3f} Hz for the largest amp {largest:.4f} rad) "
              f"-> {n} samples")
    elif cfg["freq_auto"]:
        start = round(max(0.25, f_slew * 0.4), 3)
        inc = round(max(0.05, (f_slew - start) / max(1, cfg["freq_n"] - 1)), 3)
        n = cfg["freq_n"]
        print(f"\n  frequency range auto-derived from the slew limit and the "
              f"amplitudes found: largest amp {largest:.4f} rad -> servo follows to "
              f"~{f_slew:.3f} Hz")
    else:
        start, inc, n = cfg["freq_start"], cfg["freq_inc"], cfg["freq_n"]
    freqs = sc.build_points(start, inc, n)
    points = [{"label": f"FQ_{int(round(f*1000)):05d}", "frequency": f,
               "pitch_amp": pitch_amp, "heave_amp": heave_amp, "phase": phase}
              for f in freqs]
    print(f"\n{'='*70}\nSTAGE 4/4 — frequency sweep ({len(points)} missions) "
          f"{freqs[0]:.3f}..{freqs[-1]:.3f} Hz\n{'='*70}")
    sc.run_missions(node, outdir, points, cfg["cycles"], cfg["delay"])
    sc.write_info(outdir, "frequency (max + efficiency)",
                  {"pitch_amp (rad)": pitch_amp, "heave_amp (rad)": heave_amp,
                   "phase (rad)": phase},
                  f"frequency {freqs[0]:.4f}..{freqs[-1]:.4f} Hz, step {inc}",
                  points, cfg["cycles"])
    results = find_freq.analyze_folder(outdir)
    return (results,) + find_freq.report_freq(results)


def stage_confirm(node, root, cfg, phase, pitch_amp, heave_amp, frequency):
    outdir = os.path.join(root, "05_optimal_curve")
    os.makedirs(outdir, exist_ok=True)
    points = [{"label": "OPTIMAL", "frequency": frequency, "pitch_amp": pitch_amp,
               "heave_amp": heave_amp, "phase": phase}]
    print(f"\n{'='*70}\nCONFIRMATION RUN — the optimal curve\n{'='*70}")
    sc.run_missions(node, outdir, points, cfg["cycles"], cfg["delay"])
    sc.write_info(outdir, "optimal curve confirmation",
                  {"frequency (Hz)": frequency, "pitch_amp (rad)": pitch_amp,
                   "heave_amp (rad)": heave_amp, "phase (rad)": phase},
                  "single mission at the chosen curve", points, cfg["cycles"])
    res = find_amp.analyze_folder(outdir)
    return res[0] if res else None


# ===========================================================================
# Summary
# ===========================================================================
def write_summary(root, cfg, chosen, confirm):
    lines = [
        f"OPTIMAL SINE CURVE — {cfg['style']}-based propulsion",
        f"generated {datetime.datetime.now():%Y-%m-%d %H:%M:%S}",
        "=" * 66, "",
        "CHOSEN CURVE",
        f"  phase       {chosen['phase']:.6f} rad  ({chosen['phase']/D2R:.2f} deg)",
        f"  pitch_amp   {chosen['pitch_amp']:.6f} rad  ({chosen['pitch_amp']/D2R:.2f} deg)",
        f"  heave_amp   {chosen['heave_amp']:.6f} rad  ({chosen['heave_amp']/D2R:.2f} deg)",
        f"  frequency   {chosen['frequency']:.6f} Hz   ({chosen['freq_basis']})",
        f"  cycles      {cfg['cycles']}", "",
        "reproduce with:",
        f"  forward_paddle frequency:{chosen['frequency']:.4f} "
        f"pitch_amp:{chosen['pitch_amp']:.4f} heave_amp:{chosen['heave_amp']:.4f} "
        f"phase:{chosen['phase']:.4f} cycles:{cfg['cycles']} label:OPTIMAL", "",
        "ALL THREE FREQUENCIES REPORTED",
    ]
    for k, v in chosen["freqs"].items():
        lines.append(f"  {k:<44} {v}")
    lines += ["", "STAGE FOLDERS",
              "  01_phase          phase sweep + style identification",
              "  02_pitch_amp      pitch amplitude sweep",
              "  03_heave_amp      heave amplitude sweep",
              "  04_frequency      frequency sweep",
              "  05_optimal_curve  confirmation run at the chosen curve", ""]
    if confirm:
        lines += ["CONFIRMATION RUN MEASURED",
                  f"  style            {confirm.get('style','—')} "
                  f"(confidence {confirm.get('confidence',0):.2f})",
                  f"  impulse/cycle    {confirm.get('impulse_per_cycle',float('nan')):.4f} N·s",
                  f"  mean Fx          {confirm.get('mean_Fx',float('nan')):.4f} N",
                  f"  efficiency       {confirm.get('efficiency_energy',float('nan')):.4f} N·s/J",
                  f"  tracking         {confirm.get('min_track',float('nan')):.2f}",
                  f"  Fz asymmetry     {confirm.get('fz_asym',float('nan')):.3f}",
                  f"  net Fy           {confirm.get('fy_net',float('nan')):.4f} N", ""]
        if confirm.get("style") != cfg["style"]:
            lines += [f"  !! the confirmation run classified as "
                      f"'{confirm.get('style')}', NOT the requested "
                      f"'{cfg['style']}' — the amplitudes/frequency chosen after the "
                      f"phase stage changed the propulsion style.", ""]
    lines += ["view any stage:  python3 scripts/plotter.py <stage folder>",
              "re-analyze:      python3 scripts/propulsion_identifier.py --from 01_phase",
              "                 python3 scripts/find_amp.py --from 02_pitch_amp",
              "                 python3 scripts/find_freq.py --from 04_frequency", ""]
    txt = "\n".join(lines)
    with open(os.path.join(root, "RESULTS.txt"), "w") as f:
        f.write(txt)
    with open(os.path.join(root, "optimal_curve.json"), "w") as f:
        json.dump({k: v for k, v in chosen.items() if k != "freqs"}, f, indent=2)
    print("\n" + txt)
    print(f"  written -> {os.path.join(root, 'RESULTS.txt')}")
    write_sine_curve(root, cfg, chosen)


def write_sine_curve(root, cfg, chosen):
    """The final result the experiment exists to produce: the position sine
    curve for EACH servo, written to its own text file.  Matches the paddle
    gait exactly (motion_command.paddle): pitch is the phase reference, heave
    lags by the chosen phase; both about the calibrated zero (0 rad)."""
    f = chosen["frequency"]
    w = 2 * math.pi * f
    pa, ha, ph = chosen["pitch_amp"], chosen["heave_amp"], chosen["phase"]
    pc, hc = 0.0, 0.0        # centers = calibrated zero (launch pitch_zero/heave_zero)
    lines = [
        f"OPTIMAL SINE CURVE — {cfg['style']}-based propulsion",
        f"generated {datetime.datetime.now():%Y-%m-%d %H:%M:%S}",
        "=" * 66, "",
        "Position command for each servo, as a function of time t (seconds).",
        "pitch = servo 1 (phase reference), heave = servo 2 (phase-shifted).",
        "Centers are the calibrated zero (0 rad).", "",
        "SYMBOLIC",
        f"  pitch(t) = {pc:.4f} + {pa:.6f} * sin(2*pi*{f:.6f}*t)",
        f"  heave(t) = {hc:.4f} + {ha:.6f} * sin(2*pi*{f:.6f}*t + {ph:.6f})",
        "",
        "NUMERIC (omega folded in)",
        f"  pitch(t) = {pa:.6f} * sin({w:.6f} * t)",
        f"  heave(t) = {ha:.6f} * sin({w:.6f} * t + {ph:.6f})",
        "",
        "PARAMETERS",
        f"  frequency  f        = {f:.6f} Hz   (omega = {w:.6f} rad/s)",
        f"  pitch amp  A_pitch  = {pa:.6f} rad  ({pa/D2R:.2f} deg)",
        f"  heave amp  A_heave  = {ha:.6f} rad  ({ha/D2R:.2f} deg)",
        f"  phase (heave - pitch) = {ph:.6f} rad  ({ph/D2R:.2f} deg)",
        f"  frequency basis     = {chosen['freq_basis']}",
        "",
        "reproduce on the robot:",
        f"  forward_paddle frequency:{f:.4f} pitch_amp:{pa:.4f} "
        f"heave_amp:{ha:.4f} phase:{ph:.4f} cycles:{cfg['cycles']} label:OPTIMAL",
        "",
    ]
    out = os.path.join(root, "sine_curve.txt")
    with open(out, "w") as fh:
        fh.write("\n".join(lines))
    print(f"  sine curve -> {out}")


# ===========================================================================
# Main
# ===========================================================================
def config_run(path, style_arg):
    """Non-interactive: load all values from a JSON so the whole experiment runs
    unattended (given the values up front).  Keys mirror the prompts; any missing
    one uses the same default.  Required: 'folder' (created in repo root) and
    'style' (or --style)."""
    with open(os.path.expanduser(path)) as f:
        c = json.load(f)
    g = c.get
    name = c["folder"]
    # Allow a relative subpath (e.g. "in_house_wet_test/lift") so runs can be
    # organized under a parent folder; reject absolute paths or escaping '..'.
    if os.path.isabs(name) or ".." in name.replace("\\", "/").split("/"):
        sys.exit("  config 'folder' must be a relative path under the repo root")
    root = os.path.join(HERE, name)
    os.makedirs(root, exist_ok=True)
    cfg = {
        "style": style_arg or c.get("style"),
        "cycles": int(g("cycles", 10)), "delay": float(g("delay", sc.INTER_MISSION_DELAY)),
        "pitch_amp": float(g("pitch_amp", 0.235619449)),
        "heave_amp": float(g("heave_amp", 0.235619449)),
        "frequency": float(g("frequency", 0.75)),
        "ph_start": float(g("ph_start", 0.0)), "ph_inc": float(g("ph_inc", 0.174533)),
        "ph_n": int(g("ph_n", 19)),
        "pitch_start": float(g("pitch_start", 0.261799)),
        "pitch_inc": float(g("pitch_inc", 5 * D2R)), "pitch_n": int(g("pitch_n", 7)),
        "heave_start": float(g("heave_start", 0.785398)),
        "heave_inc": float(g("heave_inc", 5 * D2R)), "heave_n": int(g("heave_n", 7)),
        "freq_n": int(g("freq_n", 6)), "freq_auto": bool(g("freq_auto", True)),
        "freq_fixed_step": bool(g("freq_fixed_step", False)),
        "freq_start": float(g("freq_start", 0.5)), "freq_inc": float(g("freq_inc", 0.25)),
    }
    if cfg["style"] not in ("lift", "drag"):
        sys.exit("  config needs 'style': 'lift' or 'drag' (or pass --style)")
    n = cfg["ph_n"] + cfg["pitch_n"] + cfg["heave_n"] + cfg["freq_n"] + 1
    print(f"\n=== OPTIMAL SINE CURVE EXPERIMENT ({cfg['style']}), unattended ===")
    print(f"  {n} missions total; everything lands in: {root}")
    return root, cfg


def prompt(style_arg):
    print("\n=== OPTIMAL SINE CURVE EXPERIMENT ===")
    print("  (launch must already be up and OPERATIONAL, load cell streaming)\n")
    cfg = {}
    root = ask_root_folder()
    cfg["style"] = style_arg or sc.ask_choice("propulsion style to optimize for",
                                              ["lift", "drag"])
    print("\n  --- applies to every stage ---")
    cfg["cycles"] = sc.ask_int("cycles per mission command", 10)
    cfg["delay"] = sc.ask_delay()

    print("\n  --- stage 1: phase sweep (amps + frequency held) ---")
    cfg["pitch_amp"] = sc.ask_float("pitch amplitude (rad) [held]", 0.235619449)
    cfg["heave_amp"] = sc.ask_float("heave amplitude (rad) [held]", 0.235619449)
    cfg["frequency"] = sc.ask_float("frequency (Hz) [held through stages 1-3]", 0.75)
    cfg["ph_start"] = sc.ask_float("phase start (rad)", 0.0)
    cfg["ph_inc"] = sc.ask_float("phase increment (rad)", 0.174533)
    cfg["ph_n"] = sc.ask_int("phase samples", 19)

    print("\n  --- stage 2: pitch amplitude sweep ---")
    cfg["pitch_start"] = sc.ask_float("pitch amp start (rad)", 0.261799)
    cfg["pitch_inc"] = sc.ask_float("pitch amp increment (rad)", 5 * D2R)
    cfg["pitch_n"] = sc.ask_int("pitch amp samples", 7)

    print("\n  --- stage 3: heave amplitude sweep ---")
    cfg["heave_start"] = sc.ask_float("heave amp start (rad)", 0.785398)
    cfg["heave_inc"] = sc.ask_float("heave amp increment (rad)", 5 * D2R)
    cfg["heave_n"] = sc.ask_int("heave amp samples", 7)

    print("\n  --- stage 4: frequency sweep ---")
    cfg["freq_n"] = sc.ask_int("frequency samples", 6)
    cfg["freq_auto"] = sc.ask_choice(
        "derive the frequency range from the slew limit and the amplitudes found?",
        ["y", "n"]) == "y"
    if not cfg["freq_auto"]:
        cfg["freq_start"] = sc.ask_float("frequency start (Hz)", 0.5)
        cfg["freq_inc"] = sc.ask_float("frequency increment (Hz)", 0.25)

    n_missions = cfg["ph_n"] + cfg["pitch_n"] + cfg["heave_n"] + cfg["freq_n"] + 1
    est = n_missions * (cfg["cycles"] / cfg["frequency"] + cfg["delay"] + 5) / 60.0
    print(f"\n  {n_missions} missions total (4 sweeps + 1 confirmation)")
    print(f"  rough runtime estimate: ~{est:.0f} min")
    print(f"  everything lands in: {root}")
    if input("\n  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
        sys.exit(0)
    return root, cfg


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--style", choices=["lift", "drag"], default=None,
                    help="propulsion style to optimize for (else prompted)")
    ap.add_argument("--config", default=None,
                    help="JSON of all values -> run unattended (no prompts)")
    ap.add_argument("--no-plots", action="store_true",
                    help="skip the live plots at the end")
    args = ap.parse_args()

    root, cfg = (config_run(args.config, args.style) if args.config
                 else prompt(args.style))
    node = sc.start_ros()
    ph_res = pa_res = ha_res = fq_res = None
    try:
        # --- 1: phase → style selection ---
        ph_res, ph_best = stage_phase(node, root, cfg)
        if ph_best is None:
            sys.exit(f"\nStopped: no usable {cfg['style']}-based phase. "
                     f"Data kept in {root}")
        phase = ph_best["phase"]

        # --- 2: pitch amp @ that phase ---
        pa_res, pa_best = stage_amp(node, root, cfg, "pitch", phase,
                                    cfg["pitch_amp"], cfg["heave_amp"], 2)
        if pa_best is None:
            sys.exit(f"\nStopped: no usable pitch amplitude. Data kept in {root}")
        pitch_amp = pa_best["pitch_amp"]

        # --- 3: heave amp @ that phase + the pitch amp just found ---
        ha_res, ha_best = stage_amp(node, root, cfg, "heave", phase,
                                    pitch_amp, cfg["heave_amp"], 3)
        if ha_best is None:
            sys.exit(f"\nStopped: no usable heave amplitude. Data kept in {root}")
        heave_amp = ha_best["heave_amp"]

        # --- 4: frequency @ phase + both amps ---
        fq_res, fmax, feff, fmax_eff = stage_freq(node, root, cfg, phase,
                                                  pitch_amp, heave_amp)
        if fmax is None:
            sys.exit(f"\nStopped: no usable frequency. Data kept in {root}")

        # The efficiency-aware frequency is the curve's default: it is the
        # fastest that does not sacrifice efficiency.  All three are reported.
        pick = fmax_eff or feff or fmax
        basis = ("max f keeping >=%.0f%% of best efficiency" % (100 * ac.EFF_BAND)
                 if pick is fmax_eff else
                 "efficiency-optimal" if pick is feff else "max f")
        chosen = {"style": cfg["style"], "phase": phase, "pitch_amp": pitch_amp,
                  "heave_amp": heave_amp, "frequency": pick["frequency"],
                  "freq_basis": basis,
                  "freqs": {
                      "[1] max frequency (efficiency ignored)":
                          f"{fmax['frequency']:.4f} Hz" if fmax else "—",
                      "[2] efficiency-optimal frequency":
                          f"{feff['frequency']:.4f} Hz" if feff else "—",
                      f"[3] max frequency keeping >={ac.EFF_BAND:.0%} of best efficiency":
                          f"{fmax_eff['frequency']:.4f} Hz" if fmax_eff else "—",
                  }}

        # --- 5: confirmation ---
        confirm = stage_confirm(node, root, cfg, phase, pitch_amp, heave_amp,
                                pick["frequency"])
    finally:
        sc.stop_ros(node)

    write_summary(root, cfg, chosen, confirm)

    if not args.no_plots:
        print("\n  showing plots for every stage — close each window to advance")
        pid.plots(ph_res)
        find_amp.plots_amp(pa_res, "pitch", pa_best)
        find_amp.plots_amp(ha_res, "heave", ha_best)
        find_freq.plots_freq(fq_res, fmax, feff, fmax_eff)


if __name__ == "__main__":
    main()
