#!/usr/bin/env python3
"""
run_experiment.py — Automated flapping-foil sine-curve optimizer
================================================================
Run in a SECOND terminal while `ros2 launch soft_propulsors_control
crab_launch.py` is already up (loadcell streaming, servos 1=pitch, 2=heave).

    python3 run_experiment.py <folder_name>

Walks the 4-stage procedure, one stage folder each:
    01_phase/  02_pitch_amp/  03_heave_amp/  04_frequency/
For every sweep it: starts record_session -> publishes the sweep missions
(one persistent publisher, 20 s between missions) -> stops the recorder ->
splits per-mission -> analyzes to pick the optimum -> runs the refinement
sweep the same way -> locks the optimum -> feeds it to the next stage.
Finally runs `optimal_sine_curve/` (10 cycles at all optimums) and writes
matplotlib plots + a README (numeric optimums + how each curve evolved).

Conventions (post-rename): pitch = servo 1 (feather), heave = servo 2 (plunge).
Forces: Fx = thrust (x), Fy = lateral (y), Fz = heave (z).  10 cycles/sweep,
first 3 ignored in analysis.
"""

import os, sys, time, math, csv, glob, signal, subprocess, threading, json
from collections import defaultdict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# ---------------------------------------------------------------------------
# CONFIG — sweep specs (per the procedure) and tunable scoring weights.
# ---------------------------------------------------------------------------
HZ_CONST      = 0.75                     # constant frequency for phase+amp stages
AMP15         = 0.235619449             # "15%" amplitude (= 0.15·π/2) in rad
CYCLES        = 10                       # per sweep; first 3 ignored in analysis
IGNORE_CYCLES = 3
INTER_MISSION_DELAY = 20.0               # s between mission commands
SLEW_LIMIT    = 5.5                      # rad/s — cap for the frequency stage
D2R           = math.pi / 180.0

# Scoring weights (logged per point so they can be retuned).
W_LATERAL   = 1.0    # penalty on mean|Fy|
W_FZ_NET    = 0.5    # penalty on |mean Fz| (heave-force asymmetry)
W_FX_TROUGH = 0.5    # penalty on Fx trough magnitude
W_FX_ASYM   = 0.5    # penalty on Fx peak-to-peak asymmetry
W_TORQUE    = 0.3    # penalty on torque(current) roughness (amp stages)

RECORDER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "record_session.py")
SPLITTER = os.path.join(os.path.dirname(os.path.abspath(__file__)), "split_missions.py")


def frange(lo, hi, step):
    """Inclusive float range (tolerant of fp error at the top end)."""
    out, x = [], lo
    while x <= hi + step * 1e-6:
        out.append(round(x, 6)); x += step
    return out


# ===========================================================================
# ROS node: one persistent publisher (no pub --once drops) + status listener
# ===========================================================================
class ExperimentNode(Node):
    def __init__(self):
        super().__init__("experiment_runner")
        self.pub = self.create_publisher(String, "mission_input", 10)
        self.create_subscription(String, "mission_status", self._status_cb, 50)
        self._done = threading.Event()
        self._awaiting = None

    def _status_cb(self, msg):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        if (d.get("event") == "ACHIEVED" and self._awaiting
                and d.get("label") == self._awaiting):
            self._done.set()

    def send_mission(self, line, label, timeout):
        """Publish one mission line, block until its ACHIEVED (or timeout)."""
        self._awaiting = label
        self._done.clear()
        msg = String(); msg.data = line
        # publish a few times over ~0.5s so a late-joining sub still gets it
        for _ in range(3):
            self.pub.publish(msg); time.sleep(0.15)
        ok = self._done.wait(timeout=timeout)
        self._awaiting = None
        if not ok:
            self.get_logger().warn(f"'{label}' did not report ACHIEVED within "
                                   f"{timeout:.0f}s — continuing.")
        return ok


# ===========================================================================
# Recorder / splitter subprocess management
# ===========================================================================
def start_recorder(folder):
    os.makedirs(folder, exist_ok=True)
    p = subprocess.Popen([sys.executable, RECORDER, folder],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)   # let it subscribe/start recording
    return p


def stop_recorder(p):
    p.send_signal(signal.SIGINT)
    try:
        p.wait(timeout=90)     # matches record_session's stretched export timeout
    except subprocess.TimeoutExpired:
        p.kill(); p.wait()
    time.sleep(1.0)


def split(folder):
    subprocess.run([sys.executable, SPLITTER, folder, "--base-dir", folder],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


# ===========================================================================
# Analysis — read a mission's force + servo data, window cycles 4..N.
# ===========================================================================
def _f(row, key):
    v = row.get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_mission(folder, label, frequency):
    """
    Return per-mission analysis dict for a split mission (folder/<label>_*/).
    Uses cycles IGNORE_CYCLES..CYCLES of the force + servo streams.
    """
    hits = glob.glob(os.path.join(folder, f"{label}_*"))
    if not hits:
        return None
    mdir = hits[0]
    fb_csv = glob.glob(os.path.join(mdir, f"{label}.csv"))
    lc_csv = glob.glob(os.path.join(mdir, f"{label}_loadcell.csv"))
    if not fb_csv:
        return None
    fb = list(csv.DictReader(open(fb_csv[0])))
    lc = list(csv.DictReader(open(lc_csv[0]))) if lc_csv else []

    period = 1.0 / frequency if frequency > 1e-6 else 0.0
    t_lo, t_hi = IGNORE_CYCLES * period, CYCLES * period   # analysis window (s)

    def win(rows):
        return [r for r in rows if (v := _f(r, "time_s")) is not None and t_lo <= v <= t_hi]

    fbw, lcw = win(fb), win(lc)

    out = {"label": label, "frequency": frequency, "n_force": len(lcw), "n_fb": len(fbw)}

    # --- forces ---
    fx = [_f(r, "Fx") for r in lcw if _f(r, "Fx") is not None]
    fy = [_f(r, "Fy") for r in lcw if _f(r, "Fy") is not None]
    fz = [_f(r, "Fz") for r in lcw if _f(r, "Fz") is not None]
    out["has_force"] = len(fx) > 10
    if out["has_force"]:
        out["mean_Fx"] = _mean(fx)
        out["peak_Fx"] = _mean(_peaks(fx))                 # mean positive peak
        out["trough_Fx"] = _mean(_peaks([-v for v in fx])) # mean trough magnitude
        out["fx_asym"] = abs(out["peak_Fx"] - out["trough_Fx"])
        out["mean_abs_Fy"] = _mean([abs(v) for v in fy]) if fy else float("nan")
        out["fz_net"] = abs(_mean(fz)) if fz else float("nan")
        out["fz_asym"] = abs(_mean(_peaks(fz)) - _mean(_peaks([-v for v in fz]))) if fz else float("nan")

    # --- servo tracking + current/torque ---
    for sid, name in ((1, "pitch"), (2, "heave")):
        pos = [_f(r, f"s{sid}_position_rad") for r in fbw if _f(r, f"s{sid}_position_rad") is not None]
        cur = [abs(_f(r, f"s{sid}_current_a")) for r in fbw if _f(r, f"s{sid}_current_a") is not None]
        if len(pos) > 2:
            out[f"{name}_achieved_amp"] = (max(pos) - min(pos)) / 2.0
        if cur:
            out[f"{name}_mean_current"] = _mean(cur)
            out[f"{name}_current_rough"] = _roughness(cur)
    out["mean_current"] = _mean([out.get("pitch_mean_current", 0.0),
                                 out.get("heave_mean_current", 0.0)])
    out["torque_rough"] = (out.get("pitch_current_rough", 0.0) +
                           out.get("heave_current_rough", 0.0))
    return out


def _mean(xs):
    xs = [x for x in xs if x is not None]
    return sum(xs) / len(xs) if xs else float("nan")


def _peaks(xs):
    """Local maxima magnitudes (simple 3-point), for peak/trough symmetry."""
    pk = [xs[i] for i in range(1, len(xs) - 1)
          if xs[i] >= xs[i - 1] and xs[i] >= xs[i + 1] and xs[i] > 0]
    return pk if pk else [max(xs)] if xs else [0.0]


def _roughness(xs):
    """Mean |2nd difference| — a proxy for how 'sharp'/jerky a curve is."""
    if len(xs) < 3:
        return 0.0
    d2 = [abs(xs[i + 1] - 2 * xs[i] + xs[i - 1]) for i in range(1, len(xs) - 1)]
    return _mean(d2)


# ---- per-stage scoring (higher = better); returns (score, breakdown) -------
def score_phase(m):
    if not m.get("has_force"):
        return (-1e9, {"reason": "no force data"})
    s = (m["mean_Fx"]
         - W_LATERAL * m["mean_abs_Fy"]
         - W_FZ_NET * m["fz_net"]
         - W_FX_TROUGH * m["trough_Fx"]
         - W_FX_ASYM * m["fz_asym"])
    return (s, {"mean_Fx": m["mean_Fx"], "mean_abs_Fy": m["mean_abs_Fy"],
                "fz_net": m["fz_net"], "trough_Fx": m["trough_Fx"],
                "fz_asym": m["fz_asym"]})


def score_amp(m):
    if not m.get("has_force"):
        return (-1e9, {"reason": "no force data"})
    s = (m["peak_Fx"]
         - W_LATERAL * m["mean_abs_Fy"]
         - W_FX_TROUGH * m["trough_Fx"]
         - W_TORQUE * m["torque_rough"])
    return (s, {"peak_Fx": m["peak_Fx"], "mean_abs_Fy": m["mean_abs_Fy"],
                "trough_Fx": m["trough_Fx"], "torque_rough": m["torque_rough"]})


def efficiency(m):
    if not m.get("has_force") or not m.get("mean_current"):
        return float("nan")
    c = m["mean_current"]
    return m["peak_Fx"] / c if c > 1e-6 else float("nan")


# ===========================================================================
# Sweep driver — record, run missions, split, analyze.
# ===========================================================================
def paddle_line(label, frequency, pitch_amp, heave_amp, phase):
    return (f"forward_paddle frequency:{frequency:.6f} pitch_amp:{pitch_amp:.6f} "
            f"heave_amp:{heave_amp:.6f} phase:{phase:.6f} cycles:{CYCLES} label:{label}")


def run_sweep(node, folder, points):
    """points: list of (label, frequency, pitch_amp, heave_amp, phase).
    Records, runs each mission (20s apart), splits.  Returns list of analysis dicts."""
    print(f"\n  >> sweep in {folder}  ({len(points)} points)")
    rec = start_recorder(folder)
    try:
        for i, (label, fq, pa, ha, ph) in enumerate(points):
            line = paddle_line(label, fq, pa, ha, ph)
            dur = CYCLES / fq if fq > 1e-6 else 15.0
            print(f"     [{i+1}/{len(points)}] {label}: f={fq:.3f} pitch={pa:.4f} "
                  f"heave={ha:.4f} phase={ph:.4f}")
            node.send_mission(line, label, timeout=dur * 1.6 + 8.0)
            if i < len(points) - 1:
                time.sleep(INTER_MISSION_DELAY)
        time.sleep(2.0)
    finally:
        stop_recorder(rec)
    split(folder)
    results = []
    for (label, fq, pa, ha, ph) in points:
        m = load_mission(folder, label, fq)
        if m:
            m.update({"pitch_amp": pa, "heave_amp": ha, "phase": ph})
            results.append(m)
    return results


def check_tracking(results):
    """Flag any point where the servo didn't follow the commanded amplitude."""
    warn = []
    for m in results:
        pa, ha = m.get("pitch_amp"), m.get("heave_amp")
        pach, hach = m.get("pitch_achieved_amp"), m.get("heave_achieved_amp")
        if pa and pach and pach < 0.9 * pa:
            warn.append(f"{m['label']}: pitch {100*pach/pa:.0f}% of commanded")
        if ha and hach and hach < 0.9 * ha:
            warn.append(f"{m['label']}: heave {100*hach/ha:.0f}% of commanded")
    return warn


# ===========================================================================
# main
# ===========================================================================
def main():
    if len(sys.argv) < 2:
        print("usage: python3 run_experiment.py <folder_name>"); sys.exit(1)
    root = sys.argv[1]
    os.makedirs(root, exist_ok=True)

    rclpy.init()
    node = ExperimentNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    time.sleep(1.0)
    print(f"=== experiment '{root}' — waiting 3s for stack (must be OPERATIONAL) ===")
    time.sleep(3.0)

    evolution = {}   # stage -> chosen optimum (for the evolution graph/README)
    all_warnings = []

    # --- import analysis-plot helpers lazily (matplotlib only needed at the end)
    from experiment_plots import (plot_sweep, plot_optimal_curve,
                                   plot_evolution, write_readme)

    # ----- STAGE 1: PHASE -----------------------------------------------
    print("\n===== STAGE 1: optimal PHASE =====")
    d = os.path.join(root, "01_phase")
    coarse_pts = [(f"PH_{int(round(ph/D2R)):03d}", HZ_CONST, AMP15, AMP15, ph)
                  for ph in frange(0.0, math.pi, 0.174533)]
    res_c = run_sweep(node, os.path.join(d, "coarse"), coarse_pts)
    best_c = _pick(res_c, score_phase)
    ph_opt = best_c["phase"]
    refine_pts = [(f"PHr_{int(round(ph/D2R)):03d}", HZ_CONST, AMP15, AMP15, ph)
                  for ph in frange(max(0.0, ph_opt - 10 * D2R), ph_opt + 10 * D2R, 2 * D2R)]
    res_r = run_sweep(node, os.path.join(d, "refine"), refine_pts)
    best = _pick(res_r + [best_c], score_phase)
    ph_opt = best["phase"]
    evolution["phase"] = {"value_rad": ph_opt, "value_deg": ph_opt / D2R, "best": best}
    plot_sweep(d, "phase", res_c, res_r, "phase", score_phase, "Phase (rad)")
    all_warnings += check_tracking(res_c + res_r)
    print(f"  -> optimal PHASE = {ph_opt:.4f} rad ({ph_opt/D2R:.1f}°)")

    # ----- STAGE 2: PITCH AMPLITUDE -------------------------------------
    print("\n===== STAGE 2: optimal PITCH amplitude =====")
    d = os.path.join(root, "02_pitch_amp")
    coarse_pts = [(f"PA_{int(round(a/D2R)):02d}", HZ_CONST, a, AMP15, ph_opt)
                  for a in frange(15 * D2R, math.pi / 4, 5 * D2R)]
    res_c = run_sweep(node, os.path.join(d, "coarse"), coarse_pts)
    best_c = _pick(res_c, score_amp)
    pa_opt = best_c["pitch_amp"]
    refine_pts = [(f"PAr_{int(round(a/D2R)):02d}", HZ_CONST, a, AMP15, ph_opt)
                  for a in frange(max(5 * D2R, pa_opt - 5 * D2R), pa_opt + 5 * D2R, 1 * D2R)]
    res_r = run_sweep(node, os.path.join(d, "refine"), refine_pts)
    best = _pick(res_r + [best_c], score_amp)
    pa_opt = best["pitch_amp"]
    evolution["pitch_amp"] = {"value_rad": pa_opt, "value_deg": pa_opt / D2R, "best": best}
    plot_sweep(d, "pitch_amp", res_c, res_r, "pitch_amp", score_amp, "Pitch amplitude (rad)")
    all_warnings += check_tracking(res_c + res_r)
    print(f"  -> optimal PITCH amp = {pa_opt:.4f} rad ({pa_opt/D2R:.1f}°)")

    # ----- STAGE 3: HEAVE AMPLITUDE -------------------------------------
    print("\n===== STAGE 3: optimal HEAVE amplitude =====")
    d = os.path.join(root, "03_heave_amp")
    coarse_pts = [(f"HA_{int(round(a/D2R)):02d}", HZ_CONST, pa_opt, a, ph_opt)
                  for a in frange(math.pi / 4, math.pi / 2, 5 * D2R)]
    res_c = run_sweep(node, os.path.join(d, "coarse"), coarse_pts)
    best_c = _pick(res_c, score_amp)
    ha_opt = best_c["heave_amp"]
    refine_pts = [(f"HAr_{int(round(a/D2R)):02d}", HZ_CONST, pa_opt, a, ph_opt)
                  for a in frange(max(math.pi / 4, ha_opt - 5 * D2R),
                                  min(math.pi / 2, ha_opt + 5 * D2R), 1 * D2R)]
    res_r = run_sweep(node, os.path.join(d, "refine"), refine_pts)
    best = _pick(res_r + [best_c], score_amp)
    ha_opt = best["heave_amp"]
    evolution["heave_amp"] = {"value_rad": ha_opt, "value_deg": ha_opt / D2R, "best": best}
    plot_sweep(d, "heave_amp", res_c, res_r, "heave_amp", score_amp, "Heave amplitude (rad)")
    all_warnings += check_tracking(res_c + res_r)
    print(f"  -> optimal HEAVE amp = {ha_opt:.4f} rad ({ha_opt/D2R:.1f}°)")

    # ----- STAGE 4: FREQUENCY (efficiency) ------------------------------
    print("\n===== STAGE 4: optimal FREQUENCY =====")
    d = os.path.join(root, "04_frequency")
    largest_amp = max(pa_opt, ha_opt)
    f_max = SLEW_LIMIT / (2 * math.pi * largest_amp) if largest_amp > 1e-6 else 2.0
    f_lo = max(0.3, f_max * 0.4)
    freqs = [round(f_lo + i * (f_max - f_lo) / 5.0, 4) for i in range(6)]  # 6 samples
    print(f"  slew-capped f_max={f_max:.3f} Hz (largest amp {largest_amp:.3f} rad); "
          f"sweeping {freqs}")
    pts = [(f"FQ_{int(round(fq*1000)):04d}", fq, pa_opt, ha_opt, ph_opt) for fq in freqs]
    res_f = run_sweep(node, os.path.join(d, "coarse"), pts)
    # efficiency, reject 'sharp'/resonant points (efficiency far above neighbors)
    fq_opt = _pick_frequency(res_f)
    evolution["frequency"] = {"value_hz": fq_opt}
    plot_sweep(d, "frequency", res_f, [], "frequency", lambda m: (efficiency(m), {}),
               "Frequency (Hz)", ykey="efficiency")
    all_warnings += check_tracking(res_f)
    print(f"  -> optimal FREQUENCY = {fq_opt:.3f} Hz")

    # ----- FINAL: optimal sine curve -----------------------------------
    print("\n===== FINAL: optimal_sine_curve (10 cycles) =====")
    d = os.path.join(root, "optimal_sine_curve")
    pts = [("OPTIMAL", fq_opt, pa_opt, ha_opt, ph_opt)]
    run_sweep(node, d, pts)
    optimal = {"frequency_hz": fq_opt, "pitch_amp_rad": pa_opt, "heave_amp_rad": ha_opt,
               "phase_rad": ph_opt}
    plot_optimal_curve(d, "OPTIMAL", fq_opt, optimal)
    plot_evolution(root, evolution)
    write_readme(root, optimal, evolution, all_warnings)

    print("\n=== DONE ===")
    print(f"  phase      = {ph_opt:.4f} rad ({ph_opt/D2R:.1f}°)")
    print(f"  pitch_amp  = {pa_opt:.4f} rad ({pa_opt/D2R:.1f}°)")
    print(f"  heave_amp  = {ha_opt:.4f} rad ({ha_opt/D2R:.1f}°)")
    print(f"  frequency  = {fq_opt:.3f} Hz")
    print(f"  results + plots + README in: {root}/")
    if all_warnings:
        print("\n  ⚠ tracking warnings (servo under-followed command):")
        for w in all_warnings:
            print("    -", w)

    node.destroy_node(); rclpy.shutdown()


def _pick(results, scorer):
    scored = [(scorer(m)[0], m) for m in results if m]
    if not scored:
        raise SystemExit("No analyzable results (servos moving? recording exported?).")
    best_s, best_m = max(scored, key=lambda x: x[0])
    if best_s <= -1e8:   # every point had no force data
        raise SystemExit(
            "ABORT: no load-cell force data in any mission — the loadcell is not "
            "streaming.  Confirm LabVIEW is sending UDP to 192.168.137.1:5005, the "
            "firewall allows it, and `ros2 topic echo /load_cell_data` shows data.")
    # log the winner's score breakdown for transparency / retuning
    bd = scorer(best_m)[1]
    print(f"     best: {best_m['label']}  score={best_s:.4f}  " +
          "  ".join(f"{k}={v:.4f}" for k, v in bd.items() if isinstance(v, (int, float))))
    return best_m


def _pick_frequency(results):
    """Max efficiency, discarding 'sharp' outliers (resonant spikes) vs neighbours."""
    good = [m for m in results if m and not math.isnan(efficiency(m))]
    if not good:
        raise SystemExit("No frequency results with force+current data.")
    good.sort(key=lambda m: m["frequency"])
    effs = [efficiency(m) for m in good]
    # flag a point whose efficiency is a sharp spike (>1.5x both neighbour means)
    keep = []
    for i, m in enumerate(good):
        lo = effs[i - 1] if i > 0 else effs[i]
        hi = effs[i + 1] if i < len(good) - 1 else effs[i]
        if effs[i] > 1.5 * max(lo, hi) and 0 < i < len(good) - 1:
            print(f"     discarding {m['label']} (possible resonance spike, "
                  f"eff={effs[i]:.3f} vs neighbours {lo:.3f}/{hi:.3f})")
            continue
        keep.append(m)
    return max(keep, key=efficiency)["frequency"]


if __name__ == "__main__":
    main()
