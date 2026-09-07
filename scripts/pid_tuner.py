#!/usr/bin/env python3
"""
pid_tuner.py — automated per-servo Position PID tuner (square-wave step test)
============================================================================
Tunes each servo's onboard Position PID gains by driving square-wave position
steps and measuring the encoder response, then adjusting gains and repeating —
LIVE, without relaunching.  Run this once to tune the actuators, then run
find_optimal_curve.py (or the sweeps) with the tuned gains.

    python3 scripts/pid_tuner.py

Requires the normal launch (crab_launch) up and calibrated:
  * the tuner engages the controller's silent 'passthrough' (manual_cmd) so the
    control loop drives nothing and the tuner owns joint_cmd with pure,
    un-smoothed steps;
  * it sets gains live via the servo_actuator node's 'position_gain_overrides'
    parameter (RAM write, torque stays on — no relaunch);
  * it archives the whole run with record_session.

SAFETY — the tuner is the ONLY position guard here (it bypasses the controller,
and Extended Position Mode has no hardware clamp):
  * every commanded goal is hard-clamped inside each servo's limit;
  * step targets are kept well inside the limit so PID overshoot has headroom;
  * a live monitor watches PRESENT position every feedback sample and, if any
    servo exceeds its absolute limit, immediately aborts: commands center, drops
    to safe low gains, releases passthrough.

Tuning procedure (per servo, independently; both servos stepped together):
  Ki is held at ~0 (default 0) to avoid integral windup during the holds.
  1. Kp: start low, increment until the rise time is aggressively fast OR a
     slight overshoot appears.
  2. Kd: then increment to damp the overshoot until the servo stops precisely
     on target (overshoot and steady-state error both within tolerance).
Three metrics are read from the encoder trace each trial: Rise Time, Peak
Overshoot, Steady-State Error.
"""

import os, sys, json, time, math, signal, threading, subprocess

import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

HERE = os.path.dirname(os.path.abspath(__file__))
RECORDER = os.path.join(HERE, "record_session.py")

D2R = math.pi / 180.0
SERVO_NODE = "/servo_actuator"

# Absolute position limits per servo (rad, centered on the calibrated zero),
# matching the launch's pitch_limit (π) / heave_limit (π/2).  These are hard
# safety bounds — commands are clamped inside them and present position is
# monitored against them.
DEFAULT_LIMITS = {1: math.pi, 2: math.pi / 2}

# X-series Position gains are integers 0..16383.
GAIN_MIN, GAIN_MAX = 0, 16383


# ===========================================================================
# Prompts
# ===========================================================================
def ask_float(label, default):
    s = input(f"  {label} [{default}]: ").strip()
    try:
        return float(s) if s else float(default)
    except ValueError:
        print("    -> number expected; using default"); return float(default)


def ask_int(label, default):
    s = input(f"  {label} [{default}]: ").strip()
    try:
        return int(s) if s else int(default)
    except ValueError:
        print("    -> integer expected; using default"); return int(default)


def ask_outdir():
    while True:
        s = input("  output path (folder created; all data lands here): ").strip()
        if not s:
            print("    -> enter a path"); continue
        p = os.path.abspath(os.path.expanduser(s))
        if os.path.exists(p) and os.listdir(p):
            if input(f"    '{p}' exists and is not empty — use anyway? (y/n): "
                     ).strip().lower() not in ("y", "yes"):
                continue
        os.makedirs(p, exist_ok=True)
        return p


# ===========================================================================
# ROS node
# ===========================================================================
class TunerNode(Node):
    def __init__(self, servo_ids, limits):
        super().__init__("pid_tuner")
        self.servo_ids = servo_ids
        self.limits = limits                    # {sid: abs_limit_rad}
        self.pos = {sid: None for sid in servo_ids}
        self._capturing = False
        self._trace = []                        # [(t, {sid: pos})]
        self.abort = False
        self.abort_reason = ""

        self.cmd_pub = self.create_publisher(Float32MultiArray, "joint_cmd", 1)
        self.manual_pub = self.create_publisher(String, "manual_cmd", 10)
        self.create_subscription(Float32MultiArray, "joint_feedback", self._fb_cb, 20)
        self.param_cli = self.create_client(SetParameters, f"{SERVO_NODE}/set_parameters")

    # ---- feedback + safety monitor ----
    def _fb_cb(self, msg):
        d = msg.data
        n = len(d) // 6
        now = time.time()
        for k in range(n):
            sid = int(round(d[6 * k]))
            if sid not in self.pos:
                continue
            p = float(d[6 * k + 2])             # position_rad
            self.pos[sid] = p
            lim = self.limits[sid]
            if abs(p) > lim and not self.abort:
                self.abort = True
                self.abort_reason = (f"servo {sid} present position {p:+.3f} rad "
                                     f"exceeded ±{lim:.3f} rad limit")
        if self._capturing:
            self._trace.append((now, {s: self.pos[s] for s in self.servo_ids}))

    # ---- command (hard-clamped) ----
    def command(self, targets):
        ids, modes, vals = [], [], []
        for sid in self.servo_ids:
            lim = self.limits[sid]
            v = max(-lim, min(lim, float(targets[sid])))
            ids.append(float(sid)); modes.append(3.0); vals.append(v)
        m = Float32MultiArray(); m.data = ids + modes + vals
        self.cmd_pub.publish(m)

    def manual(self, text):
        m = String(); m.data = text
        for _ in range(3):                      # a few sends so it isn't missed
            self.manual_pub.publish(m); time.sleep(0.1)

    # ---- capture window ----
    def start_capture(self):
        self._trace = []; self._capturing = True

    def stop_capture(self):
        self._capturing = False
        return list(self._trace)

    # ---- gains via parameter service ----
    def set_gains(self, gains):
        """gains: {sid: (p, i, d)} -> set position_gain_overrides (STRING param)."""
        override = {str(sid): {"p": int(p), "i": int(i), "d": int(d)}
                    for sid, (p, i, d) in gains.items()}
        req = SetParameters.Request()
        req.parameters = [Parameter(
            name="position_gain_overrides",
            value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                 string_value=json.dumps(override)))]
        if not self.param_cli.wait_for_service(timeout_sec=5.0):
            print("  !! servo_actuator SetParameters service not available");  return False
        fut = self.param_cli.call_async(req)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.02)
        ok = fut.done() and fut.result() is not None and \
            all(r.successful for r in fut.result().results)
        time.sleep(0.3)   # let the hardware loop pick up the dirty flag and write
        return ok


def start_ros(node):
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()


# ===========================================================================
# Step-response metrics
# ===========================================================================
def analyze_edges(trace, sid, toggle_times, targets, dwell, settle_frac=0.3):
    """
    From a capture and the known toggle schedule, compute mean Rise Time,
    Peak Overshoot (%), and Steady-State Error (rad) for one servo.

    Each edge is normalized so the step is positive, then:
      rise time   = t(90% of step) - t(10% of step)
      overshoot % = (peak beyond target) / |step| * 100
      sse         = mean(last `settle_frac` of the dwell) - target
    The first edge is skipped as warm-up.  Returns a dict, or None if no clean
    edge was measurable.
    """
    rt, ov, se, npts = [], [], [], []
    for e in range(1, len(toggle_times)):       # skip edge 0 (warm-up)
        t_start = toggle_times[e]
        t_end = t_start + dwell
        target = targets[e][sid]
        seg = [(t - t_start, p[sid]) for (t, p) in trace
               if t_start <= t < t_end and p[sid] is not None]
        if len(seg) < 5:
            continue
        start_pos = seg[0][1]
        step = target - start_pos
        if abs(step) < 1e-3:
            continue
        sign = 1.0 if step > 0 else -1.0
        # normalized progress 0..1 toward target
        def prog(p):
            return (p - start_pos) / step
        # rise time 10% -> 90%
        t10 = t90 = None
        for (tt, p) in seg:
            fr = prog(p)
            if t10 is None and fr >= 0.1:
                t10 = tt
            if fr >= 0.9:
                t90 = tt; break
        if t10 is not None and t90 is not None and t90 >= t10:
            rt.append(t90 - t10)
        # overshoot: furthest past target in the step direction
        extreme = max((sign * (p - target) for (_t, p) in seg), default=0.0)
        ov.append(max(0.0, extreme) / abs(step) * 100.0)
        # steady-state error: tail mean minus target
        tail = [p for (tt, p) in seg if tt >= dwell * (1.0 - settle_frac)]
        if tail:
            se.append(sum(tail) / len(tail) - target)
        npts.append(len(seg))
    if not ov:
        return None
    def _mean(x):
        return sum(x) / len(x) if x else float("nan")
    return {"rise_time": _mean(rt), "overshoot": _mean(ov),
            "sse": _mean(se), "n_edges": len(ov), "pts_per_edge": _mean(npts)}


# ===========================================================================
# Square-wave burst
# ===========================================================================
def run_burst(node, center, amp, dwell, n_edges):
    """Drive a square wave (both servos together) and capture the response.
    Returns (trace, toggle_times, targets_per_toggle)."""
    lo = {sid: center[sid] - amp[sid] for sid in node.servo_ids}
    hi = {sid: center[sid] + amp[sid] for sid in node.servo_ids}
    toggle_times, targets = [], []
    node.command(lo); time.sleep(dwell)         # settle at low before measuring
    node.start_capture()
    t0 = time.time()
    for e in range(n_edges):
        tgt = hi if e % 2 == 0 else lo
        node.command(tgt)
        toggle_times.append(time.time() - t0)
        targets.append(dict(tgt))
        # hold the dwell, but bail immediately on a safety abort
        t_hold = time.time()
        while time.time() - t_hold < dwell:
            if node.abort:
                break
            time.sleep(0.01)
        if node.abort:
            break
    # Feedback samples are stamped with absolute time.time(); toggle_times are
    # relative to t0.  Re-base the trace to t0 so both share one clock — else
    # analyze_edges' window test never matches and every edge reads empty.
    trace = [(t - t0, pos) for (t, pos) in node.stop_capture()]
    return trace, toggle_times, targets


# ===========================================================================
# Tuning
# ===========================================================================
def _snapshot_edge(trace, sid, toggle_times, targets, dwell, edge_index=1):
    """One clean rising edge as (t, pos, target) for the checkpoint plots."""
    if edge_index >= len(toggle_times):
        edge_index = len(toggle_times) - 1
    t_start = toggle_times[edge_index]
    t_end = t_start + dwell
    target = targets[edge_index][sid]
    seg = [(t - t_start, p[sid]) for (t, p) in trace
           if t_start <= t < t_end and p[sid] is not None]
    if len(seg) < 3:
        return None
    return {"t": [x[0] for x in seg], "pos": [x[1] for x in seg], "target": target}


def tune(node, cfg):
    """
    Per-servo state machine, tuned in the classic order Kp -> Kd -> Ki:
      P : raise Kp until the rise time is aggressively fast OR slight overshoot
      D : raise Kd until the overshoot is damped
      I : raise Ki until the steady-state error is within tolerance
    Both servos step together; each advances independently.

    NOTE ON Ki: Ki removes the steady-state droop that Kp/Kd cannot.  IN WATER,
    though, the drag load makes the integrator wind up during the stroke, so the
    in-water gains should end with Ki≈0 — set ki_max=0 to pin it there.  For a
    dry bench characterization we tune all three.

    Snapshots a clean step response at each checkpoint (before / after P / after
    D / after I) for the per-servo tuning plots.
    """
    ids = node.servo_ids
    center = {sid: cfg["center"] for sid in ids}
    amp = {sid: cfg["amp"] for sid in ids}

    st = {sid: {"phase": "P", "kp": cfg["kp_start"], "kd": 0, "ki": 0,
                "last": None, "snaps": {}} for sid in ids}
    history = []

    def gains_map():
        return {sid: (st[sid]["kp"], st[sid]["ki"], st[sid]["kd"]) for sid in ids}

    for trial in range(1, cfg["max_trials"] + 1):
        if node.abort:
            break
        if all(st[sid]["phase"] == "done" for sid in ids):
            print("\n  all servos tuned.")
            break

        if not node.set_gains(gains_map()):
            print("  !! failed to set gains; aborting"); node.abort = True; break
        print(f"\n  trial {trial}:  "
              + "  ".join(f"s{sid}[{st[sid]['phase']}] Kp={st[sid]['kp']} "
                         f"Ki={st[sid]['ki']} Kd={st[sid]['kd']}" for sid in ids))

        trace, tt, targets = run_burst(node, center, amp, cfg["dwell"], cfg["n_edges"])
        if node.abort:
            break

        row = {"trial": trial}
        for sid in ids:
            m = analyze_edges(trace, sid, tt, targets, cfg["dwell"])
            s = st[sid]
            snap = _snapshot_edge(trace, sid, tt, targets, cfg["dwell"])
            if "before" not in s["snaps"] and snap:      # trial 1 = untuned baseline
                s["snaps"]["before"] = {**snap, "gains": (s["kp"], s["ki"], s["kd"])}
            if m is None:
                print(f"    s{sid}: no clean edge measured — holding gains")
                row[f"s{sid}"] = {"kp": s["kp"], "ki": s["ki"], "kd": s["kd"],
                                  "metrics": None}
                continue
            s["last"] = m
            rise_txt = (f"{m['rise_time']*1000:6.1f} ms" if m["rise_time"] == m["rise_time"]
                        else "  >dwell")
            print(f"    s{sid}: rise={rise_txt}  overshoot={m['overshoot']:5.1f}%  "
                  f"sse={m['sse']/D2R:+.2f}°")
            row[f"s{sid}"] = {"kp": s["kp"], "ki": s["ki"], "kd": s["kd"],
                              "phase": s["phase"], "metrics": m}

            def _mark(tag):
                if snap:
                    s["snaps"][tag] = {**snap, "gains": (s["kp"], s["ki"], s["kd"])}

            # ---- advance this servo's state machine ----
            if s["phase"] == "P":
                fast_enough = m["rise_time"] <= cfg["rise_target"]
                overshooting = m["overshoot"] >= cfg["overshoot_trigger"]
                if fast_enough or overshooting or s["kp"] >= cfg["kp_max"]:
                    s["phase"] = "D"; _mark("after_p")
                    why = ("rise time target met" if fast_enough else
                           "slight overshoot appeared" if overshooting else
                           "Kp ceiling reached")
                    print(f"       -> Kp set ({why}); now damping with Kd")
                else:
                    s["kp"] += cfg["kp_step"]
            elif s["phase"] == "D":
                # Kd damps OVERSHOOT (its job); it does NOT fix steady-state
                # error — that is the I phase's job — so don't chase sse here.
                damped = m["overshoot"] <= cfg["overshoot_tol"]
                if damped or s["kd"] >= cfg["kd_max"]:
                    s["phase"] = "I"; _mark("after_d")
                    why = ("overshoot within tolerance" if damped
                           else "Kd ceiling reached")
                    if cfg["ki_max"] <= 0:                # Ki pinned off -> done
                        s["phase"] = "done"; _mark("after_i")
                        print(f"       -> DONE ({why}; Ki pinned 0): "
                              f"Kp={s['kp']} Ki=0 Kd={s['kd']}")
                        if abs(m["sse"]) > cfg["sse_tol"]:
                            print(f"          (residual sse {m['sse']/D2R:+.2f}° — "
                                  f"proportional droop; Ki=0 by request)")
                    else:
                        s["ki"] = cfg["ki_start"]
                        print(f"       -> Kd set ({why}); now removing sse with Ki")
                else:
                    s["kd"] += cfg["kd_step"]
            elif s["phase"] == "I":
                # Ki removes steady-state error; watch overshoot doesn't creep back.
                on_target = abs(m["sse"]) <= cfg["sse_tol"]
                broke_overshoot = m["overshoot"] > cfg["overshoot_tol"] * 2.0
                if on_target or s["ki"] >= cfg["ki_max"] or broke_overshoot:
                    s["phase"] = "done"; _mark("after_i")
                    why = ("sse within tolerance" if on_target else
                           "overshoot returned — backing off Ki" if broke_overshoot
                           else "Ki ceiling reached")
                    if broke_overshoot:
                        s["ki"] = max(0, s["ki"] - cfg["ki_step"])
                    print(f"       -> DONE ({why}): Kp={s['kp']} Ki={s['ki']} Kd={s['kd']}")
                else:
                    s["ki"] += cfg["ki_step"]
        history.append(row)

    return st, history


# ===========================================================================
# Main
# ===========================================================================
def _load_config(path):
    """Read a JSON config so the tuner runs unattended (all values up front).
    Recognized keys (all optional; missing ones use the defaults shown):
      outdir, servo_ids[list], limits{sid:rad}, center, amp, dwell, n_edges,
      rise_target_ms, overshoot_trigger, overshoot_tol, sse_tol_deg,
      kp_start, kp_step, kp_max, kd_step, kd_max,
      ki_start, ki_step, ki_max, max_trials
    Set ki_max:0 to pin Ki=0 (in-water/drag)."""
    with open(os.path.expanduser(path)) as f:
        return json.load(f)


def _collect(preset):
    """Return (outdir, servo_ids, limits, cfg). Interactive if preset is None,
    else fully from the preset dict (no prompts)."""
    if preset is None:
        outdir = ask_outdir()
        print("\n  --- servos ---")
        raw = input("  servo ids to tune (comma-sep) [1,2]: ").strip() or "1,2"
        servo_ids = [int(x) for x in raw.split(",")]
        limits = {sid: ask_float(f"servo {sid} absolute position limit ±rad",
                                 DEFAULT_LIMITS.get(sid, math.pi)) for sid in servo_ids}
        print("\n  --- step waveform (kept well inside the limits) ---")
        center = ask_float("step center (rad)", 0.0)
        amp = ask_float("step half-amplitude (rad)", 0.3)
        dwell = ask_float("dwell per step (s, must exceed settle time)", 1.0)
        n_edges = ask_int("edges per trial (>=4; first is warm-up)", 6)
        print("\n  --- targets / thresholds ---")
        rise_ms = ask_float("rise-time target (ms) — 'aggressively fast' below this", 200.0)
        ov_trig = ask_float("Kp-phase overshoot trigger (%)", 5.0)
        ov_tol = ask_float("Kd-phase overshoot tolerance (%)", 2.0)
        sse_deg = ask_float("steady-state error tolerance (deg)", 0.5)
        print("\n  --- gains (X-series 0..16383) — tuning Kp -> Kd -> Ki ---")
        kp_start = ask_int("Kp start (low/safe)", 200)
        kp_step = ask_int("Kp increment", 100)
        kp_max = ask_int("Kp ceiling (safety)", 3000)
        kd_step = ask_int("Kd increment", 150)
        kd_max = ask_int("Kd ceiling (safety)", 3000)
        ki_start = ask_int("Ki start", 50)
        ki_step = ask_int("Ki increment", 50)
        ki_max = ask_int("Ki ceiling (0 = pin Ki off, for in-water/drag)", 2000)
        max_trials = ask_int("max trials", 60)
    else:
        g = preset.get
        outdir = os.path.abspath(os.path.expanduser(preset["outdir"]))
        os.makedirs(outdir, exist_ok=True)
        servo_ids = [int(x) for x in g("servo_ids", [1, 2])]
        limits = {int(k): float(v) for k, v in g("limits", {}).items()} or \
                 {sid: DEFAULT_LIMITS.get(sid, math.pi) for sid in servo_ids}
        for sid in servo_ids:
            limits.setdefault(sid, DEFAULT_LIMITS.get(sid, math.pi))
        center = float(g("center", 0.0)); amp = float(g("amp", 0.3))
        dwell = float(g("dwell", 1.0)); n_edges = int(g("n_edges", 6))
        rise_ms = float(g("rise_target_ms", 200.0))
        ov_trig = float(g("overshoot_trigger", 5.0))
        ov_tol = float(g("overshoot_tol", 2.0))
        sse_deg = float(g("sse_tol_deg", 0.5))
        kp_start = int(g("kp_start", 200)); kp_step = int(g("kp_step", 100))
        kp_max = int(g("kp_max", 3000))
        kd_step = int(g("kd_step", 150)); kd_max = int(g("kd_max", 3000))
        ki_start = int(g("ki_start", 50)); ki_step = int(g("ki_step", 50))
        ki_max = int(g("ki_max", 2000)); max_trials = int(g("max_trials", 60))

    # safety: waveform must leave headroom for overshoot inside every limit
    for sid in servo_ids:
        reach = abs(center) + abs(amp)
        if reach >= limits[sid]:
            sys.exit(f"  !! center±amp ({reach:.3f}) reaches servo {sid} limit "
                     f"({limits[sid]:.3f}) — reduce center/amp; overshoot needs headroom.")
        if reach > 0.8 * limits[sid]:
            print(f"  ! note: servo {sid} steps to {reach:.3f} rad "
                  f"({reach/limits[sid]:.0%} of limit) — little headroom for overshoot.")

    cfg = {"center": center, "amp": amp, "dwell": dwell, "n_edges": n_edges,
           "rise_target": rise_ms / 1000.0, "overshoot_trigger": ov_trig,
           "overshoot_tol": ov_tol, "sse_tol": sse_deg * D2R,
           "kp_start": kp_start, "kp_step": kp_step, "kp_max": kp_max,
           "kd_step": kd_step, "kd_max": kd_max,
           "ki_start": ki_start, "ki_step": ki_step, "ki_max": ki_max,
           "max_trials": max_trials}
    return outdir, servo_ids, limits, cfg


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None,
                    help="JSON of all values -> run unattended (no prompts)")
    ap.add_argument("--yes", action="store_true", help="skip the final confirmation")
    ap.add_argument("--no-plots", action="store_true")
    args = ap.parse_args()

    print("\n=== AUTOMATED PID TUNER (square-wave step test, Kp -> Kd -> Ki) ===")
    print("  (crab_launch must be up and CALIBRATED; load cell not needed)\n")

    preset = _load_config(args.config) if args.config else None
    outdir, servo_ids, limits, cfg = _collect(preset)

    print(f"\n  everything lands in: {outdir}")
    if not args.yes and preset is None:
        if input("  proceed? (y/n): ").strip().lower() not in ("y", "yes"):
            return

    # ---- bring up ROS ----
    rclpy.init()
    node = TunerNode(servo_ids, limits)
    start_ros(node)
    time.sleep(1.0)

    t0 = time.time()
    while any(node.pos[s] is None for s in servo_ids) and time.time() - t0 < 10.0:
        time.sleep(0.1)
    if any(node.pos[s] is None for s in servo_ids):
        print("  !! no joint_feedback for some servos — is the launch up and calibrated?")
        node.destroy_node(); rclpy.shutdown(); return
    print("  feedback OK, present positions: "
          + ", ".join(f"s{s}={node.pos[s]:+.3f}" for s in servo_ids))

    safe = {s: (cfg["kp_start"], 0, 0) for s in servo_ids}   # Ki=0, Kd=0 = safe
    rec = None
    try:
        print("  engaging controller passthrough...")
        node.manual("passthrough")
        time.sleep(0.5)
        node.set_gains(safe)
        node.command({s: cfg["center"] for s in servo_ids})
        time.sleep(max(1.0, cfg["dwell"]))
        if node.abort:
            raise RuntimeError(node.abort_reason)

        raw_dir = os.path.join(outdir, "raw"); os.makedirs(raw_dir, exist_ok=True)
        rec = subprocess.Popen([sys.executable, RECORDER, raw_dir],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)

        st, history = tune(node, cfg)

    except RuntimeError as e:
        print(f"\n  !! SAFETY ABORT: {e}")
        st, history = {}, []
    finally:
        try:
            node.command({s: cfg["center"] for s in servo_ids})
            node.set_gains(safe)
            time.sleep(0.5)
            node.manual("stop")
        except Exception:
            pass
        if rec is not None:
            rec.send_signal(signal.SIGINT)
            try:
                rec.wait(timeout=90)
            except subprocess.TimeoutExpired:
                rec.kill()

    if node.abort:
        print(f"\n  ABORTED: {node.abort_reason}")
        print("  servos returned to center, gains reset to safe start, passthrough released.")

    report(outdir, servo_ids, st, history, cfg)
    if not args.no_plots and st:
        plot_tuning(outdir, servo_ids, st)
    node.destroy_node(); rclpy.shutdown()


def report(outdir, servo_ids, st, history, cfg):
    # per-trial CSV
    import csv
    with open(os.path.join(outdir, "tuning_trials.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["trial", "servo", "phase", "Kp", "Kd", "Ki",
                    "rise_ms", "overshoot_pct", "sse_deg", "n_edges"])
        for row in history:
            for sid in servo_ids:
                cell = row.get(f"s{sid}")
                if not cell:
                    continue
                m = cell.get("metrics")
                w.writerow([row["trial"], sid, cell.get("phase", ""),
                            cell["kp"], cell["kd"], cell.get("ki", 0),
                            f"{m['rise_time']*1000:.1f}" if m else "",
                            f"{m['overshoot']:.1f}" if m else "",
                            f"{m['sse']/D2R:.3f}" if m else "",
                            m["n_edges"] if m else ""])

    # final gains + suggested launch override
    final = {}
    lines = ["PID TUNING RESULT", "=" * 60, ""]
    for sid in servo_ids:
        s = st.get(sid)
        if not s:
            lines.append(f"servo {sid}: not tuned"); continue
        final[str(sid)] = {"p": s["kp"], "i": s["ki"], "d": s["kd"]}
        m = s.get("last") or {}
        lines.append(f"servo {sid}:  Kp={s['kp']}  Ki={s['ki']}  Kd={s['kd']}   "
                     f"[{s['phase']}]")
        if m:
            lines.append(f"           rise {m.get('rise_time',float('nan'))*1000:.1f} ms, "
                         f"overshoot {m.get('overshoot',float('nan')):.1f}%, "
                         f"sse {m.get('sse',float('nan'))/D2R:+.2f}°")
    override_json = json.dumps(final)
    lines += ["", "make it the boot default — paste into crab_launch.py:",
              f"    'position_gain_overrides': '{override_json}',", "",
              "or set live on the running node:",
              f"    ros2 param set /servo_actuator position_gain_overrides '{override_json}'",
              ""]
    tuned_ki = any(st.get(sid, {}).get("ki", 0) > 0 for sid in servo_ids)
    if tuned_ki:
        lines += ["note: this is a DRY-BENCH tune with all three gains.  IN WATER the",
                  "      drag load makes the integrator wind up during each stroke, so",
                  "      the in-water gains should use Ki=0.  Re-tune with ki_max:0 (or",
                  "      just zero the 'i' fields above) before the water experiment.", ""]
    txt = "\n".join(lines)
    with open(os.path.join(outdir, "RESULT.txt"), "w") as f:
        f.write(txt + "\n")
    with open(os.path.join(outdir, "gains.json"), "w") as f:
        json.dump(final, f, indent=2)
    print("\n" + txt)
    print(f"  written -> {os.path.join(outdir, 'RESULT.txt')}")


def plot_tuning(outdir, servo_ids, st):
    """Per-servo step-response overlay at each checkpoint: before tuning, after
    Kp, after Kd, after Ki.  Live matplotlib windows (zoomable) + saved PNGs."""
    order = [("before", "before tuning"), ("after_p", "after Kp"),
             ("after_d", "after Kd"), ("after_i", "after Ki (final)")]
    for sid in servo_ids:
        snaps = st.get(sid, {}).get("snaps", {})
        if not snaps:
            continue
        fig, ax = plt.subplots(figsize=(10, 5.5))
        target = None
        for tag, label in order:
            snap = snaps.get(tag)
            if not snap:
                continue
            target = snap["target"]
            p, i, d = snap["gains"]
            ax.plot(snap["t"], snap["pos"], lw=1.5,
                    label=f"{label}  (Kp={p} Ki={i} Kd={d})")
        if target is not None:
            ax.axhline(target, color="k", ls="--", lw=0.8, label=f"target {target:+.3f} rad")
        ax.set_xlabel("time since step (s)"); ax.set_ylabel("position (rad)")
        ax.set_title(f"servo {sid} — step response through tuning "
                     f"(Kp → Kd → Ki)")
        ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
        fig.tight_layout()
        png = os.path.join(outdir, f"servo{sid}_tuning.png")
        fig.savefig(png, dpi=110)
        print(f"  plot -> {png}")
    print("  showing tuning plots — close the windows to finish")
    plt.show()


if __name__ == "__main__":
    main()
