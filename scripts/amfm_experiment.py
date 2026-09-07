#!/usr/bin/env python3
"""AM/FM shaping experiment: design + run.

GOAL
----
Measure, for each kinematic knob, (a) how much it moves each force-waveform
metric on Fx and Fy, and (b) how much it disturbs everything else -- so the
answer is a JACOBIAN and a coupling map, not a claim that all twenty metrics
are independently controllable.

STAGES
------
  J  JACOBIAN.   Every knob perturbed +/- around a nominal, one at a time,
                 on each servo. Gives d(metric)/d(knob) by central difference
                 and, from the rank of that matrix, how many metrics are
                 ACTUALLY independent. Central differences rather than
                 one-sided so a nonlinear-but-symmetric response is not
                 mistaken for zero sensitivity.

  S  SWEEP.      The knobs that survive stage J get 5 levels each, so the
                 response curve is characterised rather than assumed linear.
                 A knob whose effect reverses (as a2 did in the harmonic
                 campaign, peaking near 0.3 then falling back) cannot be used
                 by a controller unless that turning point is known.

  R  REPLICATES. The nominal, repeated throughout the run in randomised
                 order. Every claim of "knob X moved metric Y" is judged
                 against this spread, and drift over the session shows up as
                 a trend in it.

EXECUTION MODEL
---------------
One gait cycle is PRECOMPUTED into a position array and replayed at the
hardware rate. The gait is static within a run, so nothing needs to be
recalculated per tick, the cycle repeats exactly (no phase drift), and every
position and velocity in the commanded array is checked against the limits
BEFORE anything moves. Extended-position mode: the commanded angle is its
own bound, unlike velocity mode where an unbounded integration is possible.

usage:
  amfm_experiment.py <folder> --design            # build + verify the plan
  amfm_experiment.py <folder> --run [--stage J]   # execute on the rig
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from dataclasses import replace

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))

from amfm_waveform import (   # noqa: E402
    Knobs, KNOB_NAMES, cycle, check, max_feasible_period)

# ---- limits, mirroring crab_launch.py -----------------------------------
PITCH_LIMIT = math.pi          # servo 1, +/-180 deg
HEAVE_LIMIT = math.pi / 2      # servo 2, +/-90 deg
SLEW_LIMIT = 5.5               # rad/s
SERVO_LIMITS = {1: PITCH_LIMIT, 2: HEAVE_LIMIT}
# joint_cmd's per-servo "mode" field is a COMMAND TYPE, not the Dynamixel
# operating mode: 3 = position write, 1 = velocity write (see controller.py's
# topic contract). The interface's write phase handles only those two values
# and silently queues NOTHING for anything else -- a 4 here was accepted with
# no error and no write, so the servos held still while the script published
# happily at 100 Hz. Extended-position is selected at the register level by
# operating_mode in crab_launch.py; the goal-position write is identical.
MODE_POSITION = 3

# ---- nominal operating point -------------------------------------------
# Modest amplitudes and a slow period so that every perturbation stays
# feasible; the point of stage J is sensitivity, not maximum force.
PERIOD_S = 2.0
NOMINAL = {
    1: Knobs(A0=0.45, C=0.0, n=1),      # pitch
    2: Knobs(A0=0.35, C=0.0, n=1),      # heave
}
# Perturbation sizes, chosen large enough to clear measurement noise and
# small enough to stay in the locally-linear region.
DELTA = {"A0": 0.10, "C": 0.12, "h_diff": 0.30, "h_com": 0.30,
         "s_diff": 0.45, "s_com": 0.45, "w_diff": 0.45, "w_com": 0.45}
SWEEP_LEVELS = 5

N_CYCLES = 3          # commanded per mission; only cycle 2 is analysed
SETTLE_S = 4.0        # quiet delay between missions
PRE_QUIET_S = 1.5     # recorded BEFORE motion: the force tare baseline
IDLE_TAIL_S = 2.0
HW_RATE = 100.0       # servo write / feedback rate


def knobs_with(base: Knobs, name, value) -> Knobs:
    return replace(base, **{name: value})


def build_plan():
    """Every mission in the campaign, before randomisation."""
    pts = []

    def add(stage, servo, knob, value, rep=0):
        k = dict(NOMINAL)
        k[servo] = (knobs_with(NOMINAL[servo], knob, value)
                    if knob else NOMINAL[servo])
        label = (f"{stage}_s{servo}_{knob}_{value:+.3f}".replace("+", "p").replace("-", "m")
                 if knob else f"{stage}_nominal_{rep}")
        pts.append({"stage": stage, "servo": servo, "knob": knob or "",
                    "value": value if knob else 0.0, "label": label,
                    "k1": k[1], "k2": k[2]})

    # --- J: central differences on every knob, each servo
    for servo in (1, 2):
        for knob in KNOB_NAMES:
            if knob == "n":
                add("J", servo, "n", 2)          # integer: 1 -> 2
                continue
            d = DELTA[knob]
            base = getattr(NOMINAL[servo], knob)
            add("J", servo, knob, base + d)
            add("J", servo, knob, base - d)

    # --- S: response curves for the continuous knobs
    for servo in (1, 2):
        for knob in ("h_diff", "h_com", "s_diff", "s_com", "w_diff", "w_com"):
            d = DELTA[knob]
            base = getattr(NOMINAL[servo], knob)
            for lv in np.linspace(base - d, base + d, SWEEP_LEVELS):
                if abs(lv - base) < 1e-9:
                    continue                      # nominal covered by R
                add("S", servo, knob, float(lv))

    # --- P: GAIT C, the prediction test set.
    # Deliberately NOT +/- perturbations of the nominal and NOT points on the
    # stage-S sweeps: several knobs move together, and the period and crest
    # count differ. A model fitted on A (stage J) and selected on B (stage S)
    # has therefore never seen anything like these, so predicting them is a
    # real test rather than interpolation inside the training region.
    # Predictions are written down BEFORE these run -- see greybox_report.py.
    rng = np.random.default_rng(17)
    for i in range(12):
        k1 = Knobs(A0=float(rng.uniform(0.25, 0.60)),
                   C=float(rng.uniform(-0.15, 0.15)),
                   n=int(rng.choice([1, 1, 2])),
                   h_diff=float(rng.uniform(-0.35, 0.35)),
                   h_com=float(rng.uniform(-0.30, 0.30)),
                   s_diff=float(rng.uniform(-0.50, 0.50)),
                   s_com=float(rng.uniform(-0.50, 0.50)),
                   w_diff=float(rng.uniform(-0.50, 0.50)),
                   w_com=float(rng.uniform(-0.50, 0.50)))
        k2 = Knobs(A0=float(rng.uniform(0.20, 0.45)),
                   C=float(rng.uniform(-0.12, 0.12)),
                   n=int(rng.choice([1, 1, 2])),
                   h_diff=float(rng.uniform(-0.30, 0.30)),
                   s_diff=float(rng.uniform(-0.40, 0.40)),
                   w_diff=float(rng.uniform(-0.40, 0.40)))
        pts.append({"stage": "P", "servo": 0, "knob": "gaitC",
                    "value": float(i), "label": f"P_gaitC_{i:02d}",
                    "k1": k1, "k2": k2})

    # --- R: replicates
    for r in range(6):
        add("R", 1, None, 0.0, rep=r)
    return pts


def verify(pt):
    """Feasibility of one mission against position and slew limits."""
    problems = []
    for servo, key in ((1, "k1"), (2, "k2")):
        p = pt[key].to_params()
        ok, info = check(p, PERIOD_S, SERVO_LIMITS[servo], SLEW_LIMIT)
        if not ok:
            problems += [f"servo{servo}: {x}" for x in info["problems"]]
        pt[f"s{servo}_pos_min"] = round(info["pos_min"], 4)
        pt[f"s{servo}_pos_max"] = round(info["pos_max"], 4)
        pt[f"s{servo}_vel_peak"] = round(info["vel_peak"], 3)
        pt[f"s{servo}_Tmin"] = round(max_feasible_period(
            p, SERVO_LIMITS[servo], SLEW_LIMIT), 3)
    pt["feasible"] = (len(problems) == 0)
    pt["problems"] = "; ".join(problems)
    return pt


def design(folder, seed=5):
    pts = [verify(p) for p in build_plan()]
    rng = np.random.default_rng(seed)
    for rank, i in enumerate(rng.permutation(len(pts)), 1):
        pts[i]["run_order"] = rank
    pts.sort(key=lambda p: p["run_order"])

    os.makedirs(folder, exist_ok=True)
    rows = []
    for p in pts:
        row = {kk: p[kk] for kk in
               ("run_order", "stage", "servo", "knob", "value", "label",
                "feasible", "problems", "s1_pos_min", "s1_pos_max",
                "s1_vel_peak", "s2_pos_min", "s2_pos_max", "s2_vel_peak")}
        for sv, key in ((1, "k1"), (2, "k2")):
            for kn, vv in p[key].as_dict().items():
                row[f"s{sv}_{kn}"] = vv
        rows.append(row)
    out = os.path.join(folder, "plan.csv")
    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    n_bad = sum(1 for r in rows if not r["feasible"])
    per = N_CYCLES * PERIOD_S + SETTLE_S + PRE_QUIET_S + IDLE_TAIL_S
    print(f"{len(rows)} missions -> {out}")
    for st in ("J", "S", "P", "R"):
        print(f"   stage {st}: {sum(1 for r in rows if r['stage']==st)}")
    print(f"   infeasible: {n_bad}")
    if n_bad:
        for r in rows:
            if not r["feasible"]:
                print(f"     {r['label']}: {r['problems']}")
    print(f"   period {PERIOD_S}s x {N_CYCLES} cycles (cycle 2 analysed) "
          f"+ {SETTLE_S}s delay + {PRE_QUIET_S}s pre-quiet "
          f"+ {IDLE_TAIL_S}s tail = {per:.0f}s/mission")
    print(f"   estimated {len(rows)*per/60:.0f} min")
    return rows


# =========================================================================
# Execution
# =========================================================================
def run(folder, only_stage=None):
    import threading

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from std_msgs.msg import Float32MultiArray, String

    rows = list(csv.DictReader(open(os.path.join(folder, "plan.csv"))))
    if only_stage:
        rows = [r for r in rows if r["stage"] in only_stage]
    rows = [r for r in rows if r["feasible"] in ("True", "true", True)]
    data_dir = os.path.join(folder, "data")
    os.makedirs(data_dir, exist_ok=True)

    rclpy.init()
    node = Node("amfm_experiment")
    pub = node.create_publisher(Float32MultiArray, "joint_cmd", 10)
    # The CONTROLLER owns joint_cmd. Publishing to it while the control loop
    # is also driving means whichever message lands last wins, and in practice
    # the external commands are simply ignored -- a position-gain sweep run
    # this way produced amplitude ratio 0.00 on both servos with no error
    # anywhere, because the servos never saw the reference at all.
    # 'passthrough' tells the controller to stay armed but drive nothing, so
    # this script owns joint_cmd outright. It is released at the end.
    manual_pub = node.create_publisher(String, "manual_cmd", 10)
    st = {"fb": {}, "lc": [], "rec": False}

    def manual(text, settle=1.5):
        """Send a manual_cmd, WAITING for the controller to have discovered
        this publisher first.

        ROS 2's default QoS is volatile: anything published before the
        subscriber is matched is silently dropped. Firing three messages
        0.1 s apart immediately after creating the publisher therefore sent
        them into the void -- the controller never entered passthrough, kept
        ownership of joint_cmd, and 20 missions were recorded with the servos
        commanded 0.900 rad peak-to-peak and moving 0.002. Nothing errored;
        the data was simply meaningless.
        """
        t0 = time.time()
        while manual_pub.get_subscription_count() < 1 and time.time() - t0 < 10.0:
            time.sleep(0.05)
        if manual_pub.get_subscription_count() < 1:
            print("   WARNING: no subscriber on manual_cmd — is the controller up?")
        m = String()
        m.data = text
        t0 = time.time()
        while time.time() - t0 < settle:      # keep sending across the window
            manual_pub.publish(m)
            time.sleep(0.15)

    def fb_cb(msg):
        d = list(msg.data)
        for i in range(0, len(d), 6):
            st["fb"][int(d[i])] = (float(d[i + 2]), float(d[i + 3]))

    def lc_cb(msg):
        if st["rec"]:
            d = list(msg.data)
            # load_cell_data: flat [t, Fx, Fy, Fz, ...] per sample block
            st["lc"].append((time.time(), d))

    node.create_subscription(Float32MultiArray, "joint_feedback", fb_cb, 50)
    try:
        node.create_subscription(Float32MultiArray, "load_cell_data", lc_cb, 200)
    except Exception:
        pass

    # CALLBACKS RUN ON THEIR OWN THREAD.
    # Previously the command loop called spin_once(timeout_sec=0) once per
    # 100 Hz tick, which dispatches at most ONE callback. Feedback alone
    # arrives at ~100 Hz and the load cell adds ~10 packets/s on top, so the
    # loop could never keep up and the queues grew without bound. Two
    # consequences, both silent:
    #   - joint_feedback was dispatched later and later, so the logged
    #     'meas' column lagged the true joint angle by up to ~0.9 s -- a
    #     0.44-cycle phase error at this period, which destroys any fit of
    #     force against measured kinematics.
    #   - load-cell packets are timestamped by their callback, so their
    #     recorded arrival times inherited the same growing skew.
    # A dedicated executor thread drains both continuously, so timestamps are
    # true arrival times and the command loop is left to do nothing but keep
    # its 100 Hz cadence.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    t0 = time.time()
    while not st["fb"] and time.time() - t0 < 20:
        time.sleep(0.05)
    if not st["fb"]:
        print("no joint_feedback — is the stack up and calibrated?")
        executor.shutdown(); node.destroy_node(); rclpy.shutdown(); return 2
    print(f"feedback live: {[(k, round(v[0],4)) for k,v in sorted(st['fb'].items())]}")
    print("engaging controller passthrough (this script owns joint_cmd)...")
    manual("passthrough")

    def send(th1, th2):
        m = Float32MultiArray()
        m.data = [1.0, 2.0, float(MODE_POSITION), float(MODE_POSITION),
                  float(th1), float(th2)]
        pub.publish(m)

    # ---- TRACKING PRE-FLIGHT -------------------------------------------
    # Verify the servos actually FOLLOW a command before spending 20 minutes
    # recording. Passthrough failing is silent -- no error, no warning, just
    # commands that go nowhere -- so the only way to catch it is to move the
    # joints and look at the encoders.
    def preflight():
        amp, per, n_s = 0.12, 2.0, int(HW_RATE * 2.0)
        u = np.linspace(0, 1, n_s, endpoint=False)
        ref = amp * np.sin(2 * np.pi * u)
        p1 = st["fb"].get(1, (0.0, 0.0))[0]
        p2 = st["fb"].get(2, (0.0, 0.0))[0]
        for a in np.linspace(0, 1, int(HW_RATE)):
            send((1 - a) * p1, (1 - a) * p2)
            time.sleep(1.0 / HW_RATE)
        rec = []
        t0 = time.time()
        for j in range(n_s * 2):
            v = ref[j % n_s]
            send(v, v * 0.7)
            rec.append((v, v * 0.7,
                        st["fb"].get(1, (np.nan,) * 2)[0],
                        st["fb"].get(2, (np.nan,) * 2)[0]))
            time.sleep(1.0 / HW_RATE)
        a = np.array(rec, float)
        half = len(a) // 2
        r1 = np.ptp(a[half:, 2]) / max(np.ptp(a[half:, 0]), 1e-9)
        r2 = np.ptp(a[half:, 3]) / max(np.ptp(a[half:, 1]), 1e-9)

        # LAG, measured separately. An amplitude ratio is blind to phase: when
        # feedback dispatch fell behind, 'meas' was a 0.87 s-delayed copy of
        # the command and this ratio still read 1.00, because a delayed sine
        # has exactly the same peak-to-peak. On a periodic command the delay
        # is only defined modulo the period, so it is reported wrapped to
        # +/- half a period.
        def lag_s(cmd_col, meas_col):
            c = a[:, cmd_col] - a[:, cmd_col].mean()
            m = a[:, meas_col] - a[:, meas_col].mean()
            xc = np.correlate(m, c, "full")
            k = np.argmax(xc) - (len(xc) // 2)
            L = k / HW_RATE
            n_per = int(round(L / per))
            return L - n_per * per          # wrapped to (-per/2, +per/2]
        return r1, r2, lag_s(0, 2), lag_s(1, 3)

    r1, r2, l1, l2 = preflight()
    print(f"   tracking pre-flight: servo1 {r1:.2f}   servo2 {r2:.2f}  "
          f"(1.0 = follows exactly)")
    print(f"   feedback lag:        servo1 {l1:+.3f}s  servo2 {l2:+.3f}s  "
          f"(0 = in step)")
    if max(abs(l1), abs(l2)) > 0.15:
        print("   ABORT: joint_feedback is not keeping up with the command.")
        print("   The logged 'meas' trace would be phase-shifted against the")
        print("   force, so any fit of force to measured kinematics is void.")
        manual("stop")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 2
    if min(r1, r2) < 0.5:
        print("   ABORT: the servos are not following commands. Passthrough")
        print("   probably did not engage, so the controller still owns")
        print("   joint_cmd. Nothing has been recorded.")
        manual("stop")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        return 2
    print()


    manifest = []
    try:
        for i, r in enumerate(rows, 1):
            k1 = Knobs(**{n: (int(float(r[f"s1_{n}"])) if n == "n"
                              else float(r[f"s1_{n}"])) for n in KNOB_NAMES})
            k2 = Knobs(**{n: (int(float(r[f"s2_{n}"])) if n == "n"
                              else float(r[f"s2_{n}"])) for n in KNOB_NAMES})
            n_s = int(HW_RATE * PERIOD_S)
            _, th1, _ = cycle(k1.to_params(), PERIOD_S, n_s)
            _, th2, _ = cycle(k2.to_params(), PERIOD_S, n_s)

            # Final guard on the ACTUAL array about to be sent.
            if (np.max(np.abs(th1)) > PITCH_LIMIT
                    or np.max(np.abs(th2)) > HEAVE_LIMIT):
                print(f"[{i}/{len(rows)}] {r['label']}: SKIPPED (array exceeds limits)")
                continue

            print(f"[{i}/{len(rows)}] {r['stage']} {r['label']}")
            # ease in from wherever the joints are, so the first sample is not a step
            p1 = st["fb"].get(1, (0.0, 0.0))[0]
            p2 = st["fb"].get(2, (0.0, 0.0))[0]
            for a in np.linspace(0, 1, int(HW_RATE * 1.5)):
                send((1 - a) * p1 + a * th1[0], (1 - a) * p2 + a * th2[0])
                time.sleep(1.0 / HW_RATE)
            # Hold the start pose while the water settles. The executor
            # thread keeps draining callbacks throughout, so no backlog forms
            # and arrival timestamps stay honest.
            t_end = time.time() + SETTLE_S
            while time.time() < t_end:
                send(th1[0], th2[0])
                time.sleep(1.0 / HW_RATE)

            # PRE-MOTION QUIET BASELINE. Recorded with the fin held at the
            # cycle start pose and the water already settled, so the tare is
            # taken from genuinely still fluid. The post-motion tail cannot do
            # this job: the fin has just stopped and the wake has not decayed.
            st["lc"], st["rec"] = [], True
            t_start = time.time() + PRE_QUIET_S      # motion begins HERE
            while time.time() < t_start:
                send(th1[0], th2[0])
                time.sleep(1.0 / HW_RATE)
            kin = []
            for c in range(N_CYCLES):
                for j in range(n_s):
                    send(th1[j], th2[j])
                    kin.append((time.time() - t_start, th1[j], th2[j],
                                st["fb"].get(1, (np.nan,)*2)[0],
                                st["fb"].get(2, (np.nan,)*2)[0]))
                    time.sleep(max(0.0, 1.0 / HW_RATE
                                   - (time.time() - t_start - kin[-1][0])))
            # Idle tail. Before the executor thread existed this window
            # captured only ~0.3 s of its intended 2 s, because packets
            # arriving during a bare sleep were never dispatched.
            t_end = time.time() + IDLE_TAIL_S
            while time.time() < t_end:
                time.sleep(0.005)
            st["rec"] = False

            kp = os.path.join(data_dir, f"{r['label']}_kin.csv")
            np.savetxt(kp, np.array(kin), delimiter=",",
                       header="t,cmd1,cmd2,meas1,meas2", comments="")
            lp = os.path.join(data_dir, f"{r['label']}_force.csv")
            with open(lp, "w", newline="") as fh:
                w = csv.writer(fh)
                w.writerow(["wall", "payload"])
                for wt, d in st["lc"]:
                    w.writerow([wt - t_start, " ".join(f"{x:.6f}" for x in d)])
            manifest.append({**r, "kin_csv": os.path.basename(kp),
                             "force_csv": os.path.basename(lp),
                             "n_kin": len(kin), "n_force": len(st["lc"])})
            print(f"      {len(kin)} kin samples, {len(st['lc'])} force packets")
    finally:
        # park at the nominal start pose, release passthrough, then stop
        try:
            for _ in range(10):
                send(0.0, 0.0)
                time.sleep(0.02)
            manual("stop")          # hand joint_cmd back to the controller
        except Exception:
            pass
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        if manifest:
            mp = os.path.join(folder, "manifest.csv")
            with open(mp, "w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(manifest[0]))
                w.writeheader(); w.writerows(manifest)
            print(f"\nwrote {mp} ({len(manifest)} missions)")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--design", action="store_true")
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--stage", default="")
    a = ap.parse_args()
    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    if a.design:
        design(folder)
    if a.run:
        return run(folder, only_stage=set(a.stage) if a.stage else None)
    return 0


if __name__ == "__main__":
    sys.exit(main())
