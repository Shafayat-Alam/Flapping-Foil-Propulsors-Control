#!/usr/bin/env python3
"""Closed-loop waveform shaper over the AM/FM knobs.  Stages 2, 3 and 4.

    stage 2   --no-seed     start from nominal, tune knobs until the measured
                            force waveform matches the mathematically-defined
                            target. No prior knowledge used.
    stage 3   --seed        jump straight to the knobs the stage-1 Jacobian
                            predicts, then refine from there. Same target,
                            same optimiser, same budget -- so the ONLY
                            difference is the starting point and the
                            comparison isolates what seeding is worth.
    stage 4   --direction D shape the force along an arbitrary 3D direction
                            rather than along Fx alone (see amfm_direction.py).

WHY THE ERROR LIVES IN METRIC SPACE
-----------------------------------
The target is a shape, described mathematically (curve_spec / ref_curves),
and the natural temptation is to score with a point-by-point correlation
against it. That was tried and it failed structurally: a correlation is
scale-invariant AND has no derivative with respect to any single knob, so it
cannot carry a gain, and a controller steering on it froze at its seed while
reporting 0.72 "match" on a run producing 0.71 N against a 9 N target.

Here the target curve is reduced to the SAME eight shape metrics that stage 1
measured, and the error is a tolerance-normalised sum over them. Every term
is then a scalar with a measured sensitivity to every knob -- which is
exactly what the stage-1 Jacobian provides.

SEEDING (stage 3)
-----------------
Stage 1 measures J = d(metric)/d(knob). Seeding solves the linear inverse

    dk = J^+ (m_target - m_nominal)

with a damped (Tikhonov) pseudo-inverse. Damping matters: J is rank-deficient
-- the whole point of stage 1 is that fewer metric directions are steerable
than there are knobs -- so an undamped inverse would put enormous, meaningless
values into the null-space directions. The damping trades a slightly biased
seed for one that stays inside the region where the Jacobian was measured.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time

import numpy as np

WORKSPACE_ROOT = "/home/shafa/soft-propulsors-control"
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "scripts"))
sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))

from amfm_waveform import Knobs, KNOB_NAMES, cycle, check   # noqa: E402
from amfm_metrics import metrics, METRIC_NAMES, lobe_peaks  # noqa: E402
import amfm_experiment as EX                                # noqa: E402

# Which metrics are steered, and what "close enough" means for each.
# Tolerances double as the normalisation, so a 0.05 miss on a width and a
# 0.5 N miss on a height contribute comparably instead of the newtons
# dominating simply by being numerically larger.
TARGET_SPEC = {
    "crest_height":  {"tol": 0.40, "w": 2.0},
    "trough_depth":  {"tol": 0.40, "w": 2.0},
    "crest_width":   {"tol": 0.05, "w": 1.5},
    "trough_width":  {"tol": 0.05, "w": 1.0},
    "crest_skew":    {"tol": 0.05, "w": 1.5},
    "trough_skew":   {"tol": 0.05, "w": 1.0},
    "crest_count":   {"tol": 0.50, "w": 3.0},
    "bias":          {"tol": 0.20, "w": 1.0},
}
# The OFF-CHANNEL net force, held at zero. Shaping thrust while the vertical
# channel carries a standing net force is not the requested result, so this is
# scored as part of the objective rather than merely reported afterwards. The
# tolerance is ~2.5x the measured replicate spread on Fy_bias (0.019 N), so
# the term is quiet inside the noise and bites outside it.
OTHER_BIAS_SPEC = {"tol": 0.05, "w": 3.0}
# Knobs the optimiser may move (n stays discrete and is set from peak count).
TUNABLE = ["A0", "C", "h_diff", "h_com", "s_diff", "s_com", "w_diff", "w_com"]
BOUNDS = {"A0": (0.15, 0.80), "C": (-0.30, 0.30),
          "h_diff": (-0.55, 0.55), "h_com": (-0.55, 0.55),
          "s_diff": (-0.70, 0.70), "s_com": (-0.70, 0.70),
          "w_diff": (-0.70, 0.70), "w_com": (-0.70, 0.70)}


def target_metrics_from_curve(name="drag"):
    """The mathematically-defined target curve, reduced to shape metrics.

    Scale-free metrics come straight across; the absolute ones (heights,
    bias) are expressed as fractions of the curve's own crest and rescaled
    to the force magnitude the rig can actually reach, so the shape target
    does not silently smuggle in an unreachable magnitude demand.
    """
    # target_curves, NOT ref_curves. ref_curves keeps the width penalty from
    # the original formulation, and that objective is bang-bang in crest
    # width -- at its default weights it returns a needle (measured
    # crest_width 0.004) for drag and collapses lift to a single crest. The
    # compact-support family in target_curves produces the shapes actually
    # wanted: drag = one right-skewed crest dying to zero (crest_width 0.231,
    # skew 0.386, count 1), lift = two equal crests (count 2).
    import target_curves as tc
    sp, _ = (tc.drag_target() if name == "drag" else tc.lift_target())
    t, F = tc.evaluate(sp)
    m = metrics(np.asarray(F))
    peak = max(m["crest_height"], 1e-9)
    return {k: (v / peak if k in ("crest_height", "trough_depth", "bias") else v)
            for k, v in m.items()}, (t, F)


# Thrust objective. Net thrust is REWARDED linearly (unbounded, so the search
# keeps pushing) while the off-channel net force is penalised quadratically
# (a constraint, satisfied and then left alone). The shape objective could not
# express this: it scores the target curve rescaled to whatever crest the rig
# happened to produce, so a weak gait and a strong one of the same shape score
# identically -- and across 100 stage-1 gaits the trough/crest ratio never went
# below 0.474, meaning shape alone cannot buy thrust.
THRUST_SPEC = {"thrust_scale": 0.02,   # N per unit of reward
               "other_tol": 0.05,      # N, ~2.5x the replicate spread
               "other_w": 3.0,
               "symmetry_w": 1.5}      # penalty weight on |crest_1-crest_2|


def thrust_error(meas):
    """Scalar to MINIMISE: -net_thrust, plus the off-channel null constraint.

    The null is a HINGE, not a quadratic from zero: inside the tolerance it
    costs nothing, outside it grows quadratically. Penalising it from zero
    made standing still the optimum -- the best measured gait (+0.152 N thrust,
    -0.069 N vertical) scored WORSE than a gait producing no force at all,
    because a motionless rig satisfies a null constraint perfectly. A hinge
    says "get the vertical inside tolerance, then stop caring and chase
    thrust", which is what was actually asked for.
    """
    net = meas.get("bias", 0.0)
    # 'other_bias' is the 1D case (channel = Fx or Fy, the OTHER in-plane
    # channel is the thing to null). 'offaxis_net' is the general 3D case
    # (an arbitrary direction u, the null is whatever net force survives
    # perpendicular to u). Exactly one of the two is present depending on
    # which evaluate() branch produced `meas`.
    other = meas.get("offaxis_net", meas.get("other_bias", 0.0))
    excess = max(0.0, abs(other) - THRUST_SPEC["other_tol"])
    c1, c2 = meas.get("crest_1", 0.0), meas.get("crest_2", 0.0)
    # Symmetry between the two crests, as a FRACTION of the bigger one so it
    # is scale-free -- a large mismatch on small crests should not cost less
    # than a small mismatch on large ones. Silent (0) when only one crest
    # exists, so this never fights the search into keeping a second, unwanted
    # lobe alive just to have something to balance.
    imbalance = abs(c1 - c2) / max(c1, 1e-9) if c2 > 0 else 0.0
    terms = {"net_thrust": net, "other_bias": other, "null_excess": excess,
             "crest_1": c1, "crest_2": c2, "imbalance": imbalance}
    total = -net / THRUST_SPEC["thrust_scale"]
    total += THRUST_SPEC["other_w"] * (excess / THRUST_SPEC["other_tol"]) ** 2
    total += THRUST_SPEC["symmetry_w"] * imbalance ** 2
    return total, terms


def error(meas, target, force_scale):
    """Tolerance-normalised shape error. force_scale converts the target's
    relative heights into newtons using the measured crest, so shape is
    scored independently of how much force the rig happens to produce."""
    total, terms = 0.0, {}
    for k, spec in TARGET_SPEC.items():
        if k not in meas or k not in target:
            continue
        tgt = target[k] * force_scale if k in ("crest_height", "trough_depth", "bias") else target[k]
        e = (meas[k] - tgt) / spec["tol"]
        terms[k] = e
        total += spec["w"] * e * e
    # Off-channel net force -> 0. Target is zero absolutely, not relative to
    # the curve, so it is not scaled by force_scale.
    if "other_bias" in meas:
        e = meas["other_bias"] / OTHER_BIAS_SPEC["tol"]
        terms["other_bias"] = e
        total += OTHER_BIAS_SPEC["w"] * e * e
    return total, terms


def _split(kd):
    """Qualified knob dict ('s1.A0' -> value) into per-servo knob dicts."""
    s1, s2 = {}, {}
    for key, v in kd.items():
        servo, name = key.split(".", 1)
        (s1 if servo == "s1" else s2)[name] = v
    return s1, s2


def seed_from_jacobian(jac_path, target, nominal_meas, nominal_knobs, tunable,
                       channel="Fx", damping=0.35, log=print):
    """Predict knob values from the stage-1 Jacobian (stage 3 only)."""
    if not os.path.exists(jac_path):
        log(f"   no jacobian at {jac_path} -- falling back to nominal seed")
        return dict(nominal_knobs)
    J = json.load(open(jac_path))
    cols = [c for c in J["cols"]]
    rows = [f"{channel}_{m}" for m in TARGET_SPEC if f"{channel}_{m}|{cols[0]}" in J["jacobian"]]
    if not rows:
        log("   jacobian has no usable rows for this channel -- nominal seed")
        return dict(nominal_knobs)
    M = np.array([[J["jacobian"].get(f"{r}|{c}", 0.0) for c in cols] for r in rows])
    dm = np.array([(target[r.split("_", 1)[1]] - nominal_meas.get(r.split("_", 1)[1], 0.0))
                   for r in rows])
    # damped least squares: J^T (J J^T + lam I)^-1 dm
    lam = damping * (np.linalg.norm(M) ** 2) / max(M.shape[0], 1)
    dk = M.T @ np.linalg.solve(M @ M.T + lam * np.eye(M.shape[0]), dm)
    out = dict(nominal_knobs)
    # Apply the solution to every knob the optimiser is allowed to move. The
    # inverse is solved over ALL 18 columns, so discarding the servo-2 half
    # (as this did originally) applies only part of a solution whose predicted
    # metric change assumed the whole of it -- the seed then lands somewhere
    # the solve never intended.
    for c, d in zip(cols, dk):
        servo, knob = c.split(".", 1)
        if c in tunable:
            lo, hi = BOUNDS[knob]
            out[c] = float(np.clip(out.get(c, 0.0) + d, lo, hi))
    log("   seeded knobs: " + "  ".join(f"{k}={out[k]:+.3f}" for k in sorted(tunable)))
    return out


class Rig:
    """Command a precomputed AM/FM cycle and read back force + kinematics."""

    def __init__(self, period_s=EX.PERIOD_S):
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32MultiArray, String
        self._rclpy, self._Arr, self._Str = rclpy, Float32MultiArray, String
        rclpy.init()
        self.node = Node("amfm_shaper")
        self.pub = self.node.create_publisher(Float32MultiArray, "joint_cmd", 10)
        # See amfm_experiment: the controller owns joint_cmd unless put into
        # passthrough, and without it external commands are silently ignored.
        self._manual_pub = self.node.create_publisher(String, "manual_cmd", 10)
        self.period = period_s
        self.fb, self.lc, self.rec = {}, [], False

        def fb_cb(m):
            d = list(m.data)
            for i in range(0, len(d), 6):
                self.fb[int(d[i])] = (float(d[i + 2]), float(d[i + 3]))

        def lc_cb(m):
            if self.rec:
                self.lc.append((time.time(), list(m.data)))
        self.node.create_subscription(Float32MultiArray, "joint_feedback", fb_cb, 50)
        self.node.create_subscription(Float32MultiArray, "load_cell_data", lc_cb, 200)

        # Callbacks on their own thread -- see the long note in
        # amfm_experiment.py. Dispatching one callback per 100 Hz command tick
        # cannot keep up with ~100 Hz feedback plus the load-cell packets, and
        # the backlog silently phase-shifts both the logged joint angles and
        # the force arrival times.
        import threading
        from rclpy.executors import SingleThreadedExecutor
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    def _manual(self, text, settle=1.5):
        """Send manual_cmd only after the controller has discovered us.

        Default QoS is volatile, so messages published before the subscriber
        is matched are dropped silently -- which is how a whole campaign got
        recorded with the servos never moving.
        """
        t0 = time.time()
        while (self._manual_pub.get_subscription_count() < 1
               and time.time() - t0 < 10.0):
            time.sleep(0.05)
        m = self._Str()
        m.data = text
        t0 = time.time()
        while time.time() - t0 < settle:
            self._manual_pub.publish(m)
            time.sleep(0.15)

    def wait(self, timeout=20.0):
        t0 = time.time()
        while not self.fb and time.time() - t0 < timeout:
            time.sleep(0.05)
        if self.fb:
            self._manual("passthrough")
            time.sleep(0.5)
        return bool(self.fb)

    def send(self, a, b):
        m = self._Arr()
        m.data = [1.0, 2.0, float(EX.MODE_POSITION), float(EX.MODE_POSITION),
                  float(a), float(b)]
        self.pub.publish(m)

    def measure(self, k1: Knobs, k2: Knobs, out_dir, label):
        """One evaluation: ease in, settle, replay 3 cycles, record."""
        n_s = int(EX.HW_RATE * self.period)
        _, th1, _ = cycle(k1.to_params(), self.period, n_s)
        _, th2, _ = cycle(k2.to_params(), self.period, n_s)
        if (np.max(np.abs(th1)) > EX.PITCH_LIMIT
                or np.max(np.abs(th2)) > EX.HEAVE_LIMIT):
            return None, "commanded array exceeds position limits"
        # Slew guard. Under the shape objective the amplitude had no incentive
        # to grow; under the thrust objective it does, so the commanded
        # velocity has to be checked as well as the position.
        for th in (th1, th2):
            if np.max(np.abs(np.gradient(th, self.period / len(th)))) > EX.SLEW_LIMIT:
                return None, "commanded array exceeds slew limit"

        p1 = self.fb.get(1, (0.0, 0.0))[0]
        p2 = self.fb.get(2, (0.0, 0.0))[0]
        for a in np.linspace(0, 1, int(EX.HW_RATE * 1.5)):
            self.send((1 - a) * p1 + a * th1[0], (1 - a) * p2 + a * th2[0])
            time.sleep(1.0 / EX.HW_RATE)
        # Hold the start pose while the water settles. Callbacks are drained
        # by the executor thread throughout, so nothing queues up here.
        t_end = time.time() + EX.SETTLE_S
        while time.time() < t_end:
            self.send(th1[0], th2[0])
            time.sleep(1.0 / EX.HW_RATE)

        # Pre-motion quiet window: the force tare baseline, taken with the fin
        # held still in settled water rather than on the post-motion tail
        # where the wake is still loading the cell.
        self.lc, self.rec = [], True
        t0 = time.time() + EX.PRE_QUIET_S            # motion begins HERE
        while time.time() < t0:
            self.send(th1[0], th2[0])
            time.sleep(1.0 / EX.HW_RATE)
        kin = []
        for _ in range(EX.N_CYCLES):
            for j in range(n_s):
                self.send(th1[j], th2[j])
                kin.append((time.time() - t0, th1[j], th2[j],
                            self.fb.get(1, (np.nan,) * 2)[0],
                            self.fb.get(2, (np.nan,) * 2)[0]))
                time.sleep(max(0.0, 1.0 / EX.HW_RATE - (time.time() - t0 - kin[-1][0])))
        t_end = time.time() + EX.IDLE_TAIL_S
        while time.time() < t_end:
            time.sleep(0.005)
        self.rec = False

        os.makedirs(out_dir, exist_ok=True)
        kp = os.path.join(out_dir, f"{label}_kin.csv")
        np.savetxt(kp, np.array(kin), delimiter=",",
                   header="t,cmd1,cmd2,meas1,meas2", comments="")
        lp = os.path.join(out_dir, f"{label}_force.csv")
        with open(lp, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["wall", "payload"])
            for wt, d in self.lc:
                w.writerow([wt - t0, " ".join(f"{x:.6f}" for x in d)])
        return (kp, lp), None

    def stop(self):
        try:
            for _ in range(10):
                self.send(0.0, 0.0)
                time.sleep(0.02)
            self._manual("stop")     # return joint_cmd to the controller
        finally:
            self._executor.shutdown()
            self.node.destroy_node()
            self._rclpy.shutdown()


def evaluate(paths, channel="Fx", direction=None):
    """Force metrics for one measurement, optionally along a 3D direction."""
    import amfm_analyze as AN
    _, lp = paths
    r = AN.load_force(lp)
    if r is None:
        return None
    t, fx, fy, fz = r
    if direction is not None:
        u = np.asarray(direction, float)
        u = u / max(np.linalg.norm(u), 1e-12)
        proj = fx * u[0] + fy * u[1] + fz * u[2]
        m = metrics(proj)
        # leakage: how much force ends up off the requested axis. Reported
        # both as an RMS (shape of the leakage) and as a NET vector bias (the
        # part of it that survives averaging) -- the thrust objective's null
        # constraint needs the latter, since a leakage that averages to zero
        # over the cycle is not a standing off-axis force.
        perp_vec = (np.stack([fx, fy, fz], axis=-1)
                    - proj[:, None] * u[None, :])
        m["offaxis_rms"] = float(np.sqrt(np.mean(np.sum(perp_vec ** 2, axis=-1))))
        m["offaxis_net"] = float(np.linalg.norm(perp_vec.mean(axis=0)))
        m["fz_bias"] = float(np.mean(fz))
        peaks = lobe_peaks(proj, +1)
        m["crest_1"] = peaks[0] if len(peaks) > 0 else 0.0
        m["crest_2"] = peaks[1] if len(peaks) > 1 else 0.0
        return m
    sig = fx if channel == "Fx" else fy
    m = metrics(sig)
    m["fz_bias"] = float(np.mean(fz))
    m["other_bias"] = float(np.mean(fy if channel == "Fx" else fx))
    # Both crest peaks, not just the dominant one -- needed to score a
    # two-crest waveform for SYMMETRY (equal height) rather than only for
    # how tall its biggest lobe is.
    peaks = lobe_peaks(sig, +1)
    m["crest_1"] = peaks[0] if len(peaks) > 0 else 0.0
    m["crest_2"] = peaks[1] if len(peaks) > 1 else 0.0
    return m


def optimise(rig, folder, target, k_start, tunable, channel, direction,
             max_evals=40, step0=0.25, objective="shape", log=print):
    """Coordinate descent with shrinking steps.

    Chosen over a gradient method because each evaluation costs ~12 s of rig
    time and finite-difference gradients would spend 8 of them per step. This
    accepts any improving move immediately and shrinks the step only when a
    full sweep of all knobs fails -- so progress is made from the first few
    evaluations rather than after the first complete gradient.
    """
    k = dict(k_start)
    history, best = [], None
    step = {n: step0 * (BOUNDS[n.split('.', 1)[1]][1]
                        - BOUNDS[n.split('.', 1)[1]][0]) / 2 for n in tunable}
    ev = 0

    def score(kd, tag):
        nonlocal ev
        ev += 1
        d1, d2 = _split(kd)
        kn1 = Knobs(**{**EX.NOMINAL[1].as_dict(), **d1})
        kn2 = Knobs(**{**EX.NOMINAL[2].as_dict(), **d2})
        paths, err = rig.measure(kn1, kn2, os.path.join(folder, "data"),
                                 f"{tag}_{ev:03d}")
        if paths is None:
            log(f"   [{ev:02d}] {tag}: {err}")
            return None
        m = evaluate(paths, channel, direction)
        if m is None:
            log(f"   [{ev:02d}] {tag}: no force data")
            return None
        if objective == "thrust":
            e, terms = thrust_error(m)
        else:
            e, terms = error(m, target, max(m.get("crest_height", 0.0), 1e-6))
        rec = {"eval": ev, "tag": tag, "err": e, **{f"m_{a}": b for a, b in m.items()},
               **{f"k_{a}": b for a, b in kd.items()}}
        history.append(rec)
        log(f"   [{ev:02d}] {tag:14s} err {e:8.3f}   " +
            " ".join(f"{a}:{b:+.3f}" for a, b in list(terms.items())[:4]))
        return e

    base = score(k, "start")
    if base is None:
        return None, history
    best = (base, dict(k))

    while ev < max_evals and max(step.values()) > 0.01:
        improved = False
        for name in tunable:
            if ev >= max_evals:
                break
            for sgn in (+1, -1):
                lo, hi = BOUNDS[name.split(".", 1)[1]]
                trial = dict(best[1])
                trial[name] = float(np.clip(trial[name] + sgn * step[name], lo, hi))
                if abs(trial[name] - best[1][name]) < 1e-9:
                    continue
                e = score(trial, f"{name}{'+' if sgn > 0 else '-'}")
                if e is not None and e < best[0] - 1e-6:
                    best = (e, trial)
                    improved = True
                    break
        if not improved:
            for n in step:
                step[n] *= 0.5
            log(f"   -- no improvement, halving steps (max {max(step.values()):.3f})")
    return best, history


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("folder")
    ap.add_argument("--curve", default="drag", choices=["drag", "lift"])
    ap.add_argument("--channel", default="Fx", choices=["Fx", "Fy"])
    ap.add_argument("--seed", action="store_true", help="stage 3: seed from stage-1 Jacobian")
    ap.add_argument("--jacobian", default="amfm_shaping/jacobian.json")
    ap.add_argument("--direction", default="", help="stage 4: 'x,y,z' target thrust axis")
    ap.add_argument("--max-evals", type=int, default=40)
    ap.add_argument("--objective", default="shape", choices=["shape", "thrust"],
                    help="'shape' matches the target curve; 'thrust' maximises "
                         "net Fx with the off-channel net held at zero")
    ap.add_argument("--start-gait", default="",
                    help="label from amfm_shaping/plan.csv to start from, "
                         "e.g. P_gaitC_09 (the measured best-thrust gait)")
    ap.add_argument("--resume-from", default="",
                    help="folder with a result.json to continue from "
                         "(its best_knobs become the new starting point)")
    ap.add_argument("--period", type=float, default=EX.PERIOD_S,
                    help="gait period in seconds. Every prior campaign used "
                         "PERIOD_S=2.0 unchanged -- this lets the OPTIMISER "
                         "trade amplitude against speed at a different fixed "
                         "period, rather than replaying an already-tuned "
                         "shape faster (which thrust2's shape has no room "
                         "for: its slew floor sits at 1.985s, barely below "
                         "2.0s, because the search already spent its speed "
                         "budget on amplitude).")
    ap.add_argument("--both-servos", action="store_true",
                    help="tune BOTH servos (16 knobs) instead of servo 1 only")
    ap.add_argument("--bias-weight", type=float, default=None,
                    help="weight on net force (thrust). Default 1.0 leaves the "
                         "trough terms ~11x more important than thrust.")
    a = ap.parse_args()

    folder = a.folder if os.path.isabs(a.folder) else os.path.join(WORKSPACE_ROOT, a.folder)
    os.makedirs(folder, exist_ok=True)
    direction = ([float(x) for x in a.direction.split(",")] if a.direction else None)

    if a.bias_weight is not None:
        TARGET_SPEC["bias"]["w"] = float(a.bias_weight)

    target, (tt, tF) = target_metrics_from_curve(a.curve)
    print(f"objective: {a.objective}   curve: {a.curve}   channel: {a.channel}"
          + (f"   direction: {direction}" if direction else ""))
    print("   " + "  ".join(f"{k}={target[k]:+.3f}" for k in TARGET_SPEC if k in target))

    # Every knob of BOTH servos is carried in the state; `tunable` decides
    # which of them the search may move. Servo 2 was previously pinned at
    # nominal, which locked out s2.h_com -- the strongest measured
    # trough-reducing knob in the whole Jacobian (-0.933 on Fx_trough_depth,
    # against -0.181 for the best servo-1 alternative).
    k_nom = {f"s1.{n}": getattr(EX.NOMINAL[1], n) for n in TUNABLE}
    k_nom.update({f"s2.{n}": getattr(EX.NOMINAL[2], n) for n in TUNABLE})
    k_nom["s1.n"] = int(round(target.get("crest_count", 1)))
    k_nom["s2.n"] = EX.NOMINAL[2].n
    if a.resume_from:
        rp = os.path.join(WORKSPACE_ROOT, a.resume_from, "result.json")
        prev = json.load(open(rp))
        kb = prev.get("best_knobs") or {}
        for key, v in kb.items():
            if key in ("s1.n", "s2.n") or key.split(".", 1)[1] == "n":
                k_nom[key if "." in key else f"s1.{key}"] = int(v)
            elif key in k_nom:
                k_nom[key] = float(v)
        print(f"   resuming from {a.resume_from}  "
              f"(its best: net {prev.get('err_best', float('nan')):.3f})")

    if a.start_gait:
        # Start from a gait already MEASURED to be good, rather than from the
        # nominal. For thrust this matters: the best stage-1 gait produced
        # +0.152 N where the shape-optimised gaits produced ~+0.025 N, so the
        # nominal is a poor place to begin.
        import csv as _csv
        row = next((r for r in _csv.DictReader(
            open(os.path.join(WORKSPACE_ROOT, "amfm_shaping", "plan.csv")))
            if r["label"] == a.start_gait), None)
        if row is None:
            print(f"   no gait '{a.start_gait}' in plan.csv"); return 2
        for sv in (1, 2):
            for n in list(TUNABLE) + ["n"]:
                k_nom[f"s{sv}.{n}"] = (int(float(row[f"s{sv}_{n}"])) if n == "n"
                                       else float(row[f"s{sv}_{n}"]))
        print(f"   starting from measured gait {a.start_gait}")

    tunable = [f"s1.{n}" for n in TUNABLE]
    if a.both_servos:
        tunable += [f"s2.{n}" for n in TUNABLE]
    print(f"   tuning {len(tunable)} knobs "
          f"({'both servos' if a.both_servos else 'servo 1 only'})"
          f"   bias weight {TARGET_SPEC['bias']['w']:.1f}")

    rig = Rig(period_s=a.period)
    if not rig.wait():
        print("no joint_feedback — is the stack up and calibrated?")
        rig.stop(); return 2
    print(f"feedback live: {sorted(rig.fb)}\n")

    try:
        k_start = dict(k_nom)
        if a.seed:
            print("STAGE 3 — seeding from the stage-1 Jacobian")
            d1, d2 = _split(k_nom)
            paths, err = rig.measure(Knobs(**{**EX.NOMINAL[1].as_dict(), **d1}),
                                     Knobs(**{**EX.NOMINAL[2].as_dict(), **d2}),
                                     os.path.join(folder, "data"), "nominal_probe")
            nm = evaluate(paths, a.channel, direction) if paths else {}
            k_start = seed_from_jacobian(
                os.path.join(WORKSPACE_ROOT, a.jacobian), target, nm or {}, k_nom,
                tunable, channel=a.channel)
        elif a.start_gait:
            print(f"no Jacobian seeding — starting from measured gait {a.start_gait}")
        else:
            print("STAGE 2 — no seeding, starting from nominal")
        print()
        best, hist = optimise(rig, folder, target, k_start, tunable,
                              a.channel, direction, max_evals=a.max_evals,
                              objective=a.objective)
    finally:
        rig.stop()

    if hist:
        with open(os.path.join(folder, "history.csv"), "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=sorted({k for r in hist for k in r}))
            w.writeheader(); w.writerows(hist)
        errs = [r["err"] for r in hist]
        print(f"\n{len(hist)} evaluations   err {errs[0]:.3f} -> {min(errs):.3f}")
        if best:
            print("   best knobs: " + "  ".join(f"{k}={v:+.3f}" for k, v in best[1].items()))
        json.dump({"seeded": bool(a.seed), "curve": a.curve, "channel": a.channel,
                   "both_servos": bool(a.both_servos),
                   "objective": a.objective, "start_gait": a.start_gait,
                   "bias_weight": TARGET_SPEC["bias"]["w"],
                   "direction": direction, "n_evals": len(hist),
                   "err_start": errs[0], "err_best": min(errs),
                   "best_knobs": best[1] if best else None},
                  open(os.path.join(folder, "result.json"), "w"), indent=2)
        print(f"   wrote {folder}/history.csv and result.json")
    return 0


if __name__ == "__main__":
    sys.exit(main())
