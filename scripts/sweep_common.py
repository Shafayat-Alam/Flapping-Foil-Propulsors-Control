"""
sweep_common.py — shared machinery for the sweep scripts
========================================================
Used by sweep_phase.py / sweep_amplitude.py / sweep_frequency.py.

Responsibilities:
  * interactive input prompts
  * one persistent ROS publisher (no `pub --once` drops) + ACHIEVED wait
  * record_session start/stop, then split_missions
  * CSV loading helpers for plotter.py (the live matplotlib viewer)

No analysis, no optimum-finding — this only runs sweeps and organizes results.
"""

import os, sys, csv, glob, math, json, time, signal, subprocess, threading

# NOTE: rclpy is imported lazily inside start_ros() so that plotter.py can use
# the CSV/metric helpers below without a sourced ROS environment.

HERE     = os.path.dirname(os.path.abspath(__file__))
RECORDER = os.path.join(HERE, "record_session.py")
SPLITTER = os.path.join(HERE, "split_missions.py")

D2R = math.pi / 180.0
INTER_MISSION_DELAY = 20.0     # s between mission commands — the prompted default

# Metrics plotted per mission.  (column, source, axis-label)
# Per request: frequency, position, velocity, current, voltage (both servos)
# + Fx/Fy/Fz/Tx/Ty/Tz.  's*_mode' is excluded (constant, not a curve).
SERVO_METRICS = [
    ("frequency_hz",    "feedback", "commanded frequency (Hz)"),
    ("s1_position_rad", "feedback", "pitch position (rad)"),
    ("s1_velocity_rps", "feedback", "pitch velocity (rad/s)"),
    ("s1_current_a",    "feedback", "pitch current (A)"),
    ("s1_voltage_v",    "feedback", "pitch voltage (V)"),
    ("s2_position_rad", "feedback", "heave position (rad)"),
    ("s2_velocity_rps", "feedback", "heave velocity (rad/s)"),
    ("s2_current_a",    "feedback", "heave current (A)"),
    ("s2_voltage_v",    "feedback", "heave voltage (V)"),
]
FORCE_METRICS = [
    ("Fx", "loadcell", "Fx — thrust (N)"),
    ("Fy", "loadcell", "Fy — lateral (N)"),
    ("Fz", "loadcell", "Fz — heave (N)"),
    ("Tx", "loadcell", "Tx (N·m)"),
    ("Ty", "loadcell", "Ty (N·m)"),
    ("Tz", "loadcell", "Tz (N·m)"),
]
ALL_METRICS = SERVO_METRICS + FORCE_METRICS


# ===========================================================================
# Input prompts
# ===========================================================================
def ask_float(label, default=None):
    while True:
        d = f" [{default}]" if default is not None else ""
        s = input(f"  {label}{d}: ").strip()
        if not s and default is not None:
            return float(default)
        try:
            return float(s)
        except ValueError:
            print("    -> enter a number")


def ask_int(label, default=None):
    while True:
        d = f" [{default}]" if default is not None else ""
        s = input(f"  {label}{d}: ").strip()
        if not s and default is not None:
            return int(default)
        try:
            return int(s)
        except ValueError:
            print("    -> enter an integer")


def ask_choice(label, options):
    while True:
        s = input(f"  {label} ({'/'.join(options)}): ").strip().lower()
        if s in options:
            return s
        print(f"    -> pick one of {options}")


def ask_outdir(label="Output path (folder will be created)"):
    """Ask for a path+folder name; create it fresh. Everything lands here."""
    while True:
        s = input(f"  {label}: ").strip()
        if not s:
            print("    -> enter a path"); continue
        p = os.path.abspath(os.path.expanduser(s))
        if os.path.exists(p) and os.listdir(p):
            if input(f"    '{p}' exists and is not empty — use anyway? (y/n): "
                     ).strip().lower() not in ("y", "yes"):
                continue
        os.makedirs(p, exist_ok=True)
        return p


def ask_delay():
    """Settling delay between mission commands.  The water has to come to rest
    between runs — otherwise the next mission's forces start in the previous
    one's wake, and the load-cell readings are contaminated."""
    return ask_float("delay between mission commands (s)", INTER_MISSION_DELAY)


def build_points(start, increment, n_samples):
    """start, start+inc, ... n_samples values."""
    return [round(start + i * increment, 6) for i in range(n_samples)]


# ===========================================================================
# ROS: one persistent publisher + ACHIEVED wait
# ===========================================================================
def _make_sweep_node_class():
    """Defined lazily: subclassing Node requires rclpy, which plotter.py lacks."""
    from rclpy.node import Node
    from std_msgs.msg import String

    class SweepNode(Node):
        def __init__(self):
            super().__init__("sweep_runner")
            self.pub = self.create_publisher(String, "mission_input", 10)
            self.create_subscription(String, "mission_status", self._cb, 50)
            self._done = threading.Event()
            self._awaiting = None

        def _cb(self, msg):
            try:
                d = json.loads(msg.data)
            except Exception:
                return
            if (d.get("event") == "ACHIEVED" and self._awaiting
                    and d.get("label") == self._awaiting):
                self._done.set()

        def send(self, line, label, timeout):
            # Publish ONCE. mission_input is RELIABLE QoS and this node has been
            # connected to crab since start_ros()'s post-discovery sleep, well
            # before the first send() — so DDS handles delivery; no need to
            # resend at the application level.  crab.py has no dedup on
            # identical mission_input lines, so resending here used to queue
            # the SAME mission multiple times: it would run, report ACHIEVED,
            # then a duplicate copy still in crab's queue would dispatch again,
            # snapping tau back to 0 mid-experiment — a real, physical jerk on
            # the servos, not a comms nicety.
            self._awaiting = label
            self._done.clear()
            m = String(); m.data = line
            self.pub.publish(m)
            ok = self._done.wait(timeout=timeout)
            self._awaiting = None
            if not ok:
                print(f"    ! '{label}' no ACHIEVED within {timeout:.0f}s — continuing")
            return ok

    return SweepNode


def start_ros():
    import rclpy
    rclpy.init()
    node = _make_sweep_node_class()()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)
    return node


def stop_ros(node):
    import rclpy
    node.destroy_node()
    rclpy.shutdown()


# ===========================================================================
# Recording / splitting
# ===========================================================================
def paddle_line(label, frequency, pitch_amp, heave_amp, phase, cycles,
                freq_ratio=1.0, pitch_k=0.0):
    # float format handles inf natively -> "inf"; float("inf") parses it back.
    return (f"forward_paddle frequency:{frequency:.6f} pitch_amp:{pitch_amp:.6f} "
            f"heave_amp:{heave_amp:.6f} phase:{phase:.6f} freq_ratio:{freq_ratio:.6f} "
            f"pitch_k:{pitch_k:.6f} cycles:{cycles} label:{label}")


RECORDING_COMPLETE_MARKER = "RECORDING_COMPLETE"


def run_missions(node, outdir, points, cycles, delay=INTER_MISSION_DELAY, split=True):
    """
    points: list of dicts {label, frequency, pitch_amp, heave_amp, phase}
    Records ONE session for the whole sweep, runs each mission `delay` seconds
    apart, then splits into per-mission folders under outdir (unless
    split=False, deferring that to a later batch pass — see split_pending.py).

    Either way, once the recorder exits cleanly a RECORDING_COMPLETE marker is
    written in `raw/` — this is what resume/dedup logic should check for, NOT
    "does the block folder have anything in it". A block killed mid-recording
    has a raw/ folder (bag partially written) but no marker, so it's correctly
    seen as incomplete and re-run; a block that finished recording but hasn't
    been split yet still has the marker, so it's correctly seen as done and
    skipped, even though there are no PH_* mission folders yet.
    """
    raw = os.path.join(outdir, "raw")
    os.makedirs(raw, exist_ok=True)
    print(f"\n  recording -> {raw}")
    rec = subprocess.Popen([sys.executable, RECORDER, raw],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(3.0)
    try:
        for i, p in enumerate(points):
            line = paddle_line(p["label"], p["frequency"], p["pitch_amp"],
                               p["heave_amp"], p["phase"], cycles,
                               p.get("freq_ratio", 1.0), p.get("pitch_k", 0.0))
            dur = cycles / p["frequency"] if p["frequency"] > 1e-6 else 15.0
            fr_tag = f" ratio={p['freq_ratio']:.3f}" if p.get("freq_ratio", 1.0) != 1.0 else ""
            k_tag = f" k={p['pitch_k']}" if p.get("pitch_k", 0.0) != 0.0 else ""
            print(f"  [{i+1}/{len(points)}] {p['label']}: f={p['frequency']:.4f}Hz{fr_tag}{k_tag} "
                  f"pitch={p['pitch_amp']:.4f} heave={p['heave_amp']:.4f} "
                  f"phase={p['phase']:.4f}")
            node.send(line, p["label"], timeout=dur * 1.6 + 8.0)
            if i < len(points) - 1:
                time.sleep(delay)
        time.sleep(2.0)
    finally:
        rec.send_signal(signal.SIGINT)
        try:
            # Generous: record_session exports the whole bag to CSV on SIGINT,
            # which for a full stage is large — a short timeout here kills the
            # export mid-write and the split then finds no missions.
            rec.wait(timeout=900)
        except subprocess.TimeoutExpired:
            rec.kill(); rec.wait()
        time.sleep(1.0)

    # Recorder has exited cleanly (SIGINT export finished, or we killed+waited
    # on a timeout — either way the bag/session.csv on disk are whatever they
    # are, not going to change further). Mark this block's data as captured,
    # independent of whether splitting happens now or later.
    open(os.path.join(raw, RECORDING_COMPLETE_MARKER), "w").close()

    if split:
        print("  splitting missions...")
        subprocess.run([sys.executable, SPLITTER, raw, "--base-dir", outdir],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        print("  recording complete, split deferred (run split_pending.py later)")


# ===========================================================================
# CSV helpers (used by plotter.py)
# ===========================================================================
def _read(path):
    try:
        return list(csv.DictReader(open(path)))
    except OSError:
        return []


def _num(rows, key):
    out = []
    for r in rows:
        v = r.get(key, "")
        try:
            out.append(float(v))
        except (TypeError, ValueError):
            out.append(None)
    return out


def _title(label, cmd):
    bits = [f"{label}"]
    for k, disp in (("frequency", "f"), ("pitch_amp", "pitch"),
                    ("heave_amp", "heave"), ("phase", "phase"), ("cycles", "cyc")):
        v = cmd.get(f"cmd.{k}")
        if v not in (None, ""):
            try:
                fv = float(v)
                bits.append(f"{disp}={fv:g}" if k == "cycles" else f"{disp}={fv:.4f}")
            except ValueError:
                bits.append(f"{disp}={v}")
    return "  ".join(bits)


def mission_dirs(outdir, points):
    """Map label -> split mission folder (or None if the mission didn't run)."""
    found = {}
    for p in points:
        hits = glob.glob(os.path.join(outdir, f"{p['label']}_*"))
        found[p["label"]] = hits[0] if hits else None
    return found


def load_metric(mdir, label, col, src):
    """(time, values, title) for one metric of one mission; ([],[],'') if absent."""
    rows = _read(os.path.join(
        mdir, f"{label}.csv" if src == "feedback" else f"{label}_loadcell.csv"))
    if not rows or col not in rows[0]:
        return [], [], ""
    t, y = _num(rows, "time_s"), _num(rows, col)
    if not any(v is not None for v in y):
        return [], [], ""
    fb = _read(os.path.join(mdir, f"{label}.csv"))
    return t, y, _title(label, fb[0] if fb else {})


def write_info(outdir, kind, constants, swept, points, cycles):
    """Record exactly what was run, so the folder is self-describing."""
    lines = [f"sweep: {kind}", f"cycles per command: {cycles}",
             f"samples: {len(points)}", "", "constants:"]
    lines += [f"  {k} = {v}" for k, v in constants.items()]
    lines += ["", f"swept: {swept}", ""]
    lines += ["missions:"]
    for p in points:
        lines.append(f"  {p['label']}: f={p['frequency']:.6f} "
                     f"pitch_amp={p['pitch_amp']:.6f} heave_amp={p['heave_amp']:.6f} "
                     f"phase={p['phase']:.6f}")
    open(os.path.join(outdir, "sweep_info.txt"), "w").write("\n".join(lines) + "\n")


def banner(kind):
    print(f"\n=== {kind} sweep ===")
    print("  (launch must already be up and OPERATIONAL, loadcell streaming)\n")
