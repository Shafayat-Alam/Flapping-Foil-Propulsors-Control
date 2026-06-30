"""
controller.py — Autonomous Execution Engine
============================================
The controller is the robot's reactive brain.  It owns every real-time sensor
(servo feedback, IMU, AprilTag detections), runs a state machine that drives the
robot toward whatever mission crab handed it, and reports high-level progress
back up.  crab never sees raw sensor data — only interpreted mission status.

Division of responsibility
---------------------------
  crab        decides *what* to do        (mission goals, retries, human prompts)
  controller  decides *how* to do it       (gait, direction, when to reacquire)
  hardware    just executes / publishes    (servos, IMU, camera — no logic)

State machine
-------------
  BOOT      counting down ``startup_delay`` s from launch (deployment settling)
  WAIT      configured but no active mission yet
  SCANNING  searching for the mission's tag(s) — sweeps one body axis at a time
  LOCKING   tag(s) first seen; confirming a stable detection before committing
  HEADING   actively swimming toward the heading (flap only; differential to turn)
  STUCK     zero progress over the stuck window — hover and let crab decide
  HOVERING  level, upright, IMU-stabilised hold (between missions / all done)

Topics
------
Subscribes : robot_config        (std_msgs/String, transient_local) from crab
             mission_cmd          (std_msgs/String) from crab
             manual_cmd           (std_msgs/String) lab teleop — overrides missions
             joint_feedback       (std_msgs/Float32MultiArray) from servo hw
             imu_data             (sensor_msgs/Imu) from IMU hw
             apriltag_detections  (std_msgs/Float32MultiArray) from perception
Publishes  : joint_cmd            (std_msgs/Float32MultiArray) to servo hw
             mission_status       (std_msgs/String) to crab
             telemetry            (std_msgs/Float32MultiArray) for logging

Wire formats
------------
joint_cmd           : [id0..idN-1, mode0..modeN-1, val0..valN-1]
                      mode 3.0 = position, 1.0 = velocity
joint_feedback      : [id, mode, pos_rad, vel_rad_s, curr_A, volt_V] per servo
apriltag_detections : [tag_id, distance_m, bearing_rad, elevation_rad, valid]
                      per tag (valid 1.0/0.0; bearing +left, elevation +up)
mission_status      : JSON — see _publish_status()
telemetry           : [seq, ts, goal,pos,vel,curr,volt per servo]
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Imu
import json
import math
import time
from collections import deque

from soft_propulsors_control import motion_command as mc


# Mode codes shared with the hardware interface
MODE_POSITION = 3.0
MODE_VELOCITY = 1.0

# Heading vocabulary.  Cardinals point at their own tag; intercardinals are the
# bisector of two adjacent cardinal tags (both must be in view to compute one).
CARDINALS = ('N', 'E', 'S', 'W')
INTERCARDINALS = {'NE': ('N', 'E'), 'SE': ('S', 'E'),
                  'SW': ('S', 'W'), 'NW': ('N', 'W')}


class Fin:
    """One propulsor: a roll servo (orients the fin) + a pitch servo (sweeps it)."""

    def __init__(self, set_id, roll_id, pitch_id):
        self.set_id = set_id
        self.roll_id = roll_id
        self.pitch_id = pitch_id


class PID:
    """
    Scalar PID controller with clamped integral (anti-windup) and gap-safe dt.

    The setpoint is folded into the ``error`` the caller passes in (error =
    target - measured), so this class stays unit-agnostic.  ``update`` takes the
    current monotonic time and derives dt itself; the first call, or any call
    after a gap longer than ``reset_dt`` (e.g. the loop spent a while in another
    state), skips integration and the derivative for that step so a stale term
    can't spike the output.  With all gains 0 the output is identically 0.
    """

    def __init__(self, kp=0.0, ki=0.0, kd=0.0,
                 out_limit=None, i_limit=None, reset_dt=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.out_limit = out_limit      # clamp on the returned command (None = off)
        self.i_limit = i_limit          # clamp on the integrator (anti-windup)
        self.reset_dt = reset_dt        # s — gap beyond which we restart cleanly
        self.reset()

    def reset(self):
        self._i = 0.0
        self._prev_err = None
        self._prev_t = None

    def update(self, error, t):
        dt = 0.0 if self._prev_t is None else (t - self._prev_t)
        self._prev_t = t

        d = 0.0
        if 0.0 < dt <= self.reset_dt:
            self._i += error * dt
            if self.i_limit is not None:
                self._i = max(-self.i_limit, min(self.i_limit, self._i))
            if self._prev_err is not None:
                d = self.kd * (error - self._prev_err) / dt
        # else: first sample or a gap — hold the integrator, skip the derivative
        self._prev_err = error

        out = self.kp * error + self.ki * self._i + d
        if self.out_limit is not None:
            out = max(-self.out_limit, min(self.out_limit, out))
        return out


class Controller(Node):

    # ---- Behaviour tuning (robot-agnostic defaults; refine on hardware) ----
    STABLE_FRAMES = 5          # consecutive valid detections to lock a tag
    ARRIVE_DISTANCE = 0.30     # m — within this of the tag = mission achieved
    ALIGN_BEARING = 0.10       # rad — bearing considered "aligned"
    TURN_BEARING = 0.30        # rad — above this we paddle to turn, else flap to cruise
    STUCK_WINDOW = 6.0         # s — progress must rise within this window
    STATUS_PERIOD = 0.2        # s — minimum spacing between status publishes (5 Hz)
    SCAN_AXIS_PERIOD = 8.0     # s — time spent sweeping each axis before rotating axes
    SCAN_RATE = 0.15           # Hz — slow sweep frequency while scanning
    SCAN_SPAN = 1.2            # rad — peak-to-peak sweep range while scanning
    ATTITUDE_OUT_LIMIT = math.pi / 2.0  # rad — cap on the hover PID's corrective offset
    ATTITUDE_I_LIMIT = 1.0     # rad·s — integral clamp on the hover PID (anti-windup)

    def __init__(self):
        super().__init__('controller')

        # ------------------------------------------------------------------
        # Parameters (outer-loop PID + rates; gait nominal values).  Most
        # structural config arrives from crab via robot_config.
        # ------------------------------------------------------------------
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('control_rate', 400.0)
        self.declare_parameter('telemetry_decimation', 1)
        self.declare_parameter('gait_velocity', 3.77)  # nominal peak stroke rate (rad/s, 2π·f·A)
        self.declare_parameter('gait_effort', 0.6)     # nominal stroke amplitude (rad)
        self.declare_parameter('max_freq', 3.0)        # Hz — cap when deriving freq from velocity
        self.declare_parameter('startup_delay', 10.0)  # fallback; crab config wins
        self.declare_parameter('manual_timeout', 30.0) # s — deadman for manual_cmd; 0 = off

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.control_rate = self.get_parameter('control_rate').value
        self.telem_decim = self.get_parameter('telemetry_decimation').value
        # Nominal stroke = velocity (peak rate, rad/s) + effort (amplitude, rad).
        # A mission may override either; freq is derived as velocity / (2π·amp).
        self.nominal_velocity = self.get_parameter('gait_velocity').value
        self.nominal_effort = self.get_parameter('gait_effort').value
        self.max_freq = self.get_parameter('max_freq').value
        self.startup_delay = self.get_parameter('startup_delay').value
        self.manual_timeout = self.get_parameter('manual_timeout').value
        # Active stroke for the current mission (set when a mission is applied).
        self.cur_freq, self.cur_amp = self._resolve_gait(None)

        # Manual teleop override (lab/bench).  When set, it takes over the servo
        # output and the mission state machine is skipped until 'stop'/timeout.
        self.manual = None           # parsed manual command, or None
        self.manual_time = 0.0       # monotonic-ish time of the last manual_cmd

        # Attitude-hold PID — one instance per axis so roll and pitch integrate
        # independently; both share the tuned kp/ki/kd.  Drives the hover/STUCK
        # level-and-upright correction (see _command_hover).  All gains default
        # to 0, so stabilisation is off until they're tuned up on hardware.
        self.roll_pid = PID(self.kp, self.ki, self.kd,
                            out_limit=self.ATTITUDE_OUT_LIMIT,
                            i_limit=self.ATTITUDE_I_LIMIT)
        self.pitch_pid = PID(self.kp, self.ki, self.kd,
                             out_limit=self.ATTITUDE_OUT_LIMIT,
                             i_limit=self.ATTITUDE_I_LIMIT)

        # ------------------------------------------------------------------
        # Robot structure (populated from robot_config, or left empty)
        # ------------------------------------------------------------------
        self.configured = False
        self.operating_mode = 'position'
        self.all_ids = []
        self.limits = {}             # sid -> (min, max) (rad)
        self.custom = {}             # sid -> spare per-servo value (unused for now)
        self.cardinal_map = {}       # "N"/"E"/"S"/"W" -> tag id (from robot_config)
        self.fins = []               # list[Fin]

        # ------------------------------------------------------------------
        # Live sensor state
        # ------------------------------------------------------------------
        self.feedback = {}           # sid -> {"pos","vel","curr","volt"}
        self.last_pos = {}           # sid -> last position (for velocity-mode derivative)
        self.last_pos_time = {}      # sid -> timestamp of last position
        self.orientation = (0.0, 0.0, 0.0)   # roll, pitch, yaw (rad)
        self.detections = {}         # tag_id -> {"dist","bearing","elev","stamp"}

        # ------------------------------------------------------------------
        # Mission + state machine
        # ------------------------------------------------------------------
        self.state = 'BOOT'
        self.mission = None          # most recent mission dict from crab
        self._mission_dirty = False  # a new mission is waiting to be applied
        self.mission_seq = 0
        self.mission_kind = 'hover'  # 'heading' | 'scan' | 'hover' | 'tag'
        self.required_tags = []      # tag ids that must all be in view to head
        self.arrive_distance = self.ARRIVE_DISTANCE  # per-mission arrival distance (m)
        self.target_bearing = 0.0    # commanded heading bearing (rad); 0 = straight ahead
        self.t0 = self.get_clock().now().nanoseconds / 1e9

        # Heading bookkeeping
        self.dist0 = None            # distance at lock (for progress normalisation)
        self.bearing0 = None         # |bearing| at lock
        self.progress = 0.0          # 0..1
        self._best_progress = 0.0    # highest progress seen this heading
        self._best_progress_time = 0.0  # when that high was last reached
        self._stable_count = 0       # consecutive valid frames during LOCKING

        # Scan bookkeeping
        self._scan_axis = 0          # 0=X,1=Y,2=Z body axis being swept
        self._scan_axis_t0 = self.t0

        # Status throttle
        self._last_status_time = 0.0
        self._last_event = None

        self.sample_counter = 0
        self._telem_buf = deque(maxlen=200)

        # ------------------------------------------------------------------
        # ROS2 interfaces
        # ------------------------------------------------------------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, 'robot_config', self._config_cb, latched)
        self.create_subscription(String, 'mission_cmd', self._mission_cb, 10)
        self.create_subscription(String, 'manual_cmd', self._manual_cb, 10)
        self.create_subscription(Float32MultiArray, 'joint_feedback', self._feedback_cb, 1)
        self.create_subscription(Imu, 'imu_data', self._imu_cb, 10)
        self.create_subscription(Float32MultiArray, 'apriltag_detections',
                                 self._apriltag_cb, 10)

        self.cmd_pub = self.create_publisher(Float32MultiArray, 'joint_cmd', 1)
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        self.telem_pub = self.create_publisher(Float32MultiArray, 'telemetry', 10)

        self.create_timer(1.0 / self.control_rate, self._control_loop)

        self.get_logger().info(
            f"Controller up — awaiting robot_config. "
            f"control_rate={self.control_rate} Hz, startup_delay={self.startup_delay}s."
        )

    # ======================================================================
    # Config + mission intake
    # ======================================================================

    def _config_cb(self, msg: String):
        """Receive the one-shot robot configuration broadcast by crab."""
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad robot_config JSON: {e}")
            return

        self.operating_mode = cfg.get('operating_mode', 'position')
        self.control_rate = cfg.get('control_rate', self.control_rate)
        self.startup_delay = cfg.get('startup_delay', self.startup_delay)
        self.nominal_velocity = cfg.get('gait_velocity', self.nominal_velocity)
        self.nominal_effort = cfg.get('gait_effort', self.nominal_effort)
        self.cur_freq, self.cur_amp = self._resolve_gait(self.mission)
        self.cardinal_map = {str(k).upper(): int(v)
                             for k, v in cfg.get('cardinal_map', {}).items()}

        self._build_structure(cfg.get('actuator_map', []))
        self.configured = True
        self.get_logger().info(
            f"Configured: {len(self.all_ids)} servos, {len(self.fins)} fins, "
            f"mode={self.operating_mode}."
        )

    def _build_structure(self, actuator_map):
        """
        Turn the actuator map into servo tables and fin (roll/pitch) pairs.

        Entry format: [id, homing_offset, set_id, min, max, custom?]
          homing_offset is written to the servo by the hardware node (the
          controller does not apply it).  Within each set the FIRST entry is the
          roll servo and the SECOND is the pitch servo — order is the source of
          truth for role, so only the gaits need to care which is which.
          custom (optional) is a spare per-servo value, stored and passed through.
        """
        self.all_ids, self.limits, self.custom, self.fins = [], {}, {}, []
        sets = {}   # set_id -> list of sid, in map order (1st = roll, 2nd = pitch)

        for entry in actuator_map:
            sid = float(entry[0])
            set_id = int(entry[2])
            lo = float(entry[3]) if len(entry) > 3 else -3.14
            hi = float(entry[4]) if len(entry) > 4 else 3.14
            custom = float(entry[5]) if len(entry) > 5 else None

            self.all_ids.append(sid)
            self.limits[sid] = (lo, hi)
            if custom is not None:
                self.custom[sid] = custom
            sets.setdefault(set_id, []).append(sid)

        self.all_ids.sort()

        for set_id, members in sets.items():
            if len(members) < 2:
                self.get_logger().warn(
                    f"Set {set_id} has {len(members)} servo(s); a fin needs a "
                    f"roll + pitch pair — skipping.")
                continue
            roll_id, pitch_id = members[0], members[1]   # positional convention
            self.fins.append(Fin(set_id, roll_id, pitch_id))

        # Initialise feedback bookkeeping
        for sid in self.all_ids:
            self.feedback.setdefault(sid, {"pos": 0.0, "vel": 0.0, "curr": 0.0, "volt": 0.0})

    def _mission_cb(self, msg: String):
        """
        Store a new mission from crab. It always supersedes the current one, but
        is only *applied* once the BOOT deployment countdown has elapsed (see
        _apply_pending_mission), so missions sent at launch don't skip settling.
        """
        try:
            m = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad mission_cmd JSON: {e}")
            return
        self.mission = m
        self._mission_dirty = True

    def _apply_pending_mission(self):
        """
        Begin executing the most recently received mission.  The controller owns
        all sequencing: a heading mission first scans for the tag(s) it needs,
        then heads, with no further commands from crab.  Standalone scan/hover
        missions just hold that behaviour until crab preempts them.
        """
        self._mission_dirty = False
        m = self.mission
        label = m.get('label', '?')
        self.cur_freq, self.cur_amp = self._resolve_gait(m)
        self.arrive_distance = float(m.get('distance', self.ARRIVE_DISTANCE))
        self.mission_seq += 1

        # Mission kind: explicit 'kind' from crab, else infer from legacy fields.
        kind = m.get('kind')
        if kind is None:
            target = m.get('target_tag_id', -1)
            kind = ('tag' if (target is not None and target >= 0
                              and str(label).upper() != 'HOVER') else 'hover')
        self.mission_kind = kind

        if kind == 'hover':
            self.required_tags = []
            self._enter('HOVERING', event='ALL_MISSIONS_DONE')
            self.get_logger().info(f"Mission '{label}': hover / idle.")
            return

        self._reset_heading()
        if kind == 'scan':
            self.required_tags = []
            self._enter('SCANNING', event='MISSION_BEGIN')
            self.get_logger().info(f"Mission '{label}': scanning.")
        elif kind == 'heading':
            self.required_tags = self._resolve_heading(m.get('heading'))
            if not self.required_tags:
                self.get_logger().error(
                    f"Mission '{label}': heading {m.get('heading')!r} unresolved "
                    f"(cardinal_map={self.cardinal_map}) — hovering.")
                self.mission_kind = 'hover'
                self._enter('HOVERING', event='ALL_MISSIONS_DONE')
                return
            self._enter('SCANNING', event='MISSION_BEGIN')
            self.get_logger().info(
                f"Mission '{label}': heading {m.get('heading')} "
                f"(tags {self.required_tags}), arrive <= {self.arrive_distance} m.")
        else:   # legacy single-tag mission
            target = int(m.get('target_tag_id', -1))
            self.required_tags = [target]
            self._enter('SCANNING', event='MISSION_BEGIN')
            self.get_logger().info(f"Mission '{label}': seeking tag {target}.")

    def _resolve_heading(self, name):
        """Map a heading (N..NW) to the tag id(s) needed, via the cardinal map."""
        name = str(name or '').upper()
        if name in CARDINALS and name in self.cardinal_map:
            return [self.cardinal_map[name]]
        if name in INTERCARDINALS:
            a, b = INTERCARDINALS[name]
            if a in self.cardinal_map and b in self.cardinal_map:
                return [self.cardinal_map[a], self.cardinal_map[b]]
        return []

    # ======================================================================
    # Sensor callbacks
    # ======================================================================

    def _feedback_cb(self, msg: Float32MultiArray):
        """Decode joint_feedback: [id, mode, pos, vel, curr, volt] per servo."""
        data = msg.data
        now = self.get_clock().now().nanoseconds / 1e9
        for i in range(0, len(data) - 5, 6):
            sid = data[i]
            if sid not in self.feedback:
                continue
            pos = data[i + 2]
            vel = data[i + 3]
            # Derive velocity from position if the sensor doesn't report it
            if vel == 0.0 and sid in self.last_pos:
                dt = now - self.last_pos_time.get(sid, now)
                if dt > 1e-6:
                    vel = (pos - self.last_pos[sid]) / dt
            self.feedback[sid].update({
                "pos": pos, "vel": vel, "curr": data[i + 4], "volt": data[i + 5]
            })
            self.last_pos[sid] = pos
            self.last_pos_time[sid] = now

    def _imu_cb(self, msg: Imu):
        """Convert the IMU quaternion to roll/pitch/yaw (rad)."""
        q = msg.orientation
        self.orientation = self._quat_to_euler(q.x, q.y, q.z, q.w)

    def _apriltag_cb(self, msg: Float32MultiArray):
        """Store the latest detections: [tag_id, dist, bearing, elev, valid] each."""
        data = msg.data
        now = self.get_clock().now().nanoseconds / 1e9
        for i in range(0, len(data) - 4, 5):
            valid = data[i + 4]
            if valid < 0.5:
                continue
            tag_id = int(data[i])
            self.detections[tag_id] = {
                "dist": data[i + 1], "bearing": data[i + 2],
                "elev": data[i + 3], "stamp": now,
            }

    @staticmethod
    def _quat_to_euler(x, y, z, w):
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sp)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return (roll, pitch, yaw)

    def _fresh_detection(self, tag_id):
        """Latest non-stale detection of one tag, or None."""
        det = self.detections.get(int(tag_id))
        now = self.get_clock().now().nanoseconds / 1e9
        if det and (now - det["stamp"]) < 0.5:   # ignore stale detections
            return det
        return None

    def _heading_state(self):
        """
        Resolve the commanded heading from the required tags, or None if any are
        not currently in view.  Returns ``(target_bearing, ref_distance)``:
          * cardinal     → that tag's bearing and distance
          * intercardinal → bisector of the two bearings; distance to the nearer
        """
        dets = []
        for tid in self.required_tags:
            det = self._fresh_detection(tid)
            if det is None:
                return None
            dets.append(det)
        if not dets:
            return None
        bearing = sum(d["bearing"] for d in dets) / len(dets)
        ref_dist = min(d["dist"] for d in dets)
        return bearing, ref_dist

    # ======================================================================
    # Real-time control loop
    # ======================================================================

    def _control_loop(self):
        if not self.configured or not self.fins:
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Manual teleop override (lab/bench) takes precedence over the mission
        # state machine.  A deadman holds neutral if commands stop arriving.
        if self.manual is not None:
            if self.manual_timeout > 0 and (now - self.manual_time) > self.manual_timeout:
                self.get_logger().warn("Manual command timed out — holding neutral.")
                self.manual = None
                self._command_targets({})
            else:
                self._do_manual(now)
            return

        # Deployment settling countdown — nothing starts until it elapses
        if self.state == 'BOOT':
            if (now - self.t0) >= self.startup_delay:
                self._enter('WAIT', event='READY')
            else:
                self._command_hover()
                return

        # Apply a mission that arrived (possibly during BOOT) now that we're past it
        if self._mission_dirty:
            self._apply_pending_mission()

        t = now
        if self.state == 'WAIT':
            self._command_hover()
        elif self.state == 'SCANNING':
            self._do_scanning(t)
        elif self.state == 'LOCKING':
            self._do_locking(t)
        elif self.state == 'HEADING':
            self._do_heading(t)
        elif self.state in ('STUCK', 'HOVERING'):
            self._command_hover()

        self._publish_status()

    # ---- States ----------------------------------------------------------

    def _do_scanning(self, t):
        """
        Sweep one body axis at a time, rotating axes.  A pure scan mission (no
        required tags) sweeps forever; a heading mission commits to LOCKING only
        once every tag it needs is simultaneously in view.
        """
        if self.required_tags and self._heading_state() is not None:
            self._stable_count = 1
            self._enter('LOCKING', event='TAG_DETECTED')
            return

        # Rotate which axis we sweep every SCAN_AXIS_PERIOD seconds
        if (t - self._scan_axis_t0) >= self.SCAN_AXIS_PERIOD:
            self._scan_axis = (self._scan_axis + 1) % 3
            self._scan_axis_t0 = t
        self._command_scan(t, self._scan_axis)

    def _do_locking(self, t):
        """Require STABLE_FRAMES consecutive full detections before committing."""
        hs = self._heading_state()
        if hs is None:
            # Lost a required tag during confirmation — rescan, keep moving
            self._stable_count = 0
            self._enter('SCANNING', event='TAG_LOST')
            return

        bearing, ref_dist = hs
        self.target_bearing = bearing
        self._stable_count += 1
        # Keep gently orienting toward the heading while confirming
        self._command_heading(t, bearing)
        if self._stable_count >= self.STABLE_FRAMES:
            self.dist0 = max(ref_dist, 1e-3)
            self.bearing0 = max(abs(bearing), 1e-3)
            self.progress = 0.0
            self._best_progress = 0.0
            self._best_progress_time = t
            self._enter('HEADING', event='TAG_ACQUIRED')

    def _do_heading(self, t):
        """Swim toward the heading, track progress, watch for arrival or stall."""
        hs = self._heading_state()
        if hs is None:
            # A required tag dropped out — reacquire it before continuing
            self._enter('SCANNING', event='TAG_LOST')
            return

        bearing, ref_dist = hs
        self.target_bearing = bearing
        self.progress = self._compute_progress(bearing, ref_dist)
        self._command_heading(t, bearing)

        # Arrived?  Facing the heading and within the mission's arrival distance.
        # Hand back to WAIT (crab dispatches the next mission, or HOVER if idle).
        if ref_dist <= self.arrive_distance and abs(bearing) <= self.ALIGN_BEARING:
            self.progress = 1.0
            self._enter('WAIT', event='ACHIEVED')
            return

        # Stuck?  Any positive progress resets the clock; no improvement for the
        # whole stuck window means we're not getting closer at all.
        if self.progress > self._best_progress:
            self._best_progress = self.progress
            self._best_progress_time = t
        elif (t - self._best_progress_time) >= self.STUCK_WINDOW:
            self._enter('STUCK', event='STUCK')

    # ---- Progress --------------------------------------------------------

    def _compute_progress(self, bearing, ref_dist):
        """
        Mission completion fraction, weighting distance over angle:
            progress = 0.7·(1 - dist/dist0) + 0.3·(1 - |bearing|/bearing0)
        Clamped to [0, 1] and normalised against the values captured at lock.
        """
        if self.dist0 is None or self.dist0 < 1e-6:
            return 0.0
        dist_term = 1.0 - ref_dist / self.dist0
        bearing_term = 1.0 - abs(bearing) / self.bearing0
        p = 0.7 * dist_term + 0.3 * bearing_term
        return max(0.0, min(1.0, p))

    # ======================================================================
    # Motion generation — composes motion_command primitives onto the fins
    # ======================================================================

    def _resolve_gait(self, mission):
        """
        Resolve (freq, amp) for a mission from its stroke knobs.

        The mission speaks ``velocity`` (peak stroke rate, rad/s) and ``effort``
        (amplitude, rad); either may be absent, falling back to the nominal
        config.  Since peak stroke rate is ``2π·f·A``, frequency is derived:

            amp  = effort
            freq = velocity / (2π·amp)        (capped at max_freq)

        So velocity sets how fast the stroke moves and effort sets how big it is,
        with frequency taking whatever value reaches that speed at that size.
        """
        mission = mission or {}
        velocity = float(mission.get('velocity', self.nominal_velocity))
        amp = float(mission.get('effort', self.nominal_effort))
        amp = max(1e-3, amp)
        freq = velocity / (mc.TWO_PI * amp)
        freq = max(0.0, min(self.max_freq, freq))
        return freq, amp

    # ---- Manual teleop (lab/bench, off the mission path) ------------------

    def _manual_cb(self, msg: String):
        """
        Parse a manual command and arm the override.  Token grammar (raw freq/amp
        in rad, intended for bench testing — limits still apply on publish):

            gait set:1 freq:1.0 amp:0.6 [wave:sine] [type:flap|paddle]
            gait id:3 rate:0.2 span:1.0          (single-servo sweep)
            drive id:3 pos:0.5                   (static hold; pos: or vel:)
            stop                                 (release manual, hand back)

        While armed, the control loop runs this instead of the mission machine.
        """
        tokens, flags = {}, set()
        for part in msg.data.strip().split():
            if ':' in part:
                k, v = part.split(':', 1)
                tokens[k.lower()] = v
            else:
                flags.add(part.lower())

        if 'stop' in flags:
            self.manual = None
            self._command_targets({})   # neutral hold once
            self.get_logger().info("Manual: stop — released to mission flow.")
            return

        def _num(key, default=None):
            try:
                return float(tokens[key])
            except (KeyError, ValueError):
                return default

        manual = None
        if 'gait' in flags and 'set' in tokens:
            fin = next((f for f in self.fins if f.set_id == int(float(tokens['set']))), None)
            if fin is None:
                self.get_logger().error(f"Manual gait: no fin for set {tokens['set']!r}.")
                return
            manual = {
                'type': 'gait',
                'kind': 'paddle' if tokens.get('type', 'flap').lower() == 'paddle' else 'flap',
                'roll': fin.roll_id, 'pitch': fin.pitch_id,
                'freq': _num('freq', 1.0), 'amp': _num('amp', 0.3),
                'wave': tokens.get('wave', 'sine'),
            }
        elif 'gait' in flags and 'id' in tokens:
            manual = {
                'type': 'sweep', 'id': float(tokens['id']),
                'rate': _num('rate', 0.2), 'span': _num('span', 1.0),
            }
        elif 'drive' in flags and 'id' in tokens:
            value = _num('pos', _num('vel', 0.0))
            manual = {'type': 'drive', 'targets': {float(tokens['id']): value}}
        else:
            self.get_logger().error(f"Manual: unrecognised command {msg.data!r}.")
            return

        self.manual = manual
        self.manual_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"Manual: {manual['type']} armed — {msg.data.strip()!r}.")

    def _do_manual(self, t):
        """Generate one tick of the armed manual command and publish it."""
        m = self.manual
        targets = {}
        if m['type'] == 'drive':
            targets = dict(m['targets'])
        elif m['type'] == 'sweep':
            targets.update(mc.sweep(m['id'], t, m['rate'], m['span']))
        else:   # gait
            fn = mc.paddle if m['kind'] == 'paddle' else mc.flap
            targets.update(fn(m['roll'], m['pitch'], t,
                              m['freq'], m['amp'], waveform=m['wave']))
        self._command_targets(targets)

    def _command_heading(self, t, bearing):
        """
        Drive toward ``target_bearing`` using the flap gait only (paddle and the
        adaptive optimizer come later).  Large bearing error → differential flap
        (the fins flap in opposite phase) so the body yaws toward the heading;
        small error → synchronous flap for broadside forward thrust.
        """
        targets = {}
        if abs(bearing) > self.TURN_BEARING:
            # Turning manoeuvre: opposed flapping biased by bearing sign
            for idx, fin in enumerate(self.fins):
                side = 1.0 if idx % 2 == 0 else -1.0
                turn = side * math.copysign(1.0, bearing)
                phase = 0.0 if turn >= 0 else math.pi
                targets.update(mc.flap(
                    fin.roll_id, fin.pitch_id, t,
                    self.cur_freq, self.cur_amp, phase=phase, waveform=mc.sine))
        else:
            # Cruise: synchronous flapping for broadside forward thrust
            for fin in self.fins:
                targets.update(mc.flap(
                    fin.roll_id, fin.pitch_id, t,
                    self.cur_freq, self.cur_amp, waveform=mc.sine))
        self._command_targets(targets)

    def _command_scan(self, t, axis):
        """
        Sweep one body axis while holding the others, searching for the tag.
        Axis→fin mapping (tune to the platform's hydrodynamics):
          0 (yaw)   : fins sweep opposed   → body rotates about vertical
          1 (pitch) : fins sweep together  → body pitches up/down
          2 (roll)  : fins sweep opposed on roll servos → body rolls
        """
        targets = {}
        for idx, fin in enumerate(self.fins):
            side = 1.0 if idx % 2 == 0 else -1.0
            if axis == 0:        # yaw: opposed pitch sweep, roll held neutral
                targets.update(mc.sweep(fin.pitch_id, t, self.SCAN_RATE,
                                        side * self.SCAN_SPAN))
                targets.update(mc.drive(fin.roll_id, 0.0))
            elif axis == 1:      # pitch: synchronous pitch sweep
                targets.update(mc.sweep(fin.pitch_id, t, self.SCAN_RATE, self.SCAN_SPAN))
            else:                # roll: opposed roll sweep
                targets.update(mc.sweep(fin.roll_id, t, self.SCAN_RATE,
                                        side * self.SCAN_SPAN))
        self._command_targets(targets)

    def _command_hover(self):
        """
        Hold level and upright with a per-axis PID on IMU roll/pitch error
        (target = level).  Roll is countered with an opposed (differential) fin
        offset, pitch with a common offset — a best-effort self-righting hold.

        The error fed to each PID is ``0 - measured`` so a positive output drives
        the body back toward level.  With all gains 0 the output is 0 and the
        fins simply sit at neutral; raise kp/ki/kd to engage stabilisation.
        """
        now = self.get_clock().now().nanoseconds / 1e9
        roll, pitch, _ = self.orientation
        roll_cmd = self.roll_pid.update(-roll, now)
        pitch_cmd = self.pitch_pid.update(-pitch, now)
        targets = {}
        for idx, fin in enumerate(self.fins):
            side = 1.0 if idx % 2 == 0 else -1.0
            targets.update(mc.drive(fin.roll_id, side * roll_cmd))
            targets.update(mc.drive(fin.pitch_id, pitch_cmd))
        self._command_targets(targets)

    def _command_targets(self, targets: dict):
        """
        Apply limits to a {servo_id: value} map and publish joint_cmd.  Servos
        not named in ``targets`` are held at neutral (0.0 = the homed zero; the
        hardware node owns the homing offset, so the controller adds nothing).
        """
        if not self.all_ids:
            return
        if self.operating_mode == 'velocity':
            mode_code = MODE_VELOCITY
        else:
            mode_code = MODE_POSITION

        ids, modes, values = [], [], []
        for sid in self.all_ids:
            raw = targets.get(sid, 0.0)
            if mode_code == MODE_POSITION:
                # Position targets are relative to the servo's homed zero
                lo, hi = self.limits.get(sid, (-3.14, 3.14))
                val = max(lo, min(hi, raw))
            else:
                # Velocity targets are absolute rad/s
                val = raw
            ids.append(sid)
            modes.append(mode_code)
            values.append(val)

        self._publish_cmd(ids, modes, values)
        self._buffer_telemetry(ids, modes, values)

    # ======================================================================
    # State helpers + publishers
    # ======================================================================

    def _reset_heading(self):
        now = self.get_clock().now().nanoseconds / 1e9
        self.dist0 = None
        self.bearing0 = None
        self.progress = 0.0
        self._best_progress = 0.0
        self._best_progress_time = now
        self._stable_count = 0
        self._scan_axis = 0
        self._scan_axis_t0 = now

    def _enter(self, new_state, event=None):
        if new_state != self.state:
            self.get_logger().info(f"State: {self.state} -> {new_state}"
                                   + (f" ({event})" if event else ""))
            # Start the attitude hold fresh whenever a hover-driven state begins,
            # so integrator/derivative state from an earlier hold can't carry over.
            if new_state in ('WAIT', 'STUCK', 'HOVERING'):
                self.roll_pid.reset()
                self.pitch_pid.reset()
        self.state = new_state
        if event:
            self._last_event = event
            self._publish_status(force=True)

    def _publish_status(self, force=False):
        """Send interpreted mission status to crab (throttled, plus on events)."""
        now = self.get_clock().now().nanoseconds / 1e9
        if not force and (now - self._last_status_time) < self.STATUS_PERIOD:
            return
        self._last_status_time = now

        roll, pitch, yaw = self.orientation
        status = {
            "label": self.mission.get('label') if self.mission else None,
            "kind": self.mission_kind,
            "heading": self.mission.get('heading') if self.mission else None,
            "target_tag_id": self.mission.get('target_tag_id') if self.mission else None,
            "state": self.state,
            "progress": round(self.progress * 100.0, 3),
            "orientation": [round(roll, 4), round(pitch, 4), round(yaw, 4)],
            "event": self._last_event,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        self._last_event = None   # events are one-shot

    def _publish_cmd(self, ids, modes, values):
        msg = Float32MultiArray()
        msg.data = ([float(i) for i in ids]
                    + [float(m) for m in modes]
                    + [float(v) for v in values])
        self.cmd_pub.publish(msg)

    def _buffer_telemetry(self, ids, modes, values):
        self._telem_buf.append({
            'seq': float(self.mission_seq), 'ts': float(self.sample_counter),
            'ids': ids, 'modes': modes, 'vals': values,
            'fb': {sid: self.feedback[sid].copy() for sid in ids if sid in self.feedback},
        })
        self.sample_counter += 1
        if self.telem_decim and self.sample_counter % self.telem_decim == 0:
            self._publish_telemetry()

    def _publish_telemetry(self):
        if not self._telem_buf:
            return
        e = self._telem_buf[-1]
        payload = [e['seq'], e['ts']]
        for sid, val in zip(e['ids'], e['vals']):
            fb = e['fb'].get(sid, {"pos": 0.0, "vel": 0.0, "curr": 0.0, "volt": 0.0})
            payload.extend([float(val), fb["pos"], fb["vel"], fb["curr"], fb["volt"]])
        msg = Float32MultiArray()
        msg.data = payload
        self.telem_pub.publish(msg)

    # ------------------------------------------------------------------
    def destroy_node(self):
        """Hold neutral on shutdown."""
        if self.all_ids:
            self._command_hover()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
