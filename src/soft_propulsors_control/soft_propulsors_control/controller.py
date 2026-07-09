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
  WAIT      idle — no mission controlling the servos; fins track the IMU
            (roll→IMU x, pitch→IMU y) referenced to the calibration/zero pose
  CALIBRATION  driving every servo to the launch-configured zero pose
            (roll_zero / pitch_zero) — the merged home/standby pose.  Must be
            commanded at least once: nothing else moves the servos until it
            completes.  Arms IMU-follow; available on demand afterward.
  SCANNING  searching for the mission's tag(s) — sweeps one body axis at a time
  LOCKING   tag(s) first seen; confirming a stable detection before committing
  HEADING   actively swimming toward the heading (flap only; differential to turn)
  FLAPPING  directional flap — active fins oscillate pitch; idle fin + rolls
            follow IMU (forward = both fins; turn = one fin)
  PADDLING  directional paddle — both fins run the phase-locked gait; direction
            reverses fins (backward = both; turn = one fin reversed → pivot)
  LATERAL   one fin's pitch oscillates, other fin's pitch held 0; rolls → IMU
  DRIVING   hold named servos at explicit positions; the rest follow IMU
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

# Per-role calibration ZERO pose and position LIMITS are launch parameters
# (see __init__): roll/pitch zero (rad) and a symmetric ± limit (rad) around
# that zero.  The servos run in Extended Position Control Mode (hardware does
# not clamp), so the controller's software clamp — built from these params in
# _build_structure — is the SOLE position guard.  The defaults below are the
# param fallbacks; the launch file is the place to change them.
#   roll  zero = 0,  limit = ±π   → roll  clamped to [-π, +π]
#   pitch zero = 0,  limit = ±π/2 → pitch clamped to [-π/2, +π/2]
# So 'calibration' drives every servo to 0.  effort is a fraction 0..1 of the
# per-axis limit (effort 1 = swing to the limit); a gait's radian amplitude =
# effort · <axis> limit.
DEFAULT_ROLL_ZERO = 0.0
DEFAULT_PITCH_ZERO = 0.0
DEFAULT_ROLL_LIMIT = math.pi
DEFAULT_PITCH_LIMIT = math.pi / 2.0

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
    ARRIVE_DISTANCE = 0.16     # m — within this of the tag = mission achieved
    ALIGN_BEARING = 0.10       # rad — bearing considered "aligned"
    TURN_BEARING = 0.30        # rad — above this we turn (differential flap), else cruise
    STUCK_WINDOW = 6.0         # s — progress must rise within this window
    POSE_TOLERANCE = 0.05     # rad — a servo counts as "arrived" within this of target
    # After a gait's cycles finish, the controller drives the fins back to
    # neutral and waits for joint_feedback to confirm arrival before reporting
    # ACHIEVED (so crab only advances the queue once the move physically
    # completed).  This caps that wait so a non-reporting servo can't hang the
    # whole mission queue — past it we finish anyway with a warning.
    LOCO_SETTLE_TIMEOUT = 3.0  # s
    # No POSE_TIMEOUT: CALIBRATION completion is decided purely by
    # encoder feedback (_pose_settled), so a slow move can never trip a false
    # STUCK.  The controller keeps commanding and waits as long as it takes.
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
        self.declare_parameter('manual_timeout', 30.0) # s — deadman for manual_cmd; 0 = off
        # IMU-follow (idle behaviour): when no mission is controlling the servos,
        # each fin tracks the IMU — roll servo follows IMU roll (x), pitch servo
        # follows IMU pitch (y) — added on top of the calibration zero pose (IMU zero
        # = zero pose).  Gains scale/flip the mapping (negative to reverse); set to
        # 0 to freeze that axis at the zero pose.
        self.declare_parameter('imu_follow_roll_gain', 1.0)
        self.declare_parameter('imu_follow_pitch_gain', 1.0)
        # Paddle gait (sine): roll + pitch sinusoids at one frequency, pitch
        # phase-shifted.  Defaults: velocity 5, pitch phase π/2 (90°).  Per
        # command: velocity:, pitch_phase: (or pshift:), roll_effort:,
        # pitch_effort: (each 0..1, else the shared effort:, default 100%).
        self.declare_parameter('paddle_velocity', 5.0)
        self.declare_parameter('paddle_pitch_phase', math.pi / 2.0)
        self.declare_parameter('paddle_slow_factor', 0.10)   # (unused by sine paddle)
        self.declare_parameter('paddle_roll_amp', math.pi / 4.0)  # (unused by sine paddle)
        # Default number of gait cycles per locomotion command (paddle/flap/
        # lateral) when the mission doesn't say.  Per-command 'cycles:N' (or
        # 'periods:N') overrides; 0 = run forever.
        self.declare_parameter('paddle_cycles', 1)
        # Calibration ZERO pose (rad) and symmetric position LIMIT (± rad) per
        # role — the pose calibration drives to and the sole software clamp
        # (Extended Position Mode has no hardware clamp).  Change these in the
        # launch file to retune the rest pose / travel.
        self.declare_parameter('roll_zero', DEFAULT_ROLL_ZERO)
        self.declare_parameter('pitch_zero', DEFAULT_PITCH_ZERO)
        self.declare_parameter('roll_limit', DEFAULT_ROLL_LIMIT)    # roll clamped to zero ± this
        self.declare_parameter('pitch_limit', DEFAULT_PITCH_LIMIT)  # pitch clamped to zero ± this
        # Output smoothing: an exponential low-pass on every outgoing position
        # command (0 = off/passthrough; →1 = heavier smoothing/more lag).  It's
        # the weight kept on the previous command each cycle, so it filters the
        # high-frequency jitter in the setpoint stream (encoder/IMU noise, gait
        # discretisation) while barely touching the slow gait sinusoid itself.
        self.declare_parameter('command_smoothing', 0.5)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.control_rate = self.get_parameter('control_rate').value
        self.telem_decim = self.get_parameter('telemetry_decimation').value
        # Nominal stroke = velocity (peak rate, rad/s) + effort (amplitude as a
        # fraction 0..1 of the per-axis max travel).  A mission may override
        # either; freq is derived from velocity (no upper cap).
        self.nominal_velocity = self.get_parameter('gait_velocity').value
        self.nominal_effort = self.get_parameter('gait_effort').value
        self.manual_timeout = self.get_parameter('manual_timeout').value
        self.imu_follow_roll_gain = self.get_parameter('imu_follow_roll_gain').value
        self.imu_follow_pitch_gain = self.get_parameter('imu_follow_pitch_gain').value
        self.paddle_slow_factor = self.get_parameter('paddle_slow_factor').value
        self.paddle_roll_amp = self.get_parameter('paddle_roll_amp').value
        self.paddle_cycles = self.get_parameter('paddle_cycles').value
        self.paddle_velocity = self.get_parameter('paddle_velocity').value
        self.paddle_pitch_phase = self.get_parameter('paddle_pitch_phase').value
        self.paddle_roll_effort = 1.0    # per-mission (see _apply_pending_mission)
        self.paddle_pitch_effort = 1.0
        # Output low-pass state: last smoothed command per servo (rad).  Cleared
        # on every state change so a new behaviour seeds fresh (no carried lag).
        self.command_smoothing = max(0.0, min(0.95,
            float(self.get_parameter('command_smoothing').value)))
        self._cmd_smooth = {}
        # Calibration zero pose + per-axis limit (rad).  roll_max_amp/pitch_max_amp
        # are the effort full-scale (effort 1 = swing to the limit).
        self.calib_roll = self.get_parameter('roll_zero').value
        self.calib_pitch = self.get_parameter('pitch_zero').value
        self.roll_limit = self.get_parameter('roll_limit').value
        self.pitch_limit = self.get_parameter('pitch_limit').value
        self.roll_max_amp = self.roll_limit
        self.pitch_max_amp = self.pitch_limit
        # Active stroke for the current mission (set when a mission is applied):
        # cur_freq (Hz) and cur_effort (amplitude fraction 0..1).
        self.cur_freq, self.cur_effort = self._resolve_gait(None)

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
        self.operating_mode = 'extended_position'
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
        # Start idle in WAIT: hold neutral (attitude PID, gains 0 by default)
        # and do nothing until crab dispatches the first mission.  crab owns the
        # init sequence now (calibration), so there is no built-in
        # settling countdown here anymore.
        self.state = 'WAIT'
        self.mission = None          # most recent mission dict from crab
        self._mission_dirty = False  # a new mission is waiting to be applied
        self.mission_seq = 0
        self.mission_kind = 'hover'  # 'heading'|'scan'|'hover'|'flap'|'paddle'|'drive'|'tag'|'calibration'
        self.calib_target_servo_id = None  # None = all servos; set = single-servo calibration (crab's per-servo walk)
        self.drive_targets = {}      # {sid: pos} for a 'drive' mission
        self.loco_dir = 'forward'    # locomotion direction: forward|backward|turn_left|turn_right
        self.loco_gait = 'paddle'    # locomotion gait: paddle|flap
        self.loco_periods = 1.0      # gait periods to run per locomotion mission (default 1)
        self._loco_t0 = 0.0          # mission-relative gait clock origin
        self._loco_settle_t0 = None  # when the post-gait "return to neutral" wait began (None = not settling)
        self._loco_period = 0.0      # seconds per gait period (paddle: emergent; flap: 1/freq)
        self._loco_entry = 0.0       # paddle entry/exit ramp duration (neutral <-> gait pose)
        self.cur_velocity = 0.0      # current mission's velocity (paddle: pitch angular speed)
        # IMU-follow is armed only after a full calibration has completed at
        # least once this boot (IMU-follow is referenced to the calibration
        # zero pose, so it must not run until that pose has been established).
        # Until then, servos nothing is driving are simply held, not followed.
        self._calibration_done = False
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

        # Pose-move bookkeeping (used by CALIBRATION)
        self._pose_t0 = self.t0
        self._pose_achieved = False   # ACHIEVED already fired for this pose instance?

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
        # Must match crab's publisher QoS (TRANSIENT_LOCAL) — otherwise a
        # mission_cmd published before this subscription exists is lost for
        # good, which can silently stall the whole init sequence depending on
        # which process happens to finish starting up first.
        self.create_subscription(String, 'mission_cmd', self._mission_cb, latched)
        self.create_subscription(String, 'manual_cmd', self._manual_cb, 10)
        self.create_subscription(Float32MultiArray, 'joint_feedback', self._feedback_cb, 1)
        self.create_subscription(Imu, 'imu_data', self._imu_cb, 10)
        self.create_subscription(Float32MultiArray, 'apriltag_detections',
                                 self._apriltag_cb, 10)

        self.cmd_pub = self.create_publisher(Float32MultiArray, 'joint_cmd', 1)
        # TRANSIENT_LOCAL for the same reason as mission_cmd: if this
        # publishes ACHIEVED before crab's subscription is up, a volatile
        # topic would lose it for good.  No downside for a late-connecting
        # subscriber on an ongoing status stream — it just gets the latest
        # cached message immediately instead of waiting for the next one.
        self.status_pub = self.create_publisher(String, 'mission_status', latched)
        self.telem_pub = self.create_publisher(Float32MultiArray, 'telemetry', 10)

        self.create_timer(1.0 / self.control_rate, self._control_loop)

        self.get_logger().info(
            f"Controller up — awaiting robot_config. "
            f"control_rate={self.control_rate} Hz."
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
        self.nominal_velocity = cfg.get('gait_velocity', self.nominal_velocity)
        self.nominal_effort = cfg.get('gait_effort', self.nominal_effort)
        self.cur_freq, self.cur_effort = self._resolve_gait(self.mission)
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

        Entry format: [id, set_id, custom?]
          Homing is calibrated once on the servo itself (e.g. via Dynamixel
          Wizard) and never touched by this stack — Present Position 0 is
          always trusted as home.  Within each set the FIRST entry is the
          roll servo and the SECOND is the pitch servo — order is the source of
          truth for role, so only the gaits need to care which is which.
          custom (optional) is a spare per-servo value, stored and passed through.

        Position limits are NOT read from the map — they're fixed by role from
        the launch params: each roll servo is clamped to roll_zero ± roll_limit,
        each pitch servo to pitch_zero ± pitch_limit (same for both sets).
        """
        self.all_ids, self.limits, self.custom, self.fins = [], {}, {}, []
        sets = {}   # set_id -> list of sid, in map order (1st = roll, 2nd = pitch)

        for entry in actuator_map:
            sid = float(entry[0])
            set_id = int(entry[1])
            custom = float(entry[2]) if len(entry) > 2 else None

            self.all_ids.append(sid)
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
            # Limits are fixed by role: each servo is clamped to its calibration
            # zero ± the role's limit (both from launch params, same for both
            # sets).  The software clamp in _command_targets enforces them —
            # the sole guard in Extended Position Mode.
            self.limits[roll_id] = (self.calib_roll - self.roll_limit,
                                    self.calib_roll + self.roll_limit)
            self.limits[pitch_id] = (self.calib_pitch - self.pitch_limit,
                                     self.calib_pitch + self.pitch_limit)

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
        self.cur_freq, self.cur_effort = self._resolve_gait(m)
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

        # Directional locomotion (paddle/flap gaits).  Accepts the plain
        # 'paddle'/'flap' (= forward) and directional '<dir>_<gait>' forms —
        # forward / backward / turn_left / turn_right.  crab forwards the kind
        # verbatim; the CONTROLLER decides fin-selection and thrust reversal
        # from the direction (see _do_paddle / _do_flap).
        loco = self._parse_locomotion(kind)
        if loco == 'invalid':
            self.get_logger().error(
                f"Mission '{label}': '{kind}' not supported "
                f"(flap has no reverse) — hovering.")
            self.mission_kind = 'hover'
            self._enter('HOVERING', event='ALL_MISSIONS_DONE')
            return
        if loco is not None:
            direction, gait = loco
            self.loco_dir = direction
            self.loco_gait = gait
            # Run a fixed number of gait periods (default 1), then finish and
            # hand back to WAIT — enforced in _do_paddle/_do_flap via a
            # mission-relative clock starting now.
            # Number of cycles: 'cycles:N' or 'periods:N' per command, else the
            # paddle_cycles param default (1).  0 = run forever.
            self.loco_periods = float(m.get('cycles', m.get('periods', self.paddle_cycles)))
            self.cur_velocity = float(m.get('velocity', self.paddle_velocity))
            self.cur_effort = max(0.0, min(1.0, float(m.get('effort', 1.0))))
            if gait == 'paddle':
                # Sine paddle: roll + pitch sinusoids at one frequency; pitch is
                # phase-shifted.  Roll and pitch efforts are independent (each
                # falls back to the shared 'effort', default 100%).
                self.paddle_roll_effort = max(0.0, min(1.0,
                    float(m.get('roll_effort', self.cur_effort))))
                self.paddle_pitch_effort = max(0.0, min(1.0,
                    float(m.get('pitch_effort', self.cur_effort))))
                self.paddle_pitch_phase = float(m.get('pitch_phase',
                    m.get('pshift', self.paddle_pitch_phase)))
                # freq from velocity, referenced to full pitch travel so
                # 'velocity' is the peak stroke rate at 100% pitch.
                self.cur_freq = self.cur_velocity / (mc.TWO_PI * self.pitch_max_amp)
                self._loco_period = (1.0 / self.cur_freq) if self.cur_freq > 1e-6 else 0.0
            else:   # flap — sinusoid at a derived frequency
                self.cur_freq, _ = self._resolve_gait(m)
                self._loco_period = (1.0 / self.cur_freq) if self.cur_freq > 1e-6 else 0.0
            self._loco_t0 = self.get_clock().now().nanoseconds / 1e9
            self._loco_settle_t0 = None
            self.required_tags = []
            self._enter('PADDLING' if gait == 'paddle' else 'FLAPPING',
                        event='MISSION_BEGIN')
            count = 'forever' if self.loco_periods <= 0 else f'{self.loco_periods:g} period(s)'
            self.get_logger().info(
                f"Mission '{label}': {direction} {gait} × {count} — "
                f"velocity={self.cur_velocity:.3f}, effort={self.cur_effort:.2f}, "
                f"period={self._loco_period:.2f}s.")
            return

        if kind in ('lateral_left', 'lateral_right'):
            # Lateral sculling: oscillate ONE fin's pitch (sinusoidal flap),
            # hold the other fin's pitch at 0; both rolls follow the IMU (held).
            #   lateral_left  → set 1 (right) pitch oscillates
            #   lateral_right → set 2 (left)  pitch oscillates
            self.loco_dir = 'left' if kind == 'lateral_left' else 'right'
            self.loco_gait = 'lateral'
            self.loco_periods = float(m.get('cycles', m.get('periods', self.paddle_cycles)))
            self.cur_velocity = float(m.get('velocity', self.nominal_velocity))
            self.cur_effort = max(0.0, min(1.0, float(m.get('effort', self.nominal_effort))))
            self.cur_freq, _ = self._resolve_gait(m)           # flap sinusoid frequency
            self._loco_period = (1.0 / self.cur_freq) if self.cur_freq > 1e-6 else 0.0
            self._loco_t0 = self.get_clock().now().nanoseconds / 1e9
            self._loco_settle_t0 = None
            self.required_tags = []
            self._enter('LATERAL', event='MISSION_BEGIN')
            active = 'set 1 (right)' if self.loco_dir == 'left' else 'set 2 (left)'
            self.get_logger().info(
                f"Mission '{label}': lateral {self.loco_dir} — {active} pitch oscillating, "
                f"other pitch held 0 (freq={self.cur_freq:.3f} Hz, effort={self.cur_effort:.2f}).")
            return

        if kind == 'drive':
            # Drive the named servos to explicit positions and hold; every other
            # servo follows the IMU (once armed).  Runs until preempted.
            self.required_tags = []
            ids = m.get('drive_ids', [])
            pos = m.get('drive_positions', [])
            self.drive_targets = {float(i): float(p) for i, p in zip(ids, pos)}
            self._enter('DRIVING', event='MISSION_BEGIN')
            self.get_logger().info(
                f"Mission '{label}': drive {self.drive_targets} "
                f"(other servos follow IMU).")
            return

        if kind == 'calibration':
            # Calibration = the merged home/standby pose: drive to 0 (the
            # extended-mode zero the servos are re-homed to) and, once a FULL
            # calibration settles, arm IMU-follow and hand off to WAIT.
            self.required_tags = []
            self.calib_target_servo_id = m.get('target_servo_id')  # None = all servos
            # Force-reset pose bookkeeping explicitly: _enter only resets it on
            # an actual state transition, but crab's per-servo walk can dispatch
            # several calibration missions in a row without ever leaving
            # CALIBRATION, so each new mission needs a fresh achieved state.
            self._pose_achieved = False
            self._pose_t0 = self.get_clock().now().nanoseconds / 1e9
            self._enter('CALIBRATION', event='MISSION_BEGIN')
            target_desc = (f"servo {self.calib_target_servo_id}"
                           if self.calib_target_servo_id is not None else "all servos")
            self.get_logger().info(f"Mission '{label}': calibration — driving {target_desc} to 0.")
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

    @staticmethod
    def _parse_locomotion(kind):
        """
        Parse a locomotion mission kind into (direction, gait).

        Accepts the plain ``paddle`` / ``flap`` (= forward) and the directional
        ``<direction>_<gait>`` forms, with direction ∈ {forward, backward,
        turn_left, turn_right} and gait ∈ {paddle, flap}.  Returns:
          (direction, gait) — a valid locomotion command,
          'invalid'         — recognised but unsupported (backward_flap: flap
                               has no reverse), or
          None              — not a locomotion kind (caller handles it).
        """
        if kind in ('paddle', 'flap'):
            return ('forward', kind)
        direction, _, gait = kind.rpartition('_')
        if gait in ('paddle', 'flap') and direction in (
                'forward', 'backward', 'turn_left', 'turn_right'):
            if direction == 'backward' and gait == 'flap':
                return 'invalid'
            return (direction, gait)
        return None

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

        # Hard gate: until a calibration mission has completed at least once
        # this boot, NOTHING moves the servos — not manual, not any mission —
        # except the calibration mission itself.  Calibration must be commanded
        # first; it establishes the reference pose everything else builds on.
        if self.manual is not None:
            if not self._calibration_done:
                return   # manual blocked pre-calibration
            if self.manual_timeout > 0 and (now - self.manual_time) > self.manual_timeout:
                self.get_logger().warn("Manual command timed out — holding neutral.")
                self.manual = None
                self._command_targets({})
            else:
                self._do_manual(now)
            return

        # Apply a mission as soon as one arrives — crab owns all timing/settling.
        if self._mission_dirty:
            self._apply_pending_mission()

        # Before crab's very first mission ever arrives, stay fully idle —
        # command nothing at all, rather than defaulting to WAIT's IMU-follow
        # behaviour, which would drive every servo toward the calibration pose at
        # once.  The operator establishes the pose explicitly with a calibration
        # mission first (launch → calibration → idle IMU-follow), so nothing moves
        # in this window (after torque enables, before the first dispatch).
        if self.mission is None:
            return

        # Pre-calibration gate (mission side): only the calibration mission is
        # allowed to drive the servos until calibration has completed once.
        if not self._calibration_done and self.state != 'CALIBRATION':
            return

        t = now
        if self.state == 'WAIT':
            # Idle: no mission is controlling the servos, so track the IMU
            # (referenced to the calibration pose).  Missions preempt this by
            # entering their own state.
            self._do_imu_follow(t)
        elif self.state == 'CALIBRATION':
            self._do_calibration(t)
        elif self.state == 'SCANNING':
            self._do_scanning(t)
        elif self.state == 'LOCKING':
            self._do_locking(t)
        elif self.state == 'HEADING':
            self._do_heading(t)
        elif self.state == 'FLAPPING':
            self._do_flap(t)
        elif self.state == 'PADDLING':
            self._do_paddle(t)
        elif self.state == 'LATERAL':
            self._do_lateral(t)
        elif self.state == 'DRIVING':
            self._do_drive(t)
        elif self.state in ('STUCK', 'HOVERING'):
            self._command_hover()

        self._publish_status()

    # ---- States ----------------------------------------------------------

    def _calibration_targets(self, target_servo_id=None):
        """
        Calibration/zero pose for every fin: roll → roll_zero, pitch → pitch_zero
        (the launch-configured rest pose).  Both sets use the same targets.
        target_servo_id restricts to just that one servo (crab's interactive
        per-servo walk); omitted/None targets every servo (full calibration,
        e.g. sent on demand via mission_input).  This same pose is the reference
        for IMU-follow.
        """
        pose = {}
        for fin in self.fins:
            pose[fin.roll_id] = self.calib_roll
            pose[fin.pitch_id] = self.calib_pitch
        if target_servo_id is not None:
            return {target_servo_id: pose[target_servo_id]} if target_servo_id in pose else {}
        return pose

    def _do_calibration(self, t):
        """
        Drive the target servo(s) to the calibration/zero pose (0 rad) — the
        merged home/standby behaviour.  crab's calib_target_servo_id restricts
        this to one servo at a time for the interactive per-servo walk; None
        means every servo (on-demand full calibration).

        On completion of a FULL calibration, arm IMU-follow and hand off to
        WAIT so idle IMU-follow takes over — seamless because IMU-follow is
        referenced to this very zero pose, so the servos don't jump.  A
        single-servo calibration (the interactive walk) instead stays put in
        CALIBRATION, since IMU-follow would move every servo and defeat the
        one-at-a-time walk.

        Completion is decided PURELY by encoder feedback (_pose_settled) — no
        timeout; a slow move (low profile velocity) can't trip a false STUCK.
        """
        if self._pose_achieved:
            return

        targets = self._calibration_targets(self.calib_target_servo_id)
        self._command_targets(targets, only_named=self.calib_target_servo_id is not None)
        if self._pose_settled(targets):
            self._pose_achieved = True
            if self.calib_target_servo_id is None:
                # Full calibration reached: arm IMU-follow for the rest of this
                # boot and hand off to WAIT (idle IMU-follow).
                self._calibration_done = True
                self._enter('WAIT', event='ACHIEVED')   # full calibration → idle IMU-follow
            else:
                self._enter(self.state, event='ACHIEVED')   # single-servo walk: stay put

    def _pose_settled(self, targets):
        """
        True once every REPORTING targeted servo is within tolerance of its
        target.  A servo that never reports feedback (disconnected / unpowered —
        the hardware node skips it) is ignored so it can't hang calibration; we
        just need at least one servo to report and all reporters to be settled.
        """
        if not targets:
            return False
        checked = 0
        for sid, tgt in targets.items():
            if sid not in self.last_pos_time:
                continue   # no feedback — treat as absent, don't block on it
            checked += 1
            if abs(self.feedback[sid]['pos'] - tgt) > self.POSE_TOLERANCE:
                return False
        return checked > 0

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

    def _loco_settle(self, t, neutral):
        """
        End-of-gait handling for a locomotion mission.  Returns True once the
        gait's commanded cycles have elapsed — at which point the fins are
        driven to ``neutral`` (the gait's rest pose) and HELD there until
        joint_feedback confirms arrival (``_pose_settled``), and only then is
        ACHIEVED reported so crab advances the queue.  A non-reporting servo
        can't hang the queue: past LOCO_SETTLE_TIMEOUT we finish anyway with a
        warning.  A zero/near-zero period (no motion) settles immediately.

        Returns:
          False — still cycling; the caller drives the gait this tick.
          True  — cycles done; this method owns the servos now (settling or
                  finished), so the caller must NOT also command them.
        """
        if self.loco_periods <= 0:
            return False   # run forever (no auto-stop) — the default for gaits
        cycling = (self._loco_period > 1e-6
                   and (t - self._loco_t0) < self.loco_periods * self._loco_period)
        if cycling:
            return False

        # Cycles complete → hold the neutral pose and wait for encoder confirm.
        if self._loco_settle_t0 is None:
            self._loco_settle_t0 = t
        self._command_imu_filled(neutral)   # un-named servos keep following the IMU
        settled = (not neutral) or self._pose_settled(neutral)
        timed_out = (t - self._loco_settle_t0) >= self.LOCO_SETTLE_TIMEOUT
        if settled or timed_out:
            if timed_out and not settled:
                self.get_logger().warn(
                    "Gait settle timed out — no encoder confirmation of neutral; "
                    "finishing mission anyway.")
            self._enter('WAIT', event='ACHIEVED')
        return True

    def _do_flap(self, t):
        """
        Directional flap.  Only the ACTIVE fins (per self.loco_dir) oscillate
        their pitch servo; every other servo — the idle fin on a turn, plus all
        roll servos — is left un-named and so keeps following the IMU (see
        _command_imu_filled).  Flap has no clean reverse, so forward drives both
        fins and a turn drives only the fin on that side.  Runs a fixed number
        of periods (self.loco_periods), then finishes.
            forward     : both fins flap
            turn_left   : set 1 (right) flaps; set 2 idle → IMU
            turn_right  : set 2 (left)  flaps; set 1 idle → IMU
        """
        # Neutral (rest) pose for the servos this gait drives: active fins'
        # pitch back to centre (rolls are un-named → IMU-follow).
        neutral = {fin.pitch_id: self.calib_pitch
                   for fin in self.fins if self._flap_fin_active(fin)}
        if self._loco_settle(t, neutral):
            return
        tau = max(0.0, t - self._loco_t0)  # mission-relative clock (period starts at 0)
        pitch_amp = self.cur_effort * self.pitch_max_amp
        targets = {}
        for fin in self.fins:
            if not self._flap_fin_active(fin):
                continue   # idle fin → un-named → IMU-follow
            targets.update(mc.flap(
                fin.roll_id, fin.pitch_id, tau,
                self.cur_freq, pitch_amp, waveform=mc.sine,
                pitch_center=self.calib_pitch))
        self._command_imu_filled(targets)

    def _flap_fin_active(self, fin):
        """Which fins flap for the current direction (see _do_flap)."""
        if self.loco_dir == 'turn_left':
            return fin.set_id == 1     # right fin only
        if self.loco_dir == 'turn_right':
            return fin.set_id == 2     # left fin only
        return True                    # forward: both

    def _paddle_fin_reversed(self, fin):
        """
        Which fins run REVERSED paddle (thrust flipped) for the current
        direction — paddle always drives BOTH fins:
            forward     : neither reversed
            backward    : both reversed
            turn_left   : set 2 (left)  reversed  → pivot left
            turn_right  : set 1 (right) reversed  → pivot right
        """
        d = self.loco_dir
        if d == 'backward':
            return True
        if d == 'turn_left':
            return fin.set_id == 2
        if d == 'turn_right':
            return fin.set_id == 1
        return False                   # forward

    def _imu_follow_targets(self):
        """
        IMU-follow target for every fin servo, referenced to the calibration zero
        pose.  IMU roll (x) drives the roll servos, IMU pitch (y) the pitch
        servos, each added on top of that servo's calibration target so an IMU
        reading of zero holds exactly the calibration pose:

            roll_servo  = roll_zero  + roll_gain  · imu_roll
            pitch_servo = pitch_zero + pitch_gain · imu_pitch
        """
        roll, pitch, _ = self.orientation
        base = self._calibration_targets()   # per-set standby pose = the IMU-zero reference
        targets = {}
        for fin in self.fins:
            targets[fin.roll_id] = base[fin.roll_id] + self.imu_follow_roll_gain * roll
            targets[fin.pitch_id] = base[fin.pitch_id] + self.imu_follow_pitch_gain * pitch
        return targets

    def _command_imu_filled(self, explicit: dict):
        """
        Publish ``explicit`` servo targets, with EVERY servo the caller did not
        name falling back to IMU-follow (referenced to the calibration pose).

        This is the "a servo follows the IMU whenever nothing is actively
        driving it" rule: an active behaviour (a gait, a sweep, …) names only
        the servos it's moving, and every other servo keeps tracking the IMU on
        the same cycle.  e.g. flap names only the pitch servos, so the roll
        servos keep following the IMU while pitch oscillates.  Limits are
        enforced downstream by _command_targets (past a limit → clamp to it).

        IMU-follow is armed only once a full calibration has completed this boot
        (see self._calibration_done).  Before that, the un-named servos are left
        untouched (held) rather than IMU-followed — only the explicit targets
        are driven.
        """
        if not self._calibration_done:
            self._command_targets(explicit, only_named=True)
            return
        targets = self._imu_follow_targets()
        targets.update(explicit)   # active commands override IMU-follow per servo
        self._command_targets(targets)

    def _do_imu_follow(self, t):
        """Idle (WAIT): nothing is driving any servo, so all fins track the IMU."""
        self._command_imu_filled({})

    def _do_lateral(self, t):
        """
        Lateral sculling: one fin's pitch oscillates (sinusoidal flap at the
        mission velocity/effort), the other fin's pitch is held at 0, and both
        roll servos are left un-named so they follow the IMU (held near rest).
            lateral 'left'  → set 1 (right) pitch oscillates → slides left
            lateral 'right' → set 2 (left)  pitch oscillates → slides right
        Runs until preempted (periods <= 0).
        """
        # Neutral: both fins' pitch at centre (rolls un-named → IMU-follow).
        neutral = {fin.pitch_id: self.calib_pitch for fin in self.fins}
        if self._loco_settle(t, neutral):
            return
        tau = max(0.0, t - self._loco_t0)
        pitch_amp = self.cur_effort * self.pitch_max_amp
        targets = {}
        for fin in self.fins:
            active = ((self.loco_dir == 'left' and fin.set_id == 1) or
                      (self.loco_dir == 'right' and fin.set_id == 2))
            if active:
                targets.update(mc.flap(fin.roll_id, fin.pitch_id, tau,
                                       self.cur_freq, pitch_amp, waveform=mc.sine,
                                       pitch_center=self.calib_pitch))
            else:
                targets[fin.pitch_id] = self.calib_pitch     # other fin pitch held at 0
        self._command_imu_filled(targets)   # rolls (un-named) follow IMU

    def _do_drive(self, t):
        """
        Drive mission: hold the mission's explicitly-named servos at their
        commanded positions while every other servo follows the IMU (see
        _command_imu_filled).  Runs until crab preempts it with another mission.
        """
        self._command_imu_filled(self.drive_targets)

    def _do_paddle(self, t):
        """
        Directional sine paddle (see mc.paddle): roll + pitch sinusoids at one
        frequency, pitch phase-shifted.  Roll amplitude = roll_effort·roll_max,
        pitch = pitch_effort·pitch_max (independent efforts).  Direction flips
        the roll sign (forward/backward/turns).  Sinusoids are continuous, so
        multiple cycles flow smoothly; runs loco_periods cycles then finishes.
        """
        # Neutral: every driven servo (roll + pitch of all fins) back to centre.
        neutral = {}
        for fin in self.fins:
            neutral[fin.roll_id] = self.calib_roll
            neutral[fin.pitch_id] = self.calib_pitch
        if self._loco_settle(t, neutral):
            return
        tau = max(0.0, t - self._loco_t0)
        roll_amp_mag = self.paddle_roll_effort * self.roll_max_amp
        pitch_amp = self.paddle_pitch_effort * self.pitch_max_amp
        targets = {}
        for fin in self.fins:
            # Sign of roll amplitude selects thrust direction per fin.
            roll_amp = -roll_amp_mag if self._paddle_fin_reversed(fin) else roll_amp_mag
            targets.update(mc.paddle(
                fin.roll_id, fin.pitch_id, tau, self.cur_freq,
                roll_amp, pitch_amp,
                roll_center=self.calib_roll, pitch_center=self.calib_pitch,
                pitch_phase=self.paddle_pitch_phase))
        self._command_imu_filled(targets)

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
        Resolve (freq, effort) for a mission from its stroke knobs.

        The mission speaks ``velocity`` (peak stroke rate, rad/s) and ``effort``
        (a fraction 0..1 of the per-axis max travel — effort 1 swings all the
        way to the position limits; >1 is clamped to 1).  Either may be absent,
        falling back to the nominal config.  Peak stroke rate is 2π·f·A with A
        the roll sweep amplitude (= effort·roll_limit), so frequency is:

            freq = velocity / (2π · effort · roll_limit)

        There is no upper cap on freq — velocity may be arbitrarily large.  The
        gait methods scale ``effort`` into a per-axis radian amplitude.
        """
        mission = mission or {}
        velocity = float(mission.get('velocity', self.nominal_velocity))
        effort = float(mission.get('effort', self.nominal_effort))
        effort = max(0.0, min(1.0, effort))          # fraction of max amplitude
        roll_amp = effort * self.roll_max_amp        # radian amplitude of the roll sweep
        freq = velocity / (mc.TWO_PI * roll_amp) if roll_amp > 1e-6 else 0.0
        return freq, effort

    # ---- Manual teleop (lab/bench, off the mission path) ------------------

    def _manual_cb(self, msg: String):
        """
        Parse a manual command and arm the override.  Token grammar (raw freq/amp
        in rad, intended for bench testing — limits still apply on publish):

            gait set:1 freq:1.0 amp:0.6 [wave:sine] [type:flap|paddle]
                                                 (paddle: roll sweeps ±amp, pitch
                                                  feathers in unison, phase-locked)
            gait id:3 rate:0.2 span:1.0          (single-servo sweep)
            drive id:3 pos:0.5                   (static hold; pos: or vel:)
            drive id:1,3 pos:3.14159             (same target, multiple servos)
            drive id:1,2 pos:3.14159,0.785       (per-servo targets, comma-paired
                                                   with id: — every servo not
                                                   named here is driven to 0.0)
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
                'vel': _num('vel', 1.0),   # paddle: pitch angular speed (rad/s)
            }
        elif 'gait' in flags and 'id' in tokens:
            manual = {
                'type': 'sweep', 'id': float(tokens['id']),
                'rate': _num('rate', 0.2), 'span': _num('span', 1.0),
            }
        elif 'drive' in flags and 'id' in tokens:
            ids = [float(x) for x in tokens['id'].split(',')]
            value_key = 'pos' if 'pos' in tokens else 'vel'
            vals = [float(x) for x in tokens.get(value_key, '0.0').split(',')]
            if len(vals) == 1 and len(ids) > 1:
                vals = vals * len(ids)   # one value broadcast to every id
            if len(vals) != len(ids):
                self.get_logger().error(
                    f"Manual drive: {len(ids)} id(s) but {len(vals)} {value_key} "
                    f"value(s) — counts must match.")
                return
            manual = {'type': 'drive', 'targets': dict(zip(ids, vals))}
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
        only_named = False
        if m['type'] == 'drive':
            targets = dict(m['targets'])
        elif m['type'] == 'sweep':
            targets.update(mc.sweep(m['id'], t, m['rate'], m['span']))
        elif m['kind'] == 'paddle':
            # paddle drives BOTH roll (sweep) and pitch (feather), shaped stroke.
            # Roll amp from the direction param (forward sign); pitch amp = manual
            # amp; vel is the pitch angular speed.  Centred on the calibration pose.
            targets.update(mc.paddle(m['roll'], m['pitch'], t,
                                     m['vel'], self.paddle_roll_amp, m['amp'],
                                     roll_center=self.calib_roll,
                                     pitch_center=self.calib_pitch,
                                     slow_factor=self.paddle_slow_factor))
            only_named = True
        else:   # flap
            # flap omits the roll servo (held at its current position); publish
            # only what the gait names so the un-commanded roll isn't reset.
            targets.update(mc.flap(m['roll'], m['pitch'], t,
                                   m['freq'], m['amp'], waveform=m['wave'],
                                   pitch_center=self.calib_pitch))
            only_named = True
        self._command_targets(targets, only_named=only_named)

    def _command_heading(self, t, bearing):
        """
        Drive toward ``target_bearing`` using the flap gait only (paddle and the
        adaptive optimizer come later).  Large bearing error → differential flap
        (the fins flap in opposite phase) so the body yaws toward the heading;
        small error → synchronous flap for forward thrust.

        flap oscillates only the pitch servos; each fin's roll servo is not
        driven here, so it keeps following the IMU (see _command_imu_filled).
        """
        pitch_amp = self.cur_effort * self.pitch_max_amp
        targets = {}
        if abs(bearing) > self.TURN_BEARING:
            # Turning manoeuvre: opposed flapping biased by bearing sign
            for idx, fin in enumerate(self.fins):
                side = 1.0 if idx % 2 == 0 else -1.0
                turn = side * math.copysign(1.0, bearing)
                phase = 0.0 if turn >= 0 else math.pi
                targets.update(mc.flap(
                    fin.roll_id, fin.pitch_id, t,
                    self.cur_freq, pitch_amp, phase=phase, waveform=mc.sine,
                    pitch_center=self.calib_pitch))
        else:
            # Cruise: synchronous flapping for forward thrust
            for fin in self.fins:
                targets.update(mc.flap(
                    fin.roll_id, fin.pitch_id, t,
                    self.cur_freq, pitch_amp, waveform=mc.sine,
                    pitch_center=self.calib_pitch))
        self._command_imu_filled(targets)

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
        # Whichever axis isn't being swept this pass is left un-named, so it
        # keeps following the IMU (see _command_imu_filled).
        self._command_imu_filled(targets)

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

    def _command_targets(self, targets: dict, only_named: bool = False):
        """
        Apply limits to a {servo_id: value} map and publish joint_cmd.

        By default, servos not named in ``targets`` are held at neutral (0.0
        = the servo's own Homing Offset zero) — what most missions want (e.g.
        a fin-only gait leaving other axes at rest).  Pass only_named=True to
        instead touch *only* the servos in ``targets`` and leave every other
        servo's command completely untouched this cycle (not even defaulted
        to 0.0) — used by the interactive per-servo calibration walk,
        so targeting one servo never implicitly moves any other.

        Position-mode servos already sitting within POSE_TOLERANCE of their
        computed target are dropped from the outgoing command entirely — a
        global "don't re-send a position we're already at" rule.  Torque
        stays on and the servo holds where it is via its own onboard PID, so
        omitting it from this cycle's packet is safe; this just avoids
        redundant bus writes/jitter for a servo that was never asked to move.
        Velocity-mode servos are exempt (rates aren't "arrived at and held").
        """
        if not self.all_ids:
            return
        if self.operating_mode == 'velocity':
            mode_code = MODE_VELOCITY
        else:
            mode_code = MODE_POSITION

        servo_ids = list(targets.keys()) if only_named else self.all_ids

        ids, modes, values = [], [], []
        for sid in servo_ids:
            raw = targets.get(sid, 0.0)
            if mode_code == MODE_POSITION:
                # Position targets are relative to the servo's homed zero.
                # Limits are ±inf for now (clamping removed), so this passes
                # the target through unchanged; restore finite ROLL_LIMIT/
                # PITCH_LIMIT to re-enable the clamp.
                lo, hi = self.limits.get(sid, (-math.inf, math.inf))
                val = max(lo, min(hi, raw))
                # Output low-pass: ease each command toward its target so the
                # high-frequency jitter in the setpoint stream (encoder/IMU
                # noise, gait discretisation) is filtered out.  Clamp again so
                # the eased value can't ride outside the limits.
                a = self.command_smoothing
                if a > 0.0:
                    prev = self._cmd_smooth.get(sid)
                    val = val if prev is None else (a * prev + (1.0 - a) * val)
                    val = max(lo, min(hi, val))
                    self._cmd_smooth[sid] = val
                current = self.feedback.get(sid, {}).get('pos')
                if (sid in self.last_pos_time and current is not None
                        and abs(current - val) <= self.POSE_TOLERANCE):
                    continue   # already there — skip this servo this cycle
            else:
                # Velocity targets are absolute rad/s
                val = raw
            ids.append(sid)
            modes.append(mode_code)
            values.append(val)

        if not ids:
            return   # every targeted servo is already at its commanded position
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
            # Reseed the output smoother on any behaviour change so lag from the
            # previous behaviour's command stream can't carry over.
            self._cmd_smooth.clear()
            # Start the attitude hold fresh whenever a hover-driven state begins,
            # so integrator/derivative state from an earlier hold can't carry over.
            if new_state in ('WAIT', 'STUCK', 'HOVERING'):
                self.roll_pid.reset()
                self.pitch_pid.reset()
            if new_state == 'CALIBRATION':
                self._pose_t0 = self.get_clock().now().nanoseconds / 1e9
                self._pose_achieved = False
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
