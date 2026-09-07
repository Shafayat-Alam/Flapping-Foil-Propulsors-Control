"""
Dynamixel_XW430_T200_interface.py — Write-Only Hardware Interface
=================================================================
This node is the exclusive owner of the physical serial bus.  It translates
ROS2 Float32MultiArray joint commands into Dynamixel protocol 2.0 packets
and writes them to the servos.

Topics
------
Subscribes : robot_config       (String, transient_local) from crab — read only
                                 for the servo id list, to auto-initialize
                                 hardware as soon as config arrives.  Homing is
                                 calibrated once on the servo itself (e.g. via
                                 Dynamixel Wizard) and NEVER touched here —
                                 this node never reads or writes the Homing
                                 Offset register.
             joint_cmd          (Float32MultiArray) [ids.. modes.. vals..]
Publishes  : joint_feedback     (Float32MultiArray) — control feedback, 6/servo:
                                 [id, mode, pos_rad, vel_rad_s, curr_A, volt_V]
             servo_diagnostics  (Float32MultiArray) — full present-data block,
                                 13 values/servo (see SERVO_DIAG_STRIDE below):
                                 [id, mode, pwm_raw, current_A, velocity_rad_s,
                                  position_rad, vel_traj_rad_s, pos_traj_rad,
                                  voltage_V, temperature_C, moving, moving_status,
                                  hw_error]
                                 hw_error is the latched Hardware Error Status
                                 byte (bit0 input-voltage, bit2 overheating,
                                 bit3 motor-encoder, bit4 electrical-shock,
                                 bit5 overload), polled at a reduced rate.

Configuration / Reconfiguration
--------------------------------
Hardware is configured ONCE at startup. No reconfiguration during runtime.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32MultiArray, String
from dynamixel_sdk import (
    PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
    DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD, COMM_SUCCESS
)
import ctypes
import json
import math
import time
import signal
import sys


def _to_int32(value: int) -> int:
    """Reinterpret an unsigned 32-bit Dynamixel register value as signed."""
    return ctypes.c_int32(int(value)).value


def _pack4(value: int) -> list:
    """Split a 32-bit integer into the 4-byte list expected by addParam."""
    v = ctypes.c_uint32(value).value
    return [
        DXL_LOBYTE(DXL_LOWORD(v)),
        DXL_HIBYTE(DXL_LOWORD(v)),
        DXL_LOBYTE(DXL_HIWORD(v)),
        DXL_HIBYTE(DXL_HIWORD(v)),
    ]


class DynamixelXW430Interface(Node):
    # Control-table addresses (Protocol 2.0 / XW430-T200)
    ADDR_BAUD_RATE     = 8
    ADDR_OPERATING_MODE = 11
    ADDR_MAX_POSITION_LIMIT = 48   # EEPROM, 4 bytes — hard upper clamp (ticks)
    ADDR_MIN_POSITION_LIMIT = 52   # EEPROM, 4 bytes — hard lower clamp (ticks)
    ADDR_CURRENT_LIMIT = 38
    ADDR_TORQUE_ENABLE = 64
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_HARDWARE_ERROR = 70
    ADDR_VELOCITY_LIMIT = 44   # EEPROM, 4 bytes — hard slew cap (0.229 rev/min/unit, max 1023)
    ADDR_PROFILE_ACCELERATION = 108
    ADDR_PROFILE_VELOCITY = 112
    ADDR_GOAL_VELOCITY = 104
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_DATA  = 126

    # Full present-data block read in one shot: Moving(122) .. Temperature(146).
    PRESENT_BLOCK_START = 122
    PRESENT_BLOCK_LEN   = 25
    HW_ERROR_DECIMATION = 50    # poll Hardware Error Status every Nth loop
    # A servo that stops answering the feedback read for this many consecutive
    # loops is declared LOST (power loss / disconnect / brownout) — enough to
    # ride through the odd dropped packet without a false alarm.
    FB_MISS_THRESHOLD = 5

    SERVO_DIAG_STRIDE = 13      # values per servo in servo_diagnostics

    # Hard position clamps written to each servo's Min/Max Position Limit
    # registers so it physically cannot be driven past its role's range,
    # independent of (and as a backstop to) the controller's software clamp.
    # Ticks: roll spans a full turn (0..2π → 0..4095), pitch a half turn
    # (0..π → 0..2048).  first servo in a set = roll, second = pitch.
    PITCH_TICK_LIMITS  = (0, 4095)
    HEAVE_TICK_LIMITS = (0, 2048)
    # Set 1 pitch uses the same positive range as set 2 — this hardware rejects
    # a negative Min Position Limit in standard Position Control Mode, so the
    # reversal isn't done via negative positions.
    HEAVE_TICK_LIMITS_SET1 = HEAVE_TICK_LIMITS

    BAUD_MAP = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6, 4500000: 7}
    TICKS_PER_RAD    = 4096.0 / (2.0 * math.pi)
    RADS_TO_VEL_UNIT = 1.0 / (0.229 * (2.0 * math.pi / 60.0))
    VEL_UNIT_TO_RADS = 0.229 * (2.0 * math.pi / 60.0)
    CURRENT_UNIT_A   = 0.00269  # A per current register unit
    VOLTAGE_UNIT_V   = 0.1      # V per input-voltage register unit

    def __init__(self):
        super().__init__('servo_actuator')

        # Serial port for the Dynamixel bus.  Default is the FTDI adapter's
        # stable by-id path (survives replugs / ttyUSB0<->ttyUSB1 renumbering);
        # override in the launch file for a different adapter.
        self.declare_parameter(
            'port', '/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT9MIR5U-if00-port0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('hardware_rate', 100.0)
        self.declare_parameter('current_limit', 800)
        self.declare_parameter('servo_velocity_i_gain', 1920)
        self.declare_parameter('servo_velocity_p_gain', 100)
        self.declare_parameter('servo_position_d_gain', 0)
        self.declare_parameter('servo_position_i_gain', 0)
        self.declare_parameter('servo_position_p_gain', 900)
        # Per-servo Position PID override, as a JSON map keyed by servo id, e.g.
        #   '{"1": {"p": 950, "i": 0, "d": 400}, "2": {"p": 700, "d": 200}}'
        # Any key omitted for a servo falls back to the scalar params above.
        # Empty string = no overrides (all servos use the scalars).  This param
        # is watched at runtime (see _on_set_params): setting it while the node
        # is up re-writes the Position P/I/D Gain RAM registers live — no
        # relaunch, torque stays on — which is how pid_tuner.py tunes gains.
        self.declare_parameter('position_gain_overrides', '')
        # Velocity Limit (register 44) is NOT written here — it's set by hand
        # on the servo (Dynamixel Wizard).  This node trusts whatever the servo
        # already has.
        # Caps Position Control Mode's point-to-point trajectory (raw register
        # units, 0 = unlimited/max speed — that's the current "snap to target
        # instantly" behaviour).  Small positive values make home_state/
        # standby moves slow and visually inspectable.
        self.declare_parameter('profile_velocity', 30)
        self.declare_parameter('profile_acceleration', 10)
        # Servos whose rotation direction is physically reversed (e.g. a
        # mirror-mounted fin).  Their goal position/velocity is negated on the
        # way out and their present position/velocity negated on the way in, so
        # the rest of the stack commands every servo in one logical frame.
        # JSON list of ids, e.g. '[3, 4]' for the left fin.
        self.declare_parameter('reverse_servos', '[]')
        try:
            self.reversed_ids = set(int(x) for x in
                                    json.loads(self.get_parameter('reverse_servos').value))
        except (ValueError, TypeError):
            self.reversed_ids = set()

        port_name = self.get_parameter('port').value
        init_baud = self.get_parameter('baudrate').value
        hw_rate = self.get_parameter('hardware_rate').value

        self.port = PortHandler(port_name)
        self.packet_handler = PacketHandler(2.0)
        self._port_name = port_name
        self._init_baud = init_baud
        self._port_warned = False
        # Opening is non-fatal: a missing adapter logs an error and the node
        # keeps running, retrying the port so it never crashes the launch.
        self.port_open = self._open_port()

        self.current_baudrate = init_baud
        self.latest_command = None
        self.active_ids = []
        self.id_modes = {}
        # Operating Mode (control-table reg 11) written to every servo at setup:
        #   1 = Velocity, 3 = Position (single-turn), 4 = Extended Position
        #   (multi-turn, allows negative goals).  Default is Extended Position;
        #   _config_cb overrides from robot_config's operating_mode string.
        self.op_mode_code = 4
        # Per-servo dropout detection: consecutive feedback-read misses, and the
        # set of servos currently flagged LOST (edge-logged once each way).
        self._fb_miss = {}
        self._servo_lost = set()
        self.position_tick_limits = {}   # sid -> (min_tick, max_tick), from robot_config
        self.is_configured = False
        self.is_configuring = False
        self.pos_sync_writer = None
        self.vel_sync_writer = None
        # Set by the parameter callback when a Position-gain param changes; the
        # hardware loop re-writes the gains at the top of its next cycle so the
        # write happens in the loop's execution context (no port contention).
        self._gains_dirty = False
        self.add_on_set_parameters_callback(self._on_set_params)

        # crab broadcasts the actuator map on a latched topic; we read it only
        # for the servo id list, to auto-initialize hardware as soon as config
        # arrives.  Homing is calibrated once on the servo itself (e.g. via
        # Dynamixel Wizard) — this node never reads or writes that register.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.config_sub = self.create_subscription(
            String, 'robot_config', self._config_cb, latched)

        self.joint_sub = self.create_subscription(Float32MultiArray, 'joint_cmd', self._cmd_cb, 1)

        self.feedback_pub = self.create_publisher(Float32MultiArray, 'joint_feedback', 1)
        self.diag_pub = self.create_publisher(Float32MultiArray, 'servo_diagnostics', 1)

        self._loop_count = 0
        self.hw_error = {}            # sid -> latched Hardware Error Status byte
        self.hw_error_read = None

        hw_period = 1.0 / hw_rate
        self.hw_timer = self.create_timer(hw_period, self._hardware_loop)

        # Config held until the port is open (adapter may arrive after launch):
        # the last robot_config we heard, replayed to _setup_hardware once the
        # port comes up.
        self._pending_ids = None
        self._pending_modes = None
        self._port_retry_timer = self.create_timer(2.0, self._retry_open_port)

        # Register SIGINT handler for immediate torque disable
        signal.signal(signal.SIGINT, self._emergency_stop)

        self.get_logger().info(f"DynamixelXW430Interface online — port={port_name} baud={init_baud} hw_rate={hw_rate} Hz")

    def _open_port(self):
        """Open the serial port; non-fatal. Returns True on success.

        A missing adapter (by-id path absent) makes serial.Serial raise inside
        openPort(); we catch it so the node stays alive and retries later
        instead of crashing the whole launch.
        """
        try:
            if not self.port.openPort():
                raise RuntimeError("openPort() returned False")
            if not self.port.setBaudRate(self._init_baud):
                raise RuntimeError("setBaudRate() returned False")
            # Let the USB-serial adapter and bus settle before the first packet —
            # writing immediately after openPort() is a common source of dropped
            # first-packets (and therefore intermittently-missing torque enable).
            time.sleep(0.25)
            self.get_logger().info(
                f"Serial port {self._port_name} open @ {self._init_baud} baud.")
            self._port_warned = False
            return True
        except Exception as e:
            if not self._port_warned:
                self.get_logger().error(
                    f"Serial port {self._port_name} not available ({e}) — running "
                    f"WITHOUT servos; will keep retrying.")
                self._port_warned = True
            try:
                self.port.closePort()
            except Exception:
                pass
            return False

    def _retry_open_port(self):
        """Periodic reopen while the port is down; re-run setup once it's back."""
        if self.port_open:
            return
        if not self._open_port():
            return
        self.port_open = True
        # Port came back — replay the last config so the servos get configured.
        if self._pending_ids and not self.is_configured and not self.is_configuring:
            self._setup_hardware(list(self._pending_ids), list(self._pending_modes),
                                 self.get_parameter('baudrate').value)

    def _config_cb(self, msg: String):
        """
        Read the actuator map from crab and auto-initialize hardware (torque
        on, gains) as soon as config arrives — don't wait for the first
        mission's joint_cmd to bring the servos up.  Homing Offset is never
        touched: it's calibrated once on the servo itself and this node just
        trusts whatever Present Position 0 already means to the hardware.
        """
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad robot_config JSON: {e}")
            return

        ids = []
        sets = {}   # set_id -> [sid, ...] in map order (1st = roll, 2nd = pitch)
        for entry in cfg.get('actuator_map', []):
            try:
                sid = int(round(entry[0]))
                set_id = int(round(entry[1]))
            except (IndexError, ValueError, TypeError):
                continue
            ids.append(sid)
            sets.setdefault(set_id, []).append(sid)

        # Per-servo hard position clamps by role: first in a set = roll, second
        # = pitch.  Applied to the Min/Max Position Limit registers at setup so
        # the servo hardware itself refuses any goal outside its range.  Set 1
        # pitch is reversed (negative range) to mirror set 2.
        self.position_tick_limits = {}
        for set_id, members in sets.items():
            for i, sid in enumerate(members):
                if i == 1:   # pitch
                    self.position_tick_limits[sid] = (
                        self.HEAVE_TICK_LIMITS_SET1 if set_id == 1 else self.HEAVE_TICK_LIMITS)
                else:        # roll
                    self.position_tick_limits[sid] = self.PITCH_TICK_LIMITS

        if self.is_configured or self.is_configuring or not ids:
            return

        op = cfg.get('operating_mode', 'extended_position')
        self.op_mode_code = {'velocity': 1, 'position': 3,
                             'extended_position': 4}.get(op, 4)
        modes = [self.op_mode_code] * len(ids)
        # Remember the config so a late/reconnected port can be configured by
        # the retry timer even if it wasn't open when config first arrived.
        self._pending_ids, self._pending_modes = ids, modes
        if not self.port_open:
            self.get_logger().warn(
                "robot_config received but serial port is down — will configure "
                "servos once the port comes up.")
            return
        self._setup_hardware(ids, modes, self.get_parameter('baudrate').value)

    def _resolve_gains(self, sid):
        """(pos_p, pos_i, pos_d) for a servo: per-servo override if present in
        position_gain_overrides, else the scalar servo_position_*_gain params."""
        p = self.get_parameter('servo_position_p_gain').value
        i = self.get_parameter('servo_position_i_gain').value
        d = self.get_parameter('servo_position_d_gain').value
        raw = self.get_parameter('position_gain_overrides').value
        if raw:
            try:
                ov = json.loads(raw).get(str(sid), {})
                p = int(ov.get('p', p)); i = int(ov.get('i', i)); d = int(ov.get('d', d))
            except (ValueError, TypeError) as e:
                self.get_logger().warn(f"bad position_gain_overrides ({e}); using scalars")
        # X-series Position gains are 0..16383
        clamp = lambda v: max(0, min(16383, int(v)))
        return clamp(p), clamp(i), clamp(d)

    def _apply_position_gains(self):
        """Re-write Position P/I/D Gain (RAM) for every active servo, live.
        Torque may stay on — these are RAM registers.  Called from the hardware
        loop when a gain param changed."""
        ph = self.packet_handler
        for sid in self.active_ids:
            p, i, d = self._resolve_gains(sid)
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_POSITION_P_GAIN, p, "position-p-gain")
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_POSITION_I_GAIN, i, "position-i-gain")
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_POSITION_D_GAIN, d, "position-d-gain")
            self.get_logger().info(f"servo {sid} position gains -> P={p} I={i} D={d}")

    def _on_set_params(self, params):
        """Flag a live gain re-write whenever a Position-gain param is set."""
        watched = {'servo_position_p_gain', 'servo_position_i_gain',
                   'servo_position_d_gain', 'position_gain_overrides'}
        if any(pm.name in watched for pm in params):
            self._gains_dirty = True
        return SetParametersResult(successful=True)

    def _cmd_cb(self, msg: Float32MultiArray):
        try:
            data = msg.data
            n = len(data) // 3
            if n == 0:
                return
            self.latest_command = (
                [int(round(data[i])) for i in range(n)],
                [int(round(data[n + i])) for i in range(n)],
                [float(data[2 * n + i]) for i in range(n)],
            )
        except Exception as e:
            self.get_logger().warn(f"Invalid joint command: {e}")

    def _hardware_loop(self):
        # Port down (adapter unplugged / never present): do nothing this cycle.
        # The retry timer reopens it and replays config when it returns.
        if not self.port_open:
            return
        if self.latest_command is None or self.is_configuring:
            return

        ids, modes, values = self.latest_command

        # ONLY configure once at startup
        if not self.is_configured:
            self._setup_hardware(ids, modes, self.get_parameter('baudrate').value)
            return

        # Live gain re-write requested via a parameter change (pid_tuner.py).
        # Done here, in the loop's context, so it never races the sync read/write
        # on the shared serial port.
        if self._gains_dirty:
            self._gains_dirty = False
            self._apply_position_gains()

        # Write phase
        for i, sid in enumerate(ids):
            if sid not in self.active_ids:
                continue

            mode = modes[i]
            value = values[i]
            direction = -1 if sid in self.reversed_ids else 1   # mirror-mounted → invert

            if mode == 3:
                raw = direction * int(round(value * self.TICKS_PER_RAD))
                self.pos_sync_writer.addParam(sid, _pack4(raw))
            elif mode == 1:
                raw = direction * int(round(value * self.RADS_TO_VEL_UNIT))
                self.vel_sync_writer.addParam(sid, _pack4(raw))

        try:
            self.pos_sync_writer.txPacket()
            self.pos_sync_writer.clearParam()
            self.vel_sync_writer.txPacket()
            self.vel_sync_writer.clearParam()
        except Exception as e:
            # A raise here (vs a comm-error return code) means the serial handle
            # itself died — adapter unplugged mid-run.  Drop the port so the
            # retry timer reopens and reconfigures instead of erroring forever.
            self.get_logger().error(f"SyncWrite error ({e}) — dropping port, will re-open.")
            self.port_open = False
            self.is_configured = False

        # Read phase
        self._loop_count += 1
        # Poll Hardware Error Status occasionally (separate register at addr 70)
        if self.hw_error_read is not None and (self._loop_count % self.HW_ERROR_DECIMATION == 0):
            try:
                if self.hw_error_read.txRxPacket() == COMM_SUCCESS:
                    for sid in self.active_ids:
                        if self.hw_error_read.isAvailable(sid, self.ADDR_HARDWARE_ERROR, 1):
                            self.hw_error[sid] = self.hw_error_read.getData(
                                sid, self.ADDR_HARDWARE_ERROR, 1)
            except Exception as e:
                self.get_logger().error(f"HW-error SyncRead error: {e}")

        read_ok = False
        try:
            read_ok = (self.feedback_read_sync.txRxPacket() == COMM_SUCCESS)
        except Exception as e:
            self.get_logger().error(f"Feedback SyncRead error: {e}")

        # A whole-bus read failure means no servo was heard this cycle — count
        # a miss for every active servo (e.g. all power lost, or USB dropped).
        if not read_ok:
            for sid in self.active_ids:
                self._track_servo_presence(sid, present=False)

        try:
            if read_ok:
                base = self.PRESENT_BLOCK_START
                fb_data = []     # control feedback (joint_feedback): 6 per servo
                diag_data = []   # full diagnostics (servo_diagnostics): 13 per servo
                for sid in self.active_ids:
                    present = self.feedback_read_sync.isAvailable(sid, base, self.PRESENT_BLOCK_LEN)
                    self._track_servo_presence(sid, present)
                    if not present:
                        continue
                    g = self.feedback_read_sync.getData
                    moving     = g(sid, base + 0, 1)
                    moving_st  = g(sid, base + 1, 1)
                    pwm        = ctypes.c_int16(g(sid, base + 2, 2)).value
                    curr       = ctypes.c_int16(g(sid, base + 4, 2)).value
                    vel        = _to_int32(g(sid, base + 6, 4))
                    pos        = _to_int32(g(sid, base + 10, 4))
                    vel_traj   = _to_int32(g(sid, base + 14, 4))
                    pos_traj   = _to_int32(g(sid, base + 18, 4))
                    volt       = g(sid, base + 22, 2)
                    temp       = g(sid, base + 24, 1)

                    # Reversed servos report in the same logical frame they're
                    # commanded in — negate position/velocity (goal + present).
                    direction = -1.0 if sid in self.reversed_ids else 1.0
                    mode = float(self.id_modes.get(sid, 0))
                    position_rad = direction * float(pos) / self.TICKS_PER_RAD
                    velocity_rps = direction * float(vel) * self.VEL_UNIT_TO_RADS
                    current_a    = float(curr) * self.CURRENT_UNIT_A
                    voltage_v    = float(volt) * self.VOLTAGE_UNIT_V

                    fb_data.extend([
                        float(sid), mode, position_rad, velocity_rps, current_a, voltage_v,
                    ])
                    diag_data.extend([
                        float(sid), mode, float(pwm), current_a, velocity_rps, position_rad,
                        direction * float(vel_traj) * self.VEL_UNIT_TO_RADS,
                        direction * float(pos_traj) / self.TICKS_PER_RAD,
                        voltage_v, float(temp), float(moving), float(moving_st),
                        float(self.hw_error.get(sid, 0)),
                    ])

                if fb_data:
                    msg = Float32MultiArray()
                    msg.data = fb_data
                    self.feedback_pub.publish(msg)
                if diag_data:
                    dmsg = Float32MultiArray()
                    dmsg.data = diag_data
                    self.diag_pub.publish(dmsg)
        except Exception as e:
            self.get_logger().error(f"SyncRead error: {e}")

    def _track_servo_presence(self, sid, present):
        """
        Watch each servo's feedback for a sudden dropout (power loss / unplug /
        brownout).  A servo that answers resets its miss counter (and, if it was
        flagged LOST, logs recovery).  A servo that misses FB_MISS_THRESHOLD
        reads in a row is edge-logged as an ERROR exactly once — no per-loop spam.
        """
        if present:
            self._fb_miss[sid] = 0
            if sid in self._servo_lost:
                self._servo_lost.discard(sid)
                self.get_logger().warn(f"Servo {sid} back ONLINE — feedback restored.")
            return

        self._fb_miss[sid] = self._fb_miss.get(sid, 0) + 1
        if self._fb_miss[sid] == self.FB_MISS_THRESHOLD and sid not in self._servo_lost:
            self._servo_lost.add(sid)
            self.get_logger().error(
                f"!!! SERVO {sid} POWER LOST — no feedback for "
                f"{self.FB_MISS_THRESHOLD} consecutive reads "
                f"(power loss / disconnected / brownout).")

    def _write_checked(self, write_fn, sid, addr, value, label, retries=3):
        """
        Write a register, retrying on comm failure/servo error instead of the
        SDK default of silently moving on.  Returns True only once the servo's
        status packet actually confirms the write.
        """
        ph = self.packet_handler
        comm_result, error = None, None
        for attempt in range(retries):
            comm_result, error = write_fn(self.port, sid, addr, value)
            if comm_result == COMM_SUCCESS and error == 0:
                return True
            time.sleep(0.01)
        self.get_logger().error(
            f"Servo {sid}: {label} write failed after {retries} attempts "
            f"(comm={ph.getTxRxResult(comm_result)}, err={ph.getRxPacketError(error)}).")
        return False

    def _setup_hardware(self, ids: list, modes: list, requested_baud: int):
        self.is_configuring = True
        self.get_logger().info(f"Configuring hardware: ids={ids} modes={modes} baud={requested_baud}")

        ph = self.packet_handler

        # Torque OFF — blind best-effort sweep over the full id range to clear
        # any leftover state; most of these ids won't physically exist so
        # failures here are expected and not logged.
        for sid in range(1, 11):
            ph.write1ByteTxRx(self.port, sid, self.ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.005)

        # Ping each requested servo and keep only the ones that respond, so a
        # missing / unpowered servo produces a clear error but the connected
        # ones still come up and move.  ids/modes are filtered together.
        present, missing = [], []
        for i, sid in enumerate(ids):
            _, comm, _err = ph.ping(self.port, sid)
            (present if comm == COMM_SUCCESS else missing).append(i)
        if missing:
            self.get_logger().error(
                f"!!! Servo(s) {[ids[i] for i in missing]} NOT RESPONDING "
                f"(not connected / unpowered) — skipping them; bringing up "
                f"connected servos {[ids[i] for i in present]}.")
        if not present:
            self.get_logger().error("No servos responded — nothing to configure.")
            self.is_configuring = False
            return
        ids = [ids[i] for i in present]
        modes = [modes[i] for i in present]

        # Torque OFF (checked) for the servos we actually care about — Operating
        # Mode is an EEPROM register that silently fails to change while torque
        # is enabled, so this must be confirmed before writing the mode below.
        for sid in ids:
            self._write_checked(ph.write1ByteTxRx, sid, self.ADDR_TORQUE_ENABLE, 0, "torque-off")

        # Get parameters
        current_limit = self.get_parameter('current_limit').value
        vel_i = self.get_parameter('servo_velocity_i_gain').value
        vel_p = self.get_parameter('servo_velocity_p_gain').value
        profile_vel = self.get_parameter('profile_velocity').value
        profile_accel = self.get_parameter('profile_acceleration').value

        op_mode = self.op_mode_code   # 1=velocity, 3=position, 4=extended position

        for i, sid in enumerate(ids):
            if op_mode in (0, 1, 3, 4):
                self._write_checked(ph.write1ByteTxRx, sid, self.ADDR_OPERATING_MODE, op_mode, "operating-mode")

            # Homing Offset is never read or written here — it's calibrated
            # once on the servo itself (e.g. via Dynamixel Wizard) and this
            # node always trusts whatever Present Position 0 already means.
            # In Extended Position mode the servo is re-homed so that the
            # STANDBY pose reads Present Position 0 (see the controller).

            # Hard position clamps (Min/Max Position Limit) only take effect in
            # Position Control Mode (3).  Extended Position Control Mode (4)
            # ignores them entirely (goal spans the full multi-turn range and
            # may be negative), so we don't write them there — the controller's
            # software clamp is the sole position guard in extended mode.
            if op_mode == 3:
                min_tick, max_tick = self.position_tick_limits.get(sid, (0, 4095))
                self._write_checked(ph.write4ByteTxRx, sid, self.ADDR_MAX_POSITION_LIMIT,
                                    ctypes.c_uint32(max_tick).value, "max-position-limit")
                self._write_checked(ph.write4ByteTxRx, sid, self.ADDR_MIN_POSITION_LIMIT,
                                    ctypes.c_uint32(min_tick).value, "min-position-limit")

            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_CURRENT_LIMIT, current_limit, "current-limit")
            # Velocity Limit (register 44) intentionally NOT written — set on the
            # servo via Wizard; this node leaves it untouched.
            pos_p, pos_i, pos_d = self._resolve_gains(sid)   # per-servo override or scalar
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_VELOCITY_I_GAIN, vel_i, "velocity-i-gain")
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_VELOCITY_P_GAIN, vel_p, "velocity-p-gain")
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_POSITION_D_GAIN, pos_d, "position-d-gain")
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_POSITION_I_GAIN, pos_i, "position-i-gain")
            self._write_checked(ph.write2ByteTxRx, sid, self.ADDR_POSITION_P_GAIN, pos_p, "position-p-gain")
            # Slows down Position Control Mode point-to-point moves (home_state
            # / standby) so motion is slow and visually inspectable instead of
            # snapping instantly to the goal.
            self._write_checked(ph.write4ByteTxRx, sid, self.ADDR_PROFILE_VELOCITY, profile_vel, "profile-velocity")
            self._write_checked(ph.write4ByteTxRx, sid, self.ADDR_PROFILE_ACCELERATION, profile_accel, "profile-acceleration")

            self.id_modes[sid] = op_mode
            time.sleep(0.005)

        # Torque ON (checked) — track failures so a dropped packet is loud
        # instead of a servo silently staying limp.
        torque_failed = []
        for i, sid in enumerate(ids):
            if modes[i] != -1:
                if not self._write_checked(ph.write1ByteTxRx, sid, self.ADDR_TORQUE_ENABLE, 1, "torque-on"):
                    torque_failed.append(sid)
                time.sleep(0.005)

        if torque_failed:
            self.get_logger().error(
                f"Torque FAILED to enable on servos {torque_failed} — they are limp.")
        else:
            self.get_logger().info(f"Torque enabled on all servos {ids}.")

        # Rebuild sync handlers
        self.pos_sync_writer = GroupSyncWrite(self.port, ph, self.ADDR_GOAL_POSITION, 4)
        self.vel_sync_writer = GroupSyncWrite(self.port, ph, self.ADDR_GOAL_VELOCITY, 4)
        self.feedback_read_sync = GroupSyncRead(
            self.port, ph, self.PRESENT_BLOCK_START, self.PRESENT_BLOCK_LEN)
        self.hw_error_read = GroupSyncRead(self.port, ph, self.ADDR_HARDWARE_ERROR, 1)

        for sid in ids:
            self.feedback_read_sync.addParam(sid)
            self.hw_error_read.addParam(sid)
        
        self.active_ids = list(ids)
        self._fb_miss = {sid: 0 for sid in ids}   # reset dropout tracking
        self._servo_lost = set()
        self.is_configured = True
        self.is_configuring = False
        # Apply the Position P/I/D gains on the NEXT loop tick. They were only
        # ever written on a live parameter change, so gains passed in at launch
        # (position_gain_overrides in crab_launch.py) were declared and then
        # never sent to the servo -- it ran on whatever its EEPROM defaults
        # were, and the identified gains silently did nothing. Deferred to the
        # loop rather than written here so it cannot race the sync read/write
        # on the shared serial port; also covers the config replay after a
        # port reopen, where the servo may have power-cycled its RAM.
        self._gains_dirty = True

        self.get_logger().info("Hardware configuration complete.")

    def _emergency_stop(self, sig, frame):
        """Emergency stop handler - disables torque immediately on SIGINT."""
        print("\n!!! EMERGENCY STOP - DISABLING TORQUE !!!")
        try:
            # Disable all servos 1-10 (don't rely on active_ids which may be empty)
            for sid in range(1, 11):
                try:
                    self.packet_handler.write1ByteTxRx(self.port, sid, self.ADDR_TORQUE_ENABLE, 0)
                except:
                    pass
            print("Torque disabled on all servos")
        except Exception as e:
            print(f"Emergency stop error: {e}")
        finally:
            try:
                self.port.closePort()
            except:
                pass
            sys.exit(0)

    def destroy_node(self):
        self.get_logger().info("Shutting down — disabling servo torque.")
        for sid in range(1, 11):
            try:
                self.packet_handler.write1ByteTxRx(self.port, sid, self.ADDR_TORQUE_ENABLE, 0)
            except Exception:
                pass
        try:
            self.port.closePort()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DynamixelXW430Interface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
