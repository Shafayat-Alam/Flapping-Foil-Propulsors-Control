"""
Dynamixel_XW430_T200_interface.py — Write-Only Hardware Interface
=================================================================
This node is the exclusive owner of the physical serial bus.  It translates
ROS2 Float32MultiArray joint commands into Dynamixel protocol 2.0 packets
and writes them to the servos.

Topics
------
Subscribes : robot_config       (String, transient_local) from crab — read only
                                 for per-servo homing offsets (written to the
                                 Homing Offset register at setup).
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
    ADDR_HOMING_OFFSET = 20
    ADDR_CURRENT_LIMIT = 38
    ADDR_TORQUE_ENABLE = 64
    ADDR_VELOCITY_I_GAIN = 76
    ADDR_VELOCITY_P_GAIN = 78
    ADDR_POSITION_D_GAIN = 80
    ADDR_POSITION_I_GAIN = 82
    ADDR_POSITION_P_GAIN = 84
    ADDR_HARDWARE_ERROR = 70
    ADDR_GOAL_VELOCITY = 104
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_DATA  = 126

    # Full present-data block read in one shot: Moving(122) .. Temperature(146).
    PRESENT_BLOCK_START = 122
    PRESENT_BLOCK_LEN   = 25
    HW_ERROR_DECIMATION = 50    # poll Hardware Error Status every Nth loop

    SERVO_DIAG_STRIDE = 13      # values per servo in servo_diagnostics

    BAUD_MAP = {9600: 0, 57600: 1, 115200: 2, 1000000: 3, 2000000: 4, 3000000: 5, 4000000: 6, 4500000: 7}
    TICKS_PER_RAD    = 4096.0 / (2.0 * math.pi)
    RADS_TO_VEL_UNIT = 1.0 / (0.229 * (2.0 * math.pi / 60.0))
    VEL_UNIT_TO_RADS = 0.229 * (2.0 * math.pi / 60.0)
    CURRENT_UNIT_A   = 0.00269  # A per current register unit
    VOLTAGE_UNIT_V   = 0.1      # V per input-voltage register unit

    def __init__(self):
        super().__init__('servo_actuator')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('hardware_rate', 500.0)
        self.declare_parameter('current_limit', 800)
        self.declare_parameter('servo_velocity_i_gain', 1920)
        self.declare_parameter('servo_velocity_p_gain', 100)
        self.declare_parameter('servo_position_d_gain', 0)
        self.declare_parameter('servo_position_i_gain', 0)
        self.declare_parameter('servo_position_p_gain', 800)

        port_name = self.get_parameter('port').value
        init_baud = self.get_parameter('baudrate').value
        hw_rate = self.get_parameter('hardware_rate').value

        self.port = PortHandler(port_name)
        self.packet_handler = PacketHandler(2.0)

        if not self.port.openPort():
            self.get_logger().fatal(f"Cannot open port {port_name}")
        if not self.port.setBaudRate(init_baud):
            self.get_logger().fatal(f"Cannot set baudrate {init_baud}")

        self.current_baudrate = init_baud
        self.latest_command = None
        self.active_ids = []
        self.id_modes = {}
        self.homing_offsets = {}      # sid -> homing offset (rad), from robot_config
        self.is_configured = False
        self.is_configuring = False
        self.pos_sync_writer = None
        self.vel_sync_writer = None

        # crab broadcasts the actuator map on a latched topic; we read it only for
        # the per-servo homing offsets, written to each servo's Homing Offset
        # register at setup so the servo itself reads 0 at mechanical home.
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

        # Register SIGINT handler for immediate torque disable
        signal.signal(signal.SIGINT, self._emergency_stop)

        self.get_logger().info(f"DynamixelXW430Interface online — port={port_name} baud={init_baud} hw_rate={hw_rate} Hz")

    def _config_cb(self, msg: String):
        """Read per-servo homing offsets (rad) from crab's actuator map."""
        try:
            cfg = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad robot_config JSON: {e}")
            return
        offsets = {}
        for entry in cfg.get('actuator_map', []):
            try:
                offsets[int(round(entry[0]))] = float(entry[1])
            except (IndexError, ValueError, TypeError):
                continue
        self.homing_offsets = offsets
        self.get_logger().info(
            f"Homing offsets received for servos {sorted(self.homing_offsets)}.")

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
        if self.latest_command is None or self.is_configuring:
            return

        ids, modes, values = self.latest_command

        # ONLY configure once at startup
        if not self.is_configured:
            self._setup_hardware(ids, modes, self.get_parameter('baudrate').value)
            return

        # Write phase
        for i, sid in enumerate(ids):
            if sid not in self.active_ids:
                continue

            mode = modes[i]
            value = values[i]

            if mode == 3:
                raw = int(round(value * self.TICKS_PER_RAD))
                self.pos_sync_writer.addParam(sid, _pack4(raw))
            elif mode == 1:
                raw = int(round(value * self.RADS_TO_VEL_UNIT))
                self.vel_sync_writer.addParam(sid, _pack4(raw))

        try:
            self.pos_sync_writer.txPacket()
            self.pos_sync_writer.clearParam()
            self.vel_sync_writer.txPacket()
            self.vel_sync_writer.clearParam()
        except Exception as e:
            self.get_logger().error(f"SyncWrite error: {e}")

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

        try:
            if self.feedback_read_sync.txRxPacket() == COMM_SUCCESS:
                base = self.PRESENT_BLOCK_START
                fb_data = []     # control feedback (joint_feedback): 6 per servo
                diag_data = []   # full diagnostics (servo_diagnostics): 13 per servo
                for sid in self.active_ids:
                    if not self.feedback_read_sync.isAvailable(sid, base, self.PRESENT_BLOCK_LEN):
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

                    mode = float(self.id_modes.get(sid, 0))
                    position_rad = float(pos) / self.TICKS_PER_RAD
                    velocity_rps = float(vel) * self.VEL_UNIT_TO_RADS
                    current_a    = float(curr) * self.CURRENT_UNIT_A
                    voltage_v    = float(volt) * self.VOLTAGE_UNIT_V

                    fb_data.extend([
                        float(sid), mode, position_rad, velocity_rps, current_a, voltage_v,
                    ])
                    diag_data.extend([
                        float(sid), mode, float(pwm), current_a, velocity_rps, position_rad,
                        float(vel_traj) * self.VEL_UNIT_TO_RADS,
                        float(pos_traj) / self.TICKS_PER_RAD,
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

    def _setup_hardware(self, ids: list, modes: list, requested_baud: int):
        self.is_configuring = True
        self.get_logger().info(f"Configuring hardware: ids={ids} modes={modes} baud={requested_baud}")

        ph = self.packet_handler
        port = self.port

        # Torque OFF
        for sid in range(1, 11):
            ph.write1ByteTxRx(port, sid, self.ADDR_TORQUE_ENABLE, 0)
            time.sleep(0.005)

        # Get parameters
        current_limit = self.get_parameter('current_limit').value
        vel_i = self.get_parameter('servo_velocity_i_gain').value
        vel_p = self.get_parameter('servo_velocity_p_gain').value
        pos_d = self.get_parameter('servo_position_d_gain').value
        pos_i = self.get_parameter('servo_position_i_gain').value
        pos_p = self.get_parameter('servo_position_p_gain').value
        
        for i, sid in enumerate(ids):
            mode = modes[i]

            if mode in (0, 1, 3):
                ph.write1ByteTxRx(port, sid, self.ADDR_OPERATING_MODE, mode)

            # Homing offset (rad -> signed ticks) so the servo reads 0 at home.
            offset_rad = self.homing_offsets.get(sid, 0.0)
            offset_ticks = ctypes.c_uint32(
                int(round(offset_rad * self.TICKS_PER_RAD))).value
            ph.write4ByteTxRx(port, sid, self.ADDR_HOMING_OFFSET, offset_ticks)

            ph.write2ByteTxRx(port, sid, self.ADDR_CURRENT_LIMIT, current_limit)
            ph.write2ByteTxRx(port, sid, self.ADDR_VELOCITY_I_GAIN, vel_i)
            ph.write2ByteTxRx(port, sid, self.ADDR_VELOCITY_P_GAIN, vel_p)
            ph.write2ByteTxRx(port, sid, self.ADDR_POSITION_D_GAIN, pos_d)
            ph.write2ByteTxRx(port, sid, self.ADDR_POSITION_I_GAIN, pos_i)
            ph.write2ByteTxRx(port, sid, self.ADDR_POSITION_P_GAIN, pos_p)

            self.id_modes[sid] = mode
            time.sleep(0.005)

        # Torque ON
        for i, sid in enumerate(ids):
            if modes[i] != -1:
                ph.write1ByteTxRx(port, sid, self.ADDR_TORQUE_ENABLE, 1)
                time.sleep(0.005)

        # Rebuild sync handlers
        self.pos_sync_writer = GroupSyncWrite(port, ph, self.ADDR_GOAL_POSITION, 4)
        self.vel_sync_writer = GroupSyncWrite(port, ph, self.ADDR_GOAL_VELOCITY, 4)
        self.feedback_read_sync = GroupSyncRead(
            port, ph, self.PRESENT_BLOCK_START, self.PRESENT_BLOCK_LEN)
        self.hw_error_read = GroupSyncRead(port, ph, self.ADDR_HARDWARE_ERROR, 1)

        for sid in ids:
            self.feedback_read_sync.addParam(sid)
            self.hw_error_read.addParam(sid)
        
        self.active_ids = list(ids)
        self.is_configured = True
        self.is_configuring = False

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
