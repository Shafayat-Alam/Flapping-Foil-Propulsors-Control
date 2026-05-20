#!/usr/bin/env python3
"""
gazebo_dynamixel_interface.py — Hybrid Gazebo/Real Dynamixel Interface
========================================================================
Automatically detects connected real servos and merges with simulated servos.

If real servos detected: uses real hardware feedback
If real servos NOT detected: uses Gazebo simulation feedback

Gazebo always shows all 4 servos (visual), but feedback comes from real hardware when available.

Topics
------
Subscribes: joint_cmd (Float32MultiArray) - from controller.py
            /joint_states (JointState) - from Gazebo (for simulated servos)
Publishes:  joint_feedback (Float32MultiArray) - merged real + sim feedback to controller.py
            /joint_trajectory_controller/joint_trajectory (JointTrajectory) - to Gazebo
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import math
import ctypes

# Try to import Dynamixel SDK (optional - only needed if real hardware present)
try:
    from dynamixel_sdk import (
        PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead,
        DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD, COMM_SUCCESS
    )
    DYNAMIXEL_AVAILABLE = True
except ImportError:
    DYNAMIXEL_AVAILABLE = False


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


class GazeboDynamixelInterface(Node):
    """
    Hybrid Dynamixel interface - auto-detects real servos, merges with Gazebo sim.
    """
    
    # Dynamixel constants
    ADDR_OPERATING_MODE = 11
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_VELOCITY = 104
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_DATA = 126
    
    TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
    RADS_TO_VEL_UNIT = 1.0 / (0.229 * (2.0 * math.pi / 60.0))
    VEL_UNIT_TO_RADS = 0.229 * (2.0 * math.pi / 60.0)
    
    def __init__(self):
        super().__init__('gazebo_dynamixel_interface')
        
        # Parameters
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 'joint_4'])
        self.declare_parameter('servo_ids', [1, 2, 3, 4])
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_position_p_gain', 800)
        
        self.joint_names = self.get_parameter('joint_names').value
        self.servo_ids = self.get_parameter('servo_ids').value
        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        
        # Servo ID to joint name mapping
        self.id_to_joint = dict(zip(self.servo_ids, self.joint_names))
        self.joint_to_id = dict(zip(self.joint_names, self.servo_ids))
        
        # Detect real hardware
        self.real_servos = []
        self.sim_servos = list(self.servo_ids)
        self.port_handler = None
        self.packet_handler = None
        self.pos_sync_writer = None
        self.vel_sync_writer = None
        self.feedback_read_sync = None
        
        if DYNAMIXEL_AVAILABLE:
            self._detect_real_servos()
        else:
            self.get_logger().info("Dynamixel SDK not available - using pure simulation")
        
        # Current joint states from Gazebo (for simulated servos)
        self.joint_positions = {name: 0.0 for name in self.joint_names}
        self.joint_velocities = {name: 0.0 for name in self.joint_names}
        
        # Subscribers
        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            'joint_cmd',
            self._cmd_callback,
            1
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            10
        )
        
        # Publishers
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            1
        )
        
        self.feedback_pub = self.create_publisher(
            Float32MultiArray,
            'joint_feedback',
            1
        )
        
        # Feedback timer
        self.create_timer(1.0 / 500.0, self._publish_feedback)
        
        self.get_logger().info(
            f"Hybrid Dynamixel interface ready - "
            f"Real servos: {self.real_servos}, Simulated: {self.sim_servos}"
        )
    
    def _detect_real_servos(self):
        """Detect which servos are physically connected."""
        try:
            self.port_handler = PortHandler(self.port_name)
            self.packet_handler = PacketHandler(2.0)
            
            if not self.port_handler.openPort():
                self.get_logger().info(f"Cannot open {self.port_name} - using pure simulation")
                return
            
            if not self.port_handler.setBaudRate(self.baudrate):
                self.get_logger().info(f"Cannot set baudrate - using pure simulation")
                self.port_handler.closePort()
                return
            
            # Ping each servo to see if it responds
            for sid in self.servo_ids:
                result, error = self.packet_handler.ping(self.port_handler, sid)
                if result == COMM_SUCCESS:
                    self.real_servos.append(sid)
                    self.sim_servos.remove(sid)
                    self.get_logger().info(f"Detected real servo ID {sid}")
            
            if self.real_servos:
                self._setup_real_servos()
            else:
                self.get_logger().info("No real servos detected - using pure simulation")
                self.port_handler.closePort()
                self.port_handler = None
        
        except Exception as e:
            self.get_logger().info(f"Servo detection failed: {e} - using pure simulation")
            if self.port_handler:
                try:
                    self.port_handler.closePort()
                except:
                    pass
                self.port_handler = None
    
    def _setup_real_servos(self):
        """Configure detected real servos."""
        pos_p = self.get_parameter('servo_position_p_gain').value
        
        for sid in self.real_servos:
            # Set position mode
            self.packet_handler.write1ByteTxRx(
                self.port_handler, sid, self.ADDR_OPERATING_MODE, 3
            )
            
            # Set PID gains
            self.packet_handler.write2ByteTxRx(
                self.port_handler, sid, 84, pos_p  # Position P gain
            )
            
            # Enable torque
            self.packet_handler.write1ByteTxRx(
                self.port_handler, sid, self.ADDR_TORQUE_ENABLE, 1
            )
        
        # Create sync read/write handlers
        self.pos_sync_writer = GroupSyncWrite(
            self.port_handler, self.packet_handler, self.ADDR_GOAL_POSITION, 4
        )
        self.vel_sync_writer = GroupSyncWrite(
            self.port_handler, self.packet_handler, self.ADDR_GOAL_VELOCITY, 4
        )
        self.feedback_read_sync = GroupSyncRead(
            self.port_handler, self.packet_handler, self.ADDR_PRESENT_DATA, 20
        )
        
        for sid in self.real_servos:
            self.feedback_read_sync.addParam(sid)
        
        self.get_logger().info(f"Configured real servos: {self.real_servos}")
    
    def _cmd_callback(self, msg: Float32MultiArray):
        """
        Receive joint commands and send to BOTH real servos AND Gazebo.
        Format: [id0, id1, ...] [mode0, mode1, ...] [val0, val1, ...]
        """
        data = msg.data
        n = len(data) // 3
        if n == 0:
            return
        
        ids = [int(round(data[i])) for i in range(n)]
        modes = [int(round(data[n + i])) for i in range(n)]
        values = [float(data[2 * n + i]) for i in range(n)]
        
        # Send to REAL servos if available
        if self.real_servos and self.pos_sync_writer:
            for sid, mode, value in zip(ids, modes, values):
                if sid not in self.real_servos:
                    continue
                
                if mode == 3:  # Position mode
                    raw = int(round(value * self.TICKS_PER_RAD))
                    self.pos_sync_writer.addParam(sid, _pack4(raw))
                elif mode == 1:  # Velocity mode
                    raw = int(round(value * self.RADS_TO_VEL_UNIT))
                    self.vel_sync_writer.addParam(sid, _pack4(raw))
            
            try:
                self.pos_sync_writer.txPacket()
                self.pos_sync_writer.clearParam()
                self.vel_sync_writer.txPacket()
                self.vel_sync_writer.clearParam()
            except Exception as e:
                self.get_logger().error(f"Real servo write error: {e}")
        
        # Send to GAZEBO for visualization (all servos)
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        
        point = JointTrajectoryPoint()
        
        for sid, mode, value in zip(ids, modes, values):
            if sid not in self.id_to_joint:
                continue
            
            joint_name = self.id_to_joint[sid]
            traj_msg.joint_names.append(joint_name)
            
            if mode == 3:
                point.positions.append(value)
                point.velocities.append(0.0)
            elif mode == 1:
                point.positions.append(self.joint_positions.get(joint_name, 0.0))
                point.velocities.append(value)
            else:
                point.positions.append(0.0)
                point.velocities.append(0.0)
        
        point.time_from_start = Duration(sec=0, nanosec=10000000)  # 10ms
        traj_msg.points.append(point)
        
        self.trajectory_pub.publish(traj_msg)
    
    def _joint_state_callback(self, msg: JointState):
        """Update joint states from Gazebo feedback (for simulated servos)."""
        for i, name in enumerate(msg.name):
            if name in self.joint_positions:
                self.joint_positions[name] = msg.position[i]
                if i < len(msg.velocity):
                    self.joint_velocities[name] = msg.velocity[i]
    
    def _publish_feedback(self):
        """
        Publish joint feedback - MERGED from real hardware + Gazebo simulation.
        Format: [id, mode, pos_rad, vel_rad_s, curr_A, volt_V] repeated per servo
        """
        fb_data = []
        
        # Read REAL servo feedback
        real_feedback = {}
        if self.real_servos and self.feedback_read_sync:
            try:
                if self.feedback_read_sync.txRxPacket() == COMM_SUCCESS:
                    for sid in self.real_servos:
                        if self.feedback_read_sync.isAvailable(sid, self.ADDR_PRESENT_DATA, 20):
                            curr = self.feedback_read_sync.getData(sid, self.ADDR_PRESENT_DATA, 2)
                            vel = self.feedback_read_sync.getData(sid, self.ADDR_PRESENT_DATA + 2, 4)
                            pos = self.feedback_read_sync.getData(sid, self.ADDR_PRESENT_DATA + 6, 4)
                            volt = self.feedback_read_sync.getData(sid, self.ADDR_PRESENT_DATA + 18, 2)
                            
                            real_feedback[sid] = {
                                'pos': float(_to_int32(pos)) / self.TICKS_PER_RAD,
                                'vel': float(_to_int32(vel)) * self.VEL_UNIT_TO_RADS,
                                'curr': float(ctypes.c_int16(curr).value) * 0.00269,
                                'volt': float(volt) * 0.1
                            }
            except Exception as e:
                self.get_logger().error(f"Real servo read error: {e}")
        
        # Build feedback for ALL servos (real + simulated)
        for sid in self.servo_ids:
            if sid in real_feedback:
                # Use REAL hardware feedback
                fb = real_feedback[sid]
                fb_data.extend([
                    float(sid),
                    3.0,  # Mode
                    fb['pos'],
                    fb['vel'],
                    fb['curr'],
                    fb['volt']
                ])
            elif sid in self.id_to_joint:
                # Use SIMULATED feedback from Gazebo
                joint_name = self.id_to_joint[sid]
                pos = self.joint_positions.get(joint_name, 0.0)
                vel = self.joint_velocities.get(joint_name, 0.0)
                
                fb_data.extend([
                    float(sid),
                    3.0,  # Mode
                    pos,
                    vel,
                    0.0,  # Current (simulated)
                    12.0  # Voltage (simulated)
                ])
        
        if fb_data:
            msg = Float32MultiArray()
            msg.data = fb_data
            self.feedback_pub.publish(msg)
    
    def destroy_node(self):
        """Cleanup on shutdown."""
        if self.real_servos and self.port_handler:
            self.get_logger().info("Disabling real servo torque")
            for sid in self.real_servos:
                try:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, sid, self.ADDR_TORQUE_ENABLE, 0
                    )
                except:
                    pass
            try:
                self.port_handler.closePort()
            except:
                pass
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboDynamixelInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
