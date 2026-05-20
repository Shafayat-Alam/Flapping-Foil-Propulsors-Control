"""
crab_controller.py — Abstract Feedforward Motion Controller with Outer-Loop PID
================================================================================
This node sits between the gait engine (crab.py) and the hardware interface.
It applies an outer-loop PID correction on top of the servo's internal PID.

Cascaded PID Structure
-----------------------
- Inner loop (servo hardware): Position/velocity PID runs at servo update rate
- Outer loop (this node): Position/velocity PID runs at 400 Hz on top

When outer-loop gains (kp, ki, kd) are all 0.0, this becomes a pure passthrough
(open-loop). The servo's internal PID still runs.

What this node does
-------------------
  1. Receives motion_cmd from the gait engine.
  2. Receives joint_feedback from the hardware interface.
  3. Computes error = (commanded - actual)
  4. Applies outer-loop PID correction if gains > 0
  5. Re-publishes corrected command on joint_cmd.
  6. Publishes telemetry (commanded vs actual trajectory log).

Topics
------
Subscribes : motion_cmd   (Float32MultiArray)  from crab.py
             joint_feedback (Float32MultiArray) from servo_actuator
Publishes  : joint_cmd    (Float32MultiArray)  to hardware interface
             telemetry    (Float32MultiArray)

motion_cmd / joint_cmd wire format
--------------------------------------
Three concatenated equal-length segments:
  [id_0, id_1, ..., id_N-1,
   mode_0, mode_1, ..., mode_N-1,
   value_0, value_1, ..., value_N-1]

Telemetry wire format
---------------------
  [cmd_ts, id_0, mode_0, raw_0, filtered_0,  id_1, mode_1, raw_1, filtered_1, ...]
  (5 floats per servo after the leading timestamp)
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray
import time
from collections import deque


# ---------------------------------------------------------------------------
# Per-servo trajectory state
# ---------------------------------------------------------------------------

class ServoState:
    """Tracks PID state for one servo."""

    def __init__(self, kp: float, ki: float, kd: float):
        # PID state
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error_sum = 0.0
        self.last_error = 0.0

    def pid_update(self, error: float, dt: float) -> float:
        """
        Compute PID correction.
        If all gains are 0, returns 0 (open-loop passthrough).
        """
        if self.kp == 0.0 and self.ki == 0.0 and self.kd == 0.0:
            return 0.0
        
        self.error_sum += error * dt
        d_error = (error - self.last_error) / dt if dt > 0.0 else 0.0
        self.last_error = error
        return (self.kp * error) + (self.ki * self.error_sum) + (self.kd * d_error)
    
    def reset_pid(self):
        """Reset PID integrator and derivative state."""
        self.error_sum = 0.0
        self.last_error = 0.0


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class Controller(Node):
    """
    Abstract, DOF-agnostic motion controller.

    Acts as a real-time trajectory conditioner.  All DOF knowledge lives
    upstream in crab.py — this node only sees servo IDs, modes, and target
    positions.
    """

    def __init__(self):
        super().__init__('controller')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        # Outer loop PID (set to 0 for open-loop passthrough)
        self.declare_parameter('kp', 0.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('telemetry_decimation', 1)  # Publish every sample
        self.declare_parameter('control_rate', 400.0)  # Hz - PID processing rate

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.telem_decim = self.get_parameter('telemetry_decimation').value
        self.control_rate = self.get_parameter('control_rate').value

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.servo_states: dict  = {}    # sid(float) -> ServoState
        self.full_feedback: dict = {}    # sid(float) -> {"pos": rad, "vel": rad/s, "curr": A, "volt": V}
        self.latest_raw:   tuple = None  # (cmd_id, ids, modes, values) last received
        self.prev_time:    float = 0.0
        self.sample_counter: int = 0

        self._telem_buf:   deque = deque(maxlen=200)

        # ------------------------------------------------------------------
        # ROS2 interfaces
        # ------------------------------------------------------------------
        self.cmd_sub   = self.create_subscription(
            Float32MultiArray, 'motion_cmd', self._cmd_cb, 1
        )
        self.feedback_sub = self.create_subscription(
            Float32MultiArray, 'joint_feedback', self._feedback_cb, 1
        )
        self.cmd_pub   = self.create_publisher(
            Float32MultiArray, 'joint_cmd', 1
        )
        self.telem_pub = self.create_publisher(
            Float32MultiArray, 'telemetry', 10
        )
        
        # Control loop timer
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)

        self.get_logger().info(f"Controller ready (control_rate={self.control_rate} Hz, PID outer-loop).")

    # ------------------------------------------------------------------
    # Incoming command handler
    # ------------------------------------------------------------------

    def _cmd_cb(self, msg: Float32MultiArray):
        """
        Store latest motion_cmd. Processing happens in control_loop timer.
        Wire format: [cmd_id, id0, id1, ...] [mode0, mode1, ...] [val0, val1, ...] [min0, max0, min1, max1, ...]
        """
        data = msg.data
        if len(data) < 1:
            return
        
        cmd_id = data[0]
        remaining = data[1:]
        n = len(remaining) // 5  # Now 5 segments: ids, modes, values, limits (min/max pairs)
        if n == 0:
            return

        ids    = [float(remaining[i])         for i in range(n)]
        modes  = [float(remaining[n + i])     for i in range(n)]
        values = [float(remaining[2 * n + i]) for i in range(n)]
        
        # Parse position limits (min, max pairs)
        limits_start = 3 * n
        position_limits = {}
        for i in range(n):
            sid = ids[i]
            min_limit = float(remaining[limits_start + 2 * i])
            max_limit = float(remaining[limits_start + 2 * i + 1])
            position_limits[sid] = (min_limit, max_limit)
        
        self.latest_raw = (cmd_id, ids, modes, values, position_limits)

        # Register any new servo IDs we haven't seen before
        for sid in ids:
            if sid not in self.servo_states:
                self.servo_states[sid] = ServoState(self.kp, self.ki, self.kd)
            if sid not in self.full_feedback:
                self.full_feedback[sid] = {"pos": 0.0, "vel": 0.0, "curr": 0.0, "volt": 0.0}
    
    # ------------------------------------------------------------------
    # Control loop (timer-driven at control_rate)
    # ------------------------------------------------------------------
    
    def _control_loop(self):
        """
        Timer-driven control loop at control_rate Hz.
        Applies PID and publishes commands.
        """
        if self.latest_raw is None:
            return
        
        cmd_id, ids, modes, values, position_limits = self.latest_raw
        
        # Compute dt from control_rate
        dt = 1.0 / self.control_rate

        # Get operating mode from mode values
        operating_mode = 'position' if modes[0] == 3.0 else 'velocity'

        final_values = []
        for sid, mode, raw in zip(ids, modes, values):
            state = self.servo_states.get(sid)
            if state is None:
                final_values.append(raw)
                continue

            # Get position limits for this servo
            min_limit, max_limit = position_limits.get(sid, (-3.14, 3.14))

            # Apply outer-loop PID correction if feedback available
            if operating_mode == 'position' and sid in self.full_feedback:
                error = raw - self.full_feedback[sid]["pos"]
                correction = state.pid_update(error, dt)
                final_val = raw + correction
                # Clamp position commands to limits
                final_val = max(min_limit, min(max_limit, final_val))
                
            elif operating_mode == 'velocity' and sid in self.full_feedback:
                error = raw - self.full_feedback[sid]["vel"]
                correction = state.pid_update(error, dt)
                final_val = raw + correction
                
                # Enforce position limits in velocity mode by zeroing velocity at limits
                actual_pos = self.full_feedback[sid]["pos"]
                if (actual_pos >= max_limit and final_val > 0) or \
                   (actual_pos <= min_limit and final_val < 0):
                    final_val = 0.0  # Stop at position limit
                    
            else:
                final_val = raw
                # Clamp passthrough position commands
                if operating_mode == 'position':
                    final_val = max(min_limit, min(max_limit, final_val))
            
            final_values.append(final_val)

        self._publish_cmd(ids, modes, final_values)

        # Buffer for telemetry
        self._telem_buf.append({
            'cmd_id':  cmd_id,
            'ts':      float(self.sample_counter),
            'ids':     ids,
            'modes':   modes,
            'raw':     list(values),
            'filt':    final_values,
            'fb':      {sid: self.full_feedback[sid].copy() for sid in ids if sid in self.full_feedback}
        })

        # Publish telemetry every Nth sample
        self.sample_counter += 1
        if self.sample_counter % self.telem_decim == 0:
            self._publish_telemetry()

    # ------------------------------------------------------------------
    # Feedback callback
    # ------------------------------------------------------------------

    def _feedback_cb(self, msg: Float32MultiArray):
        """
        Decode joint_feedback from hardware interface.
        Wire format: [id, mode, pos_rad, vel_rad_s, curr_A, volt_V] repeated per servo
        """
        data = msg.data
        for i in range(0, len(data), 6):
            sid = data[i]
            if sid in self.full_feedback:
                self.full_feedback[sid].update({
                    "pos":  data[i + 2],
                    "vel":  data[i + 3],
                    "curr": data[i + 4],
                    "volt": data[i + 5]
                })

    # ------------------------------------------------------------------
    # Publisher helpers
    # ------------------------------------------------------------------

    def _publish_cmd(self, ids, modes, values):
        msg = Float32MultiArray()
        msg.data = (
            [float(i) for i in ids] +
            [float(m) for m in modes] +
            [float(v) for v in values]
        )
        self.cmd_pub.publish(msg)

    def _publish_telemetry(self):
        if not self._telem_buf:
            return

        entry = self._telem_buf[-1]   # most recent sample
        
        # Format: [cmd_id, timestamp, goal0, pos0, vel0, curr0, volt0, goal1, pos1, vel1, curr1, volt1, ...]
        payload = [entry['cmd_id'], entry['ts']]
        
        for sid, mode, raw, filt in zip(
            entry['ids'], entry['modes'], entry['raw'], entry['filt']
        ):
            fb = entry['fb'].get(sid, {"pos": 0.0, "vel": 0.0, "curr": 0.0, "volt": 0.0})
            payload.extend([
                float(filt),      # commanded goal (after filtering + PID)
                fb["pos"],        # actual position from encoder
                fb["vel"],        # actual velocity
                fb["curr"],       # actual current
                fb["volt"]        # actual voltage
            ])

        msg = Float32MultiArray()
        msg.data = payload
        self.telem_pub.publish(msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Publish a zero-velocity hold command before shutting down."""
        if self.latest_raw is not None:
            ids, modes, vals = self.latest_raw
            # Drive values toward zero offsets (hold at mechanical zero)
            hold_vals = []
            for sid, v in zip(ids, vals):
                state = self.servo_states.get(sid)
                hold_vals.append(state.filtered_pos if state else v)
            self._publish_cmd(ids, modes, hold_vals)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

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
