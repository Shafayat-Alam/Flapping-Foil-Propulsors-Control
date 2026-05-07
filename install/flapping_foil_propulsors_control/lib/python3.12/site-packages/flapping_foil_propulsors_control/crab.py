"""
crab.py — Gait Library and Parameter Host
==========================================
This node owns two things:
  1. The actuator map (servo IDs, zero offsets, set assignments) declared
     as a ROS2 parameter so they can be overridden from the launch file
     without touching code.
  2. Every gait / motion primitive.  Each primitive is a plain method that
     accepts (t, freq, amp) and returns a dict whose keys are DOF labels
     (e.g. "roll", "pitch", "heave").  The controller maps those labels to
     servos dynamically based on the set definition — no hardcoded DOF count.

Adding a new gait:  write a method that returns a dict, done.
Adding a new DOF:   add a servo to a set in the launch parameter and write
                    a gait function that returns values for that many sets.

Communication
-------------
Subscribes : robot_cmd   (std_msgs/String)
Publishes  : motion_cmd  (std_msgs/Float32MultiArray)  — write-only downstream

robot_cmd wire format
----------------------
A single string with two whitespace-separated sections:

  cmd_id:[<id>] func:[<name>] freq:[<hz>] amp:[<rad>] cycles:[<n>]
  sets:[<s0>,<s1>,...] phases:[<rad_s0>,<rad_s1>,...]
  config:[<JSON array of {id,offset,set}>]

All keys are lowercase, no spaces inside brackets.

Example
-------
  cmd_id:[1] func:[hover] freq:[0.5] amp:[0.4] cycles:[10]
  sets:[1,2] phases:[0.0,1.5708]
  config:[{"id":1,"offset":3.6,"set":1},{"id":2,"offset":2.7,"set":1},
          {"id":3,"offset":3.0,"set":2},{"id":4,"offset":3.42,"set":2}]
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float32MultiArray, String
import math
import json
from collections import deque

# Import user-defined motion library
from flapping_foil_propulsors_control.motion_library import MotionLibrary


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class CrabGaitEngine(Node):
    """
    Gait engine node.

    Owns the actuator map parameter and dispatches user-defined gait functions.
    Parses inbound robot_cmd strings, evaluates the requested function at 400 Hz,
    and publishes raw position targets to motion_cmd.

    Users define their own gait functions as methods in this class. Each function
    receives (t, freq, amp, phase) and returns a dict with numeric keys matching
    set IDs: {1: angle_rad, 2: angle_rad, ...}

    No feedback is consumed here — this is an open-loop trajectory generator.
    """

    def __init__(self):
        super().__init__('crab_gait_engine')

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------
        # actuator_map: JSON string  [[id, offset_rad, set_id], ...]
        # Readable/overridable from the launch file without touching code.
        self.declare_parameter(
            'actuator_map',
            '[[1, 3.60, 1], [2, 2.70, 1], [3, 3.00, 2], [4, 3.42, 2]]'
        )
        self.declare_parameter('operating_mode', 'position')  # 'position' or 'velocity'

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self.all_ids:       list  = []
        self.offsets:       dict  = {}   # sid -> zero offset (rad)
        self.set_map:       dict  = {}   # set_id -> [sid, ...]
        self.set_phase_map: dict  = {}   # set_id -> phase offset (rad)
        self.position_limits: dict = {}  # sid -> (min, max) in radians

        # Command queue and execution state
        self.command_queue: deque = deque()
        self.current_cmd_id:  float = 0.0
        self.active_type:     str   = None  # 'gait' or 'direct'
        self.active_func:     str   = None
        self.active_freq:     float = 1.0
        self.active_amp:      float = 0.0
        self.active_phase:    float = 0.0
        self.active_freq_ratio:    float = 1.0
        self.active_amp_ratio:     float = 1.0
        self.active_phase_offset:  float = 0.0
        self.active_sets:     list  = []
        self.active_params:   dict  = {}
        self.total_samples:   int   = 0
        self.sample_count:    int   = 0
        
        # Direct command state (persists until overridden)
        self.direct_goals:    dict  = {}  # sid -> value (holds between commands)

        # Load actuator map from parameter
        self._load_actuator_map()
        
        # Initialize user motion library
        self.gaits = MotionLibrary(self)

        # Queue calibration command at startup
        self.command_queue.append({
            'cmd_id':        0.0,
            'type':          'gait',
            'func':          'calibration',
            'freq':          1.0,
            'amp':           0.0,
            'phase':         0.0,
            'freq_ratio':    1.0,
            'amp_ratio':     1.0,
            'phase_offset':  0.0,
            'cycles':        1.0,
            'sets':          list(self.set_map.keys()),
            'phases':        [0.0] * len(self.set_map),
            'config':        None
        })
        self.get_logger().info("Calibration queued at startup")

        # ------------------------------------------------------------------
        # ROS2 interfaces
        # ------------------------------------------------------------------
        self.motion_pub = self.create_publisher(Float32MultiArray, 'motion_cmd',  1)
        self.robot_cmd_sub = self.create_subscription(String, 'robot_cmd', self._motion_cb, 10)
        self.telemetry_sub = self.create_subscription(Float32MultiArray, 'telemetry', self._telemetry_cb, 10)
        
        self.get_logger().info(
            f"CrabGaitEngine ready — {len(self.all_ids)} servos across "
            f"{len(self.set_map)} sets. Telemetry-driven mode."
        )
        
        # Kickstart the telemetry loop by sending initial motion command
        self.startup_timer = self.create_timer(1.0, self._send_startup_command)

    # ------------------------------------------------------------------
    # Actuator map management
    # ------------------------------------------------------------------

    def _load_actuator_map(self):
        """
        Parse the actuator_map parameter and populate internal lookup tables.
        Format: [[id, offset_rad, set_id, min_limit, max_limit], ...]
        min_limit and max_limit are optional - if not provided, defaults to ±3.14
        Can be called again at runtime if the parameter is updated.
        """
        raw = self.get_parameter('actuator_map').value
        try:
            entries = json.loads(raw)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Bad actuator_map JSON: {e}")
            return

        self.all_ids.clear()
        self.offsets.clear()
        self.set_map.clear()
        self.position_limits = {}  # sid -> (min, max)

        for entry in entries:
            sid    = float(entry[0])
            offset = float(entry[1])
            set_id = int(entry[2])
            
            # Optional position limits (default ±π if not provided)
            min_limit = float(entry[3]) if len(entry) > 3 else -3.14
            max_limit = float(entry[4]) if len(entry) > 4 else 3.14

            self.all_ids.append(sid)
            self.offsets[sid] = offset
            self.position_limits[sid] = (min_limit, max_limit)

            if set_id not in self.set_map:
                self.set_map[set_id] = []
            self.set_map[set_id].append(sid)

        self.all_ids = sorted(self.all_ids)
        self.get_logger().info(
            f"Actuator map loaded: {len(self.all_ids)} servos, "
            f"sets: {sorted(self.set_map.keys())}"
        )

    # ------------------------------------------------------------------
    # User-defined gait functions
    # ------------------------------------------------------------------
    
    def _send_startup_command(self):
        """Send initial motion command to kickstart telemetry loop."""
        if not self.all_ids:
            return
        
        # Send hold at zero command
        operating_mode = self.get_parameter('operating_mode').value
        if operating_mode == 'position':
            modes = [3.0] * len(self.all_ids)
        else:
            modes = [1.0] * len(self.all_ids)
        
        values = [self.offsets.get(sid, 0.0) for sid in self.all_ids]
        self._publish_motion_cmd(self.all_ids, modes, values)
        self.get_logger().info("Sent startup command to kickstart telemetry loop")
        
        # Cancel timer after first use
        if hasattr(self, 'startup_timer'):
            self.startup_timer.cancel()
        
        # Cancel timer after first send
        if hasattr(self, 'startup_timer'):
            self.startup_timer.cancel()
    
    def calibration(self, t: float, freq: float, amp: float, phase: float, **kwargs):
        """
        Calibration function - moves all servos to their zero offset positions.
        Call with: func:[calibration] freq:[1.0] amp:[0.0] phase:[0.0] cycles:[1] sets:[1]
        """
        return {set_id: 0.0 for set_id in self.set_map.keys()}

    # ------------------------------------------------------------------
    # Motion command parser
    # ------------------------------------------------------------------

    def _motion_cb(self, msg: String):
        """
        Parse a robot_cmd string and add to command queue.
        
        Two command formats supported:
        
        1. Gait commands (time-based):
           cmd_id:[1] func:[hover] freq:[1.0] amp:[0.3] cycles:[5] sets:[1,2] phases:[0,1.57]
           
        2. Direct commands (immediate):
           cmd_id:[1] func:[drive] servo:[3] value:[0.5]
        
        Commands execute sequentially.
        """
        raw = msg.data

        try:
            # --- Optional inline config override ----------------------------
            config_data = None
            if ' config:' in raw:
                cmd_raw, config_raw = raw.split(' config:', 1)
                config_data = json.loads(config_raw.strip())
            else:
                cmd_raw = raw

            # --- Parse key:[value,...] tokens --------------------------------
            tokens = {}
            for part in cmd_raw.lower().replace(' ', '').split(']'):
                if ':[' not in part:
                    continue
                key, vals = part.split(':[', 1)
                tokens[key] = vals.split(',')

            cmd_id   = float(tokens['cmd_id'][0])
            func_name = tokens['func'][0].strip()
            
            # --- Detect command type -----------------------------------------
            if 'servo' in tokens:
                # DIRECT COMMAND FORMAT
                cmd_type = 'direct'
                
                # Parse direct command parameters (flexible - function defines what it needs)
                params = {}
                for key, vals in tokens.items():
                    if key not in ['cmd_id', 'func']:
                        # Convert single values, keep lists as lists
                        if len(vals) == 1:
                            # Try to convert to number, else keep as string
                            try:
                                params[key] = float(vals[0])
                            except ValueError:
                                params[key] = vals[0]
                        else:
                            params[key] = [float(v) for v in vals]
                
                # Validate function exists
                if callable(getattr(self, func_name, None)):
                    pass  # Function exists on node
                elif callable(getattr(self.gaits, func_name, None)):
                    pass  # Function exists in user gaits
                else:
                    raise ValueError(f"Unknown function: '{func_name}'")
                
                # Queue direct command
                self.command_queue.append({
                    'cmd_id':   cmd_id,
                    'type':     cmd_type,
                    'func':     func_name,
                    'params':   params,
                    'config':   config_data
                })
                
            else:
                # GAIT COMMAND FORMAT
                cmd_type = 'gait'
                
                freq     = float(tokens['freq'][0])
                amp      = float(tokens['amp'][0])
                phase    = float(tokens.get('phase', ['0.0'])[0])  # Optional, default 0
                cycles   = float(tokens['cycles'][0])
                sets     = [int(s) for s in tokens['sets']]
                phases   = [float(p) for p in tokens.get('phases', ['0.0'] * len(sets))]
                
                # Optional ratio parameters for differential servo behavior
                freq_ratio    = float(tokens.get('freq_ratio', ['1.0'])[0])
                amp_ratio     = float(tokens.get('amp_ratio', ['1.0'])[0])
                phase_offset  = float(tokens.get('phase_offset', ['0.0'])[0])

                if len(phases) != len(sets):
                    raise ValueError(
                        f"'phases' length ({len(phases)}) must match 'sets' length ({len(sets)})"
                    )

                # Validate gait function exists
                if callable(getattr(self, func_name, None)):
                    pass  # Function exists on node (e.g., calibration)
                elif callable(getattr(self.gaits, func_name, None)):
                    pass  # Function exists in user gaits
                else:
                    raise ValueError(f"Unknown gait function: '{func_name}'")

                # Queue gait command
                self.command_queue.append({
                    'cmd_id':        cmd_id,
                    'type':          cmd_type,
                    'func':          func_name,
                    'freq':          freq,
                    'amp':           amp,
                    'phase':         phase,
                    'freq_ratio':    freq_ratio,
                    'amp_ratio':     amp_ratio,
                    'phase_offset':  phase_offset,
                    'cycles':        cycles,
                    'sets':          sets,
                    'phases':        phases,
                    'config':        config_data
                })

            self.get_logger().info(
                f"Queued cmd_id={cmd_id} type={cmd_type} func={func_name} "
                f"(queue depth: {len(self.command_queue)})"
            )

        except Exception as e:
            self.get_logger().error(f"motion_cb parse failure: {e}")

    # ------------------------------------------------------------------
    # Event-driven control loop (triggered by joint_feedback)
    # ------------------------------------------------------------------

    def _telemetry_cb(self, msg: Float32MultiArray):
        """
        Triggered when telemetry arrives from controller.
        Evaluates next gait sample or updates direct command goals.
        Telemetry-driven - no timer.
        """
        if not self.all_ids:
            return

        # Check if we need to start a new command
        if self.active_func is None:
            if self.command_queue:
                self._start_next_command()

        # Build goals (start with direct command state or zeros)
        goals: dict = {sid: self.direct_goals.get(sid, 0.0) for sid in self.all_ids}

        if self.active_func is not None:
            if self.active_type == 'gait':
                # GAIT COMMAND - time-based execution
                # Assume controller runs at 400 Hz (configurable via controller's control_rate)
                control_rate = 400.0  # Hz - matches controller default
                t = self.sample_count / control_rate
                
                # Get gait function
                if hasattr(self.gaits, self.active_func):
                    gait_fn = getattr(self.gaits, self.active_func)
                else:
                    gait_fn = getattr(self, self.active_func)
                
                result = gait_fn(
                    t, self.active_freq, self.active_amp, self.active_phase,
                    freq_ratio=self.active_freq_ratio,
                    amp_ratio=self.active_amp_ratio,
                    phase_offset=self.active_phase_offset
                )
                
                # Map gait results to servos
                # Handle both set_id keys (all servos in set move together)
                # and servo_id keys (per-servo control for differential motion)
                for set_id in self.active_sets:
                    if set_id in result:
                        # Set-level control - all servos in set move together
                        base_angle = result[set_id]
                        for sid in self.set_map[set_id]:
                            goals[sid] = base_angle
                    else:
                        # Servo-level control - check if individual servos are in result
                        for sid in self.set_map[set_id]:
                            if sid in result:
                                goals[sid] = result[sid]

                self.sample_count += 1

                # Check if command finished
                if self.sample_count >= self.total_samples:
                    self.get_logger().info(
                        f"Gait complete — cmd_id={self.current_cmd_id} ({self.sample_count} samples)"
                    )
                    self.active_func = None
                    
            elif self.active_type == 'direct':
                # DIRECT COMMAND - execute and persist
                
                # Get direct function
                if hasattr(self.gaits, self.active_func):
                    direct_fn = getattr(self.gaits, self.active_func)
                else:
                    direct_fn = getattr(self, self.active_func)
                
                self.get_logger().info(f"Calling {self.active_func} with params: {self.active_params}")
                
                # Call with unpacked parameters
                result = direct_fn(**self.active_params)
                
                self.get_logger().info(f"Result from {self.active_func}: {result}")
                
                # Update direct_goals (these persist across telemetry callbacks)
                for key, value in result.items():
                    if key in self.set_map:
                        # Key is a set_id
                        for sid in self.set_map[key]:
                            self.direct_goals[sid] = value
                            goals[sid] = value
                    elif key in self.all_ids:
                        # Key is a servo_id
                        self.direct_goals[key] = value
                        goals[key] = value
                
                self.get_logger().info(f"Updated direct_goals: {self.direct_goals}")
                self.get_logger().info(f"Direct command complete — cmd_id={self.current_cmd_id}")
                self.active_func = None  # Done immediately, but goals persist

        # Build and publish joint command
        operating_mode = self.get_parameter('operating_mode').value
        if operating_mode == 'position':
            modes = [3.0] * len(self.all_ids)
        elif operating_mode == 'velocity':
            modes = [1.0] * len(self.all_ids)
        else:
            modes = [3.0] * len(self.all_ids)
        
        # Apply offsets
        values = [goals[sid] + self.offsets.get(sid, 0.0) for sid in self.all_ids]
        
        self._publish_motion_cmd(self.all_ids, modes, values)

    def _start_next_command(self):
        """Pop next command from queue and initialize execution."""
        cmd = self.command_queue.popleft()
        
        # Apply config if present
        if cmd.get('config') is not None:
            self.all_ids.clear()
            self.offsets.clear()
            self.set_map.clear()
            for item in cmd['config']:
                sid    = float(item['id'])
                offset = float(item['offset'])
                set_id = int(item['set'])
                self.all_ids.append(sid)
                self.offsets[sid] = offset
                if set_id not in self.set_map:
                    self.set_map[set_id] = []
                self.set_map[set_id].append(sid)
            self.all_ids = sorted(self.all_ids)
        
        # Initialize common fields
        self.current_cmd_id = cmd['cmd_id']
        self.active_func    = cmd['func']
        self.active_type    = cmd['type']
        
        if self.active_type == 'gait':
            # Initialize gait-specific fields
            self.active_freq         = cmd['freq']
            self.active_amp          = cmd['amp']
            self.active_phase        = cmd['phase']
            self.active_freq_ratio   = cmd['freq_ratio']
            self.active_amp_ratio    = cmd['amp_ratio']
            self.active_phase_offset = cmd['phase_offset']
            self.active_sets         = cmd['sets']
            self.set_phase_map       = {s: cmd['phases'][i] for i, s in enumerate(cmd['sets'])}
            
            # Calculate total samples based on controller rate (400 Hz default)
            control_rate = 400.0  # Hz - should match controller's control_rate
            duration = cmd['cycles'] / cmd['freq']
            self.total_samples = int(duration * control_rate)
            self.sample_count = 0
            
            self.get_logger().info(
                f"Starting gait cmd_id={self.current_cmd_id} func={self.active_func} "
                f"freq={self.active_freq} amp={self.active_amp} phase={self.active_phase} "
                f"freq_ratio={self.active_freq_ratio} amp_ratio={self.active_amp_ratio} "
                f"phase_offset={self.active_phase_offset} "
                f"({self.total_samples} samples, {len(self.command_queue)} remaining)"
            )
            
        elif self.active_type == 'direct':
            # Initialize direct-specific fields
            self.active_params = cmd['params']
            
            self.get_logger().info(
                f"Starting direct cmd_id={self.current_cmd_id} func={self.active_func} "
                f"params={self.active_params} ({len(self.command_queue)} remaining)"
            )

    # ------------------------------------------------------------------
    # Publisher helper
    # ------------------------------------------------------------------

    def _publish_motion_cmd(self, ids, modes, values):
        """
        motion_cmd wire format: 
        [cmd_id, id0, id1, ...] [mode0, mode1, ...] [val0, val1, ...] [min0, max0, min1, max1, ...]
        
        Position limits are appended as pairs (min, max) for each servo.
        """
        # Get position limits for each servo
        limits = []
        for sid in ids:
            min_limit, max_limit = self.position_limits.get(sid, (-3.14, 3.14))
            limits.extend([min_limit, max_limit])
        
        msg = Float32MultiArray()
        msg.data = (
            [float(self.current_cmd_id)] +
            [float(i) for i in ids] +
            [float(m) for m in modes] +
            [float(v) for v in values] +
            limits
        )
        self.motion_pub.publish(msg)

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        """Zero all servos on shutdown."""
        if self.all_ids:
            self._publish_motion_cmd(
                self.all_ids,
                [3.0] * len(self.all_ids),
                [self.offsets.get(sid, 0.0) for sid in self.all_ids]
            )
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CrabGaitEngine()
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