"""
motion_library.py - Motion Library Functions
=============================================
Motion library contains all motion functions that can be composed to create
complex behaviors. Users add custom motion functions to the MotionLibrary class.

Each motion function signature:
    def my_motion(self, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict
    
Core Parameters (always provided):
    t:     Elapsed time (seconds)
    freq:  Oscillation frequency (Hz)
    amp:   Peak amplitude (radians or rad/s depending on operating_mode)
    phase: Phase offset (radians)

Optional Parameters (via **kwargs):
    Any custom parameters can be passed via robot_cmd and accessed through kwargs.
    Use kwargs.get('param_name', default_value) to extract with defaults.
    
Returns:
    dict: {servo_id: value} or {set_id: value}
        - servo_id keys for per-servo control (differential motion)
        - set_id keys for synchronized motion within a set

Motion primitives have access to:
    self.node.all_ids          - List of all servo IDs
    self.node.set_map          - {set_id: [servo_ids...]}
    self.node.offsets          - {servo_id: offset_rad}
    self.node.position_limits  - {servo_id: (min, max)}
"""

import math


class MotionLibrary:
    """
    User-defined motion functions.
    
    Add custom motion methods to this class. They will automatically be
    available when sending robot_cmd with func:[your_method_name].
    """
    
    def __init__(self, node):
        """
        Initialize with reference to parent ROS2 node.
        
        Args:
            node: Reference to crab_gait_engine node for accessing servo configuration
        """
        self.node = node
    
    # -----------------------------------------------------------------------
    # DIRECT SERVO CONTROL FUNCTIONS
    # -----------------------------------------------------------------------
    
    def drive(self, servo, value, **kwargs):
        """
        Single servo direct control - command one servo to a specific position/velocity.
        
        Core Parameters:
            servo: Servo ID (float from parser)
            value: Position (rad) or velocity (rad/s) depending on operating_mode
        
        Kwargs:
            None - this function ignores kwargs for simplicity
        
        Returns:
            dict: {servo_id: value}
        
        Usage:
            cmd_id:[1] func:[drive] servo:[3] value:[0.5]
        
        Notes:
            - Command persists until overwritten by another command
            - Operating mode (position/velocity) set in launch file
        """
        return {float(servo): value}
    
    def drive_multi(self, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Multiple servo direct control - command multiple servos simultaneously.
        
        Core Parameters:
            t, freq, amp, phase: Unused for this function (required by interface)
        
        Kwargs (REQUIRED - one of these formats):
            servos (dict): Direct mapping {servo_id: value, ...}
                Example: servos:{3:0.5, 4:-0.3, 5:1.2}
            
            OR individual servo IDs as keys:
                Example: 3:0.5 4:-0.3 5:1.2
        
        Returns:
            dict: {servo_id: value} for all commanded servos
        
        Usage Examples:
            # Via servos dict (preferred):
            cmd_id:[1] func:[drive_multi] servos:{3:0.5, 4:-0.3}
            
            # Via individual kwargs:
            cmd_id:[1] func:[drive_multi] 3:0.5 4:-0.3
        
        Notes:
            - Commands persist until overwritten
            - Can command any subset of servos
            - Unconfigured servos are ignored
        """
        result = {}
        
        # Try to get servos dict first
        if 'servos' in kwargs:
            servos_dict = kwargs['servos']
            for servo_id, value in servos_dict.items():
                result[float(servo_id)] = float(value)
        else:
            # Parse individual servo IDs from kwargs
            for key, value in kwargs.items():
                try:
                    servo_id = float(key)
                    if servo_id in self.node.all_ids:
                        result[servo_id] = float(value)
                except (ValueError, TypeError):
                    # Ignore non-numeric keys
                    pass
        
        return result
    
    # -----------------------------------------------------------------------
    # WAVEFORM PRIMITIVES - Building blocks for custom motions
    # -----------------------------------------------------------------------
    
    def sine(self, servo: float, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Sine wave - smooth sinusoidal oscillation.
        
        Core Parameters:
            servo: Servo ID to command
            t: Time (seconds)
            freq: Frequency (Hz)
            amp: Amplitude (rad)
            phase: Phase offset (rad)
        
        Kwargs:
            None - basic waveform, no customization
        
        Returns:
            dict: {servo: amp * sin(2πft + φ)}
        
        Usage:
            Helper function for building complex motions
        """
        value = amp * math.sin(2.0 * math.pi * freq * t + phase)
        return {servo: value}
    
    def square(self, servo: float, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Square wave - instant switching between +amp and -amp.
        
        Core Parameters:
            servo: Servo ID to command
            t: Time (seconds)
            freq: Frequency (Hz)
            amp: Amplitude (rad)
            phase: Phase offset (rad)
        
        Kwargs:
            duty_cycle (float): Fraction of cycle at +amp (default: 0.5)
        
        Returns:
            dict: {servo: ±amp}
        
        Usage:
            Helper function for bang-bang control patterns
        """
        duty_cycle = kwargs.get('duty_cycle', 0.5)
        phase_total = (freq * t + phase / (2.0 * math.pi)) % 1.0
        value = amp if phase_total < duty_cycle else -amp
        return {servo: value}
    
    def triangle(self, servo: float, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Triangle wave - linear ramps up and down.
        
        Core Parameters:
            servo: Servo ID to command
            t: Time (seconds)
            freq: Frequency (Hz)
            amp: Amplitude (rad)
            phase: Phase offset (rad)
        
        Kwargs:
            None - symmetric triangle wave
        
        Returns:
            dict: {servo: value}
        
        Usage:
            Helper function for constant-velocity sweeps
        """
        phase_total = (freq * t + phase / (2.0 * math.pi)) % 1.0
        value = amp * (1.0 - 4.0 * abs(phase_total - 0.5))
        return {servo: value}
    
    def sawtooth(self, servo: float, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Sawtooth wave - linear ramp up, instant reset.
        
        Core Parameters:
            servo: Servo ID to command
            t: Time (seconds)
            freq: Frequency (Hz)
            amp: Amplitude (rad)
            phase: Phase offset (rad)
        
        Kwargs:
            None - standard sawtooth
        
        Returns:
            dict: {servo: value}
        
        Usage:
            Helper function for rowing/paddling motions
        """
        phase_total = (freq * t + phase / (2.0 * math.pi)) % 1.0
        value = amp * (2.0 * phase_total - 1.0)
        return {servo: value}
    
    def step(self, servo: float, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Step function - holds +amp for half cycle, -amp for half cycle.
        
        Core Parameters:
            servo: Servo ID to command
            t: Time (seconds)
            freq: Frequency (Hz)
            amp: Amplitude (rad)
            phase: Phase offset (rad)
        
        Kwargs:
            duty_cycle (float): Fraction of cycle at +amp (default: 0.5)
        
        Returns:
            dict: {servo: ±amp}
        
        Usage:
            Helper function for discrete state switching
        """
        duty_cycle = kwargs.get('duty_cycle', 0.5)
        phase_total = (freq * t + phase / (2.0 * math.pi)) % 1.0
        value = amp if phase_total < duty_cycle else -amp
        return {servo: value}
    
    def trapezoid(self, servo: float, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Trapezoid wave - ramp up, hold, ramp down, hold.
        
        Core Parameters:
            servo: Servo ID to command
            t: Time (seconds)
            freq: Frequency (Hz)
            amp: Amplitude (rad)
            phase: Phase offset (rad)
        
        Kwargs:
            ramp_fraction (float): Fraction of cycle spent ramping (default: 0.25 per ramp)
        
        Returns:
            dict: {servo: value}
        
        Usage:
            Helper function for smooth acceleration/deceleration profiles
        """
        phase_total = (freq * t + phase / (2.0 * math.pi)) % 1.0
        
        if phase_total < 0.25:
            value = amp * (4.0 * phase_total)
        elif phase_total < 0.5:
            value = amp
        elif phase_total < 0.75:
            value = amp * (3.0 - 4.0 * phase_total)
        else:
            value = -amp
        
        return {servo: value}
    
    # -----------------------------------------------------------------------
    # FLAPPING MOTION PRIMITIVES
    # -----------------------------------------------------------------------
    
    def sine_flap(self, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Sine flap - servo pairs with differential phase/frequency/amplitude.
        
        Core Parameters:
            t: Time (seconds)
            freq: Base oscillation frequency (Hz)
            amp: Base amplitude (rad)
            phase: Base phase offset (rad)
        
        Kwargs:
            freq_ratio (float): Frequency multiplier for even-indexed servos (default: 1.0)
                                Set to 0.0 to hold even servos at constant position
            amp_ratio (float): Amplitude multiplier for even-indexed servos (default: 1.0)
                               Used as hold position when freq_ratio=0.0
            phase_offset (float): Phase shift for even-indexed servos in radians (default: π/2)
        
        Returns:
            dict: {servo_id: value} for all active servos
        
        Logic:
            - Sorts all active servos in ascending order
            - Pairs as (even_index, odd_index): (0,1), (2,3), (4,5), ...
            - Even-indexed servos get modified parameters (freq*freq_ratio, amp*amp_ratio, phase+phase_offset)
            - Odd-indexed servos get base parameters (freq, amp, phase)
        
        Usage Examples:
            # Basic 90° phase offset flap (default):
            func:[sine_flap] freq:[0.5] amp:[1.0] phase:[0.0] cycles:[3] sets:[1]
            
            # Custom phase offset (180° out of phase):
            func:[sine_flap] freq:[0.5] amp:[1.0] phase_offset:[3.14] cycles:[3] sets:[1]
            
            # Different frequencies (even servo 2x faster):
            func:[sine_flap] freq:[0.5] amp:[1.0] freq_ratio:[2.0] cycles:[3] sets:[1]
            
            # Hold even servo, oscillate odd (forward thrust):
            func:[sine_flap] freq:[0.5] amp:[1.0] freq_ratio:[0.0] amp_ratio:[1.0] cycles:[3] sets:[1]
            
            # Asymmetric amplitude (even servo half amplitude):
            func:[sine_flap] freq:[0.5] amp:[1.0] amp_ratio:[0.5] cycles:[3] sets:[1]
        
        Notes:
            - Default phase_offset of π/2 (90°) provides standard flapping motion
            - freq_ratio=0.0 holds even servos at position defined by amp_ratio
            - Works with any even number of servos (2, 4, 6, etc.)
        """
        # Parse optional parameters with defaults
        freq_ratio = kwargs.get('freq_ratio', 1.0)
        amp_ratio = kwargs.get('amp_ratio', 1.0)
        phase_offset = kwargs.get('phase_offset', math.pi / 2)  # Default 90° phase offset
        
        result = {}
        
        # Get all servos from active sets
        active_servos = []
        for set_id in self.node.set_map.keys():
            active_servos.extend(self.node.set_map[set_id])
        
        # Sort in ascending order
        active_servos.sort()
        
        # Calculate even-index parameters
        even_freq = freq * freq_ratio
        even_amp = amp * amp_ratio
        even_phase = phase + phase_offset
        
        # Pair servos: (even, odd), (even, odd), ...
        for i in range(0, len(active_servos) - 1, 2):
            even_servo = active_servos[i]      # Even index gets modified parameters
            odd_servo = active_servos[i + 1]   # Odd index gets base parameters
            
            # Even servo behavior
            if freq_ratio == 0.0:
                # Hold constant position
                result[float(even_servo)] = even_amp
            else:
                # Oscillate with modified parameters
                result[float(even_servo)] = even_amp * math.sin(2.0 * math.pi * even_freq * t + even_phase)
            
            # Odd servo behavior (always oscillates with base parameters)
            result[float(odd_servo)] = amp * math.sin(2.0 * math.pi * freq * t + phase)
        
        return result
    
    def sine_paddle(self, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
        """
        Sine paddle - rowing/paddling motion with alternating waveforms.
        
        Core Parameters:
            t: Time (seconds)
            freq: Base oscillation frequency (Hz)
            amp: Base amplitude (rad)
            phase: Base phase offset (rad)
        
        Kwargs:
            freq_ratio (float): Frequency multiplier for even-indexed servos (default: 1.0)
                                Set to 0.0 to hold even servos constant
            amp_ratio (float): Amplitude multiplier for even-indexed servos (default: 1.0)
                               Used as hold position when freq_ratio=0.0
            phase_offset (float): Phase shift for even-indexed servos in radians (default: 0.0)
        
        Returns:
            dict: {servo_id: value} for all active servos
        
        Logic:
            - Sorts all active servos in ascending order
            - Pairs as (even_index, odd_index): (0,1), (2,3), (4,5), ...
            - Even-indexed servos: sawtooth wave (linear power stroke, instant recovery)
            - Odd-indexed servos: sine wave (smooth oscillation)
        
        Usage Examples:
            # Basic paddle (sawtooth + sine):
            func:[sine_paddle] freq:[0.5] amp:[1.0] phase:[0.0] cycles:[5] sets:[1]
            
            # Faster power stroke (even servo 2x frequency):
            func:[sine_paddle] freq:[0.5] amp:[1.0] freq_ratio:[2.0] cycles:[5] sets:[1]
        
        Notes:
            - Sawtooth provides fast return stroke, slow power stroke
            - Adjust phase parameter to reverse stroke direction
        """
        # Parse optional parameters with defaults
        freq_ratio = kwargs.get('freq_ratio', 1.0)
        amp_ratio = kwargs.get('amp_ratio', 1.0)
        phase_offset = kwargs.get('phase_offset', 0.0)
        
        result = {}
        
        # Get all servos from active sets
        active_servos = []
        for set_id in self.node.set_map.keys():
            active_servos.extend(self.node.set_map[set_id])
        
        # Sort in ascending order
        active_servos.sort()
        
        # Calculate even-index parameters
        even_freq = freq * freq_ratio
        even_amp = amp * amp_ratio
        even_phase = phase + phase_offset
        
        # Pair servos: (even, odd), (even, odd), ...
        for i in range(0, len(active_servos) - 1, 2):
            even_servo = active_servos[i]      # Sawtooth (power stroke)
            odd_servo = active_servos[i + 1]   # Sine (smooth)
            
            # Even servo - sawtooth for rowing motion
            if freq_ratio == 0.0:
                # Hold constant position
                result[float(even_servo)] = even_amp
            else:
                # Sawtooth with modified parameters
                phase_total = (even_freq * t + even_phase / (2.0 * math.pi)) % 1.0
                result[float(even_servo)] = even_amp * (2.0 * phase_total - 1.0)
            
            # Odd servo - sine for smooth oscillation
            result[float(odd_servo)] = amp * math.sin(2.0 * math.pi * freq * t + phase)
        
        return result
    
    # -----------------------------------------------------------------------
    # ADD CUSTOM MOTION FUNCTIONS BELOW
    # -----------------------------------------------------------------------
    
    # Example template:
    # def my_custom_motion(self, t: float, freq: float, amp: float, phase: float, **kwargs) -> dict:
    #     """
    #     Brief description of what this motion does.
    #     
    #     Core Parameters:
    #         t, freq, amp, phase: Standard parameters
    #     
    #     Kwargs:
    #         my_param (type): Description (default: value)
    #     
    #     Returns:
    #         dict: {servo_id or set_id: value}
    #     
    #     Usage:
    #         func:[my_custom_motion] freq:[0.5] amp:[1.0] my_param:[2.5] cycles:[3] sets:[1]
    #     """
    #     # Extract custom parameters
    #     my_param = kwargs.get('my_param', default_value)
    #     
    #     result = {}
    #     # ... your motion logic here ...
    #     
    #     return result
