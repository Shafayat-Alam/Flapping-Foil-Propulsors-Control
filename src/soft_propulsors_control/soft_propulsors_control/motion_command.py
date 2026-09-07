"""
motion_command.py — Pure Motion Math Library
=============================================
A flat module of stateless functions that turn time + parameters into servo
targets.  Nothing here knows about ROS, sensors, or robot state — the
controller owns all of that and calls into these helpers in its real-time loop.

The library is layered so larger motions are built from smaller ones:

  Layer 1  Waveforms        sine / cosine / square / triangle / sawtooth /
                            trapezoid — pure ``(t, freq, amp, phase) -> float``.
                            The *shape* of a motion is just which one you pass in.

  Layer 2  Servo targets    drive / drive_multi / hold — turn raw values into the
                            ``{servo_id: value}`` dict the controller consumes.

  Layer 3  Gaits            flap / paddle — coordinated roll+pitch fin motions
                            built from Layer 1 + Layer 2.  The waveform is an
                            argument, so ``flap`` with ``sine`` and ``flap`` with
                            ``square`` are the same function, different feel.

  Layer 4  Search helpers   sweep — a slow one-axis ramp the controller composes
                            into scanning / reorientation behaviours.

Every function returns either a ``float`` (waveforms) or a ``{servo_id: value}``
dict (everything else), where ``value`` is radians in position mode or rad/s in
velocity mode.  The controller applies offsets, limits, and feedback on top.

Convention: a "fin" is a roll servo plus a pitch servo.  Roll orients the fin
relative to the flow (broadside vs. edge-on); pitch sweeps it.  The controller
knows which physical servo is roll and which is pitch and passes their IDs in.
"""

import math

TWO_PI = 2.0 * math.pi


# ===========================================================================
# Layer 1 — Waveforms   (t, freq, amp, phase) -> float
# ===========================================================================
# Each returns a single scalar at time ``t``.  ``phase`` is in radians.  These
# are the interchangeable "shape" of any oscillating motion.

def sine(t: float, freq: float, amp: float, phase: float = 0.0, **_) -> float:
    """Smooth sinusoid: ``amp * sin(2π·freq·t + phase)``."""
    return amp * math.sin(TWO_PI * freq * t + phase)


def cosine(t: float, freq: float, amp: float, phase: float = 0.0, **_) -> float:
    """Smooth cosinusoid: ``amp * cos(2π·freq·t + phase)`` (sine shifted +90°)."""
    return amp * math.cos(TWO_PI * freq * t + phase)


def shaped_sine(t: float, freq: float, amp: float, phase: float = 0.0,
               k: float = 0.0, **_) -> float:
    """
    Sine wave whose corners are squared off by exponent ``k``:

        y = amp · sign(sin θ) · |sin θ|^(1/(k+1)),   θ = 2π·freq·t + phase

    k=0 is an EXACT plain sine (identical to ``sine()``).  Increasing k
    progressively flattens the top/bottom and sharpens the reversal, k→∞
    (pass ``float('inf')``) is an exact square wave.  Peak amplitude is
    ``amp`` at every k — only the shape between peaks changes.  sign(sin θ) is
    taken as exactly 0 at θ = 0 (mod π) so the curve crosses zero cleanly
    (rather than a spurious ±1 jump) at every k, including k=inf.
    """
    s = math.sin(TWO_PI * freq * t + phase)
    if s == 0.0:
        return 0.0
    exponent = 1.0 / (k + 1.0)
    return amp * math.copysign(abs(s) ** exponent, s)


def square(t: float, freq: float, amp: float, phase: float = 0.0,
           duty: float = 0.5, **_) -> float:
    """Square wave snapping between +amp and -amp. ``duty`` is the +amp fraction."""
    cycle = (freq * t + phase / TWO_PI) % 1.0
    return amp if cycle < duty else -amp


def triangle(t: float, freq: float, amp: float, phase: float = 0.0, **_) -> float:
    """Symmetric triangle wave — constant-rate ramps up then down."""
    cycle = (freq * t + phase / TWO_PI) % 1.0
    return amp * (1.0 - 4.0 * abs(cycle - 0.5))


def sawtooth(t: float, freq: float, amp: float, phase: float = 0.0, **_) -> float:
    """Sawtooth wave — linear ramp from -amp to +amp, then reset."""
    cycle = (freq * t + phase / TWO_PI) % 1.0
    return amp * (2.0 * cycle - 1.0)


def trapezoid(t: float, freq: float, amp: float, phase: float = 0.0,
              ramp: float = 0.25, **_) -> float:
    """
    Trapezoid wave — ramp up, hold at +amp, ramp down, hold at -amp.
    ``ramp`` is the fraction of the half-cycle spent ramping (0..0.5).
    """
    cycle = (freq * t + phase / TWO_PI) % 1.0
    ramp = max(1e-6, min(0.5, ramp))
    if cycle < ramp:                      # rising edge
        return amp * (cycle / ramp)
    if cycle < 0.5:                       # high hold
        return amp
    if cycle < 0.5 + ramp:                # falling edge
        return amp * (1.0 - (cycle - 0.5) / ramp)
    return -amp                           # low hold


# Registry so callers can select a waveform by name (e.g. from a parameter)
WAVEFORMS = {
    'sine': sine,
    'cosine': cosine,
    'square': square,
    'triangle': triangle,
    'sawtooth': sawtooth,
    'trapezoid': trapezoid,
}


def get_waveform(name):
    """Resolve a waveform by name, or pass a callable straight through."""
    if callable(name):
        return name
    return WAVEFORMS.get(str(name).lower(), sine)


def ease(x: float) -> float:
    """
    Smoothstep ease 0→1 over x∈[0,1] (zero velocity at both ends).
    Used to make transitions graceful rather than abrupt.
    """
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)


# ===========================================================================
# Layer 2 — Servo targets   -> {servo_id: value}
# ===========================================================================

def drive(servo_id, value: float) -> dict:
    """Command a single servo. Returns ``{servo_id: value}``."""
    return {float(servo_id): float(value)}


def drive_multi(targets: dict) -> dict:
    """Command several servos at once from a ``{servo_id: value}`` mapping."""
    return {float(sid): float(val) for sid, val in targets.items()}


def hold(servo_ids, value: float = 0.0) -> dict:
    """Hold every servo in ``servo_ids`` at the same value (default neutral)."""
    return {float(sid): float(value) for sid in servo_ids}


# ===========================================================================
# Layer 3 — Gaits   (coordinated roll + pitch fin motion)
# ===========================================================================

def flap(pitch_id, heave_id, t: float, freq: float, amp: float,
         phase: float = 0.0, waveform=sine,
         heave_center: float = 0.0, **kwargs) -> dict:
    """
    Flapping gait — pitch oscillation with the roll servo left untouched.

    Only the pitch servo is commanded: it oscillates about ``heave_center``
    (the standby pose, 0 in extended mode) with the chosen ``waveform`` (amplitude
    ``amp`` in rad, ``freq`` set by the controller).  The roll servo is
    deliberately NOT commanded, so it holds whatever position it was last driven
    to (or keeps following the IMU) — the caller publishes only the pitch servo
    so the omitted roll is left alone rather than reset.

    ``pitch_id`` is accepted for call-site symmetry with the other gaits but is
    intentionally unused (roll is held, not set).

    Args:
        pitch_id : this fin's roll servo — accepted but not commanded (held)
        heave_id : this fin's pitch servo — the one that oscillates
        t, freq, amp, phase : standard waveform parameters (amp in rad)
        waveform : Layer-1 function or name selecting the pitch shape
        heave_center : oscillation midpoint (rad; standby pose = 0 in extended mode)

    Extra ``kwargs`` (e.g. ``duty``, ``ramp``) are forwarded to the waveform.

    Returns: ``{heave_id: heave_value}``  (roll omitted → held at current pos)
    """
    wave = get_waveform(waveform)
    heave_value = heave_center + wave(t, freq, amp, phase, **kwargs)
    return drive(heave_id, heave_value)


def harmonic_wave(t: float, freq: float, amp: float, phase: float = 0.0,
                  a2: float = 0.0, phi2: float = 0.0, a3: float = 0.0,
                  bias: float = 0.0, _norm_cache={}) -> float:
    """Fundamental + 2nd + 3rd harmonic, peak-normalised, plus a DC bias.

        y = bias + amp * [sin(th) + a2*sin(2th + phi2) + a3*sin(3th)] / peak

    The three terms do different jobs, and the split is not arbitrary:

      * a2 is an EVEN harmonic. It is the only term here that breaks the
        half-wave symmetry y(t + T/2) = -y(t). Under any odd force law
        (F ~ v, v|v| or v^3 alike) that symmetry forces the positive and
        negative force lobes to be exact mirror images, which is why a plain
        sine -- and pitch_k, which adds only ODD harmonics -- can never make
        one lobe bigger than the other. phi2 selects WHICH lobe grows.
      * a3 is odd, so it changes lobe WIDTH only and leaves the +/- balance
        untouched. It is the same thing pitch_k already does, exposed here
        as an amplitude so it can be tuned continuously.
      * bias shifts the neutral angle without touching velocity at all (a
        constant differentiates to zero). It acts purely through angle of
        attack, which makes it an independent route to asymmetry -- the one
        worth testing separately from a2.

    PEAK NORMALISATION is what keeps shape independent of size: without it,
    raising a2 would also raise the stroke amplitude, and a force change
    could not be attributed to shape rather than to a bigger sweep. The
    normaliser depends only on (a2, phi2, a3), so it is cached rather than
    recomputed at every servo tick.
    """
    key = (round(a2, 4), round(phi2, 3), round(a3, 4))
    norm = _norm_cache.get(key)
    if norm is None:
        n = 720
        peak = 0.0
        for i in range(n):
            th = TWO_PI * i / n
            v = (math.sin(th) + a2 * math.sin(2 * th + phi2)
                 + a3 * math.sin(3 * th))
            peak = max(peak, abs(v))
        norm = peak if peak > 1e-9 else 1.0
        _norm_cache[key] = norm
    th = TWO_PI * freq * t + phase
    y = (math.sin(th) + a2 * math.sin(2 * th + phi2) + a3 * math.sin(3 * th))
    return bias + amp * y / norm


def paddle_harmonic(pitch_id, heave_id, tau: float, freq: float,
                    pitch_amp: float, heave_amp: float,
                    pitch_center: float = 0.0, heave_center: float = 0.0,
                    pitch_phase: float = 0.0, heave_freq_ratio: float = 1.0,
                    p_a2: float = 0.0, p_phi2: float = 0.0, p_a3: float = 0.0,
                    h_a2: float = 0.0, h_phi2: float = 0.0, h_a3: float = 0.0,
                    pitch_bias: float = 0.0, **kwargs) -> dict:
    """paddle() with independent harmonic content on each servo.

    Harmonics go on the servo whose force channel is being shaped: pitch
    for Fx, heave for Fy. Only one at a time is meaningful -- with two
    actuators and three force components the rig is underactuated, so
    shaping both channels simultaneously is not attempted.

    pitch_bias is ADDED to pitch_center, so it offsets the stroke relative
    to the calibrated neutral rather than replacing it.
    """
    pitch_value = harmonic_wave(tau, freq, pitch_amp, 0.0,
                                p_a2, p_phi2, p_a3,
                                bias=pitch_center + pitch_bias)
    heave_value = harmonic_wave(tau, freq * heave_freq_ratio, heave_amp,
                                pitch_phase, h_a2, h_phi2, h_a3,
                                bias=heave_center)
    targets = drive(pitch_id, pitch_value)
    targets.update(drive(heave_id, heave_value))
    return targets


def paddle(pitch_id, heave_id, tau: float, freq: float,
           pitch_amp: float, heave_amp: float,
           pitch_center: float = 0.0, heave_center: float = 0.0,
           pitch_phase: float = 0.0, heave_freq_ratio: float = 1.0,
           pitch_k: float = 0.0, **kwargs) -> dict:
    """
    Sine paddling gait — pitch and heave are sinusoids about the rest pose
    (pitch_center, heave_center).  Pitch is the frequency/phase reference; heave
    runs at ``heave_freq_ratio × freq`` and is phase-shifted from pitch by
    ``pitch_phase`` (rad) at tau=0.  Roll and pitch amplitudes are independent
    (A_r, A_p) — the controller scales them from separate efforts.

        pitch_value  = pitch_center + shaped_sine(A_r, freq, k=pitch_k)
        heave_value = heave_center + A_p · sin(2π·(freq·heave_freq_ratio)·tau
                                               + pitch_phase)

    ``pitch_k`` shapes ONLY the pitch curve (see ``shaped_sine``) — heave stays
    a plain sinusoid.  k=0 (default) is a plain sine, identical to the original
    ``paddle()`` behaviour.

    ``heave_freq_ratio`` defaults to 1.0 (both servos at the same frequency —
    the original behaviour, where pitch_phase is then a genuine fixed lag).
    Away from 1.0 the two sinusoids run at different frequencies, so their
    relative phase drifts continuously; pitch_phase only sets the offset at
    tau=0.  ``pitch_amp`` (A_r) is SIGNED — its sign flips the roll sinusoid to
    reverse thrust (forward fin +A_r, reversed fin −A_r).  Sinusoids are
    naturally continuous, so multiple cycles flow smoothly with no reset.
    ``tau`` is mission-relative time (s).

    Returns: ``{pitch_id: pitch_value, heave_id: heave_value}``
    """
    pitch_value = pitch_center + shaped_sine(tau, freq, pitch_amp, 0.0, k=pitch_k)
    heave_value = heave_center + sine(tau, freq * heave_freq_ratio, heave_amp,
                                      pitch_phase)
    targets = drive(pitch_id, pitch_value)
    targets.update(drive(heave_id, heave_value))
    return targets


# ===========================================================================
# Layer 4 — Search helpers
# ===========================================================================

def sweep(servo_id, t: float, rate: float, span: float,
          center: float = 0.0, phase: float = 0.0) -> dict:
    """
    Slow continuous one-axis sweep — the building block for scanning.

    Drives ``servo_id`` back and forth across ``±span/2`` around ``center`` at
    ``rate`` Hz using a smooth triangle profile, so the body reorients steadily
    on a single axis while the controller holds the others constant.

    Args:
        servo_id : servo to sweep
        t : elapsed time (s)
        rate : sweep frequency (Hz) — keep low for a slow search
        span : peak-to-peak sweep range (rad)
        center : midpoint of the sweep (rad)
        phase : phase offset (rad)

    Returns: ``{servo_id: value}``
    """
    value = center + triangle(t, rate, span / 2.0, phase)
    return drive(servo_id, value)


# ===========================================================================
# Layer 5 — GREY-BOX HIL BRIDGE  (ROS-aware exception to this file's own
# "stateless, no ROS/sensors/robot-state" design — placed here deliberately,
# by request, rather than in a separate script, so the real-hardware plant
# implementation for scripts/force_control.py lives next to the motion
# primitives it ultimately drives. Nothing above this line needs or imports
# rclpy; every ROS-dependent name below is imported lazily so Layers 1-4
# stay usable without a sourced ROS workspace.)
# ===========================================================================
#
# force_control.py's closed-loop tuner (see that file's own long docstring)
# needs one thing this module didn't have before: a real run_plant(params,
# n_cycles) that actually drives the fin and returns real load-cell +
# encoder data, matching run_plant_SIMULATED's exact call/return signature
# so nothing else in force_control.py has to change.
#
# Driving the real rig means:
#   1. Publishing a 'forward_paddle ...' line on mission_input (the same
#      wire format scripts/sweep_common.py already uses) and waiting for a
#      matching 'ACHIEVED' on mission_status -- crab.py parses the line,
#      controller.py executes it via mc.paddle() (Layer 3, above).
#   2. Capturing load_cell_data (Fx/Fy/Fz, 10 kHz) and joint_feedback
#      (encoder position) for the duration of that mission.
#
# THE UNIT/CONVENTION TRAP (read this before changing anything below):
# force_control.py's own abstract parameters -- amp_ratio=A1/A2,
# freq_ratio=f1/f2 (PITCH/heave), delta_phi=phi1-phi2 -- use the same
# theta1/theta2/phi1/phi2 convention as the original bench-test claims
# analysis (independent phase per axis). This project's REAL mission_input
# fields do NOT use that convention:
#   * mc.paddle() (Layer 3, above) hardcodes the PITCH sinusoid's phase to
#     0.0 -- only HEAVE carries a phase term (its `pitch_phase` argument,
#     named for historical reasons, actually offsets heave). So phi1 must
#     be held at 0 here, and delta_phi = phi1 - phi2 = -phi2, i.e.
#     mission `phase` = -delta_phi.
#   * this project's stored/mission `freq_ratio` field is HEAVE/PITCH (see
#     mc.paddle's `heave_freq_ratio` argument) -- the RECIPROCAL of
#     force_control's own freq_ratio = f1/f2 = PITCH/heave. So
#     mission `freq_ratio` = 1 / (force_control's freq_ratio).
# Both conversions happen ONCE, here, in decode_params_to_mission() -- this
# is the only function in the whole HIL path that needs to know either of
# them. Get either sign/inversion backwards and the WITH-relationships
# controller's fixed gains (PRIMARY, in force_control.py) will push every
# tuning cycle in the wrong direction on real hardware while looking
# perfectly fine in the terminal log (the math is unaware of which physical
# direction "increase phase" actually corresponds to).

import math as _math

LOADCELL_SAMPLE_RATE_HZ = 10000.0   # must match load_cell_interface.py's 'sample_rate' param

# Low-pass cutoff for run_plant_HARDWARE, as a multiple of the commanded
# pitch frequency (see the filtering block there for why this exists).
# 10x keeps every hydrodynamic harmonic the bench analysis ever found
# meaningful (~4x and below) with wide margin, while sitting far under the
# ~29-30 Hz structural resonance measured on this rig.
RESONANCE_FILTER_HARMONIC = 10.0
LOADCELL_AXES = ('Fx', 'Fy', 'Fz', 'Tx', 'Ty', 'Tz')   # inner-dim order of load_cell_data


# Pitch-curve shape exponent commanded for every HIL mission. MUST match the
# value the bench sweep was recorded at, because every PRIMARY gain in
# force_control.py was fitted on that data: in_house_wet_test_3D was run
# entirely at cmd.pitch_k = 1.0 (verified against its recorded cmd.* columns).
# This was previously hardcoded to 0.0 in the mission line -- a pure sine --
# so every HIL run commanded a DIFFERENT pitch waveform than the one the
# gains describe. k=1 squares the pitch curve off slightly (see
# shaped_sine): k=0 is a plain sine, k->inf a square wave.
PITCH_K = 1.0


def decode_params_to_mission(params, n_cycles, label,
                             center_amp_deg=36.0, center_freq_hz=0.5):
    """
    Converts force_control's abstract [amp_ratio, freq_ratio, delta_phi]
    into a literal 'forward_paddle ...' mission_input line, applying the
    two convention conversions described in the module comment above.

    Uses the GEOMETRIC-MEAN-PRESERVING convention -- pitch = center*sqrt(r),
    heave = center/sqrt(r) -- identical to the in_house_wet_test_3D sweep
    (verified against its recorded cmd.* columns) and to
    force_control.decode_params(). This REPLACES an earlier "heave pinned at
    15deg, pitch = amp_ratio*15deg" scheme, which commanded roughly a
    QUARTER of the bench's motion on both axes and so could never reproduce
    the forces the gains were fitted to.

    center_amp_deg / center_freq_hz mirror force_control's CENTER_AMP_DEG /
    CENTER_FREQ_HZ -- passed in rather than imported, so this function has no
    dependency on force_control.py at all (force_control.py depends on THIS
    module, not the other way around).

    Returns (mission_line, pitch_freq_hz) -- the caller needs pitch_freq_hz
    separately to size a sensible ACHIEVED timeout.
    """
    # params is [amp_ratio, freq_ratio, delta_phi] or, since the addition of
    # an overall amplitude SCALE channel, [amp_ratio, freq_ratio, delta_phi,
    # scale]. Accept both -- scale defaults to 1.0 -- so a saved 3-element
    # result from before that change still replays correctly.
    p = list(params)
    while len(p) < 5:
        p = p + [1.0]
    amp_ratio, freq_ratio, delta_phi, scale, freq_scale = p[0], p[1], p[2], p[3], p[4]
    s_a = _math.sqrt(max(amp_ratio, 1e-9))
    s_f = _math.sqrt(max(freq_ratio, 1e-9))

    # scale multiplies BOTH amplitudes (it is the tip-speed / overall-force
    # lever); the ratio still sets how that total is split between the axes.
    pitch_amp_deg = center_amp_deg * s_a * scale
    heave_amp_deg = center_amp_deg / s_a * scale
    # freq_scale multiplies BOTH frequencies, moving their geometric mean
    # (which freq_ratio alone cannot do -- it only redistributes speed).
    pitch_freq_hz = center_freq_hz * s_f * freq_scale
    heave_phase_rad = -delta_phi                              # phi1 held at 0 -> phi2 = -delta_phi
    # mission 'freq_ratio' field is heave/pitch (the RECIPROCAL of
    # force_control's pitch/heave). heave_freq = pitch_freq * this
    #   = center*sqrt(r) * (1/r) = center/sqrt(r)  -- the geometric partner. Correct.
    mission_freq_ratio = (1.0 / freq_ratio) if abs(freq_ratio) > 1e-9 else 1.0

    pitch_amp_rad = _math.radians(pitch_amp_deg)
    heave_amp_rad = _math.radians(heave_amp_deg)

    line = (f"forward_paddle frequency:{pitch_freq_hz:.6f} pitch_amp:{pitch_amp_rad:.6f} "
            f"heave_amp:{heave_amp_rad:.6f} phase:{heave_phase_rad:.6f} "
            f"freq_ratio:{mission_freq_ratio:.6f} pitch_k:{PITCH_K:.6f} "
            f"cycles:{n_cycles} label:{label}")
    return line, pitch_freq_hz


def _make_hil_node_class():
    """Defined lazily (mirrors scripts/sweep_common.py's pattern exactly)
    so importing motion_command doesn't require rclpy unless a caller
    actually asks for the HIL bridge."""
    import threading
    import json as _json
    from rclpy.node import Node
    from std_msgs.msg import String, Float32MultiArray
    from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy

    class HILControlNode(Node):
        """Publishes mission_input, waits for the matching mission_status
        ACHIEVED, and buffers load_cell_data + joint_feedback for exactly
        the duration of one measurement -- the real-hardware counterpart
        of force_control.run_plant_SIMULATED."""

        def __init__(self):
            super().__init__('force_control_hil')
            latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 history=HistoryPolicy.KEEP_LAST)
            self.pub = self.create_publisher(String, 'mission_input', 10)
            self.create_subscription(String, 'mission_status', self._status_cb, 50)
            self.create_subscription(Float32MultiArray, 'load_cell_data', self._loadcell_cb, 50)
            self.create_subscription(Float32MultiArray, 'joint_feedback', self._feedback_cb, 50)
            self.create_subscription(String, 'robot_config', self._config_cb, latched)

            self._done = threading.Event()
            self._awaiting = None
            self._recording = False
            self._lc_buf = []    # [(t_s, Fx, Fy, Fz), ...]
            self._fb_buf = []    # [(t_s, {sid: pos_rad}), ...]
            self.pitch_id = None   # first fin's oscillating servo IDs, for the
            self.heave_id = None   # diagnostic servo-position plot only (not control)
            self.baseline = (0.0, 0.0, 0.0)   # (Fx0, Fy0, Fz0) rest-baseline, see
                                              # capture_rest_baseline() -- subtracted
                                              # from every run_plant_HARDWARE measurement
                                              # so descriptors reflect real forces, not
                                              # the mount's static weight/preload offset.

        def capture_rest_baseline(self, duration_s=1.5):
            """
            Records load_cell_data for `duration_s` with the servos assumed
            IDLE (call this right after calibration, before any paddle
            missions run) and stores the per-axis MEAN as self.baseline.
            run_plant_HARDWARE subtracts this from every subsequent Fx/Fy/Fz
            measurement -- without it, the raw loadcell reading includes a
            large constant mechanical offset (the mount's own weight/preload:
            observed on real hardware as Fy~+3.3N, Fz~-28N even at rest),
            which the "fy_net should be ~0" / "fz_skew should be ~0"
            secondary objectives can then never satisfy no matter how the
            servos are tuned -- that offset has nothing to do with kinematics.

            This mirrors the taring already done throughout the bench-test
            analysis (prove_claims.py's load_raw(), which subtracts the
            post-gait REST period's median) -- same idea, applied here to a
            dedicated pre-experiment rest window instead of a post-gait tail,
            since a HIL run doesn't have a "post-gait" segment to borrow one
            from.
            """
            # Wait for the load_cell_data subscription to actually be
            # delivering before timing the window. start_hil_node() only
            # sleeps 1s for DDS discovery, which is a RACE: if discovery is
            # still in flight the whole capture window can pass with zero
            # samples. That happened on a real run and, because the old code
            # only WARNED and returned a (0,0,0) baseline, the entire run
            # proceeded completely untared -- producing a plausible-looking
            # but false "no trough" result (trough_min ~ +2.8, which was just
            # the raw ~3.5 N mechanical offset). Never again: wait for real
            # samples, and hard-fail if none arrive.
            import time as _time
            deadline = _time.time() + 15.0
            while _time.time() < deadline:
                self.start_recording()
                _time.sleep(duration_s)
                lc_buf, _ = self.stop_recording()
                if lc_buf:
                    break
                self.get_logger().warn(
                    "capture_rest_baseline: no load-cell samples yet -- "
                    "waiting for load_cell_data and retrying")
                _time.sleep(1.0)
            if not lc_buf:
                raise RuntimeError(
                    "capture_rest_baseline: no load-cell samples after 15s. "
                    "Refusing to run untared -- every force descriptor would be "
                    "offset by the rig's mechanical bias (~3.5 N on Fx here). "
                    "Check that load_cell_interface is publishing 'load_cell_data'.")
            fx0 = sum(r[1] for r in lc_buf) / len(lc_buf)
            fy0 = sum(r[2] for r in lc_buf) / len(lc_buf)
            fz0 = sum(r[3] for r in lc_buf) / len(lc_buf)
            self.baseline = (fx0, fy0, fz0)
            self.get_logger().info(
                f"Rest baseline captured ({len(lc_buf)} samples over {duration_s}s): "
                f"Fx0={fx0:.4f} Fy0={fy0:.4f} Fz0={fz0:.4f} -- subtracting from all "
                f"subsequent measurements.")
            return self.baseline

        def _config_cb(self, msg):
            try:
                cfg = _json.loads(msg.data)
                sets = {}
                for entry in cfg.get('actuator_map', []):
                    sid, set_id = float(entry[0]), int(entry[1])
                    sets.setdefault(set_id, []).append(sid)
                first = next(iter(sets.values()))
                if len(first) >= 2:
                    self.pitch_id, self.heave_id = first[0], first[1]
            except Exception:
                pass   # diagnostic-only info; a parse miss just leaves theta plots empty

        def _status_cb(self, msg):
            try:
                d = _json.loads(msg.data)
            except Exception:
                return
            if (d.get('event') == 'ACHIEVED' and self._awaiting
                    and d.get('label') == self._awaiting):
                self._done.set()

        def _loadcell_cb(self, msg):
            if not self._recording:
                return
            now_ns = self.get_clock().now().nanoseconds
            data = msg.data
            n = len(data) // 6
            dt_ns = 1e9 / LOADCELL_SAMPLE_RATE_HZ
            # sample i within a packet sits at packet_time + i/sample_rate
            # (same convention scripts/split_missions.py uses on the bag).
            for s in range(n):
                base = s * 6
                self._lc_buf.append((
                    (now_ns + s * dt_ns) / 1e9,
                    data[base + 0], data[base + 1], data[base + 2],
                ))

        def _feedback_cb(self, msg):
            if not self._recording:
                return
            now_s = self.get_clock().now().nanoseconds / 1e9
            data = msg.data
            pos = {}
            for i in range(0, len(data) - 5, 6):
                pos[data[i]] = data[i + 2]
            self._fb_buf.append((now_s, pos))

        def start_recording(self):
            self._lc_buf, self._fb_buf = [], []
            self._recording = True

        def stop_recording(self):
            self._recording = False
            return list(self._lc_buf), list(self._fb_buf)

        def send(self, line, label, timeout):
            self._awaiting = label
            self._done.clear()
            m = String(); m.data = line
            self.pub.publish(m)
            ok = self._done.wait(timeout=timeout)
            self._awaiting = None
            return ok

    return HILControlNode


def start_hil_node():
    """Brings up rclpy + one HILControlNode, spinning in a background
    thread -- mirrors scripts/sweep_common.py's start_ros()/stop_ros()
    exactly. Call once before any run_plant_HARDWARE() calls; pass the
    returned node into run_plant_HARDWARE via force_control.main(hardware=True)."""
    import threading
    import rclpy
    rclpy.init()
    node = _make_hil_node_class()()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    import time as _time
    _time.sleep(1.0)   # let discovery settle before the first mission_input publish
    return node


def stop_hil_node(node):
    import rclpy
    node.destroy_node()
    rclpy.shutdown()


def run_plant_HARDWARE(params, n_cycles=1, node=None):
    """
    THE real-hardware plant, matching force_control.run_plant_SIMULATED's
    signature and return contract exactly: (params, n_cycles=1) ->
    (t, Fx, Fy, Fz, theta1, theta2), all numpy arrays except params.

    Sequence: encode params -> mission line (see decode_params_to_mission
    and the unit/convention comment above), arm the node's buffers, publish
    and wait for ACHIEVED, snapshot the buffers, convert to numpy, align
    encoder feedback onto the load-cell time grid (nearest-sample; feedback
    arrives far less often than the 10kHz load-cell stream). theta1/theta2
    are diagnostic-plot-only in force_control.py (never fed to the cost
    function), so a best-effort nearest-match here is sufficient -- Fx/Fy/Fz
    (which DO drive the controller) come directly off the real load-cell
    stream with proper per-sample timestamps.

    A missed ACHIEVED (the mission_status packet dropped, but the motion
    almost certainly still happened) is NOT fatal here -- this matches the
    established behavior of scripts/sweep_common.py's SweepNode.send(),
    which logs a warning and continues rather than aborting an entire sweep
    over one flaky status message. Only raises if the load-cell buffer is
    also empty (i.e. something is actually wrong, not just a dropped ACK).

    Raises RuntimeError only if no load-cell samples were captured at all
    (load_cell_interface not publishing, or the mission truly never ran) --
    fail loudly rather than silently feeding the controller an empty/zero
    measurement it would happily "optimize" against.
    """
    import numpy as _np

    if node is None:
        raise RuntimeError("run_plant_HARDWARE needs a live HILControlNode "
                           "(see start_hil_node()) -- pass node=...")

    run_plant_HARDWARE._counter = getattr(run_plant_HARDWARE, '_counter', 0) + 1
    label = f"HIL_{run_plant_HARDWARE._counter:05d}"

    line, pitch_freq_hz = decode_params_to_mission(params, n_cycles, label)
    timeout = (n_cycles / pitch_freq_hz if pitch_freq_hz > 1e-6 else 4.0) * 1.6 + 8.0

    node.start_recording()
    ok = node.send(line, label, timeout=timeout)
    lc_buf, fb_buf = node.stop_recording()

    if not ok:
        node.get_logger().warn(
            f"mission '{label}' ({line}) did not report ACHIEVED within "
            f"{timeout:.1f}s -- continuing with whatever was captured "
            f"(status message likely dropped, not a real failure)")
    if not lc_buf:
        raise RuntimeError(f"no load-cell samples captured for mission '{label}' "
                           f"-- is load_cell_interface publishing 'load_cell_data', "
                           f"or did the mission genuinely never run?")

    lc_buf.sort(key=lambda r: r[0])
    fx0, fy0, fz0 = node.baseline
    t_abs = _np.array([r[0] for r in lc_buf])
    Fx = _np.array([r[1] for r in lc_buf]) - fx0
    Fy = _np.array([r[2] for r in lc_buf]) - fy0
    Fz = _np.array([r[3] for r in lc_buf]) - fz0

    # ---- structural-resonance removal -------------------------------------
    # The raw 10 kHz load-cell stream carries a large ~29-30 Hz component
    # that is NOT hydrodynamic force: at a commanded pitch frequency near
    # 0.7 Hz it sits at roughly 40x the driving rate, and an FFT of a real
    # captured cycle showed it at nearly the same magnitude as the genuine
    # 1x/2x force content (545 vs 613). It is mount/gear/structure ringing,
    # and with no filtering anywhere in the HIL path it landed directly in
    # every descriptor and every plot -- which is what made the measured
    # force curves look like noise rather than waveforms.
    #
    # Zero-phase 4th-order Butterworth (filtfilt, so no group delay is
    # introduced and peak TIMING is preserved -- important because skew and
    # waveform_match both depend on where features sit within the cycle).
    # Cutoff at RESONANCE_FILTER_HARMONIC x the commanded pitch frequency:
    # high enough to pass every plausible hydrodynamic harmonic (the bench
    # analysis never found meaningful Fx content above ~4x), far enough
    # below the ~29 Hz ringing to attenuate it by orders of magnitude.
    # Applied BEFORE the cycle trim below so filtfilt has the full captured
    # buffer to work with and its edge transients fall outside the kept window.
    if pitch_freq_hz > 1e-6 and len(Fx) > 32:
        fs = 1.0 / _np.median(_np.diff(t_abs)) if len(t_abs) > 1 else 0.0
        cutoff = RESONANCE_FILTER_HARMONIC * pitch_freq_hz
        if fs > 2.5 * cutoff:
            from scipy.signal import butter, filtfilt
            b, a = butter(4, cutoff / (fs / 2.0), btype="low")
            Fx, Fy, Fz = filtfilt(b, a, Fx), filtfilt(b, a, Fy), filtfilt(b, a, Fz)

    # Trim to the last n_cycles periods by REAL TIME (not sample count).
    # The capture window starts the instant we arm the buffer, which is
    # before the mission_input round-trip (crab -> controller) actually
    # starts the servo moving -- so the raw buffer has leading dead-time
    # the simulated plant never has. collect_steady_measurement (in
    # force_control.py, unchanged) slices "last cycle" by raw sample count,
    # which assumes the whole array IS exactly n_cycles of clean motion;
    # left untrimmed, that naive slice can land on a start/end discontinuity
    # and produce a bogus FFT result (observed on real hardware: a
    # dominant_frequency() reading of ~16Hz on a 0.65Hz commanded motion).
    # Trimming here, once, keeps that downstream logic correct without
    # having to touch it.
    if pitch_freq_hz > 1e-6:
        period_s = 1.0 / pitch_freq_hz
        keep_from = t_abs[-1] - n_cycles * period_s
        mask = t_abs >= keep_from
        if mask.sum() >= 20:   # keep the untrimmed capture if trimming would leave too little
            t_abs, Fx, Fy, Fz = t_abs[mask], Fx[mask], Fy[mask], Fz[mask]

    t = t_abs - t_abs[0]

    theta1 = _np.zeros_like(t)
    theta2 = _np.zeros_like(t)
    if fb_buf and node.pitch_id is not None and node.heave_id is not None:
        fb_buf.sort(key=lambda r: r[0])
        fb_t = _np.array([r[0] for r in fb_buf]) - t_abs[0]
        fb_p1 = _np.array([r[1].get(node.pitch_id, 0.0) for r in fb_buf])
        fb_p2 = _np.array([r[1].get(node.heave_id, 0.0) for r in fb_buf])
        # nearest-sample alignment onto the (much finer) load-cell time grid
        idx = _np.clip(_np.searchsorted(fb_t, t), 0, len(fb_t) - 1)
        theta1, theta2 = fb_p1[idx], fb_p2[idx]

    return t, Fx, Fy, Fz, theta1, theta2
