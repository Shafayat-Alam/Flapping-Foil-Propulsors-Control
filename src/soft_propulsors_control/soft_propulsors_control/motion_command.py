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

def flap(roll_id, pitch_id, t: float, freq: float, amp: float,
         phase: float = 0.0, waveform=sine,
         pitch_center: float = 0.0, **kwargs) -> dict:
    """
    Flapping gait — pitch oscillation with the roll servo left untouched.

    Only the pitch servo is commanded: it oscillates about ``pitch_center``
    (the standby pose, 0 in extended mode) with the chosen ``waveform`` (amplitude
    ``amp`` in rad, ``freq`` set by the controller).  The roll servo is
    deliberately NOT commanded, so it holds whatever position it was last driven
    to (or keeps following the IMU) — the caller publishes only the pitch servo
    so the omitted roll is left alone rather than reset.

    ``roll_id`` is accepted for call-site symmetry with the other gaits but is
    intentionally unused (roll is held, not set).

    Args:
        roll_id : this fin's roll servo — accepted but not commanded (held)
        pitch_id : this fin's pitch servo — the one that oscillates
        t, freq, amp, phase : standard waveform parameters (amp in rad)
        waveform : Layer-1 function or name selecting the pitch shape
        pitch_center : oscillation midpoint (rad; standby pose = 0 in extended mode)

    Extra ``kwargs`` (e.g. ``duty``, ``ramp``) are forwarded to the waveform.

    Returns: ``{pitch_id: pitch_value}``  (roll omitted → held at current pos)
    """
    wave = get_waveform(waveform)
    pitch_value = pitch_center + wave(t, freq, amp, phase, **kwargs)
    return drive(pitch_id, pitch_value)


def paddle(roll_id, pitch_id, tau: float, freq: float,
           roll_amp: float, pitch_amp: float,
           roll_center: float = 0.0, pitch_center: float = 0.0,
           pitch_phase: float = 0.0, **kwargs) -> dict:
    """
    Sine paddling gait — roll and pitch are both sinusoids at the same frequency
    ``freq`` (Hz), about the rest pose (roll_center, pitch_center).  The pitch
    curve is phase-shifted from roll by ``pitch_phase`` (rad); that shift sets
    the roll/pitch phasing that produces thrust.  Roll and pitch amplitudes are
    independent (A_r, A_p) — the controller scales them from separate efforts.

        roll_value  = roll_center  + A_r · sin(2π·freq·tau)
        pitch_value = pitch_center + A_p · sin(2π·freq·tau + pitch_phase)

    ``roll_amp`` (A_r) is SIGNED — its sign flips the roll sinusoid to reverse
    thrust (forward fin +A_r, reversed fin −A_r).  Sinusoids are naturally
    continuous, so multiple cycles flow smoothly with no reset.  ``tau`` is
    mission-relative time (s).

    Returns: ``{roll_id: roll_value, pitch_id: pitch_value}``
    """
    roll_value = roll_center + sine(tau, freq, roll_amp, 0.0)
    pitch_value = pitch_center + sine(tau, freq, pitch_amp, pitch_phase)
    targets = drive(roll_id, roll_value)
    targets.update(drive(pitch_id, pitch_value))
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
