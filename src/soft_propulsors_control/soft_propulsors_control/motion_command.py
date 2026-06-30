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
         phase: float = 0.0, waveform=sine, roll_angle: float = math.pi / 2.0,
         **kwargs) -> dict:
    """
    Flapping gait — broadside thrust.

    The roll servo is held at ``roll_angle`` (default π/2) so the fin presents
    maximum surface area to the flow, and the pitch servo oscillates around 0
    with the chosen ``waveform``.  Think of holding an open palm perpendicular
    to the flow and waving the whole hand.

    Args:
        roll_id, pitch_id : servo IDs for this fin
        t, freq, amp, phase : standard waveform parameters (amp in rad)
        waveform : Layer-1 function or name selecting the pitch shape
        roll_angle : fixed roll hold position in rad (broadside exposure)

    Extra ``kwargs`` (e.g. ``duty``, ``ramp``) are forwarded to the waveform.

    Returns: ``{roll_id: roll_angle, pitch_id: pitch_value}``
    """
    wave = get_waveform(waveform)
    pitch_value = wave(t, freq, amp, phase, **kwargs)
    targets = drive(roll_id, roll_angle)
    targets.update(drive(pitch_id, pitch_value))
    return targets


def paddle(roll_id, pitch_id, t: float, freq: float, amp: float,
           phase: float = 0.0, power_fraction: float = 0.5,
           power_angle: float = math.pi / 2.0, feather_angle: float = 0.0,
           **kwargs) -> dict:
    """
    Paddling gait — rowing stroke with a graceful, low-drag recovery.

    One cycle has two smoothly-eased phases:

      * Power stroke   (0 → ``power_fraction``): roll is held broadside at
        ``power_angle`` for maximum surface area while pitch sweeps from -amp
        to +amp, pushing water.
      * Recovery stroke (``power_fraction`` → 1): roll rotates to
        ``feather_angle`` (edge-on, minimum drag) while pitch slips back from
        +amp to -amp.  Nothing snaps — both strokes are eased.

    Drag asymmetry comes from feathering the fin on recovery, not from a fast
    return, so the whole motion stays graceful.

    Args:
        roll_id, pitch_id : servo IDs for this fin
        t, freq, amp, phase : standard parameters (amp in rad)
        power_fraction : fraction of the cycle spent on the power stroke (0..1)
        power_angle : roll hold during the power stroke (broadside, rad)
        feather_angle : roll hold during recovery (edge-on, rad)

    Returns: ``{roll_id: roll_value, pitch_id: pitch_value}``
    """
    power_fraction = max(0.05, min(0.95, power_fraction))
    cycle = (freq * t + phase / TWO_PI) % 1.0

    if cycle < power_fraction:
        # Power stroke: broadside roll, eased pitch sweep -amp -> +amp
        s = ease(cycle / power_fraction)
        roll_value = power_angle
        pitch_value = amp * (2.0 * s - 1.0)
    else:
        # Recovery stroke: feathered roll, eased pitch slip +amp -> -amp
        s = ease((cycle - power_fraction) / (1.0 - power_fraction))
        roll_value = power_angle + (feather_angle - power_angle) * ease(
            # feather quickly at the start of recovery, hold edge-on, then
            # return to broadside as the next power stroke approaches
            math.sin(math.pi * s)
        )
        pitch_value = amp * (1.0 - 2.0 * s)

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
