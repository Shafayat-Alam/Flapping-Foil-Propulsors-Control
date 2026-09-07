"""
================================================================================
GREY-BOX RUN-TO-RUN CONTROLLER FOR FLAPPING-FIN KINEMATIC TUNING
================================================================================

PURPOSE
-------
Given a desired Fx(t) force curve shape (described in a JSON file) and a set
of servo sinusoids (pitch + heave), find the sinusoid parameters that produce
that force curve on the real fin/loadcell rig -- using a small amount of
known physics (from prior data analysis) plus live measurement feedback.

WHY "GREY-BOX"
--------------
- WHITE-box would mean a full first-principles fluid-structure-interaction
  model. We don't have one (per the PDF, Fz specifically is described as "a
  consequence of fluid interaction" that can't be predicted from kinematics
  alone).
- BLACK-box would mean no assumed structure at all -- pure trial and error.
- GREY-box (this file) means: we DO trust a few specific, previously-proven
  relationships (claims 1-6 from the bench data: which servo ratio moves
  which force descriptor, and in which direction), and we use REAL-TIME
  loadcell feedback to correct for everything those relationships don't
  cover or get slightly wrong.

THE THREE KINEMATIC PARAMETERS BEING TUNED
--------------------------------------------
  amp_ratio  = A1 / A2   (pitch amplitude / heave amplitude)
  freq_ratio = f1 / f2   (pitch frequency / heave frequency)
  delta_phi  = phi1 - phi2   (phase difference between pitch and heave)
These three numbers are all the controller ever changes. Everything else
(actual servo angles, actual Hz, actual phase in radians) is derived from
them via decode_params().

WHAT THE JSON FILE DESCRIBES
-----------------------------
1. "channel_definitions" -> "Fx" -> "target_points": a handful of (time,
   force) points that get interpolated into a full desired Fx(t) curve
   shape. This is ONLY used to visually check "did I describe the shape I
   meant" (see plot_target_curve) and, as a fallback, to figure out how many
   peaks per cycle you want if you don't specify peak_count explicitly.

2. "objectives": the actual math the controller optimizes against. Each
   entry is one of:
     {"type": "target",   "value": <number>, "weight": <number>}
     {"type": "maximize", "weight": <number>}
     {"type": "minimize", "weight": <number>}
   A descriptor NOT listed in "objectives" is simply not being optimized --
   its weight is implicitly 0, so it contributes nothing to the cost and
   (in the WITH-relationships controller) the kinematic parameter tied to it
   never moves.

WHAT Fy AND Fz ARE, AND WHY THEY'RE NOT IN THE JSON
------------------------------------------------------
The loadcell measures three force components: Fx (thrust), Fy (lateral),
Fz (heave/vertical). Only Fx is ever something you hand-design a shape for.
Fy and Fz instead get two FIXED, HARDCODED objectives (SECONDARY_OBJECTIVES
below) that apply to every run regardless of what's in the JSON:
  - Fy should have ~zero net thrust over a cycle (no unwanted net sideways
    push -- the fin should generate forward thrust, not drift sideways).
  - Fz should be symmetric about zero (no net skew up or down).
These are monitored every cycle and folded into the same cost function as
the Fx objectives, but (in WITH-relationships mode) they are NEVER used to
move a parameter, because no claim from the prior data analysis says which
parameter controls them -- the PDF explicitly says Fz is a "consequence of
fluid interaction" that kinematics can't directly command.

HOW COST WORKS (the actual JSON math)
----------------------------------------
Every objective, JSON-declared or hardcoded, contributes one term to a
single scalar "cost" number that the whole system is trying to reduce:
    target:    cost_term = weight * (target_value - measured_value)^2
    maximize:  cost_term = -weight * measured_value
    minimize:  cost_term =  weight * measured_value
"target" costs are bounded below at 0 (perfect match = 0 cost). "maximize"
and "minimize" costs are UNBOUNDED -- they keep improving forever unless a
physical parameter bound stops them. This matters for the convergence
check (see PLATEAU_TOL below).

For the WITH-relationships controller specifically, we also need to know
which DIRECTION to push a parameter, not just how good the current cost is.
That direction is the negative gradient of the cost term with respect to
the MEASURED value (not the parameter -- see objective_term_and_signal):
    target:    signal = weight * (target_value - measured_value)
    maximize:  signal = weight                (always "push it up")
    minimize:  signal = -weight               (always "push it down")
This signal is then multiplied by a GAIN (the physical, data-derived fact
of how much the parameter actually moves that measured value) to get the
actual parameter update. This is why "signal" and "gain" are kept as two
separate concepts in the code: signal = "which way and why", gain =
"how much that push actually moves the world".

TWO CONTROLLER MODES
---------------------
Both modes follow the same loop shape: u(k+1) = u(k) + step(k), i.e. "take
the current parameters, look at the measured result, adjust, repeat."
They differ ONLY in how step(k) is computed:

  WITH relationships (controller_step_WITH_relationships):
    step(k) = fixed_gain * signal(k)
    Uses the fixed gains in PRIMARY (below), which encode "amp_ratio moves
    peak_height by this much, freq_ratio moves rate by this much, etc." --
    facts established from the prior bench data analysis (claims 1-6).
    Only 1 plant measurement needed per cycle, because we already know
    which way to move each parameter without having to test it.

  WITHOUT relationships (controller_step_WITHOUT_relationships):
    No assumed gain or sign at all. For each of the 3 parameters, nudge it
    slightly, re-measure, and see whether TOTAL cost went up or down. Move
    a small FIXED step in whichever direction helped. This needs 3 extra
    plant measurements per cycle (one perturbation per parameter) on top of
    the baseline measurement, so it's ~4x slower per cycle than the
    WITH-relationships mode -- but it's the only mode that can improve an
    objective like trough_min, which has no known gain relationship at all.

STOPPING RULE (best-so-far, patience-based -- no fixed target)
------------------------------------------------------------------
Every objective's accuracy is a [0,1] fraction (100% = perfect):
  target objective:            accuracy = 1 - |value - measured| / |value|
  maximize/minimize objective: accuracy = measured/reference or reference/measured
                                (requires an explicit "reference" in the JSON --
                                 there's no natural 100% point for an unbounded
                                 objective otherwise; objectives without one are
                                 excluded from accuracy and reported as such)
Overall accuracy is the weighted average across every objective that has one
(JSON objectives with a value/reference + peak_count exact-match + the two
hardcoded secondary objectives, which always have a value).

There is NO fixed accuracy target. The loop always tries to push accuracy
higher and tracks the best accuracy (and the full cycle record) seen so
far. It stops once PATIENCE_CYCLES pass in a row without beating that best
(an improvement smaller than IMPROVEMENT_TOL doesn't count, to avoid
stopping/resetting on pure measurement noise). The reported final result is
the BEST cycle found, not necessarily the last one run -- the last few
cycles before stopping are, by construction, cycles that didn't improve on
it. MAX_ITERS remains a hard cap in case accuracy never plateaus at all.

THE MEASUREMENT PROTOCOL (real-hardware realism)
----------------------------------------------------
Every single plant measurement, in EITHER controller mode, goes through
collect_steady_measurement(), which:
  1. Waits SETTLE_WAIT_S seconds (mechanical/fluid transient settling time
     after commanding new servo parameters).
  2. Runs the plant for N_CYCLES_PER_MEASUREMENT full periods.
  3. Keeps ONLY the last period and discards the rest ("drop first cycle"),
     matching the same steady-state protocol used in the original bench
     tests, so a leftover transient from the PREVIOUS parameter setting
     never contaminates the measurement of the new one.

FILE STRUCTURE (5-stage pipeline)
------------------------------------
  1. INGEST    - read the JSON file
  2. INTERPRET - turn target_points into a curve, extract/define objectives
  3. VERIFY    - plot the target curve so you can visually confirm the JSON
                 actually describes the shape you meant, before tuning starts
  4. CONTROL   - run the closed loop (either mode) with live terminal logging
  5. REPORT    - print final sine parameters + timing, plot final results

HOW TO POINT THIS AT REAL HARDWARE
--------------------------------------
run_plant_SIMULATED() stands in for the real rig with a toy physics model
(clearly NOT a real fluid-structure model -- just enough to make the whole
pipeline runnable end to end for testing).

The real-hardware bridge is NOT implemented in this file -- it lives in
soft_propulsors_control.motion_command (see the "GREY-BOX HIL BRIDGE"
section appended at the end of that file), because driving the plant means
publishing mission_input / reading load_cell_data + joint_feedback, which
requires a live rclpy node. To run on real hardware:

    python3 scripts/force_control.py target_curve_drag_dominant.json with --hardware

This launches an HILControlNode (see motion_command.py), and passes
mc.run_plant_HARDWARE (bound to that node) as run_plant into
run_control_loop() -- nothing else in this file changes; decode_params()
below is still the ONLY place that knows the abstract/physical mapping
(amp_ratio, freq_ratio, delta_phi -> A1/f1/phi1/A2/f2/phi2), and
motion_command's HIL bridge is responsible for converting THAT into the
actual mission_input wire format (which uses different unit/sign
conventions -- see the long comment in motion_command.decode_params_to_mission
for exactly why simply passing these numbers through would be wrong).
================================================================================
"""

import os
import sys
import time
import json
import math
import numpy as np
from scipy.interpolate import PchipInterpolator
from scipy.stats import skew as scipy_skew
import matplotlib
matplotlib.use("Agg")   # headless backend -- no display needed, we only save PNGs
import matplotlib.pyplot as plt

# Absolute workspace root (same convention as crab_launch.py) -- lets this
# script (a) find target_curve_*.json in the project root regardless of cwd,
# and (b) import the soft_propulsors_control package for the hardware bridge
# without depending on the caller having sourced the ROS workspace first.
WORKSPACE_ROOT = '/home/shafa/soft-propulsors-control'


# ============================================================================
# FIXED KNOWLEDGE FROM YOUR DATA ANALYSIS
# Every number in this block is derived directly from the in_house_wet_test_3D
# 3D sweep (in_house_wet_test_3D/coupling/*_data.csv), NOT guessed or carried
# over from an earlier placeholder version -- see the derivation notes next
# to each one, including the honest R^2 / sample-count caveats. It is only
# ever used by the WITH-relationships controller mode -- the
# WITHOUT-relationships mode ignores all of it and discovers directions
# empirically instead.
#
# STRUCTURAL CAVEAT (applies to every amp_ratio-tied gain below): the
# in_house_wet_test_3D sweep varied pitch_amp AND heave_amp OPPOSITELY
# (geometric-mean-preserving: pitch_amp=center*sqrt(ratio), heave_amp=
# center/sqrt(ratio), so both move every time "amp_ratio" changes).
# decode_params() in THIS file holds heave_amp FIXED (A2_BASE) and only
# varies pitch_amp via amp_ratio -- a different kinematic sweep than the
# one these gains were measured on. The measured SIGN/direction is still
# the best available evidence (it reflects a real, physical asymmetry
# between how pitch vs heave motion drives Fx), but the exact MAGNITUDE may
# not transfer perfectly, since the bench data can't isolate "pitch_amp
# alone" from "heave_amp shrinking at the same time." Treat these gains as
# directionally-informed starting points, not precisely-calibrated physics.
# ============================================================================

# Maps each Fx descriptor that HAS a known relationship to a LIST of
# (index into the params array, gain) pairs -- a descriptor may respond to
# MORE than one parameter (this is new: the original design only allowed
# one). Gain is d(descriptor)/d(parameter) -- a PHYSICAL fact, always the
# same sign regardless of what the JSON objective is trying to do with it.
# controller_step_WITH_relationships applies the SAME push_signal to every
# (idx, gain) pair listed for a descriptor.
#
# Below is the result of a FULL systematic cross-check -- every descriptor
# regressed against EVERY one of the 3 parameters (not just the one each
# was assumed to belong to) -- recomputed directly from raw
# in_house_wet_test_3D/split_data using this file's OWN descriptor
# definitions (peak_to_peak, safe_skew, dominant_frequency's logic, etc.),
# so these numbers match exactly what gets measured at runtime:
#
#                 amp_ratio          freq_ratio         delta_phi
#   peak_height   -0.437 (R2=.626)   -0.347 (R2=.626)   -0.023 (R2=.006)
#   trough_min    +0.346 (R2=.674)   -0.009 (R2=.002)   -0.030 (R2=.020)
#   skew          +0.210 (R2=.756)   +0.072 (R2=.104)   +0.096 (R2=.225)
#   rate*         n/a                +0.057 (R2=.899)   n/a
#   peak_count    weak/discrete, R2<0.22 on all three -- see STABLE_BANDS instead
# (*rate's freq_ratio number is from the restricted fit, see below; amp_ratio/
#  delta_phi were not usefully regressable against a continuous frequency)
#
# THE BIG SURPRISE: skew is driven far more strongly by amp_ratio (R2=.756)
# than by delta_phi (R2=.225) -- the OPPOSITE of the original theoretical
# framing (claims 5&6: phase -> skew). Both relationships are real, so BOTH
# are kept below (that's why this dict's values are now lists, not single
# tuples) -- but be aware: if a JSON declares peak_height/trough_min/skew
# together, all three now push on params[0] (amp_ratio) simultaneously.
# That's the honest, data-driven picture (amp_ratio really is the dominant
# lever for Fx's overall shape in this dataset), not a bug.
PRIMARY = {
    # amp_ratio -> Fx peak-to-peak height. THE SIGN IS NEGATIVE --
    # increasing amp_ratio (pitch amp relative to fixed heave amp)
    # measurably SHRINKS Fx's peak-to-peak swing over the tested range
    # (0.33-3.0), the opposite of the original placeholder (+0.08), which
    # had never been checked against real data. R^2=0.63 moderate.
    "peak_height": [(0, -0.437)],
    # freq_ratio -> Fx fundamental frequency (Hz). Using all 7 rate_data.csv
    # points gives a noisy fit (R^2=0.53) because the two lowest tested
    # ratios (0.4, 0.5) show a sharp, likely resonance-contaminated drop
    # (flagged during the original claims analysis). Restricting to
    # freq_ratio>=0.667 (5 points, near-flat band around 1.0-1.1Hz) gives
    # R^2=0.90 -- used here, but the true relationship is closer to "flat,
    # locked near 2x pitch frequency" than truly proportional; this is a
    # weak local linearization, not a strong physical law.
    "rate": [(1, 0.057)],
    # skew (scipy.stats.skew(Fx), NOT the psi/biphase metric used
    # elsewhere) responds to BOTH amp_ratio (strong, R^2=0.756) and
    # delta_phi (weak, R^2=0.225) -- both listed, signal applied to both
    # every cycle. The delta_phi number: regressed against
    # delta_phi_rad=-phase_deg*pi/180 (mc.paddle() hardcodes pitch's own
    # phase to 0, so delta_phi=-phi2; see motion_command.py's
    # decode_params_to_mission comment) -- same direction as the original
    # placeholder (+0.02) but noisy point-to-point (raw skew swings -0.96
    # to +0.79 across the sweep). Without the amp_ratio term, delta_phi
    # would have gotten NO signal from ANY descriptor -- keeping the weak
    # link here is what still lets skew objectives move phase at all.
    # Signs below are for the FX_SIGN-corrected (negated) Fx. skew(-x) ==
    # -skew(x) exactly, so both terms are a clean sign flip of the values
    # fitted on the raw data (+0.210 / +0.096), R^2 unchanged at 0.76/0.23.
    "skew": [(0, -0.210), (2, -0.096)],
    # amp_ratio -> Fx's trough (float(np.min(Fx)), signed). HAD NO PRIMARY
    # ENTRY AT ALL BEFORE -- only reachable by the slow WITHOUT mode.
    # Checked against all three parameters: amp_ratio clearly wins
    # (R^2=0.67 vs 0.002 and 0.020 for freq_ratio/delta_phi). Same axis as
    # peak_height, OPPOSITE sign: as amp_ratio increases, the peak comes
    # down AND the trough comes up -- Fx's whole envelope compresses
    # toward its mean.
    # NOT a sign flip: under negation trough_min = min(-Fx) = -max(Fx), a
    # DIFFERENT quantity from -min(Fx), so this was re-fitted directly on the
    # negated bench data rather than having its sign flipped. Doing the naive
    # flip (-0.346) would have been both the wrong magnitude and the wrong
    # sign. Re-fit gives +0.0907, and note R^2 drops 0.67 -> 0.38, so this is
    # now a notably weaker relationship than it looked before.
    "trough_min": [(0, 0.0907), (3, -1.999)],
    # trough_depth is trough_min clamped at 0, so while a dip exists it moves
    # with exactly the same physics and takes the same channels.
    "trough_depth": [(0, 0.0907), (3, -1.999)],
    # trough_frac is scale-invariant BY CONSTRUCTION, so it deliberately gets
    # NO scale channel -- wiring one would just re-create the shrink-the-
    # waveform shortcut the ratio exists to close. amp_ratio is the only
    # seeded lever, since that is what redistributes motion between the two
    # servos and so actually changes waveform asymmetry rather than size.
    # The gain adapter will re-identify this from live data once it has 5+
    # cycles, and the model-free phase can move any parameter regardless.
    "trough_frac": [(0, -0.05)],
    # same physical mechanism (waveform asymmetry) as trough_frac, so it
    # shares its channel; the model-free phase explores delta_phi for it.
    "trough_frac_pre": [(0, -0.05)],
    # The (3, -1.999) scale channel is the one that actually matters for
    # "minimise the trough", and it was measured directly on this rig rather
    # than on the bench grid: over a run where the peak_height ratchet drove
    # scale 1.0 -> 1.4, trough_min tracked it at -1.999 N per unit scale with
    # R^2=0.987. Without it trough_min could only act through amp_ratio
    # (+0.0907, R^2=0.38) while peak_height pushed scale the other way at
    # +1.25 -- so the trough was guaranteed to deepen no matter what. With
    # both channels present the two objectives genuinely trade against each
    # other through the same parameter and the optimiser can balance them.
    # NOTE the gains above are 1D-slice fits (freq_ratio=1.0/phase=0 locked
    # etc). Re-fitting them over the FULL 3D grid shows they are far weaker
    # than those slices implied -- peak_height<-amp_ratio R^2 0.626 -> 0.128,
    # trough_min<-amp_ratio 0.674 -> 0.062, skew<-amp_ratio 0.756 -> 0.007,
    # skew<-delta_phi 0.225 -> 0.004. A slice through the baseline is simply
    # not representative of the interior the controller actually traverses.
    # The multivariate full-grid fit (descriptor ~ a*amp + b*freq + c*dphi)
    # reaches only R^2=0.408 for peak_height and R^2=0.011 for skew, i.e.
    # the three ratios genuinely do NOT determine skew at all. The signs
    # are retained above (they agree between slice and full grid, and sign
    # is all the controller needs to move the right way) but do not read
    # these magnitudes as calibrated physics.
    #
    # params[3] = scale -> Fx peak-to-peak. This is the one STRONG lever in
    # the dataset. peak_height's best single predictor over the full grid is
    # heave tip speed 2*pi*f_heave*A_heave (R^2=0.437, slope +0.570 N per
    # rad/s; pitch tip speed R^2=0.300; both beat every ratio). scale
    # multiplies both amplitudes, hence both tip speeds, directly:
    #     d(peak_height)/d(scale) = 0.570 * (typical heave tip speed ~2.2 rad/s)
    #                             ~ +1.25 N per unit scale
    # Binned means confirm the trend is real and monotonic, not a fit
    # artifact -- heave tip speed 0-1 rad/s: mean peak_height 2.17 N;
    # 1-2: 2.40; 2-3: 3.00; 3-4: 3.70; 4-6: 4.17 (max observed 7.52 N).
    "peak_height_scale": [(3, 1.25)],
    # Fy peak-to-peak vs amp_ratio, fitted on the bench sweep:
    # slope -1.757 N per unit amp_ratio, R^2=0.57 -- a STRONGER relationship
    # than the equivalent one for Fx (-0.437, R^2=0.63 on a 1D slice, far
    # weaker on the full grid). Scale drives both axes' swing together.
    "fy_p2p":       [(0, -1.757), (3, 1.25)],
}
# 'peak_height_scale' is not a separate measurable descriptor -- it is an
# ADDITIONAL control channel for the existing peak_height objective, letting
# it drive params[3] as well as params[0]. controller_step_WITH_relationships
# maps any PRIMARY key of the form '<descriptor>_<suffix>' back to
# '<descriptor>' when looking up the objective and its measured value.
# STRUCTURAL CAVEAT (applies to every amp_ratio-tied gain above): the
# in_house_wet_test_3D sweep varied pitch_amp AND heave_amp OPPOSITELY
# (geometric-mean-preserving: pitch_amp=center*sqrt(ratio), heave_amp=
# center/sqrt(ratio), so both move every time "amp_ratio" changes).
# decode_params() in THIS file holds heave_amp FIXED (A2_BASE) and only
# varies pitch_amp via amp_ratio -- a different kinematic sweep than the
# one these gains were measured on. The measured SIGN/direction is still
# the best available evidence (it reflects a real, physical asymmetry
# between how pitch vs heave motion drives Fx), but the exact MAGNITUDE may
# not transfer perfectly. Treat these gains as directionally-informed
# starting points, not precisely-calibrated physics.
#
# NOTE on Fy (lateral force): not wired into PRIMARY -- this file's design
# deliberately never lets Fy influence the controller (see module docstring,
# "Fy and Fz... hardcoded... NEVER used to move a parameter"). Documented
# here in case that decision is revisited: Fy_p2p vs amp_ratio (same
# scaling_data.csv) shows an even STRONGER relationship than Fx_p2p does --
# slope=-1.757 N/unit amp_ratio, R^2=0.57. amp_ratio suppresses BOTH axes'
# swing, not just Fx's.

# freq_ratio bands (force_control's pitch/heave convention) that reliably
# produce a given Fx peak count. RE-DERIVED from the FULL 3D grid
# (in_house_wet_test_3D/control_model_data.csv, 324 missions spanning
# freq_ratio x amp_ratio x phase), replacing an earlier version fitted to
# only the 3 freq values that one narrow slice happened to contain.
# Measured peak_count distribution, marginalized over all amp_ratio and
# phase combinations at each tested freq_ratio:
#     fc=0.400  n=45   1pk=28.9%   2pk=57.8%    peak_height mean 3.654
#     fc=0.500  n=45   1pk=46.7%   2pk=48.9%    peak_height mean 3.521
#     fc=1.000  n=45   1pk=11.1%   2pk=80.0%    peak_height mean 2.671
#     fc=2.000  n=48   1pk=83.3%   2pk=12.5%    peak_height mean 2.168
#     fc=2.500  n=47   1pk=83.0%   2pk=12.8%    peak_height mean 2.331
# So the reliable 1-peak region is fc ~2.0-2.5 (83%), and the reliable
# 2-peak region is fc ~1.0 (80%).
#
# THE PREVIOUS VALUES WERE BACKWARDS. STABLE_BANDS[1] was (0.55, 0.78) --
# derived when only fc in {0.5, 0.667, 1.0} was in view, where fc=0.667
# looked like a weak 1-peak majority. Over the full grid that region is
# actually 2-peak-leaning, and the genuine 1-peak zone sits at the OPPOSITE
# end of the range. Every hardware run so far was clamped into (0.55,0.78)
# while asking for peak_count=1, i.e. pinned inside a band the data says
# favors the wrong answer -- which is why measured peak_count kept flipping
# between 1 and 2 and the accuracy score oscillated with it.
# WIDENED to the full tested range. The previous 1-peak band, (2.0, 2.5), was
# chosen purely for peak-count RELIABILITY (83% single-peak there) without
# checking what thrust was available inside it -- and that region turns out to
# be the worst on the rig for force:
#     freq_ratio  1-peak share   max thrust AMONG 1-peak missions
#        0.400        28.9%              7.519 N
#        0.500        46.7%              6.949 N
#        1.000        11.1%              4.679 N
#        2.000        83.3%              3.362 N   <- old band
#        2.500        83.0%              3.364 N   <- old band
# The clamp therefore capped achievable thrust at ~3.4 N, less than half of
# what the rig has actually produced, which is why HIL runs never came near
# the ~8 N seen in the earlier sweep. Crucially the 7.519 N maximum is ITSELF
# a 1-peak mission, so peak_count=1 and high thrust are NOT in conflict --
# the band was simply excluding the region where both hold at once. Widening
# it lets the optimiser reach that region; peak_count remains a scored
# objective, so it still has to find a combination that gives one peak.
STABLE_BANDS = {1: (0.40, 2.50), 2: (0.40, 2.50)}

# Where to START freq_ratio for a given target peak count. Band centre is a
# poor default now the band is wide; 0.5 is the data-optimal compromise --
# 6.949 N reachable with the best 1-peak share (46.7%) of any high-thrust
# frequency.
# freq_ratio 0.5 is where the bench sweep's peak thrust sits, but it puts
# heave at TWICE the pitch frequency -- two heave strokes per pitch cycle,
# hence two thrust pulses per analysed window, against a single-pulse target.
# That is why Fx stopped resembling the target curve. 1.0 synchronises the
# servos (one heave stroke per pitch cycle == one pulse) and matches the
# reference run, whose period was 2.000 s with both servos at 0.5 Hz.
# Costs some ceiling thrust (bench max 4.679 N at fr=1.0 vs 6.949 N at 0.5)
# but is the only region whose waveform SHAPE can match a one-pulse target.
START_FREQ_RATIO = {1: 1.00, 2: 0.50}

# NOTE: there is deliberately NO CHAOTIC_ZONE anymore. The previous value
# (1.9, 2.1) was never data-backed -- it was carried over from a placeholder
# and explicitly flagged as unverified, because the earlier narrow slice had
# tested nothing in that range. The full grid now covers it directly, and
# fc=2.0 turns out to be the single MOST reliable peak-count region measured
# (83.3% 1-peak, the highest consistency of any tested freq_ratio). Keeping
# the old guard would have actively steered the controller out of the best
# region for exactly the target the drag curve asks for.
CHAOTIC_ZONE = None

# --- HARDCODED SECONDARY OBJECTIVES ---
# These are NOT read from JSON and NOT derived from data claims -- they are
# fixed design requirements that apply to every single run: Fy should carry
# no net thrust (no sideways drift), Fz should be symmetric about zero (no
# net skew). Written in the same {type, value, weight} format as a JSON
# objective so they can be scored by the exact same cost-function code.
SECONDARY_OBJECTIVES = {
    # LATERAL (Fy): maximise peak-to-peak swing, exactly like thrust. Fy is
    # a working direction here, not a disturbance -- the reference run shows
    # it delivering a ~6 N excursion. It is therefore a ratcheting maximise
    # (see RATCHET_CONFIG), NOT a net-zero target. The previous
    # "fy_net -> 0" objective actively fought this: it asked the optimiser to
    # cancel the very swing that is now wanted, which is why thrust kept
    # being traded away for a lateral null that was never achievable anyway.
    "fy_p2p":  {"type": "maximize", "weight": 2.0, "reference": 6.0},

    # VERTICAL (Fz): the only axis that must cancel out -- net zero and
    # symmetric about zero, matching the reference run's Fz oscillating
    # tightly around 0. This is the sole hard constraint now, and it is the
    # one the rig already gets closest to (-0.117 N last run, essentially at
    # tolerance), so gating thrust on it is realistic rather than futile.
    "fz_net":  {"type": "target", "value": 0.0, "weight": 3.0, "tolerance": 0.1},
    "fz_skew": {"type": "target", "value": 0.0, "weight": 1.0, "tolerance": 0.5},
}
# Thrust-direction convention. The load cell is mounted face-down and always
# has been, so this is not a mounting correction -- it only states which
# direction counts as positive thrust when comparing against an all-positive
# target curve. Measured on hardware: waveform_match was 0.767 using Fx as
# recorded and 0.888 using -Fx. Applied at the single measurement choke point
# so every descriptor, plot and CSV downstream sees one convention.
# Note peak-to-peak (the new primary objective) is sign-invariant, so this
# now only affects waveform_match, skew and peak_count.
FX_SIGN = -1.0

SECONDARY_TOL = 0.05    # +/- band used only to print "ok"/"off" in the log -- doesn't affect control

# GEOMETRIC-MEAN-PRESERVING CENTER (matches in_house_wet_test_3D exactly).
#
# These replace the old A2_BASE=15deg / F2_BASE=0.5Hz "fixed heave reference"
# scheme, which was a SILENT MISMATCH with the data every gain here is
# derived from. The bench sweep never held heave fixed -- it moved BOTH
# servos oppositely around a fixed geometric center:
#     pitch_amp = CENTER_AMP * sqrt(amp_ratio)     heave_amp = CENTER_AMP / sqrt(amp_ratio)
#     pitch_freq = CENTER_FREQ * sqrt(freq_ratio)  heave_freq = CENTER_FREQ / sqrt(freq_ratio)
# (verified directly against the recorded cmd.* columns: at amp_ratio=0.33
# the bench commanded pitch=20.68deg/heave=62.67deg; at 3.00 it commanded
# pitch=62.35deg/heave=20.78deg; at ratio 1.0 both were 36deg. Frequencies
# likewise: freq_ratio_fc=0.5 gave pitch=0.354Hz/heave=0.707Hz.)
#
# The old scheme commanded pitch=amp_ratio*15deg with heave pinned at 15deg,
# so at amp_ratio=0.33 it drove pitch=4.95deg/heave=15deg -- roughly a
# QUARTER of the bench's motion on both axes. That is why measured Fx
# peak-to-peak on hardware sat near 1.0N while the same nominal amp_ratio
# produced ~4.1N on the bench, and why the servos visibly barely moved.
# Every PRIMARY gain was measured under the geometric convention, so the
# controller has to command that same convention for them to mean anything.
CENTER_AMP_DEG = 36.0    # = 0.6283 rad, the bench's center_amp for BOTH servos at ratio 1.0
CENTER_FREQ_HZ = 0.5     # the bench's center_freq for BOTH servos at ratio 1.0
PHI2 = 0.0               # rad, pitch phase is pinned at 0 (mc.paddle hardcodes it);
                         # delta_phi is carried entirely by heave -- see
                         # motion_command.decode_params_to_mission for the sign derivation.

# --- Controller loop tuning knobs ---
MAX_ITERS = 100          # hard cap on cycles per run -- a safety net, not a target
PATIENCE_CYCLES = 10     # stop once this many consecutive cycles fail to beat the best accuracy seen so far
IMPROVEMENT_TOL = 0.001  # an accuracy increase smaller than this doesn't count as "improved" --
                          # guards the patience counter against pure measurement noise
# NOTE: there is no fixed accuracy target anymore. The loop always tries to
# push accuracy higher, for as long as it keeps finding improvements. It
# stops when it can no longer beat its own best result for PATIENCE_CYCLES
# cycles in a row -- "kept trying, nothing worked better, this is what it
# could achieve" -- rather than stopping at some arbitrary percentage.

FD_STEP = 0.05           # how far to nudge a parameter when probing it, WITHOUT-relationships mode only
FIXED_STEP = np.array([0.03, 0.03, 0.10, 0.03, 0.03])   # the actual step size taken once a direction is known,
                                             # one entry per [amp_ratio, freq_ratio, delta_phi, scale, freq_scale].
                                             # Fixed magnitude, not gradient-scaled -- see the long
                                             # comment in controller_step_WITHOUT_relationships for why.

# Physical/servo sanity bounds. Applied after EVERY parameter update, in
# BOTH controller modes, so neither an aggressive gain nor a noisy empirical
# gradient can push a parameter somewhere the real servos could never reach.
# Tightened from the original (0.1, 5.0) to (0.33, 3.0) -- that's the ACTUAL
# tested range in in_house_wet_test_3D/coupling/1_scaling/scaling_data.csv;
# every PRIMARY gain above is a linear fit over that range and was never
# validated outside it. 0.1 (the old lower bound) is 3.3x further from the
# nearest tested point (0.33) than the tested range itself is wide -- on the
# very first real hardware run, "maximize peak_height" (negative gain) drove
# amp_ratio straight to that untested floor in 3 cycles and got stuck there,
# extrapolating a linear model into territory it was never checked against.
AMP_RATIO_BOUNDS = (0.33, 3.0)
DELTA_PHI_BOUNDS = (-np.pi, np.pi)

# params[3] = SCALE -- an overall amplitude multiplier applied to BOTH servos.
# THIS IS NEW, and it exists because of a structural dead-end found in the
# full-grid analysis: under the geometric-mean-preserving decode,
# pitch_amp * heave_amp == CENTER_AMP^2 for EVERY amp_ratio, and likewise
# for the frequencies. So amp_ratio/freq_ratio only ever REDISTRIBUTE motion
# between the two servos -- they cannot add any. Total tip speed
# (2*pi*f*A, the single strongest predictor of Fx peak-to-peak in the data:
# R^2=0.437 for heave_tipspeed vs 0.128 for amp_ratio) was therefore
# effectively FIXED, and the controller was confined to a constant-scale
# manifold. That is the real reason it drove straight to a bound and locked
# by cycle 3: on that manifold there was genuinely nothing left to gain.
# scale multiplies both amplitudes, so it moves tip speed directly and is
# the only parameter here that can actually make the force bigger.
# Bounds keep both servos inside the amplitude envelope the bench actually
# swept (20.7deg-62.7deg about a 36deg center) once ratio and scale combine.
SCALE_BOUNDS = (0.6, 1.4)

# params[4] = FREQ_SCALE -- an overall frequency multiplier on BOTH servos,
# the frequency counterpart of `scale`. Without it the geometric mean of the
# two frequencies was PINNED at CENTER_FREQ_HZ: freq_ratio only ever
# redistributed speed between pitch and heave, so the rig could never simply
# flap faster. That matters because peak_height's strongest predictor in the
# bench data is tip speed 2*pi*f*A (R^2=0.437, vs 0.128 for amp_ratio), and
# tip speed has two factors -- with frequency locked, only half the available
# thrust lever was reachable.
#
# HONEST CAVEAT: unlike every other parameter here, this one has NO seeded
# gain, because the entire in_house_wet_test_3D sweep ran at
# center_freq = 0.5 Hz. Individual pitch/heave frequencies varied (0.316 to
# 0.791 Hz) but their geometric mean never did, so the data cannot separate
# "faster overall" from "redistributed". It therefore starts with no PRIMARY
# entry: the WITH phase cannot move it, and it is discovered by the gain
# adapter (once 5+ cycles give it variation) and by the model-free phase.
# Bounds are deliberately conservative -- 0.7x-1.3x of 0.5 Hz keeps both
# servos inside the 0.316-0.791 Hz band the bench actually exercised, so the
# rig is not driven into an untested speed regime. The slew clamp still
# applies on top and will pull amplitude back if a faster flap exceeds it.
FREQ_SCALE_BOUNDS = (0.7, 1.3)

# Per-axis servo position limits (rad), matching controller.py's launch
# defaults (pitch_limit=pi, heave_limit=pi/2). decode_params can now exceed
# these once scale is in play (e.g. amp_ratio=0.33 puts heave at 62.7deg,
# and scale=1.4 would push it to 87.7deg, right at the heave limit), so
# clamp_params enforces them explicitly rather than relying on the
# controller's own downstream clamp to silently truncate the waveform --
# a truncated sinusoid is a DIFFERENT waveform, which would quietly
# corrupt every descriptor measured from it.
PITCH_AMP_LIMIT_RAD = np.pi
HEAVE_AMP_LIMIT_RAD = np.pi / 2.0

# Hard safety cap on commanded peak angular velocity (2*pi*frequency*amplitude,
# rad/s), applied to BOTH servos -- matches SLEW_LIMIT from
# scripts/sweep_amp_freq_phase.py (the same bench sweep this file's other
# constants are derived from), which used this exact value to classify a
# mission SKIP-UNSAFE before ever commanding it. Neither AMP_RATIO_BOUNDS nor
# STABLE_BANDS individually prevent amp_ratio and freq_ratio from COMBINING
# into an unsafe peak velocity -- clamp_slew() enforces this jointly, after
# the individual per-parameter clamps. Under the geometric-mean-preserving
# decode both servos' amplitudes change with amp_ratio (in opposite
# directions), so which axis binds depends on which side of 1.0 you're on --
# hence checking both, not just pitch.
SLEW_LIMIT_RAD_S = 5.5

# --- Measurement protocol ---
SETTLE_WAIT_S = 5.0            # seconds to wait after commanding new params, before measuring
N_CYCLES_PER_MEASUREMENT = 2   # run this many periods, keep only the last (drop-first-cycle)


# ============================================================================
# 1. INGEST -- read the JSON file, no processing yet
# ============================================================================

def load_target_json(path):
    """Just a thin wrapper around json.load so the rest of the pipeline
    doesn't need to know the file is even JSON, in case that ever changes.

    If `path` isn't found as given (relative to cwd), also tries it
    relative to WORKSPACE_ROOT -- the two target_curve_*.json files live in
    the project root, and this script may be invoked from anywhere."""
    if not os.path.exists(path):
        alt = os.path.join(WORKSPACE_ROOT, path)
        if os.path.exists(alt):
            path = alt
    with open(path, "r") as f:
        return json.load(f)


# ============================================================================
# 2. INTERPRET -- turn target_points into a curve, and figure out what the
#    controller should actually be optimizing (the "objectives" dict).
# ============================================================================

def interpolate_channel(channel_spec, period_s, n_samples=400):
    """Turns a handful of (t, F) points into a dense curve.

    Uses PCHIP (shape-preserving piecewise cubic Hermite interpolation)
    rather than a plain cubic spline. This choice matters and was verified
    empirically: a plain periodic cubic spline can OVERSHOOT past your
    data's own min/max when the curve has a flat plateau next to a sharp
    corner (confirmed: it rang to -5.5N on a curve whose points never went
    below 0). PCHIP is mathematically guaranteed to never do that -- the
    interpolated curve always stays within the range implied by adjacent
    points, so "the plot matches what I described" is always true, for any
    shape you draw with the points, not just the ones tested so far.
    """
    pts = channel_spec["target_points"]
    t_pts = np.array([p["t"] for p in pts])
    F_pts = np.array([p["F"] for p in pts])
    t_dense = np.linspace(0, period_s, n_samples)

    if channel_spec.get("interpolation", "cubic") == "linear":
        # straight-line interpolation between points, no smoothing at all
        F_dense = np.interp(t_dense, t_pts, F_pts)
    else:
        # default / "cubic" in the JSON actually means PCHIP now, see docstring above
        F_dense = PchipInterpolator(t_pts, F_pts)(t_dense)

    return t_dense, F_dense


def build_target_curve(spec):
    """Reads just the Fx channel out of the full JSON spec -- Fy and Fz are
    never target-curve-described, only Fx is (see module docstring)."""
    t, Fx = interpolate_channel(spec["channel_definitions"]["Fx"], spec["period_s"])
    return t, Fx


def peak_to_peak(x):
    """Max minus min of an array -- used as the 'peak_height' descriptor
    (total swing of the Fx curve, not just the height above zero)."""
    return float(np.max(x) - np.min(x))


def safe_skew(x):
    """scipy.stats.skew is mathematically 0/0 (undefined -> NaN) for a
    perfectly constant array. A NaN here would silently poison every
    downstream sum (cost, accuracy) since NaN + anything = NaN. A constant
    signal has no asymmetry by definition, so 0.0 is the correct value,
    not an edge case to avoid -- this matters for real measurements too,
    not just synthetic tests: a genuinely flat Fy or Fz reading is a
    perfectly plausible real result."""
    if np.std(x) < 1e-6:
        return 0.0
    return float(scipy_skew(x))


def dominant_frequency(t, x, expected_freq_hz=None, margin=0.5, pad_factor=8):
    """FFT-based fundamental frequency of a signal. Used as the 'rate'
    descriptor -- roughly, how fast the curve's dominant feature repeats
    within the measurement window.

    A plain global argmax over the raw FFT (the original version of this
    function) is exactly the bug identified and fixed in prove_claims.py's
    bench-data analysis earlier in this project, and it reproduced on real
    hardware here too: a single short, un-windowed, non-padded cycle has
    very coarse frequency resolution and leaky spectral energy, so the
    global max bin can land far from the true fundamental -- observed
    directly on this rig as 'rate' readings of 16-30Hz on a ~0.65Hz
    commanded motion. Fixed the same way: a Hann window (reduces spectral
    leakage from the cycle's edges), zero-padding (pad_factor x, smoother
    peak localization -- does NOT increase true frequency resolution, that
    still depends on the real window duration), and, when the caller knows
    what frequency to expect (every real call site here does -- it's
    literally the commanded pitch frequency), restricting the peak search
    to a band around it (+/-margin fraction) so noise/harmonics elsewhere
    in the spectrum can't win. Falls back to an unrestricted global argmax
    (skipping DC) only when no expected_freq_hz is given."""
    n = len(x)
    dt = t[1] - t[0] if n > 1 else 1.0
    sample_rate = 1.0 / dt if dt > 1e-9 else 1.0
    signal_clean = x - np.mean(x)
    windowed = signal_clean * np.hanning(n)
    n_fft = max(n * pad_factor, n)
    spectrum = np.abs(np.fft.rfft(windowed, n=n_fft))
    freqs = np.fft.rfftfreq(n_fft, d=1.0 / sample_rate)
    if len(spectrum) <= 1:
        return 0.0

    if expected_freq_hz and expected_freq_hz > 1e-6:
        f_min, f_max = expected_freq_hz * (1.0 - margin), expected_freq_hz * (1.0 + margin)
        mask = (freqs >= f_min) & (freqs <= f_max)
        if mask.any():
            sub = spectrum[mask]
            return float(freqs[mask][np.argmax(sub)])

    # argmax over spectrum[1:] skips the DC bin (index 0), +1 corrects the index back
    return float(freqs[np.argmax(spectrum[1:]) + 1])


def count_peaks(t, x, expected_freq_hz=None, prominence_frac=0.2, min_distance_frac=0.07):
    """Counts distinct local maxima in one cycle.

    The ORIGINAL version of this (plain scipy.find_peaks on the raw signal,
    prominence-only) is the exact same bug identified and fixed for
    count_force_peaks() in the bench-data analysis (prove_claims.py) and
    reproduced HERE on real hardware: run on real hardware data mid-tuning,
    the raw-signal version returned 24-154 "peaks" per cycle on signals a
    human would call single- or double-peaked -- it was counting individual
    noise wiggles, and since peak_count is checked for an EXACT match
    against the JSON target, this meant the peak_count objective could
    essentially never be satisfied, capping overall accuracy regardless of
    how well-tuned everything else was.

    Fixed the same way as count_force_peaks(): circularly pad the signal
    (it spans exactly one period, so this is legitimate, not fabricated
    data) so a peak straddling the window edge isn't split in two, low-pass
    filter with a 4th-order Butterworth at 4x the expected/commanded
    frequency (passes real harmonic content, rejects load-cell/mechanical
    noise) with a moving-average fallback if the sample rate is too low for
    that filter design, then find local maxima in the top
    (1-prominence_frac) of the filtered signal's range with minimum
    separation min_distance_frac of a cycle."""
    n = len(x)
    if n < 8 or peak_to_peak(x) < 1e-9:
        return 0

    pad = max(1, n // 6)
    xp = np.concatenate([x[-pad:], x, x[:pad]])

    dt = t[1] - t[0] if n > 1 else 1.0
    sample_rate = 1.0 / dt if dt > 1e-9 else 1.0
    # Guard the Butterworth design: at a 10 kHz load-cell rate against a
    # ~0.7 Hz commanded frequency the normalised cutoff is ~5e-4, and a
    # 4th-order filter that narrow is numerically singular -- scipy raises
    # LinAlgError("Singular matrix") from lfilter_zi. That killed a 35-cycle
    # hardware run outright, losing every result. Require a workable Wn and
    # enough samples, and fall back to the moving average otherwise; wrap the
    # design+apply so any residual numerical failure degrades to the fallback
    # instead of taking down the run.
    xs = None
    wn = (4.0 * expected_freq_hz / (sample_rate / 2.0)) if (expected_freq_hz and sample_rate > 0) else 0.0
    if 1e-3 < wn < 0.99 and len(xp) > 30:
        try:
            from scipy.signal import butter, filtfilt
            b, a = butter(4, wn, btype="low")
            xs = filtfilt(b, a, xp)
        except Exception:
            xs = None
    if xs is None:
        win = max(3, len(xp) // 15)
        xs = np.convolve(xp, np.ones(win) / win, mode="same")

    lo, hi = xs.min(), xs.max()
    rng = hi - lo
    if rng < 1e-9:
        return 0
    thr = hi - prominence_frac * rng
    candidates = [i for i in range(1, len(xs) - 1)
                  if xs[i] > xs[i - 1] and xs[i] >= xs[i + 1] and xs[i] >= thr]
    core = [i for i in candidates if pad <= i < pad + n]
    if not core:
        return 0
    min_dist = max(1, int(min_distance_frac * n))
    core.sort(key=lambda i: -xs[i])
    kept = []
    for i in core:
        if all(abs(i - j) >= min_dist for j in kept):
            kept.append(i)
    return len(kept)


# Every descriptor name you might use in a JSON "objectives" block MUST have
# an entry here -- this dict is the single place that defines "how do I
# compute descriptor X from a raw Fx(t) waveform". Add a new descriptor
# (e.g. a real rise-time or width metric) by adding one line here; nothing
# else in the file needs to change to support it.
DESCRIPTOR_EXTRACTORS = {
    "peak_height": lambda t, Fx: peak_to_peak(Fx),
    "rate":        lambda t, Fx: dominant_frequency(t, Fx),
    "skew":        lambda t, Fx: safe_skew(Fx),          # + = right-skewed, - = left-skewed
    "peak_count":  lambda t, Fx: count_peaks(t, Fx),
    # How far Fx dips BELOW zero, as a one-sided quantity: 0.0 when the
    # waveform never goes negative, negative by the depth of the dip when it
    # does. Use this (not trough_min) to ask for "no trough": trough_min is
    # two-sided, so a target of 0 would push an already-all-positive Fx back
    # DOWN toward zero, actively destroying a good result. trough_depth
    # saturates at 0, so once the dip is gone there is no further pressure.
    "trough_depth": lambda t, Fx: float(min(0.0, np.min(Fx))),
    # Trough depth as a FRACTION of the waveform's total swing (0 = no dip
    # below zero, 0.5 = the dip is half the peak-to-peak). This is the one to
    # optimise against, NOT the absolute trough_depth: absolute depth is
    # trivially satisfiable by shrinking the whole waveform, and that is
    # exactly what happened on hardware -- the optimiser drove scale to its
    # 0.600 floor, cutting peak-to-peak from 1.93 N to 0.62 N. Absolute
    # trough "improved" (-0.725 -> -0.28) while the trough as a share of the
    # peak got WORSE (37.6% -> 45.2%), i.e. the shape degraded while the
    # number being optimised went down. This ratio is scale-invariant, so
    # shrinking buys nothing and only genuine asymmetry can reduce it.
    # The positive peak on its own. peak_height is peak-to-peak, which says
    # nothing about WHICH end supplied the swing -- a curve can grow its
    # negative excursion and score identically. Stage 2 pins this so the
    # trough can only be reduced by shrinking the NEGATIVE side.
    "fx_pos_peak": lambda t, Fx: float(np.max(Fx)),
    "trough_frac": lambda t, Fx: (
        float(abs(min(0.0, np.min(Fx))) / peak_to_peak(Fx))
        if peak_to_peak(Fx) > 1e-9 else 0.0),
    # Depth of the dip that occurs BEFORE the main thrust peak, as a fraction
    # of total swing. The pre-peak trough is the damaging one -- it is thrust
    # acting backwards during the stroke's build-up -- whereas the post-peak
    # dip is the stroke unloading and is acceptable. Plain trough_frac cannot
    # tell them apart: it reports whichever is deeper, so a run could be
    # penalised for an acceptable trailing dip, or (worse) look fine because
    # the trailing dip happened to be shallower while a bad leading dip sat
    # untouched. Everything left of argmax(Fx) is the build-up.
    "trough_frac_pre": lambda t, Fx: (
        float(abs(min(0.0, float(np.min(Fx[:max(1, int(np.argmax(Fx)))]))))
              / peak_to_peak(Fx))
        if peak_to_peak(Fx) > 1e-9 else 0.0),
    "trough_min":  lambda t, Fx: float(np.min(Fx)),               # the curve's lowest point --
                                                                    # maximizing this = "keep the
                                                                    # trough shallow, not deep"
}
# "rate" and "peak_count" both need an expected/commanded frequency for
# their fundamental-aware search & filter cutoff (see dominant_frequency()
# and count_peaks() above) -- everything else in DESCRIPTOR_EXTRACTORS has
# a uniform (t, Fx) signature and is called through the dict directly;
# these two are special-cased in extract_fx_descriptors() below instead.
_FREQ_AWARE_DESCRIPTORS = {
    "rate":       dominant_frequency,
    "peak_count": count_peaks,
}


def waveform_match(t, Fx, target_t, target_Fx, n=200):
    """How closely the measured cycle's SHAPE matches the JSON target
    curve's shape, as a [0,1] score (1 = identical shape).

    THIS IS THE DESCRIPTOR THAT ACTUALLY ANSWERS "does my force curve look
    like the one I asked for". Every other descriptor is a single scalar
    summary (a height, a count, a skew) -- it is entirely possible to score
    well on all of them while the waveform looks nothing like the target,
    which is exactly what happened on the first tared hardware runs (77.8%
    "accuracy" on a curve that was visually pure noise against a smooth 9N
    trapezoid target). Before this existed, the JSON's target_points were
    used ONLY for the visual-check plot and to auto-derive peak_count --
    the optimizer never compared its measurement to the target at all.

    Method: resample both curves onto the same n-point cycle-fraction grid
    (0..1), then take the best Pearson correlation over all circular shifts
    of the measured curve, scored as max(0, r).

      * Circular shift: the measured cycle's t=0 is the commanded gait's
        start, but the fluid force response lags it by an unknown amount,
        so a fixed alignment would penalize a perfectly-shaped curve purely
        for arriving late. Both curves are exactly one period long, so
        rotating one is legitimate, not fudging.
      * Correlation (not RMS error): scale- and offset-invariant, so this
        measures SHAPE ONLY. That is deliberate -- the rig cannot physically
        reach the drag target's 9N plateau (the entire bench sweep never
        exceeded ~4.3N peak-to-peak), so scoring absolute magnitude here
        would permanently cap the score for reasons no amount of tuning can
        fix. MAGNITUDE IS SCORED SEPARATELY by peak_height; this term is
        purely "is it the right shape".
    """
    if target_t is None or target_Fx is None or len(Fx) < 8:
        return float("nan")
    period_meas = t[-1] - t[0]
    period_targ = target_t[-1] - target_t[0]
    if period_meas < 1e-9 or period_targ < 1e-9:
        return float("nan")
    grid = np.linspace(0.0, 1.0, n, endpoint=False)
    meas = np.interp(grid, (t - t[0]) / period_meas, Fx)
    targ = np.interp(grid, (target_t - target_t[0]) / period_targ, target_Fx)
    meas = meas - meas.mean()
    targ = targ - targ.mean()
    if np.std(meas) < 1e-12 or np.std(targ) < 1e-12:
        return 0.0
    # circular cross-correlation, normalized -> best-alignment Pearson r
    corr = np.correlate(np.concatenate([meas, meas]), targ, mode="valid")[:n]
    r = float(np.max(corr) / (np.linalg.norm(meas) * np.linalg.norm(targ)))
    return float(np.clip(r, 0.0, 1.0))


def extract_fx_descriptors(t, Fx, names=None, expected_freq_hz=None, target=None):
    """THE single source of truth for turning a raw (t, Fx) waveform into
    named descriptor values. Called on BOTH the JSON target curve (for
    reporting) AND every real/simulated measured cycle (for control) -- so
    target and measurement are always computed the exact same way and are
    directly comparable.

    names=None means "extract everything we know how to extract"; pass a
    list to compute only specific descriptors (a small speed optimization
    used when only peak_count is needed, e.g. in resolve_objectives).

    expected_freq_hz: the commanded pitch frequency for THIS measurement,
    if known -- passed through to dominant_frequency() ('rate') and
    count_peaks() ('peak_count'), both of which need it to center their
    fundamental-aware search / filter cutoff. Every real caller in this
    file passes it; None (unrestricted search) is only the fallback for
    the target-curve JSON, which has no "commanded frequency" of its own
    -- resolve_objectives() passes 1/period_s instead, the target curve's
    own implied fundamental.

    target: (target_t, target_Fx) of the JSON curve, required for the
    'waveform_match' descriptor (omitted from the result when not given --
    e.g. when extracting descriptors OF the target curve itself, where
    matching it against itself would be trivially 1.0 and meaningless)."""
    names = names or list(DESCRIPTOR_EXTRACTORS.keys()) + ["waveform_match"]
    out = {}
    for name in names:
        if name == "waveform_match":
            if target is not None:
                out[name] = waveform_match(t, Fx, target[0], target[1])
        elif name in _FREQ_AWARE_DESCRIPTORS:
            out[name] = _FREQ_AWARE_DESCRIPTORS[name](t, Fx, expected_freq_hz=expected_freq_hz)
        else:
            out[name] = DESCRIPTOR_EXTRACTORS[name](t, Fx)
    return out


def extract_secondary(Fy, Fz):
    """The two hardcoded secondary measurements: net mean of Fy (should be
    ~0, no sustained sideways force) and the skewness of Fz (should be ~0,
    symmetric about zero). These are never JSON-described -- see module
    docstring for why."""
    return {"fy_net": float(np.mean(Fy)), "fy_skew": safe_skew(Fy),
            "fy_p2p": peak_to_peak(Fy),
            "fz_net": float(np.mean(Fz)), "fz_skew": safe_skew(Fz),
            "fz_p2p": peak_to_peak(Fz)}


def resolve_objectives(spec, t, Fx):
    """Pulls the "objectives" block straight out of the JSON, used exactly
    as written -- no merging, no defaults injected, no prose anywhere.

    The ONE exception: if you didn't specify a peak_count objective, we
    fall back to whatever peak_count the target_points shape itself implies
    (auto-extracted from the interpolated curve). This is necessary because
    peak_count isn't just another cost term -- it's used as a hard
    constraint (see clamp_freq_ratio) that the controller needs a value for
    even if you never explicitly stated an optimization goal around it."""
    objectives = dict(spec.get("objectives", {}))
    # Auto-add the shape-match objective unless the JSON overrides it. This
    # is THE objective that actually asks "does the measured force curve look
    # like the one described in target_points" -- without it, target_points
    # only ever drove the visual-check plot, and 'accuracy' was an average of
    # scalar summaries that can all score well on a waveform bearing no
    # resemblance to the target (observed: 77.8% on a curve that was visually
    # pure noise against a smooth trapezoid). tolerance=1.0 against a target
    # of 1.0 makes the reported accuracy for this term equal the raw match
    # score itself, so the number means exactly what it says.
    if "waveform_match" not in objectives:
        # weight 5.0 (was 2.0): during the MATCH stage this is the job, and
        # it must outrank everything else that is active. It stays on into
        # the later stages so the matched shape is defended, not discarded.
        objectives["waveform_match"] = {"type": "target", "value": 1.0,
                                        "weight": 5.0, "tolerance": 1.0}
    if "peak_count" not in objectives:
        implied_freq_hz = 1.0 / spec["period_s"] if spec.get("period_s") else None
        auto_peak_count = extract_fx_descriptors(
            t, Fx, ["peak_count"], expected_freq_hz=implied_freq_hz)["peak_count"]
        objectives["peak_count"] = {"type": "target", "value": auto_peak_count}
    return objectives


# Objectives held back until the MATCH stage finishes (see
# maybe_advance_stage). These are the "make it bigger" objectives -- they
# would otherwise overwhelm waveform_match and prevent the curve from ever
# being matched in the first place.
MAGNITUDE_OBJECTIVES = ("peak_height", "fy_p2p")


def split_deferred(objectives):
    """Remove the magnitude objectives from the active set and return them
    separately, to be switched on at the stage-0 -> stage-1 transition."""
    deferred = {}
    for name in MAGNITUDE_OBJECTIVES:
        if name in objectives:
            deferred[name] = objectives.pop(name)
    return objectives, deferred


# Starting point / step size / direction for the RATCHET mechanism (see
# apply_ratchets() below), keyed by descriptor name. Replaces open-ended
# "maximize"/"minimize" objectives -- those emit a CONSTANT push signal
# forever (see controller_step_WITH_relationships/objective_term_and_signal),
# so they are mathematically guaranteed to walk straight to a parameter
# bound and stay there, regardless of how good the gain is (observed: every
# hardware run so far railed amp_ratio/freq_ratio/scale to a bound within a
# handful of cycles). A ratcheting TARGET has an equilibrium instead: once
# reached, the bar is raised one step further in the same direction
# (maximize -> up, minimize -> down); the first time a raised bar ISN'T
# reached, it freezes there for the rest of the run -- so it settles at
# whatever the rig can actually sustain instead of pinning at a hard bound.
# start is set comfortably inside what's already been measured on real
# hardware (not the JSON's own literal target -- e.g. drag_dominant's
# target curve implies peak_height=9.0N, which is above anything ever
# recorded across the whole in_house_wet_test_3D sweep, max 7.52N -- an
# unreachable STARTING point would never ratchet at all, it would just sit
# failed forever).
# A ratchet is defined ONLY by its direction/step size -- there is no
# curve-specific start value. The bar is SELF-SEEDED from the first real
# measurement (see ratchet_step), so the same config works for any target
# curve: drag_dominant, lift_dominant, or anything added later. An earlier
# version hardcoded absolute starts (peak_height 0.30 N, rate 1.20 Hz)
# picked from one particular run of one particular curve -- those numbers
# are meaningless for a different target, and if a start lands on the hard
# side of what the rig does the ratchet freezes on cycle 1 and never moves.
# step is the increment applied each time the bar is cleared, and its SIGN
# is the ratchet direction:
#   +  climbs upward   (former "maximize" objectives)
#   -  climbs downward (former "minimize" objectives)
# Consecutive misses before a ratchet bar decays back to the achievable
# frontier (see ratchet_step). Small enough that an unreachable bar cannot
# dominate the cost function for long, large enough that ordinary
# cycle-to-cycle noise does not knock the bar down while the rig can still
# clear it -- measured accuracy swings ~29-45% at identical parameters, so a
# single bad draw must not count as "unreachable".
RATCHET_DECAY_AFTER = 3

RATCHET_CONFIG = {
    "peak_height": {"step": +0.10},   # was "maximize"  -- bigger force is better
    "rate":        {"step": -0.05},   # was "minimize"  -- slower is better
    # trough_min is min(Fx): the depth of the dip BELOW zero, so it is
    # negative. "Minimise the trough" means make it SHALLOWER -- drive it up
    # toward 0 -- so it climbs upward, same direction as peak_height.
    "trough_min":  {"step": +0.10},
    "fy_p2p":      {"step": +0.10},   # lateral swing: climb like thrust   # was "maximize"  -- shallower dip is better
}


def apply_ratchets(objectives):
    """Converts any 'maximize'/'minimize' objective with a RATCHET_CONFIG
    entry into a ratcheting 'target' objective, seeded at its configured
    start. Mutates and returns `objectives`. Call ONCE, right after
    resolve_objectives(), before the control loop starts -- the returned
    dict is then updated in place, cycle to cycle, by ratchet_step()."""
    for name, cfg in RATCHET_CONFIG.items():
        spec = objectives.get(name)
        if spec is None or spec["type"] not in ("maximize", "minimize"):
            continue   # not present, or already an explicit target -- leave it alone
        objectives[name] = {
            "type": "target", "value": None,   # None = not yet seeded; see ratchet_step
            "weight": spec.get("weight", 1.0),
            "tolerance": abs(cfg["step"]), "_ratchet_step": cfg["step"],
        }
    return objectives


# Objectives treated as HARD CONSTRAINTS: thrust is only allowed to climb
# while all of these sit inside their tolerance. Everything else is scored
# normally, but these three gate the peak_height ratchet (see
# constraints_satisfied / ratchet_step).
# NOTE trough_frac is deliberately NOT in this list, even though it is a
# first-priority requirement. The gate works by easing thrust down, and
# thrust is delivered through `scale` -- but trough_frac is scale-INVARIANT
# by construction (it is a ratio, built that way so the optimiser could not
# cheat by shrinking the whole waveform). So gating thrust on trough_frac
# spends thrust on an objective it is mathematically incapable of improving:
# observed costing ~80% of peak (3.5 N -> 0.64 N) while trough_frac never
# left ~0.30. fy_net and fz_net DO scale with thrust (r~0.98), so easing
# thrust genuinely helps those two -- they are what the gate is for.
# trough_frac is still a weight-3 cost term and still first-priority; it is
# simply pursued through waveform ASYMMETRY (delta_phi, amp_ratio,
# freq_ratio), which is the only mechanism that can actually move it.
CONSTRAINT_OBJECTIVES = ("fz_net",)

# Objectives whose ratchet is gated by those constraints -- i.e. maximised
# only in the slack the constraints leave.
GATED_BY_CONSTRAINTS = ("peak_height",)


def constraints_satisfied(measured_all, objectives, secondary=None):
    """True when every CONSTRAINT_OBJECTIVE is within its own tolerance."""
    secondary = secondary if secondary is not None else SECONDARY_OBJECTIVES
    for name in CONSTRAINT_OBJECTIVES:
        spec = objectives.get(name) or secondary.get(name)
        if spec is None or name not in measured_all:
            continue
        tol = spec.get("tolerance")
        if tol is None:
            continue
        if abs(spec.get("value", 0.0) - measured_all[name]) > tol:
            return False
    return True


# How many cycles waveform_match may fail to improve before the MATCH stage
# is declared finished. Deliberately generous -- matching the curve is the
# primary job and should not be cut short by a couple of noisy cycles.
MATCH_PATIENCE = 8


def maybe_advance_stage(objectives, measured_all, state, log=print):
    """Three-stage schedule.

    STAGE 0 -- MATCH: get the Fx waveform as close to the JSON target curve
    as the rig can manage. waveform_match dominates; the magnitude
    objectives (maximise peak-to-peak on Fx and Fy) are held back entirely,
    because "biggest possible swing" and "reproduce this particular shape"
    are different goals and the former swamps the latter. Running them
    together is what drove waveform_match from 0.97 down to 0.55.
    Fz net-zero stays active throughout -- it is a constraint on symmetry,
    not a shape objective, so it does not fight the match.

    STAGE 1 -- OPTIMISE: once the match stops improving, the deferred
    magnitude objectives switch on and thrust/lateral swing are maximised
    FROM the matched waveform rather than instead of it. waveform_match
    stays active so the shape earned in stage 0 is not simply thrown away.

    STAGE 2 -- REFINE: once peak-to-peak tops out, pin the positive peak and
    minimise the trough (see the stage-2 notes below).
    """
    stage = state.get("stage", 0)

    if stage == 0:
        wf = measured_all.get("waveform_match")
        if wf is None:
            return
        best = state.get("best_wf", -1.0)
        if wf > best + IMPROVEMENT_TOL:
            state["best_wf"] = wf
            state["wf_stall"] = 0
            return
        state["wf_stall"] = state.get("wf_stall", 0) + 1
        if state["wf_stall"] < MATCH_PATIENCE:
            return
        for name, spec in state.get("deferred", {}).items():
            objectives[name] = spec
        apply_ratchets(objectives)
        state["stage"] = 1
        log(f"\n--- STAGE 1: curve match has plateaued at "
            f"{state['best_wf']:.3f}. Switching on the magnitude objectives "
            f"({', '.join(state.get('deferred', {}))}) and optimising from here.\n")
        return

    maybe_enter_stage2(objectives, measured_all, state, log=log)


def maybe_enter_stage2(objectives, measured_all, state, log=print):
    """Two-stage schedule on Fx.

    STAGE 1 -- maximise the swing: peak_height (peak-to-peak) ratchets upward
    with no trough objective at all, because a deep negative excursion is
    part of the swing being maximised and penalising it would fight the goal.

    STAGE 2 -- once that ratchet has hit the rig's ceiling (signalled by its
    first decay, i.e. it missed its own bar RATCHET_DECAY_AFTER times in a
    row), lock in what was achieved and go after the trough:
      * fx_pos_peak is pinned as a target at the best POSITIVE peak seen, so
        the positive side cannot be given back;
      * trough_frac_pre is switched on, so the only way left to improve is to
        shrink the NEGATIVE excursion.
    Pinning the positive peak is what makes stage 2 meaningful -- peak_height
    alone is peak-to-peak and says nothing about which end supplied the
    swing, so without the pin the optimiser could "reduce the trough" by
    simply collapsing the whole waveform, which is the shrink-everything
    shortcut seen earlier in this project.
    """
    if state.get("stage", 1) != 1:
        return
    ph = objectives.get("peak_height")
    if ph is None or not ph.get("_decayed_once"):
        return
    best_pos = state.get("best_pos_peak")
    if best_pos is None:
        return
    state["stage"] = 2
    objectives["fx_pos_peak"] = {"type": "target", "value": float(best_pos),
                                 "weight": 3.0, "tolerance": 0.10}
    objectives["trough_frac_pre"] = {"type": "target", "value": 0.0,
                                     "weight": 3.0, "tolerance": 0.05}
    ph["_ratchet_step"] = None          # stop chasing more swing
    log(f"\n--- STAGE 2 at this cycle: peak-to-peak has topped out. Holding the "
        f"positive peak at {best_pos:.3f} N and now minimising the trough.\n")


def ratchet_step(objectives, measured_all, log=print):
    """Call once per cycle, after a measurement, before the controller
    decides its next move. For every objective carrying '_ratchet_step'
    (i.e. everything apply_ratchets() converted) that ISN'T frozen yet:
    if this cycle's measurement is within `tolerance` of the current
    target, push the target one step further in the ratchet direction and
    log it; otherwise freeze it permanently (no further raises for the
    rest of the run -- a one-way ratchet, not a pause)."""
    for name, spec in objectives.items():
        step = spec.get("_ratchet_step")
        if step is None or name not in measured_all:
            continue
        # DIRECTIONAL test: "reached" means at-or-beyond the bar in the
        # direction the ratchet is climbing, NOT "within tolerance of it".
        # A symmetric abs() test counts an OVERSHOOT as a failure, which is
        # backwards -- exceeding a maximize bar is the best possible outcome.
        # (Observed on hardware: a peak_height bar of 2.0N measured 3.689N and
        # was scored "not reached", freezing the ratchet on cycle 1 so the
        # whole run stayed pinned to a 2.0N target while actually producing
        # 3.7-7N.) The tolerance still applies on the near side, so landing
        # just short of the bar still counts and keeps the ratchet climbing.
        measured_val = measured_all[name]
        if spec["value"] is None:
            # First real measurement: seed the bar one step beyond it, so the
            # ratchet starts exactly where the rig actually is rather than at
            # an arbitrary constant. Nothing is scored against this objective
            # until it has been seeded (see the None guards in
            # objective_term_and_signal / objective_accuracy / overall_accuracy).
            spec["value"] = measured_val + step
            log(f"           ratchet: {name} seeded from first measurement "
                f"{measured_val:.3f} -> bar {spec['value']:.3f}")
            continue

        # --- CONSTRAINT GATE ---------------------------------------------
        # A gated objective (thrust) may only climb while every hard
        # constraint -- trough_frac, fy_net, fz_net -- is inside tolerance.
        # While any is violated the bar is walked DOWN one step below what
        # was just measured, so thrust actively gives ground to help the
        # constraints close. As soon as they are all satisfied the bar
        # resumes climbing from wherever thrust currently sits, so the peak
        # is re-maximised while HOLDING the constraints (if climbing breaks
        # one again, the gate closes and it backs off once more).
        # Without this, thrust and the constraints were simply summed as
        # competing cost terms and thrust won: scale railed to its 1.400
        # ceiling while trough_frac stalled at ~0.40 and fy_net/fz_net drifted
        # further out, in run after run.
        if name in GATED_BY_CONSTRAINTS and not constraints_satisfied(
                measured_all, objectives):
            old = spec["value"]
            spec["value"] = measured_val - abs(step)
            spec["_misses"] = 0
            if abs(old - spec["value"]) > 1e-9:
                log(f"           ratchet: {name} GATED (a constraint is out of "
                    f"tolerance) -- easing bar {old:.3f} -> {spec['value']:.3f}")
            continue

        if step > 0:      # climbing upward (was "maximize")
            reached = measured_val >= spec["value"] - spec["tolerance"]
        else:             # climbing downward (was "minimize")
            reached = measured_val <= spec["value"] + spec["tolerance"]
        if reached:
            old = spec["value"]
            # Jump the bar to just past what was ACTUALLY achieved, rather
            # than a single step past the old bar -- otherwise a big
            # overshoot (3.7N against a 2.0N bar) would need many cycles of
            # +0.5 steps just to catch up to where the rig already is.
            spec["value"] = max(old + step, measured_val + step) if step > 0 \
                else min(old + step, measured_val + step)
            log(f"           ratchet: {name} {measured_val:.3f} vs bar {old:.3f} "
                f"-> raising bar to {spec['value']:.3f}")
        else:
            # HOLD, do not freeze. The bar stays where it is and the
            # controller keeps trying for it; if a later cycle reaches it,
            # the ratchet resumes climbing. Freezing permanently on the
            # first miss (the previous behaviour) meant a single noisy cycle
            # ended the climb for the whole run -- observed stopping
            # peak_height's climb at cycle 5 of 30 even though later cycles
            # comfortably exceeded that bar. The run-level patience counter
            # is what ends the climb now, so it keeps pushing for as long as
            # the rig keeps delivering.
            # Track consecutive misses and, after a few, DECAY the bar back
            # toward what is actually being achieved. A bar that stays far
            # out of reach forever generates a large permanent error, and
            # since cost is (err/tol)^2 that term then dominates every other
            # objective indefinitely -- observed on hardware with a
            # peak_height bar stuck 1.17 N high, which made the optimiser
            # trade away trough depth and Fy/Fz net-zero just to chase it.
            # Decaying keeps this a CONSTRAINED maximisation: the bar tracks
            # the achievable frontier, its cost stays small while sitting at
            # that frontier, so the hard constraints (trough_frac, fy_net,
            # fz_net) win whenever they are violated -- and the bar still
            # ratchets straight back up the moment the rig delivers more.
            spec["_misses"] = spec.get("_misses", 0) + 1
            if spec["_misses"] >= RATCHET_DECAY_AFTER:
                old = spec["value"]
                spec["value"] = measured_val + step
                spec["_misses"] = 0
                spec["_decayed_once"] = True
                log(f"           ratchet: {name} missed bar {old:.3f} "
                    f"{RATCHET_DECAY_AFTER}x -- decaying to {spec['value']:.3f} "
                    f"(tracking achievable frontier)")
            else:
                log(f"           ratchet: {name} {measured_val:.3f} short of bar "
                    f"{spec['value']:.3f} -- holding bar, will retry")


# ============================================================================
# 3. VERIFY -- render the JSON-described target curve so a human can
#    visually confirm it actually looks like what they meant, BEFORE any
#    tuning happens. This never touches the controller at all.
# ============================================================================

def plot_target_curve(t, Fx, spec, out_path):
    """Plots the interpolated curve as a line, with the original JSON
    target_points overlaid as dots -- so you can see both 'what I typed'
    and 'what the interpolator turned it into' on the same picture."""
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(t, Fx, color="#378ADD", linewidth=2)
    for p in spec["channel_definitions"]["Fx"]["target_points"]:
        ax.plot(p["t"], p["F"], "o", color="#333", markersize=5)
    ax.set_xlabel("time within one period (s)")
    ax.set_ylabel("Fx (N)")
    ax.set_title("Target Fx curve as described by JSON\n(dots = your input points, line = interpolation)")
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


# ============================================================================
# 4. CONTROL -- the actual closed loop. This section contains:
#    - decode_params / the simulated plant
#    - the measurement protocol wrapper (wait + 2-cycle + drop-first)
#    - parameter bounds/clamping
#    - the cost function (the JSON math, made concrete)
#    - both controller step functions (WITH and WITHOUT relationships)
#    - the outer run_control_loop that ties it all together with logging
# ============================================================================

def unpack(params):
    """[amp_ratio, freq_ratio, delta_phi, scale, freq_scale] from a parameter
    vector; any missing trailing entries default to 1.0 so an older saved
    3- or 4-element result JSON still loads and replays."""
    p = list(params)
    while len(p) < 5:          # legacy 3- or 4-element vectors default to 1.0
        p = p + [1.0]
    return p[0], p[1], p[2], p[3], p[4]


def decode_params(params):
    """Turns the 3 abstract controller parameters into 6 literal sinusoid
    values a servo actually understands (A1/f1/phi1 = pitch, A2/f2/phi2 =
    heave). This is the ONLY place that knows about CENTER_AMP_DEG /
    CENTER_FREQ_HZ / PHI2 -- everywhere else in the file only ever deals
    with the 3 abstract ratios.

    Uses the GEOMETRIC-MEAN-PRESERVING convention, identical to the
    in_house_wet_test_3D sweep every PRIMARY gain was measured on (see the
    CENTER_AMP_DEG comment above for the verification against the recorded
    cmd.* columns):

        pitch = CENTER * sqrt(ratio)      heave = CENTER / sqrt(ratio)

    so pitch/heave == ratio exactly, and sqrt(pitch*heave) == CENTER always.
    Both servos move when a ratio changes -- that is what the bench did, and
    it is what makes the measured gains transferable.

    NOTE: this is the abstract/physical decode used by both the simulated
    plant and the cost/gain math (PRIMARY, STABLE_BANDS, etc.) below -- it
    is NOT the mission_input wire format, which additionally inverts
    freq_ratio (it stores heave/pitch) and negates the phase. That
    translation lives in motion_command.decode_params_to_mission(), applied
    only at the hardware boundary, so this function and everything above it
    never needs to know real hardware exists."""
    amp_ratio, freq_ratio, delta_phi, scale, freq_scale = unpack(params)
    s_a = math.sqrt(max(amp_ratio, 1e-9))
    s_f = math.sqrt(max(freq_ratio, 1e-9))
    return dict(
        A1=CENTER_AMP_DEG * s_a * scale,   # pitch amplitude, deg
        f1=CENTER_FREQ_HZ * s_f * freq_scale,   # pitch frequency, Hz
        phi1=PHI2,                         # pitch phase, rad -- pinned at 0 (mc.paddle)
        A2=CENTER_AMP_DEG / s_a * scale,   # heave amplitude, deg
        f2=CENTER_FREQ_HZ / s_f * freq_scale,   # heave frequency, Hz
        phi2=PHI2 - delta_phi,             # heave phase, rad -- carries the whole delta_phi
    )


def run_plant_SIMULATED(params, n_samples_per_cycle=400, noise_std=0.02, n_cycles=1):
    """STAND-IN for the real bench rig -- lets the whole pipeline run and be
    tested without any hardware attached. This is NOT a real fluid-structure
    model; it's just enough physics-flavored structure (rectified velocity
    products, like real drag/lift forces scale with velocity squared) to
    produce plausible-looking, noisy, coupled Fx/Fy/Fz waveforms that react
    to the kinematic parameters in a qualitatively sensible way.

    Returns n_cycles full periods of data -- the caller (collect_steady_
    measurement) is responsible for deciding how much of it to actually use.
    """
    cmd = decode_params(params)
    period = 1.0 / min(cmd["f2"], cmd["f1"]) if cmd["f1"] > 0 else 2.0
    # endpoint=False so consecutive cycles tile perfectly with no repeated sample at the seam
    t = np.linspace(0, n_cycles * period, n_samples_per_cycle * n_cycles, endpoint=False)

    theta1 = np.deg2rad(cmd["A1"]) * np.sin(2 * np.pi * cmd["f1"] * t + cmd["phi1"])   # pitch angle, rad
    theta2 = np.deg2rad(cmd["A2"]) * np.sin(2 * np.pi * cmd["f2"] * t + cmd["phi2"])   # heave angle, rad
    v1, v2 = np.gradient(theta1, t), np.gradient(theta2, t)   # angular velocities

    # Toy force model: forces roughly proportional to velocity-squared
    # (drag/lift-like), plus a cross-coupling term between pitch and heave.
    Fx = 2.0 * v1 * np.abs(v1) + 0.4 * v1 * v2
    Fy = 2.0 * v2 * np.abs(v2) + 0.4 * v1 * v2
    # Fz: intentionally only WEAKLY and NONLINEARLY tied to the kinematics,
    # representing the PDF's own finding that Fz is a "consequence of fluid
    # interaction" rather than something directly commanded by velocity.
    Fz = 0.05 * (v1 ** 2 - v2 ** 2)

    # measurement noise, scaled to each channel's own amplitude so a small
    # or large signal gets proportionally similar noise
    for arr in (Fx, Fy, Fz):
        arr += np.random.normal(0, noise_std * max(peak_to_peak(arr), 1e-6), size=arr.shape)

    return t, Fx, Fy, Fz, theta1, theta2


def collect_steady_measurement(params, run_plant=run_plant_SIMULATED,
                                n_cycles=N_CYCLES_PER_MEASUREMENT, wait_s=SETTLE_WAIT_S):
    """THE single choke point every plant measurement goes through, in
    EITHER controller mode. This is deliberate: it means the settling-wait
    and drop-first-cycle protocol is applied identically and automatically
    everywhere, rather than being something each caller has to remember to
    do correctly.

    Sequence:
      1. Wait `wait_s` seconds -- lets mechanical/fluid transients from the
         PREVIOUS parameter setting die out before we command a new one.
      2. Run the plant for `n_cycles` full periods.
      3. Throw away all but the LAST period. This matches your own bench
         protocol (drop the first cycle, use only steady-state data) --
         it guards against a leftover transient from the old parameters
         leaking into what's supposed to be a measurement of the new ones.
      4. Shift the returned time array to start at t=0, so descriptor
         extraction (which assumes "one cycle starting at 0") works
         correctly regardless of which period of the raw signal we
         actually sliced out.
    """
    time.sleep(wait_s)
    t_full, Fx_full, Fy_full, Fz_full, th1_full, th2_full = run_plant(params, n_cycles=n_cycles)

    n = len(t_full)
    one_cycle = n // n_cycles
    sl = slice(-one_cycle, None)   # last `one_cycle` samples = the last period

    t = t_full[sl] - t_full[sl][0]   # re-zero time so the returned cycle starts at t=0
    # FX_SIGN: thrust-direction convention, see its definition above.
    return (t, FX_SIGN * Fx_full[sl], Fy_full[sl], Fz_full[sl],
            th1_full[sl], th2_full[sl])


def clamp_freq_ratio(freq_ratio, target_peak_count):
    """Forces freq_ratio to stay inside the band known (from claim 7) to
    reliably produce `target_peak_count` peaks per cycle, and additionally
    steps it out of CHAOTIC_ZONE if the stable band itself happens to
    overlap that region."""
    lo, hi = STABLE_BANDS.get(target_peak_count, (0.3, 3.0))   # generic fallback band if
                                                                  # target_peak_count wasn't swept
    clamped = min(max(freq_ratio, lo), hi)
    if CHAOTIC_ZONE is not None and CHAOTIC_ZONE[0] <= clamped <= CHAOTIC_ZONE[1]:
        clamped = CHAOTIC_ZONE[0] - 0.05   # step just outside the chaotic band
    return clamped


def clamp_amplitude_limits(params):
    """Scales `scale` down until NEITHER servo's commanded amplitude exceeds
    its physical travel limit. Necessary now that scale exists: without it,
    a large scale combined with an off-center amp_ratio can command an
    amplitude past the servo's range, and controller.py would clamp the
    position downstream -- silently flattening the peaks of the commanded
    sinusoid. That truncated wave is a DIFFERENT waveform, so every
    descriptor measured from the resulting force would be describing a
    motion the controller doesn't think it commanded."""
    amp_ratio, freq_ratio, delta_phi, scale, freq_scale = unpack(params)
    cmd = decode_params(params)
    pitch_rad = math.radians(cmd["A1"])
    heave_rad = math.radians(cmd["A2"])
    worst_frac = max(pitch_rad / PITCH_AMP_LIMIT_RAD, heave_rad / HEAVE_AMP_LIMIT_RAD)
    if worst_frac <= 1.0 or worst_frac < 1e-9:
        return params
    params = np.array(unpack(params), dtype=float)
    params[3] = max(SCALE_BOUNDS[0], params[3] / worst_frac)
    return params


def clamp_slew(params):
    """Pulls amp_ratio back TOWARD 1.0 until neither servo's commanded peak
    angular velocity (2*pi*f*A_rad) exceeds SLEW_LIMIT_RAD_S. Both
    AMP_RATIO_BOUNDS and STABLE_BANDS are 1D clamps on a single parameter
    each -- neither alone stops the two from COMBINING into a peak velocity
    the real servo can't track.

    Checks BOTH axes: under the geometric-mean-preserving decode, moving
    amp_ratio away from 1.0 SHRINKS one servo's amplitude while GROWING the
    other's, so the binding axis flips depending on which side of 1.0 you're
    on (amp_ratio<1 makes heave the big/fast one, amp_ratio>1 makes pitch).
    The old version checked pitch only, which under this decode would have
    missed every heave-limited case entirely.

    Pulling toward 1.0 (rather than scaling amp_ratio down unconditionally,
    as the pitch-only version did) is the correct move for the same reason:
    1.0 is the geometric center where both amplitudes equal CENTER_AMP_DEG
    and neither is inflated."""
    params = np.array(unpack(params), dtype=float)
    for _ in range(60):
        cmd = decode_params(params)
        v_pitch = 2.0 * np.pi * cmd["f1"] * math.radians(cmd["A1"])
        v_heave = 2.0 * np.pi * cmd["f2"] * math.radians(cmd["A2"])
        worst = max(v_pitch, v_heave)
        if worst <= SLEW_LIMIT_RAD_S or worst < 1e-9:
            return params
        # `scale` is the direct lever (it multiplies both amplitudes, so it
        # scales both tip speeds proportionally) -- use it first, exactly to
        # the needed factor. Only if scale is already at its floor do we fall
        # back to nudging amp_ratio toward 1.0 (the geometric center, where
        # neither servo's amplitude is inflated).
        need = SLEW_LIMIT_RAD_S / worst
        if params[3] > SCALE_BOUNDS[0] + 1e-9:
            params[3] = max(SCALE_BOUNDS[0], params[3] * need)
            continue
        params[0] = params[0] + 0.10 * (1.0 - params[0])
        if abs(params[0] - 1.0) < 1e-6:
            return params
    return params


def clamp_params(params, target_peak_count):
    """Applies ALL physical/known-safe bounds to a parameter vector at
    once. Called after every single parameter update, in both controller
    modes, so neither an aggressive gain nor a noisy finite-difference
    estimate can push a parameter somewhere unreachable, previously flagged
    as unstable, or too fast for the servo to physically track."""
    params = np.array(unpack(params), dtype=float)
    params[0] = min(max(params[0], AMP_RATIO_BOUNDS[0]), AMP_RATIO_BOUNDS[1])   # amp_ratio
    params[1] = clamp_freq_ratio(params[1], target_peak_count)                    # freq_ratio
    params[2] = min(max(params[2], DELTA_PHI_BOUNDS[0]), DELTA_PHI_BOUNDS[1])    # delta_phi
    params[3] = min(max(params[3], SCALE_BOUNDS[0]), SCALE_BOUNDS[1])            # scale
    params[4] = min(max(params[4], FREQ_SCALE_BOUNDS[0]), FREQ_SCALE_BOUNDS[1])  # freq_scale
    params = clamp_amplitude_limits(params)   # servo travel limits (both axes)
    params = clamp_slew(params)   # joint amp_ratio x freq_ratio x scale, BOTH axes, last
    return params


# Multivariate model of the rig, fitted over the full 324-mission bench grid:
#     descriptor ~ a*amp_ratio + b*freq_ratio + c*delta_phi + const
# These are exactly the "which parameter goes where" relationships PRIMARY
# encodes as gradients; here the SAME knowledge is used in the other
# direction -- given the descriptors the target curve implies, solve for the
# parameters that should produce them. R^2 is carried per descriptor and used
# as the fit weight, so a well-determined relationship (peak_height, 0.41)
# dominates the solve and a meaningless one (skew, 0.01) is effectively
# ignored rather than injecting noise into the prediction.
BENCH_MODEL = {
    # descriptor:   (a_amp,  b_freq,  c_dphi,   const,   R^2)
    "peak_height": (-0.3619, -0.6512, +0.0685, +4.3290, 0.408),
    "trough_min":  (+0.1508, +0.2254, +0.0307, -1.8695, 0.159),
    "peak_count":  (+0.0318, -0.3465, -0.0087, +1.8477, 0.111),
    "skew":        (-0.0284, -0.0047, -0.0097, +0.0412, 0.011),
}


def predict_params_for_target(target_desc, target_peak_count):
    """Invert BENCH_MODEL to predict the parameters that should reproduce the
    target curve's descriptors -- the seeding step.

    Weighted least squares across every descriptor the target defines, each
    weighted by its fit R^2. Solves for [amp_ratio, freq_ratio, delta_phi];
    scale and freq_scale start at 1.0 because they are magnitude knobs and
    the match stage is about SHAPE. The result is clamped into the usual
    bounds -- a target can legitimately ask for more than the rig can do
    (this curve wants 9 N against a measured bench maximum of 7.5 N), and an
    unclamped extrapolation would start the run outside its own limits.
    """
    rows, rhs, wts = [], [], []
    for name, (a, b, c, const, r2) in BENCH_MODEL.items():
        if name not in target_desc or r2 <= 0.02:
            continue
        rows.append([a, b, c])
        rhs.append(target_desc[name] - const)
        wts.append(math.sqrt(r2))
    if not rows:
        return None
    A = np.array(rows) * np.array(wts)[:, None]
    y = np.array(rhs) * np.array(wts)
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    amp, fr, dphi = float(sol[0]), float(sol[1]), float(sol[2])
    lo, hi = STABLE_BANDS.get(target_peak_count, (0.40, 2.50))
    amp = min(max(amp, AMP_RATIO_BOUNDS[0]), AMP_RATIO_BOUNDS[1])
    fr = min(max(fr, lo), hi)
    dphi = min(max(dphi, DELTA_PHI_BOUNDS[0]), DELTA_PHI_BOUNDS[1])
    return np.array([amp, fr, dphi, 1.0, 1.0])


def initial_guess(target_peak_count, target_desc=None):
    """Starting parameter vector. When the target curve's descriptors are
    available, they are run through predict_params_for_target so the run
    STARTS at the model's best guess for that shape. Falling back to a fixed
    point only when no target descriptors are given."""
    if target_desc:
        pred = predict_params_for_target(target_desc, target_peak_count)
        if pred is not None:
            return pred
    lo, hi = STABLE_BANDS.get(target_peak_count, (0.40, 2.50))
    fr = START_FREQ_RATIO.get(target_peak_count, (lo + hi) / 2)
    return np.array([1.0, min(max(fr, lo), hi), 0.0, 1.0, 1.0])


def objective_term_and_signal(obj_spec, measured_value):
    """Turns ONE objective spec ({"type", "value"?, "weight"}) plus the
    current measured value into two numbers:
      cost_term    -- how much this objective contributes to total cost
                       right now (lower is always better)
      push_signal  -- which direction (and how strongly) increasing the
                       MEASURED value would reduce this cost term.
                       Mathematically: push_signal = -d(cost_term)/d(measured)

    This is the literal, computable definition of "what optimization means"
    for a single descriptor -- there is no other place in the code that
    defines this; everything else just calls this function.
    """
    w = obj_spec.get("weight", 1.0)
    t = obj_spec["type"]

    if obj_spec.get("value", 0) is None:
        # Un-seeded ratchet (first cycle): no bar exists yet, so it can
        # contribute neither cost nor a push direction. Seeded by
        # ratchet_step immediately after this cycle is scored.
        return 0.0, 0.0

    if t == "target":
        # cost_term = w * (value - measured)^2   (a parabola, minimum at measured == value)
        # d(cost_term)/d(measured) = -2*w*(value - measured), but we drop the
        # constant factor of 2 -- it just rescales the signal uniformly and
        # gets absorbed into the fixed gains anyway.
        value = obj_spec["value"]
        err = value - measured_value
        # Cost is normalised by the objective's TOLERANCE, so "tolerance"
        # means what it says: how much error this objective will accept.
        # Un-normalised (raw w*err^2), objectives on different natural scales
        # were incomparable -- on hardware a peak_height ratchet bar sitting
        # 1.17 N out of reach contributed 1.37, while a trough 0.50 N deep
        # against a 0.05 N tolerance contributed only 0.75 at THREE TIMES the
        # weight. The model-free optimiser therefore correctly concluded that
        # DEEPENING the trough to chase peak height lowered total cost, which
        # is exactly what was seen (trough -0.40 -> -0.54 as scale climbed).
        # Normalised, those terms become 137 vs 300 -- the priority the weight
        # was meant to express. Signal is deliberately left un-normalised: it
        # sets WITH-mode step size through the PRIMARY gains, which were
        # fitted against raw error.
        tol = obj_spec.get("tolerance")
        denom = (tol ** 2) if tol else 1.0
        return w * err ** 2 / denom, w * err

    elif t == "maximize":
        # cost_term = -w * measured   (straight line, always decreasing as measured increases)
        # slope is constant, so the push signal is just +w regardless of the
        # current value -- "always keep pushing this up."
        return -w * measured_value, w

    elif t == "minimize":
        # mirror image of maximize: always push the value down
        return w * measured_value, -w

    raise ValueError(f"unknown objective type: {t}")


def objective_accuracy(obj_spec, measured_value):
    """Turns ONE objective spec + measured value into a [0, 1] accuracy
    fraction, or None if accuracy isn't computable for this objective (a
    maximize/minimize objective with no "reference" given -- there is no
    natural 100% point for an unbounded objective without one).

    - target type: accuracy = 1 - |value - measured| / tolerance, clipped
      to [0, 1]. tolerance is an explicit obj_spec["tolerance"] if given;
      otherwise defaults to |value| for a nonzero target (relative error,
      the original behavior -- a target of 9.0 and a target of 0.5 are
      judged on comparable relative terms), or 1.0 for a target of exactly
      (or near) 0.0.
      FIXED BUG: the original formula used max(|value|, 1e-9) as the
      denominator unconditionally -- for value=0 that's a denominator of
      1e-9, so ANY nonzero measurement (even a genuinely excellent one,
      e.g. measured=0.02 against a target of 0) computed an error ratio in
      the millions and clipped to 0% accuracy. Confirmed on real hardware:
      SECONDARY_OBJECTIVES' fy_net/fz_skew both target 0.0 and were scoring
      ~0% every single cycle regardless of how close to zero they actually
      got, dragging overall accuracy down by 2 of 5 weighted terms
      unconditionally. Explicit "tolerance" values are now set on those two
      (see SECONDARY_OBJECTIVES below) instead of relying on this fallback.
    - maximize type (needs "reference"): accuracy = measured / reference,
      clipped to [0, 1]. reference is the value you're calling "as good as
      it needs to be" -- NOT a hard ceiling, the controller can and will
      keep pushing past it if the gain lets it (accuracy just reports >100%
      as 100%, since "more than the goal" is still full marks).
    - minimize type (needs "reference"): accuracy = reference / measured,
      clipped to [0, 1] (assumes reference and measured are positive and
      reference is the smallest value you're calling "good enough").
    """
    t = obj_spec["type"]

    if obj_spec.get("value", 0) is None:
        return None   # un-seeded ratchet -- excluded from accuracy this cycle

    if t == "target":
        value = obj_spec["value"]
        tolerance = obj_spec.get("tolerance")
        if tolerance is None:
            tolerance = abs(value) if abs(value) > 1e-6 else 1.0
        return float(np.clip(1.0 - abs(value - measured_value) / tolerance, 0.0, 1.0))

    reference = obj_spec.get("reference")
    if reference is None:
        return None   # no yardstick given -- excluded from accuracy, not silently assumed perfect

    if t == "maximize":
        if reference <= 0:
            return None
        return float(np.clip(measured_value / reference, 0.0, 1.0))

    elif t == "minimize":
        if measured_value <= 0:
            return 0.0
        return float(np.clip(reference / measured_value, 0.0, 1.0))

    raise ValueError(f"unknown objective type: {t}")


def overall_accuracy(measured_all, objectives):
    """Weighted average accuracy across every objective that HAS a
    computable accuracy (JSON objectives with a value/reference, plus
    peak_count as an exact-match binary check, plus the two hardcoded
    secondary objectives, which always have a value so are always
    included). Objectives with no computable accuracy are reported
    separately as `excluded` so it's always visible what the percentage
    is and isn't accounting for.

    Returns (accuracy_fraction, included_names, excluded_names)."""
    weighted_sum, weight_total = 0.0, 0.0
    included, excluded = [], []

    for name, spec in objectives.items():
        if name == "peak_count":
            # binary exact-match check rather than objective_accuracy's
            # continuous formulas, since peak_count is a discrete count
            acc = None if spec.get("value") is None else \
                  (1.0 if measured_all.get(name) == spec["value"] else 0.0)
        else:
            acc = objective_accuracy(spec, measured_all[name])

        if acc is None:
            excluded.append(name)
            continue
        w = spec.get("weight", 1.0)
        weighted_sum += w * acc
        weight_total += w
        included.append(name)

    for name, spec in SECONDARY_OBJECTIVES.items():
        if name in MAGNITUDE_OBJECTIVES and name not in objectives:
            continue   # deferred until the MATCH stage completes
        spec = objectives.get(name, spec)
        acc = objective_accuracy(spec, measured_all[name])
        w = spec.get("weight", 1.0)
        weighted_sum += w * acc
        weight_total += w
        included.append(name)

    accuracy = weighted_sum / weight_total if weight_total > 0 else 0.0
    return accuracy, included, excluded


def total_cost(measured_all, objectives):
    """Sums cost_term across every objective that applies to this cycle:
    the JSON-declared objectives PLUS the two hardcoded secondary ones
    (Fy net thrust, Fz symmetry). peak_count is skipped here on purpose --
    it's handled as a hard discrete constraint (via clamp_freq_ratio), not
    as a smooth cost term you can take a gradient of.

    measured_all is expected to be a dict containing every descriptor name
    that any current objective might reference (both Fx descriptors and the
    fy_net/fz_skew secondary ones) -- callers build this by merging the
    two extraction functions' outputs together."""
    c = 0.0
    for name, spec in objectives.items():
        if name == "peak_count":
            continue
        if name in measured_all:
            term, _ = objective_term_and_signal(spec, measured_all[name])
            c += term
    for name, spec in SECONDARY_OBJECTIVES.items():
        if name in MAGNITUDE_OBJECTIVES and name not in objectives:
            continue   # deferred until the MATCH stage completes
        term, _ = objective_term_and_signal(objectives.get(name, spec),
                                            measured_all[name])
        c += term
    return c


# --- online gain re-identification -----------------------------------------
# The PRIMARY table is a SEED, not gospel: it is fitted on bench data taken
# at one operating point, and the controller may well be driven somewhere
# that behaves differently. GainAdapter watches the run's own
# (parameters -> measured descriptor) history and, once it has enough
# evidence, replaces a seeded gain with one re-fitted from what this rig is
# actually doing right now.
#
# Guards, so a noisy handful of cycles can't hijack a good seed:
#   * ADAPT_MIN_SAMPLES cycles must be collected first (>=5, per design).
#   * the parameter must actually have MOVED (ADAPT_MIN_RANGE) -- regressing
#     a descriptor against a parameter pinned at a bound estimates nothing.
#   * the fit must reach ADAPT_MIN_R2, otherwise the seed is kept.
# Every override is logged with its R^2 and sample count, so a gain never
# changes silently.
ADAPT_MIN_SAMPLES = 5
ADAPT_MIN_R2 = 0.50
ADAPT_MIN_RANGE = {0: 0.15, 1: 0.15, 2: 0.15, 3: 0.08, 4: 0.08}   # per-parameter minimum spread
# Cap on a re-identified gain's magnitude, as the largest parameter step a
# unit objective signal is allowed to command in one cycle. A re-fit early in
# a run can legitimately come back very large (the fit is exact through few
# points), and an uncapped gain simply slams the parameter into its bound on
# the next step, throwing away the fine control the adaptation was supposed
# to buy. These are ~25% of each parameter's usable span.
ADAPT_MAX_GAIN = {0: 0.67, 1: 0.13, 2: 1.57, 3: 0.20, 4: 0.15}

# --- hybrid handover criterion -------------------------------------------
# The WITH phase should hand over only when it is genuinely STUCK, not merely
# when the noisy accuracy metric has failed to set a new record. Accuracy on
# this rig swings ~29-45% at IDENTICAL parameters (peak_count flipping 1<->2
# is enough to do it), so a patience counter alone will trip while the
# controller is still walking the parameters somewhere useful -- observed
# mid-run with amp_ratio still moving 0.535 -> 0.578 -> 0.590 per cycle.
# So handover additionally requires the PARAMETERS to have gone quiet:
# average per-cycle movement, as a fraction of each parameter's own usable
# span, below PARAM_SETTLED_FRAC over the last PARAM_SETTLED_WINDOW cycles.
# HANDOVER_MAX_WITH_CYCLES is a backstop for the case where the parameters
# never settle but keep oscillating without improving.
PARAM_SETTLED_FRAC = 0.01     # 1% of span per cycle, averaged
PARAM_SETTLED_WINDOW = 4
# Lowered 40 -> 12. The WITH phase can only move a parameter some PRIMARY
# gain points at, and the trough's real mechanism -- waveform ASYMMETRY via
# delta_phi -- has no trustworthy gain (no bench data maps parameters to
# trough asymmetry; trough_frac's amp_ratio gain is an educated guess, not a
# fit). Consequence on hardware: delta_phi moved on only 6 of 33 cycles,
# spanning 0.06 rad, so the one knob that can actually reshape the trough sat
# at zero while the WITH phase burned 30+ cycles on scale. The model-free
# phase explores every parameter regardless of gains, so hand over to it
# early and give it the bulk of the run.
HANDOVER_MAX_WITH_CYCLES = 12
PARAM_SPANS = np.array([
    AMP_RATIO_BOUNDS[1] - AMP_RATIO_BOUNDS[0],
    0.5,                                          # typical STABLE_BANDS width
    DELTA_PHI_BOUNDS[1] - DELTA_PHI_BOUNDS[0],
    SCALE_BOUNDS[1] - SCALE_BOUNDS[0],
    FREQ_SCALE_BOUNDS[1] - FREQ_SCALE_BOUNDS[0],
])


def params_settled(history, window=PARAM_SETTLED_WINDOW, frac=PARAM_SETTLED_FRAC):
    """True once the parameter vector has essentially stopped moving --
    i.e. the WITH phase really has nothing left to do, as opposed to the
    accuracy metric merely being noisy."""
    if len(history) < window + 1:
        return False
    P = np.array([unpack(h["params"]) for h in history[-(window + 1):]], dtype=float)
    steps = np.abs(np.diff(P, axis=0)) / PARAM_SPANS
    return bool(steps.mean() < frac)


class GainAdapter:
    def __init__(self):
        self._params = []
        self._measured = []
        self._last = {}

    def observe(self, params, measured_all):
        self._params.append(np.array(unpack(params), dtype=float))
        self._measured.append(dict(measured_all))

    def effective(self, primary, log=print):
        """PRIMARY with any confidently re-identified gains substituted in.

        Uses a MULTIVARIATE least-squares fit (descriptor ~ all four
        parameters + intercept), not one single-variable fit per parameter.
        That matters: the controller moves every parameter on every cycle, so
        the columns are correlated, and a univariate slope attributes shared
        variance to whichever parameter it happens to be regressing against.
        Observed doing exactly that -- peak_height's amp_ratio gain came back
        as +13.6 then +4.8 then +4.6 on consecutive cycles, sign-flipped from
        its seed, purely because scale was moving at the same time. The
        multivariate coefficient is the partial derivative with the other
        parameters held constant, which is what a PRIMARY gain actually means.
        """
        n = len(self._params)
        if n < ADAPT_MIN_SAMPLES:
            return primary
        P = np.array(self._params)
        # Only fit against parameters that genuinely moved; a pinned column
        # carries no information and would make the design matrix singular.
        movers = [i for i in range(P.shape[1])
                  if np.ptp(P[:, i]) >= ADAPT_MIN_RANGE.get(i, 0.15)]
        if not movers:
            return primary
        A = np.column_stack([P[:, movers], np.ones(n)])
        if np.linalg.matrix_rank(A) < A.shape[1]:
            return primary   # collinear design -- cannot separate the effects

        out = {}
        for name, channels in primary.items():
            desc = name if name in DESCRIPTOR_EXTRACTORS else name.rsplit("_", 1)[0]
            ys = [m.get(desc) for m in self._measured]
            if any(v is None for v in ys):
                out[name] = channels
                continue
            y = np.array(ys, dtype=float)
            if np.std(y) < 1e-9:
                out[name] = channels
                continue
            coef, *_ = np.linalg.lstsq(A, y, rcond=None)
            resid = y - A @ coef
            ss_tot = np.sum((y - y.mean()) ** 2)
            r2 = 1.0 - np.sum(resid ** 2) / ss_tot if ss_tot > 1e-12 else 0.0

            new_ch = []
            for idx, seed in channels:
                if idx not in movers or r2 < ADAPT_MIN_R2:
                    new_ch.append((idx, seed))
                    continue
                slope = float(coef[movers.index(idx)])
                # If the fit lands outside the sanity cap, KEEP THE SEED --
                # do not clamp. Clamping substitutes a fabricated number that
                # is neither the seed nor what the data said, and an
                # out-of-cap fit is itself evidence the estimate is not
                # trustworthy. This mattered badly on hardware: the
                # trough_depth<-scale seed of -1.999 (measured on this rig at
                # R^2=0.987) and peak_height<-scale seed of +1.250 were both
                # silently replaced by the cap value +-0.200, weakening the
                # trough's pull on scale ~10x. scale then climbed unopposed
                # and trough_frac stalled around 0.40 -- the adaptation was
                # destroying the best-measured relationship in the file.
                cap = ADAPT_MAX_GAIN.get(idx)
                if cap is not None and abs(slope) > cap:
                    new_ch.append((idx, seed))
                    continue
                prev = self._last.get((name, idx))
                if prev is None or abs(slope - prev) > 0.25 * max(abs(prev), 1e-6):
                    log(f"           gain: {desc} <- params[{idx}] re-identified "
                        f"{seed:+.3f} -> {slope:+.3f}  (model R2={r2:.2f}, n={n})")
                self._last[(name, idx)] = slope
                new_ch.append((idx, slope))
            out[name] = new_ch
        return out


def controller_step_WITH_relationships(params, objectives, measured, target_peak_count,
                                       primary=None):
    """The FAST controller mode: uses the fixed, data-derived gains in
    PRIMARY to move each parameter directly, no extra measurements needed.

    For each descriptor that DOES have known relationship(s) (peak_height,
    rate, skew, trough_min -- see PRIMARY): if you declared an objective
    for it in JSON, compute the push_signal from that objective and this
    cycle's measured value, then apply (gain * signal) to EVERY (idx, gain)
    pair listed for that descriptor (most have one; 'skew' has two -- see
    PRIMARY's derivation comment). If you did NOT declare an objective for
    it, the signal defaults to 0 -- nothing tied to it moves this cycle.

    Any objective for a descriptor with no PRIMARY entry (currently: none
    are fully unmapped, but this stays true in general for any future
    descriptor added to DESCRIPTOR_EXTRACTORS without a PRIMARY entry) is
    silently ignored here -- it's still MEASURED and COSTED (see
    total_cost, called by the outer loop), just not acted on. Only
    controller_step_WITHOUT_relationships can act on those.

    Returns (new_params, signals_dict_for_logging, extra_plant_evals=0) --
    0 extra evals because this mode reacts only to the measurement the
    outer loop already took this cycle; it never needs to probe anything
    itself."""
    new_params = np.array(unpack(params), dtype=float)
    signals = {}
    for name, targets in (primary or PRIMARY).items():
        # a PRIMARY key may be '<descriptor>_<suffix>' -- an extra control
        # channel for an existing descriptor (see 'peak_height_scale') --
        # so resolve the descriptor it actually refers to before looking up
        # the objective and the measurement.
        desc = name if name in DESCRIPTOR_EXTRACTORS else name.rsplit("_", 1)[0]
        if desc in objectives and desc in measured:
            _, signal = objective_term_and_signal(objectives[desc], measured[desc])
        else:
            signal = 0.0   # no JSON objective for this descriptor -> frozen
        signals[name] = signal
        for idx, gain in targets:
            new_params[idx] += gain * signal
    new_params = clamp_params(new_params, target_peak_count)
    return new_params, signals, 0


def controller_step_WITHOUT_relationships(params, objectives, measured, target_peak_count,
                                           run_plant, baseline_cost_val, settle_wait_s=SETTLE_WAIT_S,
                                           target=None):
    """The SLOW, model-free controller mode: makes NO assumption about
    which parameter affects which descriptor, or in which direction. It
    finds out empirically, every single cycle, by finite-difference
    probing.

    For each of the 3 parameters:
      1. Nudge it by a small fixed amount (FD_STEP).
      2. Take a full real measurement at that nudged point (this is why
         this mode costs 3 EXTRA plant measurements per cycle, on top of
         the 1 baseline measurement the outer loop already took).
      3. Recompute total cost at the nudged point using the FULL objective
         set (every JSON objective, whether or not it has a known gain,
         plus the hardcoded secondary ones) -- this is precisely what lets
         this mode act on something like trough_min that the WITH-mode
         cannot.
      4. Record only the SIGN of whether cost went up or down -- not the
         raw magnitude.

    Why sign-only instead of true gradient descent: an earlier version of
    this used the raw finite-difference magnitude directly (gradient
    descent), and it was empirically unstable -- a steep local slope on one
    parameter (delta_phi in particular, where a small phase shift can flip
    a skew measurement's sign) produced a huge gradient estimate that blew
    the parameter straight to its bound in one step, then the same thing
    happened in the opposite direction next cycle, oscillating forever.
    Taking a small FIXED step in the direction the sign check indicates is
    the standard, much more robust fix when a noisy single-sample gradient
    estimate can't be trusted for step SIZE, only step DIRECTION.

    Returns (new_params, empty_signals_dict, extra_plant_evals=3)."""
    direction = np.zeros(len(FIXED_STEP))
    evals = 0
    for i in range(len(FIXED_STEP)):
        perturbed = params.copy()
        perturbed[i] += FD_STEP
        perturbed = clamp_params(perturbed, target_peak_count)

        t_p, Fx_p, Fy_p, Fz_p, _, _ = collect_steady_measurement(
            perturbed, run_plant, wait_s=settle_wait_s)
        evals += 1

        meas_p = extract_fx_descriptors(t_p, Fx_p, expected_freq_hz=decode_params(perturbed)["f1"],
                                        target=target)
        sec_p = extract_secondary(Fy_p, Fz_p)
        cost_p = total_cost({**meas_p, **sec_p}, objectives)

        # +1 if the nudge made cost worse (so we should step the OTHER way),
        # -1 if the nudge made cost better (so we should step further that way)
        direction[i] = -np.sign(cost_p - baseline_cost_val)

    new_params = params + FIXED_STEP * direction
    new_params = clamp_params(new_params, target_peak_count)
    return new_params, {}, evals   # {} = no per-descriptor signals to log in this mode


def fmt(d):
    """Small helper for terminal logging: formats a dict of numbers as
    'key=value, key=value, ...', using 3 decimal places for floats and
    plain repr for anything else (e.g. integer peak_count)."""
    return ", ".join(f"{k}={v:.3f}" if isinstance(v, float) else f"{k}={v}" for k, v in d.items())


def fmt_objectives(objectives):
    """Human-readable one-line summary of the objectives dict, printed at
    the start of every run so the terminal log is self-documenting about
    what the controller is actually trying to do."""
    parts = []
    for name, spec in objectives.items():
        if spec["type"] == "target":
            parts.append(f"{name}: target={spec['value']} w={spec.get('weight', 1.0)}")
        else:
            parts.append(f"{name}: {spec['type']} w={spec.get('weight', 1.0)}")
    return "; ".join(parts)


def run_control_loop(objectives, use_relationships=True, run_plant=run_plant_SIMULATED,
                      settle_wait_s=SETTLE_WAIT_S, target=None, hybrid=False,
                      deferred_objectives=None, target_desc=None):
    """The outer control loop. Ties everything above together:
      - picks a starting point (initial_guess)
      - each cycle: take a real measurement, compute cost, ask the chosen
        controller mode for the next parameters, log everything, check for
        convergence (a cost plateau -- see PLATEAU_TOL/PLATEAU_CYCLES above)
      - tracks per-cycle and total wall-clock time throughout, since every
        measurement includes a real settling wait

    Returns (final_params, history, total_plant_evals, total_time_seconds).
    `history` is a list of one dict per cycle, containing everything about
    that cycle (params used, what was measured, cost, raw waveforms, and
    timing) -- report_final() uses the LAST entry to make the final plots,
    but the full history is available for any other analysis afterward."""
    target_peak_count = objectives["peak_count"]["value"]
    params = initial_guess(target_peak_count, target_desc=target_desc)
    if hybrid:
        # Stage-driven: match model-free, then optimise on known gains, then
        # fall back to model-free for what the gains cannot reach.
        mode = ("hybrid: MATCH model-free -> OPTIMISE with relationships "
                "-> REFINE model-free")
    else:
        mode = ("WITH relationships (fast)" if use_relationships
                else "WITHOUT relationships (slow, model-free)")

    print(f"STEP 2: mode = {mode}")
    print(f"         objectives: {fmt_objectives(objectives)}")
    print(f"         each measurement: wait {settle_wait_s:.1f}s to settle, run "
          f"{N_CYCLES_PER_MEASUREMENT} cycles, analyze only the last one")
    print(f"         starting params from stable-band lookup for peak_count={target_peak_count}: "
          f"amp_ratio={params[0]:.3f}, freq_ratio={params[1]:.3f}, "
          f"delta_phi={params[2]:.3f}, scale={params[3]:.3f}")
    print("=" * 70)
    print("STEP 3: closed-loop tuning\n")

    history, total_evals = [], 0
    stage_state = {"stage": 0, "deferred": deferred_objectives or {}}
    with_cycles = 0

    # STAGE 0 IS MODEL-FREE, ALWAYS.
    # The MATCH stage's whole job is to reshape the Fx waveform to the JSON
    # target, and its only active objective is waveform_match -- a Pearson
    # correlation over the resampled waveform. That has no PRIMARY gain and
    # cannot have one: it is not a scalar with a clean derivative w.r.t.
    # amp_ratio the way peak_height is. WITH-relationships mode only moves a
    # parameter that some objective's gain points at, so with only
    # waveform_match active it computed an all-zero signal vector and left
    # the parameters frozen at the seed for the entire run (drag_dominant_8:
    # 10 cycles, params identical to 3 decimal places, waveform_match just
    # rattling between 0.46 and 0.65 on measurement noise alone).
    # Model-free mode needs no gains -- it perturbs each parameter, remeasures
    # and keeps whatever lowered total cost -- so it can genuinely search for
    # shape. It costs 1+len(FIXED_STEP) measurements per cycle instead of 1;
    # that is the price of the matching stage actually doing something.
    if hybrid:
        use_relationships = False
    with_phase_start = None   # cycle at which the WITH (stage-1) phase began
    adapter = GainAdapter()   # re-identifies PRIMARY gains from this run's own data
    best_accuracy = -1.0      # best accuracy seen so far this run (accuracy is always >= 0, so -1 is a safe "none yet")
    best_entry = None         # full cycle record (params, measurements, waveforms) at the best accuracy
    no_improve_count = 0      # consecutive cycles that failed to beat best_accuracy
    loop_start = time.time()

    for k in range(1, MAX_ITERS + 1):
        cycle_start = time.time()

        # --- take this cycle's baseline measurement at the current params ---
        t_m, Fx_m, Fy_m, Fz_m, th1_m, th2_m = collect_steady_measurement(
            params, run_plant, wait_s=settle_wait_s)
        total_evals += 1

        measured = extract_fx_descriptors(t_m, Fx_m, expected_freq_hz=decode_params(params)["f1"],
                                          target=target)
        secondary = extract_secondary(Fy_m, Fz_m)
        measured_all = {**measured, **secondary}
        cost_val = total_cost(measured_all, objectives)
        accuracy, acc_included, acc_excluded = overall_accuracy(measured_all, objectives)

        adapter.observe(params, measured_all)

        # Track the best POSITIVE peak so stage 2 has something to pin to.
        if "fx_pos_peak" in measured_all:
            stage_state["best_pos_peak"] = max(stage_state.get("best_pos_peak", -1e9),
                                               measured_all["fx_pos_peak"])
        stage_before = stage_state.get("stage", 0)
        maybe_advance_stage(objectives, measured_all, stage_state)

        # STAGE 0 -> 1 also switches CONTROL MODE, not just objectives.
        # Matching (stage 0) is a shape problem and ran model-free. The
        # magnitude objectives switched on at this transition -- peak_height,
        # fy_p2p -- DO have measured PRIMARY gains, so stage 1 is exactly
        # where the fast gain-driven mode earns its keep: 1 measurement per
        # cycle instead of 6. This is the "optimise with known relationships"
        # phase; when it plateaus, the handover below drops back to model-free
        # for the "without relationships" phase, which is the only mode able
        # to move fy/fz (no kinematic parameter predicts them).
        if hybrid and stage_before == 0 and stage_state.get("stage", 0) == 1:
            use_relationships = True
            with_phase_start = k
            no_improve_count = 0
            print(f"    switching to WITH-relationships (gain-driven) mode for the "
                  f"optimise stage -- 1 measurement per cycle instead of "
                  f"{1 + len(FIXED_STEP)}.\n")

        # Captured BEFORE ratchet_step: were any objectives still unseeded
        # (and therefore excluded) when this cycle's accuracy was computed?
        scored_partially = any(sp.get("value") is None for sp in objectives.values())

        # cost/accuracy above are scored against what was ACTUALLY aimed for
        # this cycle; ratchet AFTER, so a raised bar only takes effect for
        # the controller step (next) and next cycle's scoring, not this one.
        ratchet_step(objectives, measured_all)

        # --- ask the chosen controller mode what to do next ---
        if use_relationships:
            new_params, signals, extra_evals = controller_step_WITH_relationships(
                params, objectives, measured, target_peak_count,
                primary=adapter.effective(PRIMARY))
        else:
            new_params, signals, extra_evals = controller_step_WITHOUT_relationships(
                params, objectives, measured, target_peak_count, run_plant, cost_val,
                settle_wait_s=settle_wait_s, target=target)
        total_evals += extra_evals   # WITHOUT-mode adds 3 here (one per parameter probed)

        cycle_duration = time.time() - cycle_start
        elapsed = time.time() - loop_start

        # pass/fail display only -- does not affect the controller itself
        def _flag(nm):
            # Only "target"-type objectives have a tolerance to be inside of;
            # a maximize (e.g. fy_p2p, where bigger is simply better) has no
            # pass/fail band, so it is reported as a bare value.
            spec = SECONDARY_OBJECTIVES.get(nm, {})
            tol = spec.get("tolerance")
            if tol is None:
                return "--"
            return "ok" if abs(spec.get("value", 0.0) - secondary[nm]) <= tol else "off"

        print(f"[cycle {k:02d}] params: amp_ratio={params[0]:.3f} freq_ratio={params[1]:.3f} "
              f"delta_phi={params[2]:.3f} scale={params[3]:.3f} "
              f"freq_scale={params[4]:.3f}  "
              f"(plant measurements so far: {total_evals})")
        print(f"           measured Fx: {fmt(measured)}")
        print(f"           cost={cost_val:.4f}" + (f"  signals: {fmt(signals)}" if signals else ""))
        print(f"           accuracy={accuracy*100:.1f}%  (included: {', '.join(acc_included)}"
              + (f"; excluded (no reference given): {', '.join(acc_excluded)}" if acc_excluded else "")
              + ")")
        print(f"           secondary:   "
              + "  ".join(f"{nm}={secondary[nm]:+.3f}[{_flag(nm)}]"
                          for nm in SECONDARY_OBJECTIVES))
        print(f"           timing:      cycle took {cycle_duration:.2f}s, elapsed {elapsed:.2f}s")

        # keep a full record of this cycle -- used by report_final and
        # available afterward for any further analysis of the whole run
        entry = dict(cycle=k, params=params.copy(), measured=measured, secondary=secondary,
                     cost=cost_val, accuracy=accuracy, t=t_m, Fx=Fx_m, Fy=Fy_m, Fz=Fz_m,
                     theta1=th1_m, theta2=th2_m,
                     cycle_duration_s=cycle_duration, elapsed_s=elapsed)
        history.append(entry)

        params = new_params   # advance to the parameters for next cycle

        # --- best-so-far / patience stopping rule ---
        # No fixed accuracy target: keep pushing for as long as accuracy
        # keeps beating its own best. Reset the patience counter on any
        # real improvement (bigger than IMPROVEMENT_TOL, to ignore noise);
        # stop once PATIENCE_CYCLES pass in a row without a new best.
        # Only cycles scored over the FULL objective set are eligible to be
        # "best". `scored_partially` is captured at scoring time, before
        # ratchet_step seeds anything -- while a ratchet is unseeded its
        # objective is excluded from the average (see the None guards in
        # objective_accuracy), so such a cycle is scored over FEWER and
        # EASIER terms and posts an inflated number. Observed: cycle 1 scored
        # 57.9% over 6 terms while every later cycle scored ~37% over 9,
        # which made cycle 1 an unbeatable "best" and froze the run's result
        # on its untuned starting parameters.
        if scored_partially:
            entry["accuracy_comparable"] = False
            continue

        if accuracy > best_accuracy + IMPROVEMENT_TOL:
            best_accuracy = accuracy
            best_entry = entry
            no_improve_count = 0
        else:
            no_improve_count += 1

        with_cycles = k if use_relationships else with_cycles
        # The backstop counts cycles spent IN the WITH phase, not absolute
        # cycles. Stage 0 (model-free matching) now runs first and can easily
        # consume more than HANDOVER_MAX_WITH_CYCLES cycles on its own; an
        # absolute test would fire on stage 1's very first cycle and skip the
        # gain-driven phase entirely.
        with_phase_cycles = (k - with_phase_start + 1) if with_phase_start else k
        if (hybrid and use_relationships
                and ((no_improve_count >= PATIENCE_CYCLES and params_settled(history))
                     or with_phase_cycles >= HANDOVER_MAX_WITH_CYCLES)):
            # The cycle-count backstop is now an INDEPENDENT trigger, not an
            # extra condition on top of an exhausted patience counter. As a
            # conjunct it was unreachable: while the WITH phase keeps finding
            # small improvements the patience counter keeps resetting, so a
            # run sat in the fast phase for 35 cycles with delta_phi frozen at
            # 0.000 and amp_ratio railed to 3.000, never reaching the
            # model-free phase that is the only thing able to move delta_phi
            # or freq_scale at all.
            # HYBRID HANDOVER: the fast, gain-driven phase has stopped making
            # progress. That is expected to happen well before the result is
            # actually optimal, because WITH-mode can only move a parameter
            # that some PRIMARY gain points at -- fy_net and fz_net have no
            # gain at all (no kinematic parameter predicts them; best R^2 was
            # 0.048/0.060 over the full bench grid), so this phase is
            # structurally blind to them. Hand over to the model-free phase,
            # which needs no gains: it probes each parameter and follows
            # whichever direction lowers TOTAL cost, so every objective --
            # including the Fy/Fz ones -- becomes actionable.
            # Resume from the BEST parameters found so far, not the current
            # ones (the last PATIENCE_CYCLES cycles are by definition ones
            # that failed to improve). Patience resets; best_accuracy does
            # NOT, so the slow phase has to genuinely beat the fast phase.
            use_relationships = False
            no_improve_count = 0
            params = np.array(unpack(best_entry["params"]), dtype=float)
            why = ("parameters have settled" if params_settled(history)
                   else f"hit the {HANDOVER_MAX_WITH_CYCLES}-cycle backstop while still moving")
            print(f"\n--- HYBRID HANDOVER at cycle {k} ({why}): WITH-relationships plateaued at "
                  f"{best_accuracy*100:.1f}% (cycle {best_entry['cycle']}). Switching to "
                  f"model-free WITHOUT-relationships mode, resuming from those parameters.")
            print(f"    (this phase costs {1 + len(FIXED_STEP)} plant measurements per cycle "
                  f"instead of 1, so cycles are ~{1 + len(FIXED_STEP)}x slower)\n")
            continue

        # In the hybrid WITH phase, an exhausted patience counter is NOT a
        # reason to stop -- only to consider handing over (above). If the
        # parameters are still moving, the phase is still doing useful work
        # and simply continues; the run can only END from the WITHOUT phase
        # (or from MAX_ITERS).
        if no_improve_count >= PATIENCE_CYCLES and hybrid and use_relationships:
            continue

        if no_improve_count >= PATIENCE_CYCLES:
            total_time = time.time() - loop_start
            print(f"\nStopped at cycle {k}: {PATIENCE_CYCLES} consecutive cycles without beating "
                  f"the best accuracy found ({best_accuracy*100:.1f}%, at cycle {best_entry['cycle']}). "
                  f"This is the best result achieved, not a fixed target -- nothing tried in the "
                  f"last {PATIENCE_CYCLES} cycles improved on it.")
            print(f"SUMMARY: {k} cycles completed, best accuracy {best_accuracy*100:.1f}% "
                  f"(at cycle {best_entry['cycle']}), total time {total_time:.2f}s "
                  f"(avg {total_time / k:.2f}s/cycle)")
            break
    else:
        # this 'else' belongs to the 'for' loop -- runs only if we never 'break'
        # (i.e. MAX_ITERS was hit while accuracy was still finding new improvements)
        total_time = time.time() - loop_start
        print(f"\nReached max iterations ({MAX_ITERS}) while still improving -- "
              f"best accuracy {best_accuracy*100:.1f}% (at cycle {best_entry['cycle']}), "
              f"total plant measurements: {total_evals}")
        print(f"SUMMARY: {MAX_ITERS} cycles completed, best accuracy {best_accuracy*100:.1f}% "
              f"(at cycle {best_entry['cycle']}), total time {total_time:.2f}s "
              f"(avg {total_time / MAX_ITERS:.2f}s/cycle)")

    return params, history, total_evals, total_time, best_entry


# ============================================================================
# 5. REPORT -- final numbers + plots, run once after the loop finishes
# ============================================================================

def plot_final_servo_position(theta1, theta2, t, out_path):
    """Plots the last measured cycle's servo encoder angles (pitch and
    heave) vs time, in degrees -- what the servos were actually
    commanded/measured to do at the final converged (or best-so-far)
    parameters."""
    fig, ax = plt.subplots(figsize=(7, 3.5))
    ax.plot(t, np.rad2deg(theta1), color="#7F77DD", linewidth=2, label="theta1 (pitch)")
    ax.plot(t, np.rad2deg(theta2), color="#1D9E75", linewidth=2, label="theta2 (heave)")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("servo encoder position (deg)")
    ax.set_title("Final servo encoder position vs time")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_final_force_curve(Fx, Fy, Fz, t, out_path):
    """Plots the last measured cycle's Fx, Fy, and Fz vs time together, so
    you can see both the tuned Fx shape AND whether the hardcoded secondary
    objectives (Fy net-zero, Fz symmetric) actually held at the final
    parameters -- a dashed zero line is drawn on all three for reference."""
    fig, axes = plt.subplots(3, 1, figsize=(7, 8), sharex=True)
    for ax, F, label, color in zip(
        axes, [Fx, Fy, Fz],
        ["Fx (thrust, JSON target)", "Fy (lateral, net-zero objective)", "Fz (heave, symmetry objective)"],
        ["#378ADD", "#EF9F27", "#D4537E"],
    ):
        ax.plot(t, F, color=color, linewidth=2)
        ax.axhline(0, color="#888", linewidth=0.8, linestyle="--")
        ax.set_ylabel(f"{label} (N)")
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel("time (s)")
    axes[0].set_title("Final tuned force curves vs time")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def report_final(final_params, history, best_entry, out_prefix):
    """Reports the BEST-found result, not necessarily the last cycle --
    the last few cycles before stopping are, by construction, cycles that
    failed to beat the best, so plotting the true best is more honest than
    plotting whatever the loop happened to end on."""
    best_params = best_entry["params"]
    cmd = decode_params(best_params)
    print("=" * 70)
    print(f"Best result found (cycle {best_entry['cycle']} of {len(history)} run):")
    print(f"  amp_ratio={best_params[0]:.3f}  freq_ratio={best_params[1]:.3f}  "
          f"delta_phi={best_params[2]:.3f} rad  scale={best_params[3]:.3f} "
          f"freq_scale={best_params[4]:.3f}")
    print(f"  A1={cmd['A1']:.2f} deg  f1={cmd['f1']:.3f} Hz  phi1={cmd['phi1']:.3f} rad")
    print(f"  A2={cmd['A2']:.2f} deg  f2={cmd['f2']:.3f} Hz  phi2={cmd['phi2']:.3f} rad")
    print(f"  cycles run: {len(history)}  |  total time: {history[-1]['elapsed_s']:.2f}s  |  "
          f"avg time/cycle: {history[-1]['elapsed_s'] / len(history):.2f}s")
    print(f"  best accuracy: {best_entry['accuracy']*100:.1f}%")
    if best_entry["cycle"] != history[-1]["cycle"]:
        print(f"  (note: the loop continued {len(history) - best_entry['cycle']} more cycles after "
              f"this one, none of which beat it, before stopping on patience)")

    servo_path = plot_final_servo_position(best_entry["theta1"], best_entry["theta2"], best_entry["t"],
                                            f"{out_prefix}_final_servo_position.png")
    force_path = plot_final_force_curve(best_entry["Fx"], best_entry["Fy"], best_entry["Fz"], best_entry["t"],
                                         f"{out_prefix}_final_force_curve.png")
    print(f"\nSTEP 5: final plots written -> {servo_path}, {force_path}")
    return servo_path, force_path


def save_cycle_history_csv(history, out_path):
    """Everything printed to the terminal every cycle (params, every
    measured/secondary descriptor, cost, accuracy, timing), one row per
    cycle -- the persisted counterpart of run_control_loop's per-cycle
    print() block, covering the WHOLE run (not just the best cycle)."""
    import csv
    fieldnames = ["cycle", "amp_ratio", "freq_ratio", "delta_phi", "scale", "freq_scale",
                  "cost", "accuracy", "waveform_match",
                  "peak_height", "rate", "skew", "peak_count", "trough_min",
                  "fy_net", "fy_skew", "fy_p2p", "fz_net", "fz_skew", "fz_p2p",
                  "trough_frac", "trough_frac_pre", "cycle_duration_s", "elapsed_s"]
    with open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for e in history:
            row = {"cycle": e["cycle"], "amp_ratio": e["params"][0], "freq_ratio": e["params"][1],
                   "delta_phi": e["params"][2], "scale": e["params"][3],
                   "freq_scale": e["params"][4],
                   "cost": e["cost"], "accuracy": e["accuracy"],
                   "cycle_duration_s": e["cycle_duration_s"], "elapsed_s": e["elapsed_s"]}
            row.update(e["measured"])
            row.update(e["secondary"])
            w.writerow(row)
    print(f"Cycle history written -> {out_path}")
    return out_path


def save_final_cycle_csv(best_entry, out_path):
    """Raw last-period waveform at the best cycle -- the exact numbers
    behind the final_force_curve / final_servo_position plots, so they can
    be replotted or reanalyzed later without rerunning the experiment."""
    import csv
    t, Fx, Fy, Fz = best_entry["t"], best_entry["Fx"], best_entry["Fy"], best_entry["Fz"]
    th1, th2 = best_entry["theta1"], best_entry["theta2"]
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "Fx_N", "Fy_N", "Fz_N", "theta1_pitch_rad", "theta2_heave_rad"])
        for i in range(len(t)):
            w.writerow([t[i], Fx[i], Fy[i], Fz[i], th1[i], th2[i]])
    print(f"Final-cycle raw data written -> {out_path}")
    return out_path


def save_result_json(folder, json_path, use_relationships, hardware, best_entry, history,
                      total_evals, total_time):
    """
    Prompts for a filename and writes the found sine parameters to
    <folder>/<name>.json -- both the abstract controller params
    (amp_ratio/freq_ratio/delta_phi) and their decoded literal values
    (A1/f1/phi1 pitch, A2/f2/phi2 heave), plus run metadata. This is the
    only place these numbers get saved -- report_final() only prints them.

    If run in --hardware mode, also includes the literal mission_input
    line that was actually sent at the best cycle (via
    motion_command.decode_params_to_mission), since that's the exact,
    ready-to-replay command for the real rig -- distinct from A1/f1/phi1,
    which are in the abstract theory units, not the mission wire format
    (see the unit/convention comment in motion_command.py).
    """
    best_params = best_entry["params"]
    cmd = decode_params(best_params)

    result = {
        "source_target_json": json_path,
        "controller_mode": "with_relationships" if use_relationships else "without_relationships",
        "hardware": bool(hardware),
        "best_cycle": best_entry["cycle"],
        "cycles_run": len(history),
        "plant_evaluations": total_evals,
        "total_time_s": total_time,
        "best_accuracy": best_entry["accuracy"],
        "abstract_params": {
            "amp_ratio": float(best_params[0]),
            "freq_ratio": float(best_params[1]),
            "delta_phi_rad": float(best_params[2]),
            "scale": float(best_params[3]),
            "freq_scale": float(best_params[4]),
        },
        "decoded_sine_params": {
            "pitch": {"A1_deg": cmd["A1"], "f1_hz": cmd["f1"], "phi1_rad": cmd["phi1"]},
            "heave": {"A2_deg": cmd["A2"], "f2_hz": cmd["f2"], "phi2_rad": cmd["phi2"]},
        },
        "measured_at_best": best_entry["measured"],
        "secondary_at_best": best_entry["secondary"],
    }

    if hardware:
        try:
            sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))
            from soft_propulsors_control import motion_command as mc
            line, _ = mc.decode_params_to_mission(best_params, n_cycles=0, label="SAVED_RESULT")
            result["mission_input_line"] = line
        except Exception as e:
            result["mission_input_line_error"] = str(e)

    default_name = (json_path.split("/")[-1].replace(".json", "")
                    + ("_with" if use_relationships else "_without") + "_result")
    name = input(f"\nSave found parameters as <name>.json in {folder} "
                 f"[{default_name}]: ").strip() or default_name
    name = name[:-5] if name.endswith(".json") else name
    out_path = os.path.join(folder, f"{name}.json")

    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(f"Saved -> {out_path}")
    return out_path


def resolve_experiment_folder(folder_arg):
    """Turns a folder argument (relative to WORKSPACE_ROOT, or absolute)
    into an existing absolute directory -- this folder is now the single
    home for everything one experiment produces: the target JSON it reads,
    and every plot/CSV/result JSON it writes."""
    folder = folder_arg if os.path.isabs(folder_arg) else os.path.join(WORKSPACE_ROOT, folder_arg)
    os.makedirs(folder, exist_ok=True)
    return folder


def resolve_target_json(folder):
    """Prompts for the target-curve JSON's filename and looks for it INSIDE
    `folder` (no more searching WORKSPACE_ROOT directly -- each experiment
    folder holds its own target JSON). If the folder contains exactly one
    .json file already, that's offered as the default so the common case
    is just pressing enter."""
    existing = sorted(f for f in os.listdir(folder) if f.endswith(".json"))
    default = existing[0] if len(existing) == 1 else None
    prompt = f"Target curve JSON filename in {folder}"
    prompt += f" [{default}]: " if default else ": "
    name = input(prompt).strip() or default
    if not name:
        raise SystemExit(f"No JSON filename given, and folder doesn't contain exactly one "
                         f".json to default to (found: {existing or 'none'}).")
    if not name.endswith(".json"):
        name += ".json"
    path = os.path.join(folder, name)
    if not os.path.exists(path):
        raise SystemExit(f"'{path}' not found.")
    return path


# ============================================================================
# MAIN -- runs the full 5-stage pipeline for one experiment folder
# ============================================================================

def main(folder, use_relationships=True, settle_wait_s=SETTLE_WAIT_S, hardware=False,
         hybrid=False):
    """Entry point: resolve the experiment folder -> find its target JSON
    (prompted, by name, inside that folder) -> build+verify target curve ->
    run the control loop -> report + save final results. EVERY output this
    script produces (target-check plot, final servo/force plots, the full
    per-cycle history CSV, the best cycle's raw waveform CSV, and the
    prompted result JSON) is written into `folder` -- nothing is scattered
    into WORKSPACE_ROOT or cwd anymore.

    hardware=False (default): runs against run_plant_SIMULATED -- always
    safe, no servos move. hardware=True drives the REAL rig: it imports
    motion_command's HIL bridge (requires a sourced ROS workspace and the
    launch stack already up + calibrated), starts an HILControlNode, and
    passes its bound run_plant_HARDWARE as the plant. This is a real,
    physically consequential action (moves servos in water) -- it is never
    the default and must be opted into explicitly via the CLI --hardware
    flag (see __main__ below)."""
    folder = resolve_experiment_folder(folder)
    json_path = resolve_target_json(folder)
    spec = load_target_json(json_path)

    t_target, Fx_target = build_target_curve(spec)
    # Descriptors of the TARGET curve itself -- what shape is being asked
    # for. These seed the parameter prediction (see initial_guess).
    target_desc = extract_fx_descriptors(
        t_target, Fx_target, expected_freq_hz=1.0 / spec["period_s"])
    objectives = resolve_objectives(spec, t_target, Fx_target)
    # Fy's "maximise swing" objective lives in SECONDARY_OBJECTIVES but is a
    # magnitude goal like the Fx one, so it is deferred alongside it. Fz's
    # net-zero/symmetry stay active from the start -- they are constraints,
    # not shape objectives, and do not fight the curve match.
    objectives.update({k: dict(v) for k, v in SECONDARY_OBJECTIVES.items()
                       if k in MAGNITUDE_OBJECTIVES})
    objectives, deferred = split_deferred(objectives)
    objectives = apply_ratchets(objectives)

    target_plot = plot_target_curve(t_target, Fx_target, spec,
                                    os.path.join(folder, "target_check.png"))

    print("=" * 70)
    print(f"Experiment folder: {folder}")
    print(f"Target curve JSON: {json_path}")
    print("STEP 1: target curve loaded and rendered ->", target_plot)
    print(f"  objectives (JSON math): {fmt_objectives(objectives)}")
    print("=" * 70)

    run_plant = run_plant_SIMULATED
    hil_node = None
    if hardware:
        sys.path.insert(0, os.path.join(WORKSPACE_ROOT, "src", "soft_propulsors_control"))
        from soft_propulsors_control import motion_command as mc
        print("STEP 1b: hardware mode -- starting HIL node (this WILL drive real servos)...")
        hil_node = mc.start_hil_node()
        print("STEP 1c: capturing rest-force baseline (servos must be idle/calibrated now)...")
        baseline = hil_node.capture_rest_baseline()
        print(f"         baseline Fx0={baseline[0]:.4f} Fy0={baseline[1]:.4f} Fz0={baseline[2]:.4f} "
             f"-- subtracted from every measurement below")
        run_plant = lambda params, n_cycles=1: mc.run_plant_HARDWARE(
            params, n_cycles=n_cycles, node=hil_node)

    mode_tag = "hybrid" if hybrid else ("with" if use_relationships else "without")
    try:
        final_params, history, total_evals, total_time, best_entry = run_control_loop(
            objectives, use_relationships=use_relationships, run_plant=run_plant,
            settle_wait_s=settle_wait_s, target=(t_target, Fx_target), hybrid=hybrid,
            deferred_objectives=deferred, target_desc=target_desc)
        report_final(final_params, history, best_entry, os.path.join(folder, mode_tag))
    finally:
        if hil_node is not None:
            mc.stop_hil_node(hil_node)

    save_cycle_history_csv(history, os.path.join(folder, f"{mode_tag}_cycle_history.csv"))
    save_final_cycle_csv(best_entry, os.path.join(folder, f"{mode_tag}_final_cycle_data.csv"))
    save_result_json(folder, json_path, use_relationships, hardware, best_entry, history,
                     total_evals, total_time)

    return total_evals, len(history), total_time, best_entry["accuracy"]


if __name__ == "__main__":
    # usage: python3 force_control.py <experiment_folder> [with|without] [--hardware]
    # <experiment_folder> : relative to WORKSPACE_ROOT, or absolute. Must
    #                       contain (or you'll be prompted to name) the
    #                       target_curve_*.json to tune against. ALL output
    #                       (plots, CSVs, result JSON) lands here too.
    # "with"    -> controller_step_WITH_relationships    (fast, uses claims 1-6)
    # "without" -> controller_step_WITHOUT_relationships (slow, model-free)
    # --hardware -> drive the REAL rig instead of the simulated plant (opt-in only)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    flags = [a for a in sys.argv[1:] if a.startswith("--")]
    folder_arg = args[0] if len(args) > 0 else "drag_dominant"
    mode = args[1] if len(args) > 1 else "with"
    # "hybrid" runs the fast gain-driven phase first and automatically hands
    # over to the model-free phase once it plateaus -- see the HYBRID
    # HANDOVER block in run_control_loop.
    # --max-cycles=N : hard-stop after N cycles regardless of the patience
    # counters. Useful for a short exploratory run on a new target curve,
    # where the point is to see how the search behaves rather than to let it
    # converge. Overrides the MAX_ITERS safety net downward only.
    for f in flags:
        if f.startswith("--max-cycles="):
            MAX_ITERS = int(f.split("=", 1)[1])
            print(f"[--max-cycles] hard cap set to {MAX_ITERS} cycles")

    main(folder_arg, use_relationships=(mode in ("with", "hybrid")),
         hardware=("--hardware" in flags), hybrid=(mode == "hybrid"))
