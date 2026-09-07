# Verification Procedures
_Generated 2026-09-03 from model/verification.yaml._
_Mark `status: pass|fail` in the model and regenerate. Tools per Round-5 confirmed set._

## VER-001 — verifies SYS-001 (Demonstration, high)
> The system shall autonomously detect a commanded AprilTag and head toward it without operator piloting.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** onboard logs (software)
- **Procedure:** Command a tag; observe SCANNING->LOCKING->HEADING->ACHIEVED in mission_status.
- **Pass:** Vehicle reaches ACHIEVED autonomously, unpiloted.
- **Phase:** validation  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-002 — verifies SYS-002 (Test, high)
> The system shall reduce range to the commanded tag at a measurable nonzero rate under nominal conditions.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** onboard apriltag log + overhead phone camera
- **Procedure:** Log apriltag range vs time during HEADING; cross-check track with overhead video + pool scale.
- **Pass:** Range decreases monotonically; mean closure rate > 0 recorded.
- **Phase:** validation  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-003 — verifies SYS-003 (Test, critical)
> The system shall operate submerged to a maximum depth of 0.75 m.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** tape measure + pool
- **Procedure:** Submerge to tape-measured 0.75 m for the run duration.
- **Pass:** Operates at 0.75 m; no ingress (per VER-004).
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-004 — verifies SYS-004 (Test, critical)
> The system shall remain watertight with zero ingress for the mission duration at <= 0.75 m.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** pool + food-colouring dye + paper-towel/tissue witness
- **Procedure:** Place tissue witness inside enclosure; add dye near seams; submerge 0.75 m for >= mission duration; open and inspect witness. (No vacuum test — no syringe.)
- **Pass:** Zero water/dye on the interior witness.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-005 — verifies SYS-005 (Demonstration, medium)
> The system shall accept missions and report status through the operator console.

- **venue:** `sil`  ·  **automated:** `partial`  ·  👤 human-in-the-loop
- **Tool:** operator console (software)
- **Procedure:** Feed a mission on /mission_input; observe /mission_status.
- **Pass:** Mission accepted and status reported.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-006b — verifies SYS-006 (Inspection, critical)
> The system shall be safe to retrieve by hand from the water at any time, presenting no shock, burn, or high-energy pinch hazard.

- **venue:** `hil`  ·  **automated:** `none`
- **Tool:** digital multimeter + thermometer/IR gun
- **Procedure:** Power off: multimeter continuity from each electrical net to enclosure/water side (expect open). After a run: measure external surface temperature; confirm the (negatively buoyant) vehicle lifts cleanly off the floor.
- **Pass:** No net-to-water continuity; surface touch-safe; retrievable by hand/pole.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-007 — verifies SYS-007 (Test, high)
> The system shall reliably detect a 200 mm tag36h11 AprilTag at ranges >= 1.5 m under nominal lighting.

- **venue:** `hil`  ·  **automated:** `partial`
- **Tool:** tape measure + 200 mm waterproof tag on stand + onboard log
- **Procedure:** Place the tag at tape-measured 1.5 m; record apriltag_detections; sweep range to find the reliable detection limit.
- **Pass:** Reliable detection at >= 1.5 m.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-008 — verifies SYS-008 (Demonstration, medium)
> The system shall declare a homing mission ACHIEVED when range to the commanded tag is <= 0.30 m and bearing is aligned.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** onboard apriltag log
- **Procedure:** On ACHIEVED, record the apriltag range.
- **Pass:** Range <= 0.30 m at ACHIEVED.
- **Phase:** validation  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-009 — verifies SYS-009 (Demonstration, medium)
> The system shall autonomously search (sweep) for the commanded tag when it is not initially within the camera field of view.

- **venue:** `sil`  ·  **automated:** `partial`
- **Tool:** kinematic Gazebo (tag in world) or onboard logs + overhead camera
- **Procedure:** Start with the tag outside the initial FOV; observe SCANNING until acquired. Runs in the kinematic sim with a tag placed in the world, then confirmed in water.
- **Pass:** Vehicle finds a tag not initially visible.
- **Phase:** validation  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-010 — verifies SYS-010 (Demonstration, high)
> On no measurable progress toward the commanded tag, the system shall auto-retry up to a configured limit and then escalate to the operator.

- **venue:** `sil`  ·  **automated:** `partial`  ·  👤 human-in-the-loop
- **Tool:** kinematic Gazebo or host-side + onboard logs
- **Procedure:** Make a tag unreachable; observe STUCK -> auto-retry -> operator escalation.
- **Pass:** No-progress detected; retries then escalates; no erratic motion.
- **Phase:** validation  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-011 — verifies SYS-011 (Demonstration, medium)
> The system shall accept a FIFO queue of tag missions and support priority missions that preempt or reorder the running mission, with a configurable wait timeout before preemption.

- **venue:** `sil`  ·  **automated:** `partial`  ·  👤 human-in-the-loop
- **Tool:** software (host-side or sim)
- **Procedure:** Feed a queue; issue a priority/override mission; observe preemption + bounded wait.
- **Pass:** Queue executes in order; override preempts per the timeout.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-012 — verifies SYS-012 (Test, high)
> The system shall be slightly negatively buoyant, resting on the pool floor when unpowered, and shall be retrievable by hand or pole from <= 0.75 m.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** digital scale + hanging/spring scale + calibration weights + pool
- **Procedure:** Weigh in air (W_air); hang fully submerged on the spring scale (W_sub); buoyant force F_b = W_air - W_sub. Release and observe.
- **Pass:** F_b < W_air (net negative); vehicle sinks slowly, rests on floor, retrievable.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-013 — verifies SYS-013 (Test, medium)
> The system shall sustain at least 30 minutes of continuous autonomous operation per deployment.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** flight 3S LiPo + digital multimeter + stopwatch
- **Procedure:** Full charge; run continuous autonomous ops; multimeter on pack voltage; stopwatch the run to low-voltage cutoff. (No bench supply — flight LiPo used.)
- **Pass:** >= 30 min continuous; clean low-voltage handling.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-014 — verifies SYS-014 (Demonstration, high)
> The system shall navigate to a defined home position (pool centre) using AprilTag bearing and range within the known, measured pool.

- **venue:** `water`  ·  **automated:** `none`
- **Tool:** onboard logs + overhead camera + tape-marked pool centre
- **Procedure:** After arrival, command return-to-home; observe the vehicle navigate to the pool centre via bearing + range. (Logic pre-verified in SIL by VER-U02.)
- **Pass:** Vehicle reaches home (pool centre) within tolerance.
- **Phase:** validation  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-015 — verifies SYS-015 (Test, critical)
> The system shall not actuate any propulsor during a configurable startup settling period (default 10 s) after power-on.

- **venue:** `sil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_no_actuation_during_startup_settle`
- **Tool:** host-side / kinematic sim + onboard joint_cmd log
- **Procedure:** Power on; feed a mission immediately; watch joint_cmd / fins for the settling period.
- **Pass:** No propulsor motion until the settle period (default 10 s) elapses.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-016 — verifies SYS-016 (Demonstration, critical)
> On loss of a sensor input or an internal fault, the system shall transition to a stable hover (HOVERING) rather than continue an uncommanded manoeuvre.

- **venue:** `sil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_sensor_loss_transitions_to_hovering`
- **Tool:** host-side / kinematic sim + onboard logs
- **Procedure:** Inject sensor loss / fault (e.g., stop IMU or detections); observe state -> HOVERING.
- **Pass:** Vehicle enters stable hover; no uncommanded manoeuvre.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-017 — verifies SYS-017 (Test, critical)
> The system shall clamp every propulsor command to its configured mechanical limits before output.

- **venue:** `sil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_joint_cmd_clamped_to_limits`
- **Tool:** host-side + joint_cmd log
- **Procedure:** Configure tight limits; command a gait that would exceed them; inspect joint_cmd values.
- **Pass:** No joint_cmd ever exceeds the configured min/max.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-018 — verifies SYS-018 (Test, high)
> The system shall reject malformed mission, configuration, or detection messages without crashing and without commanding the propulsors.

- **venue:** `sil`  ·  **automated:** `full`
- **Script:** `test_robustness.py::test_malformed_messages_rejected`
- **Tool:** host-side
- **Procedure:** Publish malformed mission/config/detection messages; observe node behaviour.
- **Pass:** Nodes log an error, keep running, and issue no propulsor command from bad input.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-019 — verifies SYS-019 (Test, high)
> The system shall ignore target detections older than a configured age and shall not act on stale perception data.

- **venue:** `sil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_stale_detections_ignored`
- **Tool:** host-side + onboard logs
- **Procedure:** Publish a detection then stop; confirm detections older than the threshold are ignored.
- **Pass:** Controller does not act on stale detections (treats as no-detection).
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-020 — verifies SYS-020 (Test, critical)
> The system shall command the propulsors only after receiving the latched robot configuration (correct actuator mapping).

- **venue:** `sil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_no_actuation_before_config`
- **Tool:** host-side + joint_cmd log
- **Procedure:** Start controller without config; confirm no joint_cmd; send latched config; confirm actuation begins.
- **Pass:** No propulsor command issued until the latched configuration is received.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-M01 — verifies MEC-001 (Inspection, medium)
> The vehicle dry mass shall be 1-3 kg (recorded by weigh-in) to meet the buoyancy/ballast budget.

- **venue:** `hil`  ·  **automated:** `none`
- **Tool:** digital scale (g)
- **Procedure:** Weigh the assembled dry vehicle.
- **Pass:** Dry mass within 1-3 kg; value recorded.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-M02 — verifies MEC-002 (Inspection, medium)
> The fins shall be Garolite reinforced with a carbon-fiber rod, with interchangeable surfaces (Icarex+PVA aerial / nylon+steel underwater).

- **venue:** `hil`  ·  **automated:** `none`
- **Tool:** visual + calipers
- **Procedure:** Confirm Garolite + CF-rod fins and the fitted surface (aerial/underwater).
- **Pass:** Materials and reinforcement as specified.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-M03 — verifies MEC-003 (Inspection, critical)
> The aluminum enclosure shall be sealed watertight with silicone at the lid/seams and cord grips at every cable penetration (strain-relieved).

- **venue:** `hil`  ·  **automated:** `none`
- **Tool:** visual
- **Procedure:** Inspect silicone bead at lid/seams and each cord grip (sealed + strain-relieved).
- **Pass:** Continuous silicone seal; all penetrations via sealed cord grips.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-E01 — verifies ELE-001 (Inspection, high)
> The vehicle shall be powered by a 3S LiPo (~2200 mAh) supplying separate servo and compute rails.

- **venue:** `hil`  ·  **automated:** `none`
- **Tool:** visual + multimeter
- **Procedure:** Confirm 3S ~2200 mAh pack; verify two separate rails (servo / compute).
- **Pass:** Correct pack; two rails present.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-E02 — verifies ELE-002 (Test, critical)
> Battery voltage shall not be driven below the LiPo safe cutoff (~9.0 V / 3.0 V per cell); operation below shall trigger a warning or safe shutdown.

- **venue:** `hil`  ·  **automated:** `partial`
- **Tool:** digital multimeter + stopwatch
- **Procedure:** Run to low charge; multimeter on pack voltage; confirm warning/shutdown at/above cutoff.
- **Pass:** Warning or shutdown before voltage falls below ~9.0 V.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-E03 — verifies ELE-003 (Test, high)
> Each servo's current limit shall be configured (factory default) and the peak per-servo current recorded and within safe thermal range.

- **venue:** `hil`  ·  **automated:** `partial`
- **Tool:** digital multimeter (in series) or servo current telemetry
- **Procedure:** Measure per-servo current during 1 Hz flapping; briefly hand-stall and confirm limit holds.
- **Pass:** Peak per-servo current recorded; limit holds on stall; no overheating.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-E04 — verifies ELE-004 (Test, critical)
> All wiring shall show end-to-end continuity < 1 ohm, no unintended shorts, and intact insulation with no exposed conductors reachable when wet.

- **venue:** `hil`  ·  **automated:** `none`
- **Tool:** digital multimeter
- **Procedure:** Power off: continuity of cables (<1 ohm); check each net-to-net and net-to-enclosure for shorts; inspect insulation.
- **Pass:** Continuity < 1 ohm; no shorts; no exposed conductors.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-E05 — verifies ELE-005 (Test, high)
> All four servos shall enumerate and respond on the RS-485 bus (Dynamixel Protocol 2.0).

- **venue:** `hil`  ·  **automated:** `full`
- **Script:** `test_servo_bus.py::test_all_four_ids_enumerate`
- **Tool:** software (Dynamixel ping) + onboard log
- **Procedure:** Run the servo ping; confirm all 4 IDs respond.
- **Pass:** 4/4 servos enumerate on RS-485.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U01 — verifies SYS-002 (Test, high)
> The system shall reduce range to the commanded tag at a measurable nonzero rate under nominal conditions.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_motion_command.py::test_triangle_symmetry_and_bounds, ::test_sweep_profile`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test the gait math: triangle wave symmetry and amplitude bounds; sweep profile center/span/phase.
- **Pass:** Waveforms stay within [center +/- amplitude]; symmetric; sweep matches the specified profile.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U02 — verifies SYS-001 (Test, high)
> The system shall autonomously detect a commanded AprilTag and head toward it without operator piloting.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_state_transitions`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test the controller state machine: SCANNING->LOCKING (STABLE_FRAMES), LOCKING->HEADING, HEADING->ACHIEVED, no-progress->STUCK, and return-home transitions on synthetic detections.
- **Pass:** Every transition fires only under its documented condition; no illegal transitions.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U03 — verifies SYS-017 (Test, critical)
> The system shall clamp every propulsor command to its configured mechanical limits before output.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_clamp_unit`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test the per-joint clamp with values inside, at, and beyond min/max.
- **Pass:** Output is clamped to [min,max] for every input; never exceeds limits.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U04 — verifies SYS-019 (Test, high)
> The system shall ignore target detections older than a configured age and shall not act on stale perception data.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_stale_window_unit`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test the staleness check at ages below, at, and above the threshold.
- **Pass:** Detections older than the window are treated as no-detection.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U05 — verifies SYS-020 (Test, critical)
> The system shall command the propulsors only after receiving the latched robot configuration (correct actuator mapping).

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_config_gate_unit`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test that the command path returns no output until a valid config is set.
- **Pass:** No command produced pre-config; commands produced post-config.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U06 — verifies SYS-011 (Test, medium)
> The system shall accept a FIFO queue of tag missions and support priority missions that preempt or reorder the running mission, with a configurable wait timeout before preemption.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_crab.py::test_queue_order, ::test_retry_count, ::test_priority_preempt, ::test_human_timeout`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test the dispatcher: FIFO ordering, retry counting to the limit, priority preemption, and the human-decision timeout.
- **Pass:** Queue order preserved; retries capped; priority preempts; timeout advances.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U07 — verifies SYS-007 (Test, high)
> The system shall reliably detect a 200 mm tag36h11 AprilTag at ranges >= 1.5 m under nominal lighting.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_apriltag.py::test_range_bearing_from_pose`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test range/bearing computation from known tag poses/intrinsics (synthetic), independent of a live camera.
- **Pass:** Computed range/bearing match the analytic values within tolerance.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U08 — verifies SYS-018 (Test, high)
> The system shall reject malformed mission, configuration, or detection messages without crashing and without commanding the propulsors.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_robustness.py::test_parser_rejects_bad_input`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test message parsers with malformed mission/config/detection payloads.
- **Pass:** Parsers raise/return cleanly; never yield an actuation command.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U09 — verifies SYS-008 (Test, medium)
> The system shall declare a homing mission ACHIEVED when range to the commanded tag is <= 0.30 m and bearing is aligned.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_achieved_declaration`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test the ACHIEVED predicate at ranges/bearings around ARRIVE_DISTANCE and ALIGN_BEARING.
- **Pass:** ACHIEVED only when range <= 0.30 m and bearing aligned.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL

## VER-U10 — verifies SYS-015 (Test, critical)
> The system shall not actuate any propulsor during a configurable startup settling period (default 10 s) after power-on.

- **venue:** `mil`  ·  **automated:** `full`
- **Script:** `test_controller.py::test_startup_settle_unit`
- **Tool:** pytest (host-side)
- **Procedure:** Unit-test that no command is emitted while the monotonic clock is within the settle period.
- **Pass:** Command path suppressed during settle; active after.
- **Phase:** verification  ·  **Status:** `open`
- Result: ______________  Date: ______  Initials: ____  [ ] PASS  [ ] FAIL
