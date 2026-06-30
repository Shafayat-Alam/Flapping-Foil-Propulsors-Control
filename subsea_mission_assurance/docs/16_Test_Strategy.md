# Test Strategy — model→reality ladder & automation
_Generated 2026-06-07 from model/verification.yaml._

**Principle:** Automate as far left on the ladder as the requirement allows; reserve water for what only reality can close.

## The venue ladder

| Venue | Name | Automatable | What it is |
|---|---|---|---|
| `mil` | Model-in-the-loop (pure logic) | full | Node logic exercised in isolation as pytest — no physics, no hardware. Fast, deterministic, runs in CI. |
| `sil` | Software-in-the-loop (kinematic Gazebo) | full | The real ROS nodes closed-loop against the kinematic Gazebo. Verifies state sequencing, scan/search geometry, topic plumbing, the safety gates and fail-safe transitions. AprilTags are placed in the sim world, so the perception pipeline (acquisition + range/bearing extraction) also runs in SIL. |
| `hil` | Hardware-in-the-loop (dry bench) | partial | Real servos/IMU/camera on the bench, out of water — bus enumeration, current limits, sensor streams, real detection range/lighting. |
| `water` | Reality (in-water) | none | The real vehicle in the real pool — the only venue for thrust, buoyancy, watertightness, endurance, and hydrodynamic stability. Human-attended. |

## Coverage

- **38 activities total.**
- By venue: mil=10, sil=10, hil=10, water=8
- By automation: full=17, partial=7, none=14  → **17 fully automated / 24 with some automation**
- Human-in-the-loop: VER-005, VER-010, VER-011

## Activities by venue

- **mil** (Model-in-the-loop (pure logic)): VER-U01, VER-U02, VER-U03, VER-U04, VER-U05, VER-U06, VER-U07, VER-U08, VER-U09, VER-U10
- **sil** (Software-in-the-loop (kinematic Gazebo)): VER-005, VER-009, VER-010, VER-011, VER-015, VER-016, VER-017, VER-018, VER-019, VER-020
- **hil** (Hardware-in-the-loop (dry bench)): VER-006b, VER-007, VER-M01, VER-M02, VER-M03, VER-E01, VER-E02, VER-E03, VER-E04, VER-E05
- **water** (Reality (in-water)): VER-001, VER-002, VER-003, VER-004, VER-008, VER-012, VER-013, VER-014

## What the kinematic Gazebo can / cannot close

- **It is:** Gazebo model is KINEMATIC ONLY — joints move as commanded; there is no hydrodynamics, buoyancy, thrust, or contact drag.
- **✅ Can verify (SIL):** State-machine sequencing, scan/search geometry, config-before-actuation, startup settle, command clamp, stale-detection reject, malformed-message reject, queue/preemption, fault->HOVERING, return-home logic, ACHIEVED declaration, and the perception pipeline — AprilTags are placed in the sim world, so SCANNING acquisition and range/bearing extraction run in SIL (VER-009).
- **❌ Cannot verify (needs HIL / water):** Closure rate / thrust (MOP-01), buoyancy (SYS-012), watertightness (SYS-004), endurance (SYS-013), REAL detection range/lighting (SYS-007), hydrodynamic control stability, servo current/thermal, RS-485 bus reality.
- **Gap / action:** Perception-SIL is enabled by the AprilTags placed in the ocean.sdf world. What still cannot be closed in sim: real detection range/lighting (SYS-007 stays HIL), and thrust/buoyancy/leak/endurance (stay water) — these drive the dry->wet integration gates.

## Human-in-the-loop

Operator acts through the console (tasking, priority override, escalation acknowledgement); these cannot run fully unattended.

Activities: VER-005, VER-010, VER-011