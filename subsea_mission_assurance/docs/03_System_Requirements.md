# System Requirements
_Generated 2026-06-07 from model/requirements.yaml._

## Electrical

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| ELE-001 | The vehicle shall be powered by a 3S LiPo (~2200 mAh) supplying separate servo and compute rails. | SYS-013 | C-POWER | I | draft |
| ELE-002 | Battery voltage shall not be driven below the LiPo safe cutoff (~9.0 V / 3.0 V per cell); operation below shall trigger a warning or safe shutdown. | SYS-016 | C-POWER | T | draft |
| ELE-003 | Each servo's current limit shall be configured (factory default) and the peak per-servo current recorded and within safe thermal range. | SYS-017 | C-SERVO | T | draft |
| ELE-004 | All wiring shall show end-to-end continuity < 1 ohm, no unintended shorts, and intact insulation with no exposed conductors reachable when wet. | SYS-006 | C-POWER | T | draft |
| ELE-005 | All four servos shall enumerate and respond on the RS-485 bus (Dynamixel Protocol 2.0). | SYS-001 | C-SERVO, C-COMPUTE | T | draft |

## Environmental

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| SYS-003 | The system shall operate submerged to a maximum depth of 0.75 m. | STK-EXP-03 | C-ENCL | T | draft |

## Functional

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| SYS-001 | The system shall autonomously detect a commanded AprilTag and head toward it without operator piloting. | MOE-01 | C-CTRL, C-APRIL | D | draft |
| SYS-004 | The system shall remain watertight with zero ingress for the mission duration at <= 0.75 m. | STK-EXP-03 | C-ENCL | T | draft |
| SYS-009 | The system shall autonomously search (sweep) for the commanded tag when it is not initially within the camera field of view. | MOE-01 | C-CTRL | D | draft |
| SYS-010 | On no measurable progress toward the commanded tag, the system shall auto-retry up to a configured limit and then escalate to the operator. | STK-EXP-01 | C-CRAB, C-CTRL | D | draft |
| SYS-008 | The system shall declare a homing mission ACHIEVED when range to the commanded tag is <= 0.30 m and bearing is aligned. | MOP-03 | C-CTRL | D | draft |
| SYS-014 | The system shall navigate to a defined home position (pool centre) using AprilTag bearing and range within the known, measured pool. | STK-EXP-01 | C-CTRL, C-APRIL | D | draft |

## Interface

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| SYS-005 | The system shall accept missions and report status through the operator console. | STK-EXP-01 | C-CRAB, C-OPC | D | draft |
| SYS-011 | The system shall accept a FIFO queue of tag missions and support priority missions that preempt or reorder the running mission, with a configurable wait timeout before preemption. | STK-EXP-01 | C-CRAB, C-OPC | D | draft |

## Mechanical

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| SYS-012 | The system shall be slightly negatively buoyant, resting on the pool floor when unpowered, and shall be retrievable by hand or pole from <= 0.75 m. | STK-EXP-02 | C-VEH, C-HULL | T | draft |
| MEC-001 | The vehicle dry mass shall be 1-3 kg (recorded by weigh-in) to meet the buoyancy/ballast budget. | SYS-012 | C-VEH | I | draft |
| MEC-002 | The fins shall be Garolite reinforced with a carbon-fiber rod, with interchangeable surfaces (Icarex+PVA aerial / nylon+steel underwater). | SYS-002 | C-FIN | I | draft |
| MEC-003 | The aluminum enclosure shall be sealed watertight with silicone at the lid/seams and cord grips at every cable penetration (strain-relieved). | SYS-004 | C-ENCL | I | draft |

## Performance

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| SYS-002 | The system shall reduce range to the commanded tag at a measurable nonzero rate under nominal conditions. | MOP-01 | C-CTRL, C-FIN | T | draft |
| SYS-007 | The system shall reliably detect a 200 mm tag36h11 AprilTag at ranges >= 1.5 m under nominal lighting. | MOP-02 | C-APRIL, C-CAM | T | draft |
| SYS-013 | The system shall sustain at least 30 minutes of continuous autonomous operation per deployment. | STK-EXP-01 | C-POWER, C-COMPUTE | T | draft |

## Safety

| ID | Requirement | Parent | Allocated | V | Status |
|---|---|---|---|---|---|
| SYS-006 | The system shall be safe to retrieve by hand from the water at any time, presenting no shock, burn, or high-energy pinch hazard. | STK-EXP-02 | C-VEH, C-ENCL | I | draft |
| SYS-015 | The system shall not actuate any propulsor during a configurable startup settling period (default 10 s) after power-on. | STK-EXP-02 | C-CTRL | T | draft |
| SYS-016 | On loss of a sensor input or an internal fault, the system shall transition to a stable hover (HOVERING) rather than continue an uncommanded manoeuvre. | STK-EXP-02 | C-CTRL | D | draft |
| SYS-017 | The system shall clamp every propulsor command to its configured mechanical limits before output. | STK-EXP-02 | C-CTRL | T | draft |
| SYS-018 | The system shall reject malformed mission, configuration, or detection messages without crashing and without commanding the propulsors. | STK-EXP-02 | C-CRAB, C-CTRL | T | draft |
| SYS-019 | The system shall ignore target detections older than a configured age and shall not act on stale perception data. | STK-EXP-02 | C-CTRL | T | draft |
| SYS-020 | The system shall command the propulsors only after receiving the latched robot configuration (correct actuator mapping). | STK-EXP-02 | C-CTRL, C-CRAB | T | draft |
