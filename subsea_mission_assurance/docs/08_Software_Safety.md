# Software Safety Analysis (NASA-GB-8719.13)
_Generated 2026-09-03 from model/software_safety.yaml._

## Determination

- **Safety-critical:** True
- **Software control category:** Autonomous (highest). The control software has sole autonomous control of the propulsors; there is no independent hardware/operator inhibit (no e-stop). Per NASA-GB-8719.13 this is the most safety-significant control category and warrants full analysis.
- **Worst credible mishap:** Hardware self-damage (servo/structure) from an out-of-limit or runaway command.
- **Severity:** Marginal (hardware damage; soft compliant fins keep human-injury energy low).
- **Fail-safe state:** Stable HOVERING; on total power/compute loss the negatively-buoyant hull settles to the floor.
- **Rigor:** Full analysis (Round 7) tailored to a research prototype: hazard causes, controls, and verification traced; no independent IV&V.

## Safety-critical requirements

- **SYS-017**: The system shall clamp every propulsor command to its configured mechanical limits before output.
- **SYS-018**: The system shall reject malformed mission, configuration, or detection messages without crashing and without commanding the propulsors.
- **SYS-020**: The system shall command the propulsors only after receiving the latched robot configuration (correct actuator mapping).

## Software hazard analysis

| SWH | Software cause | System hazard | Sev | Likelihood | Controls | Verification |
|---|---|---|---|---|---|---|
| SWH-01 | Propulsor command exceeds the joint's mechanical limit. | Hardware self-damage (servo stall / structural overtravel). | Marginal | Occasional | SYS-017 | VER-017 |
| SWH-02 | Propulsor actuation during hand placement/recovery. | Operator finger pinch from a moving fin (low energy, soft fins). | Negligible | Remote | SYS-015 | VER-015 |
| SWH-03 | Controller acts on stale or false target detections. | Erratic homing drives the vehicle into a pool wall / stresses hardware. | Marginal | Occasional | SYS-019, SYS-001 | VER-019, VER-001 |
| SWH-04 | Loss of a sensor input or an internal fault mid-run. | Uncommanded / erratic manoeuvre. | Marginal | Occasional | SYS-016 | VER-016 |
| SWH-05 | Malformed mission/config/detection message. | Node crash or undefined actuator command. | Marginal | Remote | SYS-018 | VER-018 |
| SWH-06 | Actuation before the latched configuration is received (wrong actuator map). | Wrong-joint runaway -> hardware self-damage. | Marginal | Remote | SYS-020 | VER-020 |