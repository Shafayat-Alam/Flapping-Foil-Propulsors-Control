# Commissioning — Operational Readiness Review (ORR)
_Generated 2026-09-03 from model/conops.yaml._

- **Authority:** PI declares readiness; Safety Observer (ACT-SAF) concurs before the first operational mission.
- **Gate:** Operational Readiness Review (ORR). The vehicle is NOT cleared for operational (autonomous, untethered, full-depth) missions until every checklist item has objective evidence and all technical risks sit at Low residual (appetite = NONE).
- **Recommission:** Re-run the ORR after any change to the seals, wiring, firmware, gait gains, or calibration.
- **Exit:** ORR signed -> vehicle is COMMISSIONED -> PH-1 operations may begin.

## Readiness checklist

| ID | Category | Item | Evidence | Status |
|---|---|---|---|---|
| COM-01 | readiness | Integration complete through INT-4 (every stage exit gate passed). | INT-4 | `open` |
| COM-02 | safety | Watertight leak gate passed — unpowered immersion to 0.75 m, zero ingress on the interior witness. | VER-004, VER-M03 | `open` |
| COM-03 | safety | Safety behaviours demonstrated: startup settle, command clamp, config-before-actuation, stale-reject, fault->HOVERING. | VER-015, VER-016, VER-017, VER-019, VER-020 | `open` |
| COM-04 | readiness | All four servos enumerate on RS-485 (Protocol 2.0, 1 Mbaud). | VER-E05 | `open` |
| COM-05 | safety | Electrical integrity: no net-to-water continuity, intact insulation, touch-safe surface. | VER-E04, VER-006b | `open` |
| COM-06 | calibration | Camera intrinsics calibrated and AprilTag edge size set to DP-TAG-SIZE (0.200 m). NOTE: code default tag_size=0.10 must be updated to match. | VER-007 | `open` |
| COM-07 | calibration | Detection range confirmed >= 1.5 m in the actual pool water/lighting. | VER-007 | `open` |
| COM-08 | calibration | Servo zero positions and per-joint mechanical limits configured and loaded in the latched robot_config. | VER-017 | `open` |
| COM-09 | calibration | Buoyancy trimmed slightly negative (submerged-weight test). | VER-012 | `open` |
| COM-10 | calibration | Home (pool centre) surveyed and tag positions measured/recorded. | VER-014 | `open` |
| COM-11 | risk | Every technical risk burned down to Low residual before powered in-water operation. | R-LEAK, R-PROP, R-PERC, R-GEOM, R-CTRL, R-PWR, R-BUOY | `open` |
| COM-12 | readiness | Endurance pack charged; >= 30 min continuous operation demonstrated. | VER-013 | `open` |

## Sign-off

- Integration & V&V evidence reviewed: ______________  Date: ______
- All technical risks at Low residual: ______________  Date: ______
- **PI — readiness declared:** ______________  Date: ______
- **Safety Observer — concurrence:** ______________  Date: ______

> Vehicle is **COMMISSIONED** only when all items above carry objective evidence and both signatures are present. Until then it remains in PH-0.