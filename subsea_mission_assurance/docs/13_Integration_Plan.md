# Integration & Verification Plan
_Generated 2026-09-03 from model/integration.yaml._

- **Strategy:** Incremental, bottom-up, with dry -> wet stage gates.
- **Method:** DSM-sequenced. Coupled clusters (control feedback loops) are integrated as a block, not piecewise.
- **Rationale:** Water ingress (R-LEAK) is the top risk, so water exposure is deferred until a full DRY integration gate passes. Each interface is brought up and observed in isolation before loops are closed, giving clean fault isolation per added interface. Immersion then proceeds unpowered -> powered-shallow -> full-depth.

## Stages

### INT-0 — Bench / dry component bring-up  (🔧 DRY)
_Each sensor/actuator interface alive and observable in isolation — no closed loop, no water._

- **Components added:** C-COMPUTE, C-SERVO, C-IMU, C-CAM, C-CAMIF, C-IMUIF, C-APRIL, C-DXLIF, C-POWER
- **Interfaces exercised:** IF-RS485, IF-I2C, IF-USB, IF-IMG, IF-TAG, IF-IMU, IF-JFB
- **Entry:** Components mounted on the bench; flight LiPo charged; no water.
- **Exit gate:** All 4 servos enumerate on RS-485; IMU streams at rate; camera -> apriltag detects a tag on the bench at >= 1.5 m.
- **Verification:** VER-E05, VER-E01, VER-E03, VER-007, VER-010
- **Risks addressed:** R-PERC

### INT-1 — Dry closed-loop integration  (🔧 DRY)
_Close the control loop in air/sim — mission plane + actuation + perception together, propulsors free to move but out of water._

- **Components added:** C-CRAB, C-CTRL, C-MOTION, C-OPC
- **Interfaces exercised:** IF-MISIN, IF-MISCMD, IF-MISSTAT, IF-CFG, IF-JCMD, IF-TLM
- **Entry:** INT-0 passed; controller + crab launched; robot_config latched.
- **Exit gate:** Config-before-actuation, startup-settle, command clamp, stale-reject, and fault->HOVERING all demonstrated dry; no actuation before config.
- **Verification:** VER-015, VER-016, VER-017, VER-018, VER-019, VER-020, VER-005, VER-011
- **Risks addressed:** R-CTRL

### INT-2 — Unpowered immersion (leak gate)  (💧 WET)
_Prove watertightness before any powered in-water run._

- **Components added:** C-ENCL, C-HULL, C-VEH
- **Interfaces exercised:** —
- **Entry:** INT-1 passed; enclosure sealed (silicone + cord grips); interior witness placed.
- **Exit gate:** Submerged to 0.75 m for >= mission duration with zero dye/water on the interior witness; buoyancy slightly negative and hand/pole recoverable.
- **Verification:** VER-004, VER-M03, VER-003, VER-012
- **Risks addressed:** R-LEAK, R-BUOY

### INT-3 — Powered shallow trial (tethered)  (💧 WET)
_First powered, in-water, closed-loop run at shallow depth with a short tether for abort/recovery._

- **Components added:** C-FIN, C-VEH
- **Interfaces exercised:** IF-JCMD, IF-JFB, IF-TAG, IF-IMU
- **Entry:** INT-2 leak gate passed; gentle gait gains loaded; tether attached.
- **Exit gate:** Stable closed-loop swimming; measurable closure toward a tag (MOP-01); no oscillation/runaway; safe surface temperature and recovery.
- **Verification:** VER-002, VER-006b, VER-014
- **Risks addressed:** R-PROP, R-GEOM, R-CTRL

### INT-4 — Full autonomous mission (validation)  (💧 WET)
_End-to-end untethered autonomous mission at full test depth — the system validation runs._

- **Components added:** C-VEH
- **Interfaces exercised:** IF-MISIN, IF-MISSTAT, IF-TAG, IF-JCMD
- **Entry:** INT-3 passed; gait tuned; endurance pack charged.
- **Exit gate:** Autonomous SCANNING->LOCKING->HEADING->ACHIEVED, return-to-home, and the 30-run acquisition campaign (MOP-02) at >= 0.90; >= 30 min endurance.
- **Verification:** VER-001, VER-008, VER-009, VER-013
- **Risks addressed:** R-PROP, R-PERC
