# Concept of Operations (ConOps)
_Generated 2026-09-03 from model/conops.yaml._

## Operating environment

- **site:** Backyard pool — known, measured, calm, clear water
- **depth max m:** 0.75
- **tags:** Several waterproofed AprilTags (200 mm, tag36h11) at known/surveyed spots; operator commands which one.
- **home:** Pool centre — fixed, tape-marked
- **lighting:** Daytime / controlled; tags well lit

## Operational phases

| Phase | OPM process | Vehicle state | Narrative |
|---|---|---|---|
| PH-0 Commission | (readiness) | Stowed | ONE-TIME before first operational mission: Operational Readiness Review — calibration, safety-behaviour and leak evidence reviewed, all risks at Low; PI declares readiness, Safety Observer concurs. (Recommission after any hardware/firmware change.) |
| PH-1 Deploy & Settle | (settle) | Stowed | Hand-placed from poolside; ~10 s startup settle; no propulsor motion. |
| PH-2 Task & Search | Dispatching/Searching/Detecting | Searching | Operator queues tags; vehicle sweeps until the commanded tag is detected (>= 1.5 m). |
| PH-3 Home | Homing | Homing -> Arrived | Vehicle reduces range/bearing and declares ACHIEVED at <= 0.30 m. |
| PH-4 Next in Sequence | Dispatching/Searching/Homing | Searching | Dispatcher advances to the next queued tag; search + home repeat. |
| PH-5 Return Home | Returning | AtHome | After the sequence, vehicle returns to pool-centre home via bearing + range. |
| PH-6 Recover | Recovering | Stowed | Negatively buoyant vehicle rests at home on the floor; hand/pole recovery; post-run leak inspection. |

## Validation scenarios

### SC-0 — Watertight integrity (every run)  _(phase PH-1 / PH-6)_
Pre/post each deployment: tissue witness + dye; submerge to 0.75 m; inspect for ingress.

- **Requirements:** SYS-003, SYS-004
- **Verification:** VER-003, VER-004

### SC-1 — Deploy & settle  _(phase PH-1)_
Hand-place; confirm no propulsor motion for the settle period; vehicle safe to handle.

- **Requirements:** SYS-015, SYS-006
- **Verification:** VER-015, VER-006b

### SC-2 — Acquire & approach (primary MOE)  _(phase PH-2 / PH-3)_
Command a tag; vehicle detects (>= 1.5 m), locks, homes, declares ACHIEVED at <= 0.30 m.

- **Requirements:** SYS-001, SYS-002, SYS-007, SYS-008
- **MOPs:** MOP-01, MOP-02, MOP-03
- **Verification:** VER-001, VER-002, VER-007, VER-008

### SC-3 — Search (tag not in initial FOV)  _(phase PH-2)_
Tag outside initial view; vehicle sweeps until it acquires.

- **Requirements:** SYS-009
- **Verification:** VER-009

### SC-4 — Tag sequence + preemption  _(phase PH-4)_
Operator queues several tags; vehicle completes them in order, with priority override + bounded wait.

- **Requirements:** SYS-005, SYS-011
- **Verification:** VER-005, VER-011

### SC-5 — Stuck & escalate  _(phase PH-2 / PH-3)_
A tag is unreachable; vehicle detects no progress, auto-retries, then escalates to the operator.

- **Requirements:** SYS-010
- **Verification:** VER-010

### SC-6 — Return home  _(phase PH-5)_
After the sequence, vehicle navigates to pool-centre home via bearing + range.

- **Requirements:** SYS-014
- **Verification:** VER-014

### SC-7 — Recover  _(phase PH-6)_
Vehicle rests at home on the floor; hand/pole recovery; touch-safe; post-run leak check.

- **Requirements:** SYS-012, SYS-006
- **Verification:** VER-012, VER-006b

### SC-8 — Endurance session  _(phase PH-1 .. PH-6)_
A full deployment runs continuously; confirm >= 30 min before recovery.

- **Requirements:** SYS-013
- **Verification:** VER-013
