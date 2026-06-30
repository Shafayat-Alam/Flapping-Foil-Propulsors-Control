# Technical Risk Register
_Generated 2026-06-07 from model/risks.yaml._

- **Scale:** 5x5. Score = Likelihood x Consequence. Bands: Low <= 4, Medium 5-11, High >= 12.
- **Appetite:** NONE. All risks shall be mitigated to Low residual before any powered in-water operation.

## Risks (ranked)

| ID | Title | L | C | Score | Band | Traces | Verification | Residual |
|---|---|---|---|---|---|---|---|---|
| R-PROP | Propulsion ineffective | 4 | 4 | 16 | High | SYS-002 | VER-002 | Low |
| R-LEAK | Water ingress / leak | 3 | 5 | 15 | High | SYS-004 | VER-004, VER-M03 | Low |
| R-PERC | Perception range in pool | 3 | 4 | 12 | High | SYS-007 | VER-007 | Low |
| R-GEOM | Fin geometry asymmetry | 3 | 3 | 9 | Medium | SYS-002 | VER-002, VER-014 | Low |
| R-CTRL | Control instability in water | 3 | 3 | 9 | Medium | SYS-016 | VER-016, VER-002 | Low |
| R-PWR | Endurance shortfall | 3 | 2 | 6 | Medium | SYS-013 | VER-013 | Low |
| R-BUOY | Buoyancy / recovery | 2 | 2 | 4 | Low | SYS-012 | VER-012 | Low |

## Risk matrix  (columns = Likelihood 1->5, rows = Consequence 5->1)

| C \\ L | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|
| **5** | · | · | R-LEAK | · | · |
| **4** | · | · | R-PERC | R-PROP | · |
| **3** | · | · | R-GEOM R-CTRL | · | · |
| **2** | · | R-BUOY | R-PWR | · | · |
| **1** | · | · | · | · | · |

## Mitigations & triggers

### R-PROP — Propulsion ineffective  (High, score 16)
The soft flapping fins do not generate enough thrust to make headway / close on a tag (propulsion is unproven).

- **Mitigation:** Characterise thrust on the bench / in sim; measure closure rate in an early tethered water trial; tune gait (freq/amp/waveform) before committing to full homing runs.
- **Trigger:** Mean closure rate ~0 in the first water trial (MOP-01).
- **Owner:** PI · **Residual target:** Low · **Status:** `open`

### R-LEAK — Water ingress / leak  (High, score 15)
A seal or cable penetration fails; water floods the enclosure and destroys the electronics (loss of vehicle).

- **Mitigation:** Unpowered immersion + dye + interior tissue witness BEFORE any powered run; incremental immersion (unpowered -> powered shallow -> full); pre/post leak inspection every run (SC-0). (No syringe -> immersion+dye in lieu of vacuum.)
- **Trigger:** Any moisture/dye on the interior witness, or fogging.
- **Owner:** PI · **Residual target:** Low · **Status:** `open`

### R-PERC — Perception range in pool  (High, score 12)
The 200 mm tag is not reliably detected at >= 1.5 m in real pool water/lighting, so homing cannot acquire.

- **Mitigation:** Bench/poolside detection-range test before relying on it; control lighting; upsize tag or tune intrinsics if margin is thin.
- **Trigger:** Reliable detection range < 1.5 m in the pool.
- **Owner:** PI · **Residual target:** Low · **Status:** `open`

### R-GEOM — Fin geometry asymmetry  (Medium, score 9)
Left/right fin geometry asymmetry (the known CAD left-fin joint offsets) produces asymmetric thrust / biased homing.

- **Mitigation:** Verify fin symmetry on the physical build; correct joint geometry if the asymmetry is present in hardware; check thrust symmetry in trials.
- **Trigger:** Vehicle consistently veers / cannot hold heading.
- **Owner:** PI · **Residual target:** Low · **Status:** `open`

### R-CTRL — Control instability in water  (Medium, score 9)
Untuned gains / loop timing cause oscillation or divergence once in water.

- **Mitigation:** Incremental gain tuning from gentle gaits up; fault -> stable HOVERING (SYS-016); short tethered trials first.
- **Trigger:** Sustained oscillation or runaway in a trial.
- **Owner:** PI · **Residual target:** Low · **Status:** `open`

### R-PWR — Endurance shortfall  (Medium, score 6)
2200 mAh cannot sustain the system draw for the 30-min endurance target.

- **Mitigation:** Power-budget analysis + measure mean draw (multimeter in series); reduce duty cycle or upsize pack if mean draw > ~4.4 A.
- **Trigger:** Measured run time < 30 min, or mean draw > 4.4 A.
- **Owner:** PI · **Residual target:** Low · **Status:** `open`

### R-BUOY — Buoyancy / recovery  (Low, score 4)
Buoyancy trim is wrong and the vehicle is uncontrollable or hard to recover (assessed Low — ballast adjustable, pool <= 0.75 m).

- **Mitigation:** Adjustable ballast; submerged-weight trim test (VER-012); confirm hand/pole recovery from the floor.
- **Trigger:** Cannot trim to slightly negative, or cannot recover within reach.
- **Owner:** PI · **Residual target:** Low · **Status:** `open`
