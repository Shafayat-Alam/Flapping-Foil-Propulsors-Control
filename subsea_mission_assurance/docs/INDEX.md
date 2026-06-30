# Soft Propulsors UUV — Subsea Mission Assurance (Master Index)
_Generated 2026-06-07. **Do not edit docs/** — edit `model/*.yaml`, then run `python3 tools/check.py && python3 tools/generate.py`._

Model-based systems engineering (as code). Source of truth: `model/`. Health: **0 errors, 0 warnings**.

| Area | Document | What it is |
|---|---|---|
| 0. Start here | [00_Model_Report.md](00_Model_Report.md) | Element counts, V&V policy, live consistency check |
|  | [07_ConOps.md](07_ConOps.md) | Concept of Operations — environment, phases (PH-0..PH-6), scenarios |
|  | [15_Commissioning.md](15_Commissioning.md) | Operational Readiness Review — the gate from integration to operations |
| 1. Concept & needs | [01_Stakeholder_Expectations.md](01_Stakeholder_Expectations.md) | Stakeholders and their expectations |
|  | [02_MOE_MOP.md](02_MOE_MOP.md) | Measures of Effectiveness / Performance (targets + measurement) |
| 2. Requirements | [03_System_Requirements.md](03_System_Requirements.md) | All requirements by domain |
|  | [Requirements_Traceability_Matrix.csv](Requirements_Traceability_Matrix.csv) | Req -> parent -> allocation -> verification -> status |
| 3. Architecture & model | [04_Architecture.md](04_Architecture.md) | Components + interfaces (Mermaid) |
|  | [05_OPM_OPL.md](05_OPM_OPL.md) | Object-Process model — OPL sentences + OPD diagram |
|  | [Interfaces.csv](Interfaces.csv) | Interface register |
| 4. Verification & Validation | [16_Test_Strategy.md](16_Test_Strategy.md) | MIL→SIL→HIL→water ladder, automation coverage, kinematic-sim limits |
|  | [06_Verification_Procedures.md](06_Verification_Procedures.md) | Checkable procedures (venue/automation/script tagged) |
|  | [Verification_Matrix.csv](Verification_Matrix.csv) | Activity -> requirement -> venue -> automation -> tool -> pass |
|  | [Scenario_Traceability_Matrix.csv](Scenario_Traceability_Matrix.csv) | Scenario -> requirements -> MOPs -> verification |
| 5. Safety & risk | [08_Software_Safety.md](08_Software_Safety.md) | NASA-GB-8719.13 analysis (6 software hazards) |
|  | [Software_Hazard_Matrix.csv](Software_Hazard_Matrix.csv) | SWH cause -> hazard -> control -> verification |
|  | [09_Risk_Register.md](09_Risk_Register.md) | Technical risks (5x5) + matrix + mitigations |
|  | [Risk_Register.csv](Risk_Register.csv) | Risk register |
| 6. Interfaces & integration | [10_Interface_Requirements.md](10_Interface_Requirements.md) | IRD — interface SHALLs (timing/protocol/data/ordering/error) |
|  | [11_Interface_Design.md](11_Interface_Design.md) | IDD — per-interface design (encoding, rate, QoS, error handling) |
|  | [12_ICD.md](12_ICD.md) | ICD — controlled baseline merging IRD + IDD per interface |
|  | [Interface_Requirements.csv](Interface_Requirements.csv) | IRD register |
|  | [ICD.csv](ICD.csv) | ICD baseline register |
|  | [13_Integration_Plan.md](13_Integration_Plan.md) | Incremental dry→wet integration stages + gates |
|  | [14_DSM.md](14_DSM.md) | Design Structure Matrix + coupled clusters + derived order |
|  | [Integration_Plan.csv](Integration_Plan.csv) | Integration stage register |
|  | [DSM.csv](DSM.csv) | Component dependency matrix |

## Recommended reading order

ConOps → Stakeholder Expectations → MOE/MOP → Requirements (+RTM) → Architecture → OPM → Verification procedures (+matrices, scenarios) → Software Safety → Risk Register → Model Report.

## Regenerate
```bash
python3 tools/check.py      # consistency gate
python3 tools/generate.py   # rebuild every view
```