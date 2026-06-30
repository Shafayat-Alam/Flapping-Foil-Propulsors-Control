# Subsea Mission Assurance — Model-Based Systems Engineering (as code)

**Program:** Soft Propulsors UUV · **Intent:** research/bench prototype (tailored)
**Framework:** NASA SE Handbook (SP-6105) · ISO 29148 · IEEE 1012 · NASA-GB-8719.13

This package is **model-based SE without external tools**. There is **one source
of truth — the model in `model/*.yaml`** — and every document, matrix, and
diagram in `docs/` is a **view generated from it**. You never hand-edit `docs/`.

```
        model/*.yaml   ──►  tools/check.py    (consistency: traces, coverage)
   (single source of truth) ──►  tools/generate.py ──►  docs/  (md + csv + mermaid)
```

## Workflow

```bash
# 1. Edit the model (the only thing you author by hand)
$EDITOR model/requirements.yaml      # or program / stakeholders / architecture / verification

# 2. Check it (fails on broken traces, missing verification, bad allocations)
python3 tools/check.py

# 3. Regenerate all views
python3 tools/generate.py            # writes docs/, embeds the check result
```

## The model (`model/`)

| File | Holds | NASA SE process |
|---|---|---|
| `program.yaml` | program intent, SoI boundary, external actors, MOEs/MOPs | Stakeholder Expectations / ConOps apex |
| `stakeholders.yaml` | stakeholders + their expectations | Stakeholder Expectations Definition |
| `requirements.yaml` | system requirements (ISO 29148 "shall") | Technical Requirements Definition |
| `architecture.yaml` | components + interfaces (factual, from the ROS 2 stack) | Logical Decomposition / Design Solution |
| `verification.yaml` | V&V activities (IEEE 1012) | Product Verification & Validation |

## The views (`docs/`, regenerated)

`00_Model_Report.md` (counts + live check) · `01_Stakeholder_Expectations.md` ·
`02_MOE_MOP.md` · `03_System_Requirements.md` · `04_Architecture.md` (Mermaid
diagrams) · `Requirements_Traceability_Matrix.csv` · `Verification_Matrix.csv` ·
`Interfaces.csv`.

## What `check.py` enforces (so the model can't drift)

- every requirement traces to a real expectation / MOE / MOP / parent requirement;
- every requirement has a valid verification method and is allocated to a real component;
- every interface has a real producer and consumer; every MOP maps to a real MOE;
- every verification activity targets a real requirement;
- **warns** on coverage gaps (a requirement with no V&V, an expectation with no derived requirement).

## Status

Built iteratively via a **Learn → Interview → Generate** loop. Items marked
`status: draft` / `target: TBD` are placeholders awaiting the next interview
round. The model currently passes: **0 errors, 0 warnings**.
