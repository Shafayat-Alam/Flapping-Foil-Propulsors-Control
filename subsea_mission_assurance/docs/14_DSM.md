# Design Structure Matrix (DSM)
_Generated 2026-09-03 — computed from the interface coupling in architecture.yaml._

Cell **X** at row _R_, column _C_ means **R receives from / depends on C** (an interface flows C→R). Marks below the diagonal = forward flow; marks above = feedback.

## Component coupling matrix

| ⬇R \\ C➡ | APRIL | CAM | CAMIF | COMPUTE | CRAB | CTRL | DXLIF | IMU | IMUIF | OPC | SERVO |
|---|---|---|---|---|---|---|---|---|---|---|---|
| **APRIL** | — | · | X | · | · | · | · | · | · | · | · |
| **CAM** | · | — | · | · | · | · | · | · | · | · | · |
| **CAMIF** | · | · | — | · | · | · | · | · | · | · | · |
| **COMPUTE** | · | X | · | — | · | · | · | · | · | · | · |
| **CRAB** | · | · | · | · | — | X | · | · | · | X | · |
| **CTRL** | X | · | · | · | X | — | X | · | X | · | · |
| **DXLIF** | · | · | · | · | · | X | — | · | · | · | · |
| **IMU** | · | · | · | X | · | · | · | — | · | · | · |
| **IMUIF** | · | · | · | · | · | · | · | · | — | · | · |
| **OPC** | · | · | · | · | · | X | · | · | · | — | · |
| **SERVO** | · | · | · | X | · | · | · | · | · | · | — |

## Coupled clusters (feedback loops)

These components form feedback loops and **must be integrated as a block** (piecewise bring-up is impossible — each needs the others):

- { C-CRAB (crab (mission dispatcher)), C-CTRL (controller (execution engine)), C-DXLIF (dynamixel interface), C-OPC (Operator console) }

## DSM-derived integration order

Topological order of the clusters (providers before consumers):

1. CAM
2. COMPUTE
3. IMU
4. SERVO
5. CAMIF
6. APRIL
7. IMUIF
8. CRAB, CTRL, DXLIF, OPC  ← coupled block

## Cross-check vs the authored stages

The control core (controller ↔ dynamixel ↔ crab ↔ console) is the coupled block — the hand-authored plan brings its sensor providers up first in **INT-0**, then closes that block in **INT-1** (dry) before any water. The DSM order and the staged plan agree.