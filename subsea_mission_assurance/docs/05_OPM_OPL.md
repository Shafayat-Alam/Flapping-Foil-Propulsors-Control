# OPM Model — Object-Process Language (OPL) + Diagram
_Generated 2026-09-03 from model/opm.yaml (Object-Process Methodology, ISO 19450)._

## OPL — the model as sentences

- **Vehicle** is physical and systemic.
- **Vehicle** can be Stowed, Searching, Homing, Arrived, Stuck, or AtHome.
- **Mission** is informatical and systemic.
- **Mission** can be Queued, Active, Achieved, or Failed.
- **Tag** is physical and environmental.
- **Detection** is informatical and systemic.
- **HomePosition** is physical and environmental.
- **Operator** is physical and environmental.
- **Dispatcher** is informatical and systemic.
- **Controller** is informatical and systemic.
- **Perception** is informatical and systemic.
- **Propulsor** is physical and systemic.

**Dispatching** is a process.
- Operator handles Dispatching.
- Dispatching requires Dispatcher.
- Dispatching yields Active Mission.

**Searching** is a process.
- Searching requires Controller.
- Searching requires Perception.
- Searching changes Vehicle from Stowed to Searching.

**Detecting** is a process.
- Detecting requires Perception.
- Detecting requires Tag.
- Detecting yields Detection.

**Homing** is a process.
- Homing requires Controller.
- Homing requires Propulsor.
- Homing requires Detection.
- Homing changes Vehicle from Searching to Arrived.
- Homing changes Mission from Active to Achieved.

**Returning** is a process.
- Returning requires Controller.
- Returning requires Propulsor.
- Returning requires Detection.
- Returning requires HomePosition.
- Returning changes Vehicle from Arrived to AtHome.

**Escalating** is a process.
- Operator handles Escalating.
- Escalating changes Vehicle from Stuck to Searching.

**Recovering** is a process.
- Operator handles Recovering.
- Recovering changes Vehicle from AtHome to Stowed.

## OPD — Object-Process Diagram

```mermaid
flowchart TB
  O-VEH["Vehicle<br>[Stowed | Searching | Homing | Arrived | Stuck | AtHome]"]:::obj
  O-MISSION["Mission<br>[Queued | Active | Achieved | Failed]"]:::obj
  O-TAG["Tag"]:::obj
  O-DETECTION["Detection"]:::obj
  O-HOME["HomePosition"]:::obj
  O-OPERATOR["Operator"]:::obj
  O-DISPATCHER["Dispatcher"]:::obj
  O-CONTROLLER["Controller"]:::obj
  O-PERCEPTION["Perception"]:::obj
  O-PROPULSOR["Propulsor"]:::obj
  P-DISPATCH(["Dispatching"]):::proc
  P-SEARCH(["Searching"]):::proc
  P-DETECT(["Detecting"]):::proc
  P-HOME(["Homing"]):::proc
  P-RETURN(["Returning"]):::proc
  P-ESCALATE(["Escalating"]):::proc
  P-RECOVER(["Recovering"]):::proc
  O-OPERATOR -- agent --> P-DISPATCH
  O-DISPATCHER -- instrument --> P-DISPATCH
  P-DISPATCH -- result --> O-MISSION
  O-CONTROLLER -- instrument --> P-SEARCH
  O-PERCEPTION -- instrument --> P-SEARCH
  P-SEARCH -- effect&rarr;Searching --> O-VEH
  O-PERCEPTION -- instrument --> P-DETECT
  O-TAG -- instrument --> P-DETECT
  P-DETECT -- result --> O-DETECTION
  O-CONTROLLER -- instrument --> P-HOME
  O-PROPULSOR -- instrument --> P-HOME
  O-DETECTION -- instrument --> P-HOME
  P-HOME -- effect&rarr;Arrived --> O-VEH
  P-HOME -- effect&rarr;Achieved --> O-MISSION
  O-CONTROLLER -- instrument --> P-RETURN
  O-PROPULSOR -- instrument --> P-RETURN
  O-DETECTION -- instrument --> P-RETURN
  O-HOME -- instrument --> P-RETURN
  P-RETURN -- effect&rarr;AtHome --> O-VEH
  O-OPERATOR -- agent --> P-ESCALATE
  P-ESCALATE -- effect&rarr;Searching --> O-VEH
  O-OPERATOR -- agent --> P-RECOVER
  P-RECOVER -- effect&rarr;Stowed --> O-VEH
classDef obj fill:#dbeafe,stroke:#1e3a8a,color:#000;
classDef proc fill:#dcfce7,stroke:#166534,color:#000;
```