# Measures of Effectiveness & Performance
_Generated 2026-06-07._

## MOE-01 — Autonomous tag homing (PRIMARY)

The vehicle shall autonomously detect a commanded AprilTag and reduce its range and bearing error to that tag without operator piloting.


| MOP | Definition | Target | Units | Measurement |
|---|---|---|---|---|
| MOP-01 | Mean closure rate toward the target tag during HEADING. | > 0 (closure demonstrated; mean rate recorded) | m/s | Onboard apriltag_detections range time series in the known/measured pool. |
| MOP-02 | Fraction of runs where the commanded tag is detected, locked, and approached. | >= 0.90 (>= 27 of 30 runs) | fraction of runs | 30 demonstration runs; count successful acquisition+approach. |
| MOP-03 | Distance to tag at mission-achieved declaration. | <= 0.30 | m | Onboard apriltag range at ACHIEVED, cross-checked against known pool geometry. |
