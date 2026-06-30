#!/usr/bin/env bash
#
# feed_missions.sh — stream missions into the crab dispatcher
# ===========================================================
# Publishes one mission line at a time onto /mission_input (std_msgs/String),
# pacing them so the dispatcher never gets a burst it can't buffer.  The crab
# queue itself is unbounded — pacing is just good manners, not a hard limit.
#
# Usage:
#   ./feed_missions.sh [missions_file] [pace_seconds]
#
#   missions_file : file with one mission per line (default: built-in examples).
#                   Blank lines and lines starting with '#' are ignored.
#   pace_seconds  : delay between missions (default: 3).
#
# Mission line format (see crab.py):
#   tag:<id> label:<name> retries:<n> override:<none|discard|requeue>
#
# Example missions file:
#   tag:3 label:NORTH retries:2
#   tag:7 label:DOCK  retries:2
#   tag:5 label:HOME  retries:1 override:requeue
#
# Run AFTER the stack is launched (ros2 launch ... crab_launch.py).

set -euo pipefail

MISSIONS_FILE="${1:-}"
PACE="${2:-3}"
TOPIC="/mission_input"

publish() {
  local line="$1"
  echo "[feed_missions] -> ${line}"
  ros2 topic pub --once "${TOPIC}" std_msgs/msg/String "{data: '${line}'}" >/dev/null
}

if [[ -n "${MISSIONS_FILE}" ]]; then
  [[ -f "${MISSIONS_FILE}" ]] || { echo "No such file: ${MISSIONS_FILE}" >&2; exit 1; }
  mapfile -t LINES < <(grep -vE '^\s*(#|$)' "${MISSIONS_FILE}")
else
  # Built-in demo sequence
  LINES=(
    "tag:3 label:NORTH retries:2 override:none"
    "tag:7 label:DOCK retries:2 override:none"
    "tag:5 label:HOME retries:1 override:none"
  )
fi

echo "[feed_missions] streaming ${#LINES[@]} missions to ${TOPIC} every ${PACE}s"
for line in "${LINES[@]}"; do
  publish "${line}"
  sleep "${PACE}"
done
echo "[feed_missions] done — queue drained will trigger HOVER."
