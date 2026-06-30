#!/usr/bin/env python3
"""
generate_test_scripts.py - Mission Test Script Generator
========================================================
Generates shell scripts that exercise the autonomous mission interface by
feeding missions onto /mission_input (std_msgs/String).  The robot is
closed-loop now — you no longer command gaits directly; you queue missions and
the controller decides how to execute them.  These scripts drive the dispatcher
through its main behaviours: single missions, queued sequences, overrides, and
retry/stuck escalation.

Two scripts are produced, mirroring the previous position/velocity split — they
are identical mission sequences meant to be run under each operating_mode
(set 'operating_mode' on the crab node in the launch file before running):

    test_position_mode.sh   (run with operating_mode: 'position')
    test_velocity_mode.sh    (run with operating_mode: 'velocity')

Usage:
    python3 generate_test_scripts.py
"""

import os

TOPIC = "/mission_input"


def _pub(label_comment, line, delay=3):
    """Emit an echo + a single mission publish + a sleep."""
    return (
        f'echo "{label_comment}"\n'
        f'ros2 topic pub --once {TOPIC} std_msgs/msg/String "{{data: \'{line}\'}}"\n'
        f'sleep {delay}\n\n'
    )


def build_script(mode_name):
    """Build a mission test sequence for the given operating mode label."""
    s = f"""#!/bin/bash
# test_{mode_name}_mode.sh - Mission Sequence Test ({mode_name} mode)
# Usage: ./test_{mode_name}_mode.sh
# Prerequisite: launch the stack with operating_mode: '{mode_name}'.
#   ros2 launch soft_propulsors_control crab_launch.py
# Missions are fed on {TOPIC}; the controller executes them autonomously.

DELAY=3

echo "========================================="
echo "Starting Mission Test Sequence ({mode_name} mode)"
echo "========================================="
echo ""

"""

    # --- Single mission -----------------------------------------------------
    s += "# =========================================================================\n"
    s += "# BASIC - single mission to a tag\n"
    s += "# =========================================================================\n\n"
    s += _pub("Mission 1: seek tag 3 (NORTH)", "tag:3 label:NORTH retries:2 override:none")

    # --- Queued sequence ----------------------------------------------------
    s += "# =========================================================================\n"
    s += "# QUEUE - several missions buffered back-to-back (FIFO)\n"
    s += "# =========================================================================\n\n"
    s += _pub("Mission 2: queue tag 7 (DOCK)", "tag:7 label:DOCK retries:2 override:none")
    s += _pub("Mission 3: queue tag 5 (HOME)", "tag:5 label:HOME retries:1 override:none")

    # --- Override: discard --------------------------------------------------
    s += "# =========================================================================\n"
    s += "# OVERRIDE discard - preempt the running mission and drop it\n"
    s += "# =========================================================================\n\n"
    s += _pub("Mission 4: discard-override to tag 9 (URGENT)",
              "tag:9 label:URGENT retries:2 override:discard")

    # --- Override: requeue --------------------------------------------------
    s += "# =========================================================================\n"
    s += "# OVERRIDE requeue - preempt but save the interrupted mission to the front\n"
    s += "# =========================================================================\n\n"
    s += _pub("Mission 5: requeue-override to tag 2 (DETOUR)",
              "tag:2 label:DETOUR retries:2 override:requeue")

    # --- Retry / stuck escalation -------------------------------------------
    s += "# =========================================================================\n"
    s += "# RETRY - a likely-unreachable tag to exercise STUCK -> retry -> human\n"
    s += "# (answer y/N on the crab terminal within 10 s when prompted)\n"
    s += "# =========================================================================\n\n"
    s += _pub("Mission 6: hard tag 99 with 2 retries", "tag:99 label:HARD retries:2 override:none")

    # --- Drain to hover -----------------------------------------------------
    s += "# =========================================================================\n"
    s += "# DRAIN - no more missions; the robot should fall back to HOVER\n"
    s += "# =========================================================================\n\n"
    s += 'echo "Queue will drain — controller should report HOVERING."\n'
    s += "sleep $DELAY\n\n"

    s += 'echo ""\n'
    s += 'echo "========================================="\n'
    s += f'echo "Mission Test Sequence Complete ({mode_name} mode)"\n'
    s += 'echo "========================================="\n'
    return s


def write_script(mode_name):
    filename = f"test_{mode_name}_mode.sh"
    with open(filename, 'w') as f:
        f.write(build_script(mode_name))
    os.chmod(filename, 0o755)
    print(f"✓ Generated {filename}")


if __name__ == '__main__':
    print("Generating mission test scripts...")
    write_script('position')
    write_script('velocity')
    print("\nDone! Test scripts generated and made executable.")
    print("\nUsage:")
    print("  ./test_position_mode.sh  # mission sequence under position mode")
    print("  ./test_velocity_mode.sh  # mission sequence under velocity mode")
