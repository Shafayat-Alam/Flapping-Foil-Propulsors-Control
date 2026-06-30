#!/usr/bin/env python3
"""
generate_dynamic_test_suite.py - Rapid Mission Hot-Swap Generator
=================================================================
Emits run_dynamic_test.sh: rapid, back-to-back mission changes with no settling
in between, stressing the dispatcher's queueing and override (preemption) paths.
Same soul as before (a fast hot-swap stress script), now mission-based.

  PART 1: a quick FIFO burst — many missions queued in rapid succession.
  PART 2: override storms — discard/requeue preemptions fired mid-mission.
"""

import os

tags = [3, 7, 5, 9, 2, 11]
labels = ["NORTH", "DOCK", "HOME", "URGENT", "DETOUR", "WEST"]

TOPIC = "/mission_input"
output_file = "run_dynamic_test.sh"


def pub(line, delay):
    return (f'ros2 topic pub -1 {TOPIC} std_msgs/msg/String '
            f'"{{data: \\"{line}\\"}}" --once\nsleep {delay}\n')


def generate_sh_script():
    with open(output_file, "w") as f:
        f.write("#!/bin/bash\n\n")
        f.write("# --- Dynamic Testing: Rapid Mission Hot-Swaps & Overrides ---\n")
        f.write("# Launch the stack first: ros2 launch soft_propulsors_control crab_launch.py\n\n")

        # --- PART 1: rapid FIFO burst -----------------------------------
        f.write("# PART 1: Rapid FIFO burst (missions queued ~1 s apart)\n")
        for tag, label in zip(tags, labels):
            f.write(pub(f"tag:{tag} label:{label} retries:1 override:none", 1))
        f.write("\n")

        # --- PART 2: override storms ------------------------------------
        f.write("# PART 2: Override storm (discard / requeue mid-mission)\n")
        for repeat in range(3):
            f.write(f"# Override set {repeat + 1}\n")
            f.write(pub(f"tag:{tags[0]} label:BASE_{repeat} retries:2 override:none", 2))
            f.write(pub(f"tag:{tags[1]} label:CUT_{repeat} retries:1 override:discard", 1))
            f.write(pub(f"tag:{tags[2]} label:SAVE_{repeat} retries:1 override:requeue", 2))
            f.write("\n")

    os.chmod(output_file, 0o755)
    print(f"Generated {output_file} (rapid mission hot-swaps + overrides).")


if __name__ == "__main__":
    generate_sh_script()
