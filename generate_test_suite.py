#!/usr/bin/env python3
"""
generate_test_suite.py - Comprehensive Mission Sweep Generator
==============================================================
Emits run_full_test.sh: a broad sweep over mission parameters to exercise the
dispatcher and controller across many scenarios.  The "soul" is unchanged — a
generated shell script that walks a parameter grid with safety delays between
steps — but it now feeds missions on /mission_input instead of gait commands.

Each scenario varies the target tag, retry budget, and override mode so the run
covers queueing, preemption, and retry/stuck escalation.
"""

# Target tags to cycle through (stand in for environment AprilTags)
tags = [3, 7, 5, 9, 2, 11, 4, 8]
labels = ["NORTH", "DOCK", "HOME", "URGENT", "DETOUR", "WEST", "EAST", "SOUTH"]

# (retries, override, category) sweep applied across tags
scenarios = [
    (2, "none", "baseline"),
    (1, "none", "baseline"),
    (3, "none", "sweep"),
    (2, "discard", "preempt"),
    (2, "requeue", "preempt"),
    (0, "none", "stress"),     # 0 retries -> immediate human escalation on stuck
]

TOPIC = "/mission_input"

with open("run_full_test.sh", "w") as f:
    f.write("#!/bin/bash\n\n")
    f.write("# Comprehensive mission sweep — feeds missions on "
            f"{TOPIC} with safety delays.\n")
    f.write("# Launch the stack first: ros2 launch soft_propulsors_control crab_launch.py\n\n")

    for tag, label in zip(tags, labels):
        f.write(f"# --- Target tag {tag} ({label}) ---\n")
        for retries, override, category in scenarios:
            line = f"tag:{tag} label:{label}_{category} retries:{retries} override:{override}"
            f.write(f'ros2 topic pub -1 {TOPIC} std_msgs/msg/String '
                    f'"{{data: \\"{line}\\"}}" --once\n')
            f.write("sleep 3\n")   # pacing between missions
        f.write("\n")

print("Generated run_full_test.sh (mission sweep with safety delays).")
