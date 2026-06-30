#!/bin/bash

# --- Dynamic Testing: Rapid Mission Hot-Swaps & Overrides ---
# Launch the stack first: ros2 launch soft_propulsors_control crab_launch.py

# PART 1: Rapid FIFO burst (missions queued ~1 s apart)
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH retries:1 override:none\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK retries:1 override:none\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME retries:1 override:none\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT retries:1 override:none\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR retries:1 override:none\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST retries:1 override:none\"}" --once
sleep 1

# PART 2: Override storm (discard / requeue mid-mission)
# Override set 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:BASE_0 retries:2 override:none\"}" --once
sleep 2
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:CUT_0 retries:1 override:discard\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:SAVE_0 retries:1 override:requeue\"}" --once
sleep 2

# Override set 2
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:BASE_1 retries:2 override:none\"}" --once
sleep 2
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:CUT_1 retries:1 override:discard\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:SAVE_1 retries:1 override:requeue\"}" --once
sleep 2

# Override set 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:BASE_2 retries:2 override:none\"}" --once
sleep 2
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:CUT_2 retries:1 override:discard\"}" --once
sleep 1
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:SAVE_2 retries:1 override:requeue\"}" --once
sleep 2

