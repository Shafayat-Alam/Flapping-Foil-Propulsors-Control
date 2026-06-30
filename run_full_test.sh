#!/bin/bash

# Comprehensive mission sweep — feeds missions on /mission_input with safety delays.
# Launch the stack first: ros2 launch soft_propulsors_control crab_launch.py

# --- Target tag 3 (NORTH) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:3 label:NORTH_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 7 (DOCK) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:7 label:DOCK_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 5 (HOME) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:5 label:HOME_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 9 (URGENT) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:9 label:URGENT_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 2 (DETOUR) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:2 label:DETOUR_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 11 (WEST) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:11 label:WEST_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 4 (EAST) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:4 label:EAST_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:4 label:EAST_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:4 label:EAST_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:4 label:EAST_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:4 label:EAST_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:4 label:EAST_stress retries:0 override:none\"}" --once
sleep 3

# --- Target tag 8 (SOUTH) ---
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:8 label:SOUTH_baseline retries:2 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:8 label:SOUTH_baseline retries:1 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:8 label:SOUTH_sweep retries:3 override:none\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:8 label:SOUTH_preempt retries:2 override:discard\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:8 label:SOUTH_preempt retries:2 override:requeue\"}" --once
sleep 3
ros2 topic pub -1 /mission_input std_msgs/msg/String "{data: \"tag:8 label:SOUTH_stress retries:0 override:none\"}" --once
sleep 3

