#!/bin/bash
# test_position_mode.sh - Mission Sequence Test (position mode)
# Usage: ./test_position_mode.sh
# Prerequisite: launch the stack with operating_mode: 'position'.
#   ros2 launch soft_propulsors_control crab_launch.py
# Missions are fed on /mission_input; the controller executes them autonomously.

DELAY=3

echo "========================================="
echo "Starting Mission Test Sequence (position mode)"
echo "========================================="
echo ""

# =========================================================================
# BASIC - single mission to a tag
# =========================================================================

echo "Mission 1: seek tag 3 (NORTH)"
ros2 topic pub --once /mission_input std_msgs/msg/String "{data: 'tag:3 label:NORTH retries:2 override:none'}"
sleep 3

# =========================================================================
# QUEUE - several missions buffered back-to-back (FIFO)
# =========================================================================

echo "Mission 2: queue tag 7 (DOCK)"
ros2 topic pub --once /mission_input std_msgs/msg/String "{data: 'tag:7 label:DOCK retries:2 override:none'}"
sleep 3

echo "Mission 3: queue tag 5 (HOME)"
ros2 topic pub --once /mission_input std_msgs/msg/String "{data: 'tag:5 label:HOME retries:1 override:none'}"
sleep 3

# =========================================================================
# OVERRIDE discard - preempt the running mission and drop it
# =========================================================================

echo "Mission 4: discard-override to tag 9 (URGENT)"
ros2 topic pub --once /mission_input std_msgs/msg/String "{data: 'tag:9 label:URGENT retries:2 override:discard'}"
sleep 3

# =========================================================================
# OVERRIDE requeue - preempt but save the interrupted mission to the front
# =========================================================================

echo "Mission 5: requeue-override to tag 2 (DETOUR)"
ros2 topic pub --once /mission_input std_msgs/msg/String "{data: 'tag:2 label:DETOUR retries:2 override:requeue'}"
sleep 3

# =========================================================================
# RETRY - a likely-unreachable tag to exercise STUCK -> retry -> human
# (answer y/N on the crab terminal within 10 s when prompted)
# =========================================================================

echo "Mission 6: hard tag 99 with 2 retries"
ros2 topic pub --once /mission_input std_msgs/msg/String "{data: 'tag:99 label:HARD retries:2 override:none'}"
sleep 3

# =========================================================================
# DRAIN - no more missions; the robot should fall back to HOVER
# =========================================================================

echo "Queue will drain — controller should report HOVERING."
sleep $DELAY

echo ""
echo "========================================="
echo "Mission Test Sequence Complete (position mode)"
echo "========================================="
