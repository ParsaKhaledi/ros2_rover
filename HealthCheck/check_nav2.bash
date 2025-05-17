#!/bin/bash
# nav2_healthcheck.sh
#
# This script subscribes to the /diagnostics topic,
# extracts one diagnostic message, and checks whether
# the diagnostic message related to Nav2 indicates that
# "Nav2 is inactive". If so, it returns unhealthy (exit 1);
# if it indicates "Nav2 is active", then it returns healthy (exit 0).
#
# Usage: ./nav2_healthcheck.sh
# (Intended for use as a Docker healthcheck command.)

# Source the ROS 2 environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Subscribe to the /diagnostics topic and capture one message
output=$(ros2 topic echo /diagnostics -n 1 2>/dev/null)

# Optional: Print the output for debugging (can be removed in production)
echo "Received diagnostic message:"
echo "$output"

# Check for the desired diagnostic block.
# We assume that the diagnostic message for Nav2 comes with a name:
# 'lifecycle_manager_navigation: Nav2 Health'
# Then we look for the 'message:' field in that section.
#
# In this simple example, we simply search the entire output.

if echo "$output" | grep -q "Nav2 is inactive"; then
  echo "Nav2 diagnostic indicates inactive. Unhealthy."
  exit 1
elif echo "$output" | grep -q "Nav2 is active"; then
  echo "Nav2 diagnostic indicates active. Healthy."
  exit 0
else
  echo "No definitive Nav2 diagnostic found; defaulting to unhealthy."
  exit 1
fi
