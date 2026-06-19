#!/bin/bash
# This file runs every time you open a terminal in the Docker container.
# It persists across Docker restarts because it lives in the ai_worker repository.

source /opt/ros/jazzy/setup.bash 2>/dev/null
source /root/ros2_ws/install/setup.bash 2>/dev/null

# Useful ROS 2 aliases
alias b="colcon build"
alias s="source install/setup.bash"

# CRITICAL FIX for GStreamer Hangs:
# Clear the corrupted GStreamer registry cache every time you log in
rm -rf ~/.cache/gstreamer-1.0/

echo "--------------------------------------------------------"
echo " Persistent Docker .bashrc loaded!"
echo " ROS 2 workspaces sourced. GStreamer registry cleared."
echo "--------------------------------------------------------"
