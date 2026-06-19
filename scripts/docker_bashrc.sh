#!/bin/bash
# This file runs every time you open a terminal in the Docker container.
# It persists across Docker restarts because it lives in the ai_worker repository.

# Source the default Ubuntu/Docker bashrc first to get terminal colors and PS1 prompt!
if [ -f ~/.bashrc ]; then
    source ~/.bashrc
fi

source /opt/ros/jazzy/setup.bash 2>/dev/null
source /root/ros2_ws/install/setup.bash 2>/dev/null

# Useful ROS 2 aliases
alias b="colcon build"
alias s="source install/setup.bash"

# CRITICAL FIX for Jetson Headless EGL Crash:
# The NVIDIA Docker runtime natively bind-mounts broken NVIDIA GStreamer plugins
# into the container. We can't delete them (Resource busy), and they crash GStreamer
# because they try to initialize EGL in a headless environment.
# Since the container runs as privileged, we can shadow them with /dev/null!
for f in /usr/lib/aarch64-linux-gnu/gstreamer-1.0/libgstnv*.so; do
    if [ -s "$f" ]; then
        mount --bind /dev/null "$f" 2>/dev/null
    fi
done
rm -rf ~/.cache/gstreamer-1.0/

echo "--------------------------------------------------------"
echo " Persistent Docker .bashrc loaded!"
echo " ROS 2 workspaces sourced."
echo "--------------------------------------------------------"
