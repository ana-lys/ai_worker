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
# into the container. We can't delete them, so we create a virtual plugin directory
# that explicitly excludes them and point GStreamer there instead!
export SAFE_GST_DIR="/tmp/safe_gst_plugins"
if [ ! -d "$SAFE_GST_DIR" ]; then
    mkdir -p "$SAFE_GST_DIR"
    find /usr/lib/aarch64-linux-gnu/gstreamer-1.0/ -type f ! -name "libgstnv*" -exec ln -s {} "$SAFE_GST_DIR"/ \;
fi
export GST_PLUGIN_SYSTEM_PATH="$SAFE_GST_DIR"
export GST_PLUGIN_PATH=""

echo "--------------------------------------------------------"
echo " Persistent Docker .bashrc loaded!"
echo " ROS 2 workspaces sourced. GStreamer NVIDIA plugins bypassed!"
echo "--------------------------------------------------------"
