#!/bin/bash
scp -P 9999 -o StrictHostKeyChecking=no -r root@192.168.0.157:~/ros2_ws/src/zed-ros2-wrapper /tmp/zed-ros2-wrapper
echo "Downloaded zed-ros2-wrapper to /tmp/zed-ros2-wrapper"
