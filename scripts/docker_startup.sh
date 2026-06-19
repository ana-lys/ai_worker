#!/bin/bash
# This file runs exactly ONCE when the Docker container is started.
# It persists across Docker restarts because it lives in the ai_worker repository.

echo "Running persistent Docker startup script..."

# You can add background tasks, hardware initializations, or ROS 2 launches here.
