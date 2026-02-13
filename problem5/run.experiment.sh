#!/bin/bash

cd ~/plansys2_ws || exit 1

# Setup and build
echo "Setting up environment"
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# Cleanup so that process is stopped when living terminal
cleanup() {
    echo "Stopping launch process..."
    kill "$LAUNCH_PID"
}
trap cleanup EXIT

# Start launch in background and save output in logs
echo "Starting ROS 2 Launch (output saved to launch.log)..."
ros2 launch problem5 problem5_launch.py > launch.log 2>&1 &

# Launch PID
LAUNCH_PID=$!

# Wait plansys2 launch
echo "Waiting 20 seconds for plansys2 to start..."
sleep 20

# Start plansys2 terminal
ros2 run plansys2_terminal plansys2_terminal