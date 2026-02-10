#!/bin/bash

# 1. Go to the workspace
cd ~/plansys2_ws || exit 1

# 2. Setup and Build
# We do this first so the environment is ready before launching anything
echo "Setting up and building..."
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash

# 3. Define a cleanup function
# This ensures that when you quit the terminal later, 
# the background launch process is also killed.
cleanup() {
    echo "Stopping launch process..."
    kill "$LAUNCH_PID"
}
# Trap the exit signal (CTRL+C) to run the cleanup function
trap cleanup EXIT

# 4. Start the Launch file in the BACKGROUND
# We use '&' to put it in the background.
# We redirect output to a file (> launch.log) so it doesn't mess up your terminal UI.
echo "Starting ROS 2 Launch (output saved to launch.log)..."
ros2 launch problem5 problem5_launch.py > launch.log 2>&1 &

# Capture the Process ID (PID) of the launch so we can kill it later
LAUNCH_PID=$!

# 5. Wait for initialization
echo "Waiting 20 seconds for plansys2 to start..."
sleep 20

# 6. Start the PlanSys2 Terminal in the FOREGROUND
# This is where you will type your commands.
ros2 run plansys2_terminal plansys2_terminal