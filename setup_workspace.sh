#!/bin/bash
# ROS 2 Workspace Setup Script for Humanoid Robotics Project
# This script will create a new ROS 2 workspace for the humanoid robotics project

echo "Setting up ROS 2 workspace for humanoid robotics project..."

# Create the workspace directory
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace with colcon
colcon build

# Source the workspace
source ~/humanoid_ws/install/setup.bash

echo "ROS 2 workspace setup completed!"
echo "Your workspace is located at ~/humanoid_ws"
echo "Don't forget to source the workspace with 'source ~/humanoid_ws/install/setup.bash' in new terminals"