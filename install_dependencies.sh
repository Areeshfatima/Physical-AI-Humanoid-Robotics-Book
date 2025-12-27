#!/bin/bash
# Dependencies Installation Script for Humanoid Robot Simulation
# This script installs required dependencies for humanoid robot simulation

echo "Installing dependencies for humanoid robot simulation..."

# Update package list
sudo apt update

# Install Gazebo Garden (most recent version compatible with ROS 2 Humble)
# Note: Gazebo has been renamed to Ignition Gazebo, and the latest version is called Garden
sudo apt install ignition-garden -y

# Install ROS 2 Gazebo plugins
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers -y

# Install visualization tools
sudo apt install ros-humble-rviz2 ros-humble-xacro ros-humble-joint-state-publisher ros-humble-robot-state-publisher -y

# Install Python dependencies
pip3 install transforms3d pygame numpy

# Install Unity Robotics Simulation Package dependencies (if needed)
# For now, we'll note what's needed in comments
echo "# To use Unity with ROS 2, download Unity Hub and install the latest LTS version of Unity"
echo "# Then install the Unity Robotics Simulation Package from the Unity Asset Store"
echo "# Also install the ROS TCP Connector package"

echo "Dependencies installation completed!"
echo "Note: For Unity simulation, additional manual installation of Unity Hub and relevant packages is required."