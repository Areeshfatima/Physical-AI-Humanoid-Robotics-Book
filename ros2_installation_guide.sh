#!/bin/bash
# ROS 2 Humble Hawksbill Installation Script for Ubuntu 22.04
# This script outlines the installation steps for ROS 2 Humble Hawksbill

echo "Starting ROS 2 Humble Hawksbill installation..."

# Step 1: Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US.UTF-8
sudo update-locale LANG=en_US.UTF-8

# Step 2: Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Step 3: Add the ROS 2 GPG key
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Step 4: Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Step 5: Install ROS 2 packages
sudo apt update
sudo apt install ros-humble-desktop

# Step 6: Install colcon build tools
sudo apt install python3-colcon-common-extensions

# Step 7: Install rosdep
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update

# Step 8: Source ROS 2 environment
# This command can be added to your ~/.bashrc to source automatically
source /opt/ros/humble/setup.bash

echo "ROS 2 Humble Hawksbill installation completed!"
echo "IMPORTANT: Run 'source /opt/ros/humble/setup.bash' or add it to your ~/.bashrc file to use ROS 2 in new terminals."