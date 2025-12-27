# Walking Locomotion Testing Guide

This document explains how to test the implemented walking locomotion system for the humanoid robot.

## System Overview

The walking locomotion system consists of three main components:

1. **Inverse Kinematics Solver (`ik_solver`)**: Calculates joint angles needed to achieve desired foot positions
2. **Gait Pattern Generator (`gait_generator`)**: Generates coordinated foot trajectories for walking
3. **Walking Controller (`walking_controller`)**: Coordinates the entire walking behavior based on velocity commands

## Testing Prerequisites

Before testing, ensure you have:
- ROS2 Humble Hawksbill installed
- The humanoid robot model properly configured in your simulation environment
- Appropriate controllers set up to handle joint commands

## Running the System

### Method 1: Launch the entire system
```bash
# Source ROS2 and your workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch the walking locomotion system
ros2 launch walking_locomotion walking_locomotion.launch.py
```

### Method 2: Run nodes individually (for debugging)
```bash
# Terminal 1: Start the inverse kinematics solver
ros2 run walking_locomotion ik_solver

# Terminal 2: Start the gait pattern generator  
ros2 run walking_locomotion gait_generator

# Terminal 3: Start the main walking controller
ros2 run walking_locomotion walking_controller
```

## Testing with Commands

Once the system is running, you can send velocity commands to control the robot:

```bash
# Send a velocity command to make the robot walk forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Make the robot turn
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'

# Stop the robot
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Automated Testing

You can also run the automated test script:

```bash
# Run the test script to execute a sequence of movements
ros2 run walking_locomotion test_walking
```

## Expected Behavior

- When `cmd_vel` contains forward velocity (positive x), the robot should walk forward
- When `cmd_vel` contains reverse velocity (negative x), the robot should walk backward
- When `cmd_vel` contains angular velocity (z), the robot should turn in place
- When both linear and angular velocities are present, the robot should move in an arc

## Monitoring

Monitor the system's performance using ROS2 tools:

```bash
# Check published topics
ros2 topic list

# Monitor joint commands
ros2 topic echo /joint_group_position_controller/commands

# Monitor joint states
ros2 topic echo /joint_states

# Check the robot's TF tree (if available)
ros2 run tf2_tools view_frames
```

## Simulation Integration

To test in simulation with Gazebo:

1. Launch your humanoid robot simulation with appropriate controllers
2. Run the walking locomotion nodes (as described above)
3. Ensure the robot's controllers (position, velocity, or effort) are properly subscribed to the command topics

## Troubleshooting

Common issues and solutions:

1. **Robot doesn't respond to commands**:
   - Check that all three nodes are running
   - Verify topic connections with `ros2 topic list` and `ros2 topic info`
   - Ensure your simulation environment is properly configured

2. **Joint movements are unstable or unnatural**:
   - Adjust gait parameters using the `gait_parameters` topic
   - Check the URDF joint limits and ensure they match the controller's expectations

3. **ROS2 nodes fail to start**:
   - Ensure your workspace is properly built with `colcon build`
   - Source the setup files before running the nodes

## Gait Parameter Tuning

You can adjust gait parameters dynamically:

```bash
# Adjust gait parameters [step_height, step_length, step_duration, stride_width]
ros2 topic pub /gait_parameters std_msgs/Float64MultiArray '{data: [0.05, 0.15, 1.0, 0.2]}'
```

Lower step height values make walking more stable but require more precise control.
Longer step duration values slow the walking pace but provide smoother transitions.