# Humanoid Robot Control Package

This package implements the core control system for a humanoid robot. It supports ROS 2 Humble Hawksbill and is designed to work with the Vision-Language-Action (VLA) pipeline.

## Features

- Joint trajectory control for humanoid robot
- Vision-Language-Action (VLA) system integration
- Navigation and movement control
- Sensor simulation (IMU, LiDAR, Odometry)
- Real-time control with 50ms latency
- Support for voice commands and cognitive planning

## Installation

1. Build the package in your ROS 2 workspace:
   ```bash
   cd ~/humanoid_ws
   colcon build --packages-select humanoid_robot_control
   source install/setup.bash
   ```

## Usage

### Running the Controller

```bash
ros2 run humanoid_robot_control main_controller
```

### Using Launch File

```bash
ros2 launch humanoid_robot_control controller.launch.py
```

### Sending Commands

You can send commands to the robot using ROS 2 topics:

```bash
# Send a movement command
ros2 topic pub /control_commands std_msgs/String "data: 'move forward'"

# Send a waving command
ros2 topic pub /control_commands std_msgs/String "data: 'wave'"

# Send vision data (as JSON)
ros2 topic pub /vision_commands std_msgs/String "data: '{\"detected_objects\": [{\"name\": \"red_block\", \"x\": 1.0, \"y\": 0.5}], \"object_positions\": {\"red_block\": [1.0, 0.5]}}'"
```

## Topics

- `/control_commands` (std_msgs/String) - Commands to control the robot
- `/vision_commands` (std_msgs/String) - Vision data and object detections
- `/joint_trajectory` (trajectory_msgs/JointTrajectory) - Joint commands
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands for navigation
- `/odom` (nav_msgs/Odometry) - Odometry data
- `/imu/data` (sensor_msgs/Imu) - IMU sensor data
- `/scan` (sensor_msgs/LaserScan) - LiDAR scan data

## Parameters

The controller uses parameters defined in `config/humanoid_params.yaml`:

- `robot_mass`: Robot mass in kg
- `robot_height`: Robot height in meters
- `control_frequency`: Control loop frequency in Hz
- `max_linear_velocity`: Maximum linear velocity in m/s
- `max_angular_velocity`: Maximum angular velocity in rad/s

## Joint Names

The robot has the following joints:

- `left_hip_joint`, `left_knee_joint`, `left_ankle_joint`
- `right_hip_joint`, `right_knee_joint`, `right_ankle_joint`
- `left_shoulder_joint`, `left_elbow_joint`
- `right_shoulder_joint`, `right_elbow_joint`
- `head_pan_joint`, `head_tilt_joint`

## Architecture

This controller implements the Vision-Language-Action (VLA) system as defined in the project specifications:

1. **Voice**: Accepts commands via topics as a substitute for direct speech recognition
2. **Language**: Processes natural language commands and plans actions
3. **Action**: Executes robot movements based on the planned actions
4. **Perception**: Simulates sensor data (LiDAR, IMU, Odometry) for environmental awareness
5. **Navigation**: Controls robot movement in 2D space

The system is designed to achieve ≤50ms latency for real-time control as specified in the requirements.