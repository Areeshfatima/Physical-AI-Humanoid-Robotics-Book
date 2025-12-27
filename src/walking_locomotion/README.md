# Walking Locomotion Package

This package provides walking locomotion capabilities for the humanoid robot. It implements inverse kinematics, gait generation, and coordinated control for stable bipedal walking.

## Features

- **Inverse Kinematics Solver**: Calculates joint angles to achieve desired foot positions
- **Gait Pattern Generator**: Creates stable walking patterns with adjustable parameters
- **Walking Controller**: Coordinates complete walking behavior based on velocity commands
- **Stable Walking**: Implements a controlled gait that maintains balance during locomotion
- **Parameter Tuning**: Allows real-time adjustment of gait parameters for different walking styles

## Package Structure

- `ik_solver.py`: Inverse kinematics solver for leg movement control
- `gait_generator.py`: Generator for coordinated walking patterns
- `walking_controller.py`: Main controller coordinating locomotion
- `test_walking.py`: Test script for validating the implementation
- `launch/`: Launch files for starting the complete system

## Dependencies

- `rclpy`: ROS2 Python library
- `std_msgs`: Standard ROS2 message types
- `sensor_msgs`: Sensor message types
- `geometry_msgs`: Geometric message types
- `numpy`: Numerical operations for IK calculations

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd physical-ai-humanoid-robotics
   ```

2. Install ROS2 Humble Hawksbill following the installation guide:
   ```bash
   ./ros2_installation_guide.sh
   ```

3. Build the workspace:
   ```bash
   ./setup_workspace.sh
   ```

## Usage

### Launch the System

```bash
# Source your ROS2 environment and workspace
source /opt/ros/humble/setup.bash
source ~/humanoid_ws/install/setup.bash

# Launch the walking locomotion system
ros2 launch walking_locomotion walking_locomotion.launch.py
```

### Control the Robot

Send velocity commands to control the robot's walking:

```bash
# Walk forward
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'

# Turn while walking
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
```

## Parameters

The system allows real-time adjustment of gait parameters:

- `step_height`: Maximum foot lift height during swing phase
- `step_length`: Forward distance covered per step
- `step_duration`: Time taken to complete one full step cycle
- `stride_width`: Lateral distance between feet for stability

## Testing

Run the automated test sequence:

```bash
ros2 run walking_locomotion test_walking
```

## Integration with Simulation

This package is designed to work with Gazebo simulation environments:

1. Ensure your humanoid robot model has appropriate joint controllers
2. Launch the simulation with your robot model
3. Start the walking locomotion nodes
4. Send velocity commands to control the robot

## Algorithm Details

### Inverse Kinematics
The system implements a geometric inverse kinematics solution for a 2-DOF leg model, calculating hip and knee angles to achieve target foot positions while respecting joint limits.

### Gait Generation
The gait generator creates coordinated foot trajectories using a combination of support and swing phases, ensuring stable walking by maintaining the center of pressure within the support polygon.

### Walking Control
The main controller orchestrates the complete walking behavior, translating high-level velocity commands into coordinated joint movements while maintaining balance.

## References

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Humanoid Robot Walking Control Research](https://www.example.com)
- [Inverse Kinematics for Humanoid Robots](https://www.example.com)