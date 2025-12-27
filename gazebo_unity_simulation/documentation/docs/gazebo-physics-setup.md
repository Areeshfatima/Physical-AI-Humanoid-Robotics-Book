---
sidebar_position: 2
---

# Gazebo Physics Environment Setup

This tutorial covers setting up a physics-accurate Gazebo environment for humanoid robot simulation, focusing on realistic physics properties like gravity, collisions, and joints.

## Overview

The Gazebo physics simulation environment provides a realistic physics engine for testing humanoid robots in various scenarios. This tutorial covers:

- Creating basic worlds with physics properties
- Configuring gravity and collision parameters
- Spawning humanoid robot models
- Validating physics accuracy

## Prerequisites

- ROS 2 Humble Hawksbill installed
- Gazebo Harmonic (Ignition) installed
- Basic understanding of URDF/SDF formats
- Python 3.10+

## Creating a Basic Gazebo World

Let's start by examining the basic physics world that comes with the simulation package:

### World File Structure

The basic physics world is defined in `basic_physics.world`:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="basic_physics_world">
    <!-- Physics engine configuration -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Include ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include sun light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Add some basic obstacles for testing -->
    <model name="box_obstacle">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.166667</ixx>
            <iyy>0.166667</iyy>
            <izz>0.166667</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

Key physics parameters configured:

- **Gravity**: 9.8 m/s² in the negative Z direction (Earth-like gravity)
- **Time Step**: 0.001 seconds for accurate physics simulation
- **Real Time Factor**: 1.0 for real-time simulation speed
- **Update Rate**: 1000 Hz for high-frequency updates

## Physics Configuration

The physics parameters can also be configured separately in YAML files:

```yaml
# Physics configuration file
# Defines global physics parameters for the simulation

# Gravity vector (m/s^2)
gravity_x: 0.0
gravity_y: 0.0
gravity_z: -9.8

# Simulation time step (seconds)
time_step: 0.001

# Real-time factor (simulation time / real time)
real_time_factor: 1.0

# Max update rate (Hz)
max_update_rate: 1000.0

# ODE solver parameters
ode_solver_type: "quick"
ode_min_step_size: 0.0001
ode_iters: 50
ode_precon_iters: 0
```

## Spawning the Humanoid Robot

The humanoid robot is defined in URDF format with realistic physical properties:

### Humanoid Robot URDF Structure

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.4"/>
      </geometry>
      <material name="light_blue">
        <color rgba="0.7 0.7 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <origin xyz="0 0 0"/>
      <inertial>
        <ixx value="0.05833"/>
        <ixy value="0.0"/>
        <ixz value="0.0"/>
        <iyy value="0.05833"/>
        <iyz value="0.0"/>
        <izz value="0.025"/>
      </inertial>
    </inertial>
  </link>

  <!-- Torso, Head, Arms, Legs defined similarly -->
  
  <!-- Joints connecting the links -->
  <joint name="base_torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.45"/>
  </joint>

  <!-- Example of a revolute joint with constraints -->
  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.2"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
  </joint>
</robot>
```

### Robot Spawner Service

The robot spawning functionality is implemented in the RobotSpawner node:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity

class RobotSpawner(Node):
    """
    ROS 2 service server that allows to spawn robots into Gazebo simulation
    """
    
    def __init__(self):
        super().__init__('robot_spawner')
        
        # Create service for spawning entities
        self.spawn_service = self.create_service(
            SpawnEntity,
            'spawn_robot',
            self.spawn_robot_callback
        )
        
        self.get_logger().info('Robot Spawner service initialized')
    
    # Callback implementation would go here
    # See full implementation in robot_spawner.py
```

## Launching the Simulation

To launch the basic physics simulation:

```bash
# Navigate to the workspace
cd ~/ros2_workspace

# Build the package
colcon build --packages-select gazebo_worlds

# Source the workspace
source install/setup.bash

# Launch the physics simulation
ros2 launch ros_integration basic_physics_simulation.launch.py
```

## Validating Physics Accuracy

The simulation includes validation tools to ensure physics accuracy:

### Physics Validator

The PhysicsValidator node compares actual robot behavior with theoretical physics:

```python
class PhysicsValidator(Node):
    """
    Node that validates physics simulation accuracy by comparing actual robot behavior
    with expected theoretical physics behavior.
    """
    
    def validate_physics(self):
        """
        Validate physics simulation accuracy against theoretical values
        """
        # Calculate expected motion based on initial conditions
        expected_pos, expected_vel = self.calculate_expected_motion(
            self.initial_conditions['position'],
            self.initial_conditions['velocity'],
            delta_t
        )
        
        # Compare with actual values from simulation
        actual_pos = self.robot_odom.pose.pose.position
        actual_vel = self.robot_odom.twist.twist.linear
        
        # Calculate deviations
        pos_deviation_percentage = # calculation logic
        vel_deviation_percentage = # calculation logic
        
        # Check if deviations are within ≤5% threshold
        pass_validation = pos_deviation_percentage <= 5.0 and vel_deviation_percentage <= 5.0
```

## Running Validation Tests

To run the physics accuracy validation:

```bash
# Run the physics validation tool
python3 src/physics_validator.py
```

This will run validation tests to ensure the physics simulation maintains ≤5% deviation from expected behavior.

## Key Physics Concepts

Understanding these key physics concepts is important for the simulation:

### Gravity Simulation
- Standard Earth gravity of 9.8 m/s² is applied
- Objects should fall at this rate in the absence of other forces
- Robots should experience realistic weight distribution

### Collision Detection
- Objects should not pass through each other
- Collision response should be physically realistic
- Ground contact should prevent further downward movement

### Joint Constraints
- Robot joints should respect their physical limits
- Revolute joints have position, velocity, and effort limits
- Fixed joints do not allow relative movement

### Inertial Properties
- Each link has mass and moment of inertia properties
- These affect how the robot responds to forces
- Proper mass distribution affects balance and locomotion

## Troubleshooting Common Physics Issues

### Robot Falling Through Floor
- Verify collision properties are properly defined
- Check that mass properties are reasonable
- Ensure gravity is properly configured

### Unstable Simulation
- Reduce the time step size for better stability
- Adjust ODE solver parameters
- Verify that inertial tensors satisfy the triangle inequality

### Joint Limits Ignored
- Check joint type configuration in URDF
- Verify limit values are properly specified
- Ensure joint controllers respect limits

## Next Steps

Now that you understand the physics environment setup:

1. Continue with [Sensor Simulation](../sensors/sensor-setup) to learn about sensor configuration
2. Learn about [Unity Visualization](../unity/unity-integration) for high-fidelity rendering
3. Explore [Custom Environments](../environments/custom-envs) to create your own simulation worlds

## Resources

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS 2 with Gazebo Guide](https://classic.gazebosim.org/tutorials?tut=ros2_integration)