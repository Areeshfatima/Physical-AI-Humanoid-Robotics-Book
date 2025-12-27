# URDF Robot Description Contract

## Overview
This contract specifies the Unified Robot Description Format (URDF) for humanoid robots used in the ROS 2 fundamentals tutorial. It defines the structure, properties, and validation requirements for robot models.

## URDF Structure Requirements

### Root Element
```
<robot name="robot_name">
  <!-- Links, joints, and other elements -->
</robot>
```

### Required Elements
1. **Robot Element**: Root element with a unique name attribute
2. **Links**: At least one link must exist (typically a base link)
3. **Joints**: Connections between links must form a valid kinematic tree

### Link Definition Schema
```
<link name="unique_link_name">
  <inertial>
    <origin xyz="x y z" rpy="roll pitch yaw"/>
    <mass value="mass_in_kg"/>
    <inertia ixx="ixx" ixy="ixy" ixz="ixz" iyy="iyy" iyz="iyz" izz="izz"/>
  </inertial>
  <visual>
    <origin xyz="x y z" rpy="r p y"/>
    <geometry>
      <!-- geometry type -->
    </geometry>
    <material name="material_name"/>
  </visual>
  <collision>
    <origin xyz="x y z" rpy="r p y"/>
    <geometry>
      <!-- geometry type -->
    </geometry>
  </collision>
</link>
```

### Joint Definition Schema
```
<joint name="unique_joint_name" type="joint_type">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>  <!-- for revolute, continuous, prismatic joints -->
  <limit lower="lower_limit" upper="upper_limit" effort="max_effort" velocity="max_velocity"/>
</joint>
```

## Joint Types
- `revolute`: Rotational joint with limited range
- `continuous`: Rotational joint without limits
- `prismatic`: Linear sliding joint with limits
- `fixed`: No movement, permanent connection
- `floating`: 6 DOF, floating movement
- `planar`: Planar motion with 3 DOF

## Geometric Shapes
- `<box size="x y z"/>`
- `<cylinder radius="r" length="l"/>`
- `<sphere radius="r"/>`
- `<mesh filename="path_to_mesh_file"/>`

## Performance Requirements
- URDF parsing time: ≤100ms for humanoid models up to 30 links
- Memory usage: ≤100MB for complete robot model
- Visualization refresh rate: ≥30 FPS when loaded in RViz

## Validation Requirements
- Single root link (base link)
- No disconnected link components
- Properly closed XML syntax
- Valid physical properties (mass > 0, positive inertia values)
- Valid joint limits and kinematic constraints

## Security Requirements
- URDF files must be validated for malicious content before loading
- No external resource references that could cause security issues

## Humanoid Robot Model Example
```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base body -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.002" ixy="0" ixz="0" iyy="0.002" iyz="0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Additional limbs would follow similar pattern -->
</robot>
```

## Integration Requirements

### With Robot State Publisher
- URDF must be accessible to robot_state_publisher node
- Joint states must be published on `/joint_states` topic
- TF transforms generated automatically from URDF and joint states

### With Controllers
- Transmission elements must define how joints connect to controllers
- Control interfaces must match between URDF and controller configuration

### With Simulation
- Gazebo-specific elements must be properly defined in <gazebo> tags
- Collision properties must be appropriate for physics simulation

## Error Handling
- Invalid URDF syntax: Parser provides detailed error messages
- Kinematic loops: Validation detects and reports kinematic violations
- Missing dependencies: Reports missing mesh files or materials