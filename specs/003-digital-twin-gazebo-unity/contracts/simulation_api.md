# Simulation API Contract: Digital Twin with Gazebo & Unity

## Overview
This contract specifies the simulation interface between Gazebo physics simulation and Unity visualization for the digital twin system. It defines the communication protocols, data formats, and performance requirements for the educational simulation module.

## Gazebo Simulation Services

### Robot Spawning Service
- **Service Name**: `/spawn_robot`
- **Service Type**: `gazebo_msgs/srv/SpawnEntity`
- **Request Schema**:
  ```
  string name
  string xml
  string robot_namespace
  gazebo_msgs/msgs/Pose initial_pose
  string reference_frame
  ```
- **Response Schema**:
  ```
  bool success
  string status_message
  ```

### Physics Configuration Service
- **Service Name**: `/set_physics_properties`
- **Service Type**: `gazebo_msgs/srv/SetPhysicsProperties`
- **Request Schema**:
  ```
  float64 time_step
  float64 max_update_rate
  geometry_msgs/Vector3 gravity
  gazebo_msgs/msgs/ODEPhysics ode_config
  ```
- **Response Schema**:
  ```
  bool success
  string status_message
  ```

## Sensor Data Topics

### LiDAR Sensor Topic
- **Topic Name**: `/lidar_scan`
- **Message Type**: `sensor_msgs/LaserScan`
- **QoS Profile**: Reliable, queue size 10
- **Message Schema**:
  ```
  std_msgs/Header header
  float32 angle_min
  float32 angle_max
  float32 angle_increment
  float32 time_increment
  float32 scan_time
  float32 range_min
  float32 range_max
  float32[] ranges
  float32[] intensities
  ```
- **Performance Requirements**: 
  - Update rate: 10-20 Hz
  - Range accuracy: ±2cm
  - Angular resolution: ≤0.5°

### Depth Camera Topic
- **Topic Name**: `/depth_camera/depth/image_raw`
- **Message Type**: `sensor_msgs/Image`
- **QoS Profile**: Reliable, queue size 5
- **Message Schema**:
  ```
  std_msgs/Header header
  uint32 height
  uint32 width
  string encoding
  uint8 is_bigendian
  uint32 step
  uint8[] data
  ```
- **Performance Requirements**:
  - Update rate: 15-30 Hz
  - Resolution: Configurable (e.g., 640x480, 1280x720)
  - Depth accuracy: ±3% of measured distance

### IMU Sensor Topic
- **Topic Name**: `/imu/data`
- **Message Type**: `sensor_msgs/Imu`
- **QoS Profile**: Reliable, queue size 10
- **Message Schema**:
  ```
  std_msgs/Header header
  geometry_msgs/Quaternion orientation
  float64[9] orientation_covariance
  geometry_msgs/Vector3 angular_velocity
  float64[9] angular_velocity_covariance
  geometry_msgs/Vector3 linear_acceleration
  float64[9] linear_acceleration_covariance
  ```
- **Performance Requirements**:
  - Update rate: 50-200 Hz
  - Noise characteristics: Match real IMU specifications
  - Orientation accuracy: ≤0.5°

## Unity Visualization Interface

### Robot State Topic
- **Topic Name**: `/robot_state`
- **Message Type**: `sensor_msgs/JointState`
- **QoS Profile**: Reliable, queue size 10
- **Message Schema**:
  ```
  std_msgs/Header header
  string[] name
  float64[] position
  float64[] velocity
  float64[] effort
  ```
- **Performance Requirements**:
  - Update rate: 60 Hz minimum for smooth visualization
  - Synchronization with physics simulation: <50ms delay
  - Joint position accuracy: <0.001 radians

### Model Configuration Service
- **Service Name**: `/set_model_configuration`
- **Service Type**: `gazebo_msgs/srv/SetModelConfiguration`
- **Request Schema**:
  ```
  string model_name
  string robot_namespace
  string[] joint_names
  float64[] joint_positions
  ```
- **Response Schema**:
  ```
  bool success
  string status_message
  ```

## Environment Management Interface

### World Control Service
- **Service Name**: `/world_control`
- **Service Type**: `std_srvs/srv/Empty` (for reset)
- **Request Schema**: Empty
- **Response Schema**: Empty
- **Alternative service** `/reset_world` with same types

### Simulation Time Topic
- **Topic Name**: `/clock`
- **Message Type**: `rosgraph_msgs/Clock`
- **QoS Profile**: Reliable, queue size 1
- **Message Schema**:
  ```
  builtin_interfaces/Time clock
  ```
- **Performance Requirements**:
  - Time synchronization accuracy: <1ms
  - Consistent timing across all nodes

## Performance and Quality Requirements

### Physics Simulation
- **Real-time Factor**: ≥0.8 for interactive learning
- **Physics Accuracy**: ≤5% deviation from expected behavior
- **Collision Detection**: Realistic response with appropriate materials
- **Gravity Simulation**: 9.8 m/s² with configurable variations

### Sensor Simulation
- **LiDAR**: Realistic noise modeling with adjustable parameters
- **Depth Camera**: Physically plausible depth estimation
- **IMU**: Realistic noise models matching actual sensors
- **Update Consistency**: Regular timing with minimal jitter

### Visualization
- **Frame Rate**: 60 FPS minimum for smooth visualization
- **Model Fidelity**: Visual representation matches physics model
- **Latency**: <50ms from physics simulation to Unity display
- **Coordinate System**: Consistent between Gazebo and Unity

## Error Handling
- Invalid spawn requests return descriptive error messages
- Sensor failures are logged and communicated appropriately
- Network disconnections are handled gracefully with reconnection attempts
- Physics instabilities trigger appropriate warning mechanisms

## Validation Criteria
- Sensor outputs match expected real-world equivalents within tolerance
- Physics simulation maintains stability under various conditions
- Visualization accurately reflects simulation state
- Performance requirements are consistently met during normal operation