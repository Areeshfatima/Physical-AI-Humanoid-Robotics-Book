# Isaac Sim VSLAM Navigation API Contract

## Overview
This contract specifies the interfaces between different components in the Isaac Sim VSLAM Navigation system for humanoid robotics. It defines the API endpoints, ROS interfaces, and data formats for communication between simulation, perception, and navigation modules.

## Isaac Sim Environment API

### Environment Initialization Service
- **Service Name**: `/initialize_environment`
- **Service Type**: `isaac_ros_interfaces/srv/InitializeEnvironment`
- **Request Schema**:
  ```
  string environment_path  # Path to USD file describing the environment
  bool use_random_objects   # Whether to add random objects for data augmentation
  float lighting_condition  # Lighting condition factor (0.0-2.0)
  ```
- **Response Schema**:
  ```
  bool success
  string error_message
  string environment_id  # Unique ID assigned to this environment instance
  ```

### Scene Configuration Service
- **Service Name**: `/configure_scene`
- **Service Type**: `isaac_ros_interfaces/srv/ConfigureScene`
- **Request Schema**:
  ```
  string environment_id  # ID of environment to configure
  RobotConfiguration robot_config  # Robot configuration details
  list[SensorConfiguration] sensors  # Sensor configurations
  list[ObjectConfiguration] static_objects  # Static object placements
  ```
- **Response Schema**:
  ```
  bool success
  string error_message
  float initialization_time_ms
  ```

## Sensor Simulation API

### Synthetic Data Generation Action
- **Action Name**: `/generate_synthetic_dataset`
- **Action Type**: `isaac_ros_interfaces/action/GenerateDataset`
- **Goal Schema**:
  ```
  string dataset_name
  string environment_id
  int num_frames  # Number of frames to generate
  float motion_speed  # Speed of robot during data collection
  bool include_annotations  # Whether to include ground truth annotations
  ```
- **Feedback Schema**:
  ```
  int frames_collected
  float progress_percentage
  string status_message
  ```
- **Result Schema**:
  ```
  bool success
  string dataset_path  # Path where dataset was saved
  int total_frames_collected
  float generation_time_sec
  ```

### Sensor Data Topics
- **LiDAR Topic**: `/sensors/lidar/scan`
  - **Message Type**: `sensor_msgs/LaserScan`
  - **QoS Profile**: Reliable, queue size 10
  - **Performance Requirements**: 10-20 Hz update rate

- **Depth Camera Topic**: `/sensors/depth/image_rect_raw`
  - **Message Type**: `sensor_msgs/Image`
  - **QoS Profile**: Reliable, queue size 5
  - **Performance Requirements**: 15-30 Hz update rate, depth accuracy ≤5cm

- **IMU Topic**: `/sensors/imu/data`
  - **Message Type**: `sensor_msgs/Imu`
  - **QoS Profile**: Reliable, queue size 10
  - **Performance Requirements**: 100-200 Hz update rate, drift ≤5%

## VSLAM API

### Visual SLAM Service
- **Service Name**: `/vslam/set_parameters`
- **Service Type**: `isaac_ros_interfaces/srv/SetSlamParameters`
- **Request Schema**:
  ```
  float tracking_features  # Number of features to track
  float map_resolution    # Resolution of generated map (m/cell)
  float relocalization_threshold  # Threshold for relocalization
  string slam_algorithm   # Algorithm to use (ORB-SLAM, Stereo-VIO, etc.)
  ```
- **Response Schema**:
  ```
  bool success
  string error_message
  float actual_map_resolution  # Actual resolution set after validation
  ```

### SLAM State Topic
- **Topic Name**: `/vslam/state`
- **Message Type**: `isaac_ros_interfaces/msg/SlamState`
- **QoS Profile**: Reliable, queue size 5
- **Message Schema**:
  ```
  std_msgs/Header header
  string state  # IDLE, TRACKING, MAPPING, LOCALIZING
  float tracking_confidence  # Confidence level (0.0-1.0)
  float drift_estimate  # Estimated drift percentage
  geometry_msgs/PoseStamped current_pose
  ```
- **Performance Requirements**: ≥10 Hz update rate, ≤50ms latency

## Navigation API

### Navigation Action
- **Action Name**: `/navigate_to_pose`
- **Action Type**: `nav_msgs/action/NavigateToPose` (standard Navigation2 action)
- **Goal Schema**:
  ```
  geometry_msgs/PoseStamped pose
  string behavior_tree  # Optional behavior tree override
  ```
- **Feedback Schema**:
  ```
  nav_msgs/RobotPosition path_progress
  string current_command
  ```
- **Result Schema**:
  ```
  bool succeeded
  string error_message
  ```

### Footstep Planning Service
- **Service Name**: `/footstep_planner/get_plan`
- **Service Type**: `isaac_ros_interfaces/srv/FootstepPlan`
- **Request Schema**:
  ```
  geometry_msgs/PoseStamped start_pose
  geometry_msgs/PoseStamped goal_pose
  string terrain_type  # FLAT, ROUGH, STAIRS, etc.
  ```
- **Response Schema**:
  ```
  bool success
  string error_message
  list<Footstep> footsteps  # Planned footsteps
  float plan_time_ms
  ```

## Performance and Validation APIs

### Performance Monitoring Topic
- **Topic Name**: `/validation/performance_metrics`
- **Message Type**: `isaac_ros_interfaces/msg/PerformanceMetrics`
- **QoS Profile**: Reliable, queue size 5
- **Message Schema**:
  ```
  std_msgs/Header header
  float slam_accuracy_percent  # SLAM accuracy (lower is better)
  float navigation_success_rate  # Navigation success rate (higher is better)
  float rendering_fps  # Rendering frames per second
  float processing_latency_ms  # Processing latency
  list[float] resource_usage  # [CPU, RAM, GPU utilizations as percentages]
  ```
- **Performance Requirements**: 
  - Rendering FPS: ≥30 FPS
  - SLAM drift: ≤5% over 100m
  - Navigation success: ≥95% in obstacle-free environments
  - Processing latency: ≤50ms for perception tasks

### Validation Service
- **Service Name**: `/validation/run_test`
- **Service Type**: `isaac_ros_interfaces/srv/RunValidationTest`
- **Request Schema**:
  ```
  string test_name  # TRAJECTORY_ACCURACY, NAVIGATION_SUCCESS, etc.
  string environment_id
  ValidationParams parameters  # Test-specific parameters
  ```
- **Response Schema**:
  ```
  bool passed
  string error_message
  float test_score  # How well the system performed (0.0-1.0)
  ValidationDetails details  # Detailed breakdown of results
  ```

## Error Handling

### Standard Error Response Format
For all services and actions, when an error occurs:
```
{
  "success": false,
  "error_code": "ERROR_TYPE",
  "error_message": "Descriptive error message",
  "error_details": {
    "timestamp": "ISO8601 timestamp",
    "component": "Which component failed",
    "suggested_action": "How to resolve the issue"
  }
}
```

## Quality of Service Requirements

- **High Frequency Topics** (IMU, joint states): Reliable delivery, highest priority
- **Medium Frequency Topics** (camera feeds): Reliable delivery with moderate priority  
- **Low Frequency Topics** (navigation goals): Reliable delivery with lower priority
- **Action Services**: Reliable delivery with medium priority
- **RPC Services**: Reliable delivery with high priority

## Validation Requirements

- All published topics must adhere to ROS 2 standard message types where possible
- Services must respond within 5 seconds for standard operations
- Actions must provide feedback at least every 0.5 seconds during execution
- System must maintain ≤5% drift for 100m trajectories
- System must maintain >95% navigation success rate in appropriate environments
- All interfaces must handle errors gracefully without crashing