---
sidebar_position: 3
title: "Nav2 Path Planning"
---

# Nav2 Path Planning for Humanoid Robots

## Overview

Navigation2 (Nav2) is the ROS 2 navigation stack that provides path planning, localization, and navigation capabilities for mobile robots. For humanoid robots, Nav2 requires special configurations to account for bipedal locomotion constraints, balance preservation, and unique kinematic properties.

This documentation covers how to configure and use Nav2 for humanoid robot navigation, focusing on achieving >95% navigation success rate in various environments.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Navigation2 packages
- Isaac Sim environment with humanoid robot
- Isaac ROS VSLAM for localization
- Properly configured robot description (URDF/SDF)

## Architecture

Nav2 consists of several key components:

1. **Global Planner**: Computes the overall path from start to goal
2. **Local Planner**: Executes short-term navigation and obstacle avoidance
3. **Costmap 2D**: Maintains obstacle and cost information
4. **Recovery Behaviors**: Handles navigation failures
5. **Footstep Planner**: Specialized for humanoid bipedal locomotion

## Installation

### Installing Navigation2

1. **Install Nav2 packages**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2
   sudo apt install ros-humble-nav2-bringup
   ```

2. **Install Nav2 tools**:
   ```bash
   sudo apt install ros-humble-nav2-tools
   ```

### Verifying Installation

Check that Nav2 packages are properly installed:

```bash
# List Nav2 packages
ros2 pkg list | grep nav2

# Check available launch files
find /opt/ros/humble/share/nav2_bringup/ -name "*.py" -type f
```

## Humanoid-Specific Configuration

### Robot Footprint

Humanoid robots have different footprint requirements than wheeled robots:

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: True
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 10.0
      publish_frequency: 10.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05
      footprint: "[ [0.15, 0.15], [0.15, -0.15], [-0.15, -0.15], [-0.15, 0.15] ]"
      footprint_padding: 0.01
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
```

### Footstep Planning Configuration

For bipedal locomotion, the footstep planner is crucial:

```yaml
# humanoid_footstep_planner.yaml
footstep_planner:
  ros__parameters:
    step_width: 0.20  # Distance between left and right feet
    step_length: 0.30 # Forward step distance
    step_min_height: 0.05  # Minimum step height for obstacles
    max_step_reach: 0.35   # Maximum distance to next step
    robot_width: 0.4       # Robot width for collision checking
    robot_length: 0.6      # Robot length for collision checking
    planner_frequency: 5.0 # How often to plan footsteps
```

## Launching Nav2 with Humanoid Configuration

### Basic Launch Command

```bash
# Launch Nav2 with humanoid-specific parameters
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/humanoid_nav2_params.yaml
```

### Complete Navigation Setup

```bash
# Terminal 1: Launch Isaac Sim environment
isaac-sim

# Terminal 2: Launch VSLAM
ros2 launch isaac_ros_visual_slam humanoid_vslam.launch.py

# Terminal 3: Launch Navigation
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:=/path/to/humanoid_nav2_params.yaml

# Terminal 4: Send navigation goals
ros2 run nav2_msgs navigation_goal_sender
```

## Configuring Planners

### Global Planners

Nav2 supports multiple global planners. For humanoid robots, consider:

1. **NavFn** (default):
   ```yaml
   global_costmap:
     plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
   
   NavFn:
     plugin: "nav2_navfn_planner/NavfnPlanner"
     tolerance: 0.5
     use_astar: false
     allow_unknown: true
   ```

2. **GridBased**:
   ```yaml
   GridBased:
     plugin: "nav2_navfn_planner/NavfnPlanner"
     # Additional parameters specific to grid-based planning
   ```

### Local Planners

For humanoid navigation, the local planner needs special considerations:

1. **DWA Local Planner** (recommended for humanoid):
   ```yaml
   local_planner:
     DWAPlanner:
       plugin: "dwb_core::DWBLocalPlanner"
       debug_trajectory_details: True
       min_vel_x: 0.0
       max_vel_x: 0.5
       min_vel_y: -0.1
       max_vel_y: 0.1
       max_vel_theta: 0.3
       min_speed_xy: 0.0
       max_speed_xy: 0.5
       min_speed_theta: 0.0
       acc_lim_x: 2.5
       acc_lim_y: 2.5
       acc_lim_theta: 3.2
       decel_lim_x: -2.5
       decel_lim_y: -2.5
       decel_lim_theta: -3.2
       vx_samples: 20
       vy_samples: 5
       vtheta_samples: 20
       sim_time: 1.7
       linear_granularity: 0.05
       angular_granularity: 0.025
       transform_tolerance: 0.2
       xy_goal_tolerance: 0.25
       yaw_goal_tolerance: 0.25
       rot_stopped_vel: 0.01
       trans_stopped_vel: 0.01
       short_circuit_trajectory_evaluation: True
       stateful: True
       critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
       BaseObstacle.scale: 0.02
       PathAlign.scale: 32.0
       PathAlign.forward_point_distance: 0.1
       GoalAlign.scale: 24.0
       GoalAlign.forward_point_distance: 0.1
       PathDist.scale: 32.0
       GoalDist.scale: 24.0
       RotateToGoal.scale: 32.0
       RotateToGoal.slowing_factor: 5.0
       Oscillation.scale: 1.0
       Oscillation.oscillation_threshold: 0.05
       Oscillation.oscillation_reset_angle: 0.2
   ```

### Humanoid-Specific Recovery Behaviors

Humanoid robots need specialized recovery behaviors:

```yaml
# Recovery behaviors for humanoid robots
behavior_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_tree_xml_filename: /path/to/humanoid_behavior_tree.xml
    recovery_server:
      possible_recoveries: ["spin", "backup", "wait", "humanoid_sway"]
    spin:
      plugin: "nav2_recoveries/Spin"
      spin_dist: 1.57
    backup:
      plugin: "nav2_recoveries/Backup"
      backup_dist: 0.15
      backup_speed: 0.025
    wait:
      plugin: "nav2_recoveries/Wait"
      wait_duration: 1.0
    humanoid_sway:
      plugin: "humanoid_recoveries/Sway"
      sway_angle: 0.1
      sway_duration: 2.0
```

## Performance Validation

### Navigation Success Rate

To validate that navigation achieves >95% success rate:

1. **Run navigation tests**:
   ```bash
   # Use navigation performance tests
   ros2 run nav2_msgs navigation_test \
     --ros-args -p test_scenario:=simple_navigation
   ```

2. **Analyze results**:
   ```bash
   # Check navigation logs for success/failure statistics
   ros2 run nav2_msgs navigation_analyzer
   ```

### Metrics to Monitor

Key metrics for humanoid navigation:
- **Success Rate**: >95% of goals reached in obstacle-free environments
- **Path Quality**: Optimality compared to global plan
- **Execution Time**: How quickly goals are achieved
- **Collision Avoidance**: Safe navigation around obstacles
- **Footstep Planning**: Valid and stable step sequences

## Integration with Isaac ROS VSLAM

### Localization with VSLAM

Nav2 uses the pose estimates from Isaac ROS VSLAM for localization:

1. **TF Setup**: Ensure proper TF tree between map, odom, and base_link
2. **Parameter Configuration**: Set `use_sim_time` to true for simulation
3. **Data Flow**: VSLAM provides pose while Nav2 plans paths

### Configuration for VSLAM Integration

```yaml
# Parameters for VSLAM localization
amcl:
  ros__parameters:
    use_sim_time: True
    # VSLAM already provides accurate pose, so reduce AMCL uncertainty
    initial_covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.05]  # Lower than default
    update_min_d: 0.1  # Update less frequently since VSLAM is accurate
    update_min_a: 0.1  # Update less frequently since VSLAM is accurate
```

## Troubleshooting

### Common Issues

1. **Navigation Failures**:
   - Check robot footprint configuration
   - Verify sensor data is reaching Nav2
   - Ensure proper TF frames exist
   - Validate VSLAM localization accuracy

2. **Poor Path Quality**:
   - Adjust global planner parameters
   - Modify local planner constraints
   - Check costmap resolution
   - Verify map quality and resolution

3. **Collision Issues**:
   - Increase footprint size
   - Adjust inflation parameters
   - Check sensor data quality
   - Improve obstacle detection

4. **Footstep Planning Problems**:
   - Verify footstep planner parameters
   - Check for valid terrain in the path
   - Ensure proper ground detection
   - Validate robot kinematic constraints

## Advanced Topics

### Custom Behavior Trees

Nav2 uses behavior trees to define the navigation process:

```xml
<!-- humanoid_behavior_tree.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="6" name="ComputeAndStrafe">
          <PipelineSequence name="ComputeAndStrafeSequence">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <FollowPath path="{path}" controller_id="FollowPath"/>
          </PipelineSequence>
          <RecoveryNode number_of_retries="2" name="Clearing">
            <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
            <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RecoveryNode>
      </RateController>
      <ReactiveSequence name="OnGoalReached">
        <GoalReached goal="{goal}" tolerance="0.25"/>
        <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
        <FollowPath path="{path}" controller_id="FollowPath"/>
      </ReactiveSequence>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

### Simulating Humanoid Constraints

To better simulate humanoid-specific constraints:

1. **Footprint Shapes**: Use complex polygon footprints
2. **Kinematic Constraints**: Account for bipedal movement
3. **Balance Preservation**: Add sway movements during navigation
4. **Terrain Adaptation**: Adjust for uneven surfaces

## Resources

- [Navigation2 Documentation](https://navigation.ros.org/)
- [ROS 2 Navigation Tutorials](https://navigation.ros.org/tutorials/index.html)
- [Costmap 2D Documentation](http://wiki.ros.org/costmap_2d)
- [Humanoid Robotics Research](https://www.ros.org/wiki/Humanoid%20Robots)