# Complete Digital Twin Example

This example demonstrates the complete workflow of creating a digital twin with Gazebo physics simulation and Unity visualization.

## Scenario: Humanoid Robot Navigation

In this example, we'll create a humanoid robot that navigates through a simple environment with obstacles, using sensor data for navigation decisions.

### Step 1: Create the Gazebo World

First, create a world file with obstacles:

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="humanoid_navigation">
    <!-- Physics engine -->
    <physics type="ode">
      <gravity>0 0 -9.8</gravity>
    </physics>
    
    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    
    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>
    
    <!-- Walls -->
    <model name="wall1">
      <pose>2 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.2 4 2</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.2 4 2</size></box></geometry>
          <material><diffuse>0.8 0.4 0.2 1</diffuse></material>
        </visual>
        <inertial>
          <mass>100.0</mass>
          <inertia>
            <ixx>16.8333</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>16.8333</iyy>
            <iyz>0</iyz>
            <izz>0.4167</izz>
          </inertia>
        </inertial>
      </link>
    </model>
    
    <!-- Humanoid robot will be spawned separately -->
  </world>
</sdf>
```

### Step 2: Configure Robot Sensors

Configure the humanoid robot with the necessary sensors for navigation:

```xml
<!-- Part of the robot URDF -->
<gazebo reference="lidar_sensor">
  <sensor type="ray" name="lidar">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/lidar</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_sensor_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Step 3: Launch the Simulation

Create a launch file to bring up the entire simulation:

```python
# launch/humanoid_navigation.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch Gazebo with our world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(get_package_share_directory('humanoid_robot_sim'), 'worlds', 'humanoid_navigation.world')}.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'humanoid_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_entity
    ])
```

### Step 4: Run the Navigation Algorithm

Implement a simple navigation algorithm that uses sensor data to navigate:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np


class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        
        # Create subscriber for laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Create a timer for periodic control updates
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.latest_scan = None
        self.get_logger().info('Navigation controller initialized')
    
    def scan_callback(self, msg):
        self.latest_scan = msg
    
    def control_loop(self):
        if self.latest_scan is None:
            return
        
        # Simple navigation strategy: move forward if clear path ahead, turn otherwise
        # Look at the front 30-degree sector
        front_sector_start = len(self.latest_scan.ranges) // 2 - len(self.latest_scan.ranges) // 24  # ~15 degrees
        front_sector_end = len(self.latest_scan.ranges) // 2 + len(self.latest_scan.ranges) // 24   # ~15 degrees
        
        # Get distances in the front sector
        front_distances = self.latest_scan.ranges[front_sector_start:front_sector_end]
        
        # Filter out invalid distances
        valid_distances = [d for d in front_distances if d >= self.latest_scan.range_min and d <= self.latest_scan.range_max]
        
        if not valid_distances:
            # No valid readings, stop
            cmd = Twist()
        else:
            # If closest obstacle is far enough, go forward, otherwise turn
            closest_obstacle = min(valid_distances)
            
            cmd = Twist()
            if closest_obstacle > 1.0:  # 1 meter threshold
                cmd.linear.x = 0.5  # Go forward
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5  # Turn right (positive angular velocity)
        
        # Publish the command
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    
    controller = NavigationController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Navigation controller stopped')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 5: Unity Visualization

Set up the Unity visualization to match the Gazebo physics:

1. Export the robot model from Gazebo to a format compatible with Unity (FBX or OBJ)
2. Import into Unity and set up the scene with matching lighting and environment
3. Implement the ROS-Unity bridge to synchronize the robot's pose between Gazebo and Unity

The Unity project would include:
- A robot model that matches the Gazebo simulation
- Scripts to receive pose information from ROS
- Visualization of sensor data (LiDAR point clouds, camera feeds, etc.)

## Running the Complete Example

1. Source ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
source /path/to/workspace/install/setup.bash
```

2. Launch the simulation:
```bash
ros2 launch humanoid_robot_sim humanoid_navigation.launch.py
```

3. In a new terminal, run the navigation controller:
```bash
ros2 run humanoid_robot_sim navigation_controller
```

4. In Unity Hub, open the companion Unity project and click Play to see synchronized visualization

## Learning Outcomes

This complete example demonstrates:
- Physics-accurate simulation in Gazebo
- Sensor simulation with realistic noise models
- Robot navigation using sensor feedback
- Synchronization between physics simulation and visualization
- The complete digital twin workflow

## Troubleshooting

- If the robot doesn't move, check that the `/cmd_vel` topic is connected properly
- If sensors don't work, verify plugin configurations in the URDF
- If Unity visualization is out of sync, check the ROS-Unity bridge connection
- For performance issues, ensure your computer meets the hardware requirements

## Extension Ideas

- Add more complex navigation algorithms (A*, Dijkstra, etc.)
- Implement SLAM using the sensor data
- Add more realistic physics by adjusting friction and damping parameters
- Create more complex environments with dynamic obstacles
- Add machine learning components for adaptive behavior
