# Quickstart Guide: Digital Twin with Gazebo & Unity

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended) or Windows 10/11
- At least 8GB RAM (16GB recommended for Unity)
- 20GB free disk space
- Dedicated GPU with OpenGL 4.3+ support
- ROS 2 Humble Hawksbill installed

### Software Dependencies
- Gazebo Harmonic (or compatible Ignition version)
- Unity Hub with Unity 2022.3 LTS or newer
- Unity Universal Render Pipeline (URP) package
- ROS 2 Humble Hawksbill with ros-gz bridge
- Unity Robotics Simulation package
- Git LFS for large asset tracking

## Installation

### 1. Install Gazebo Harmonic
```bash
# For Ubuntu 22.04
sudo apt update
sudo apt install ignition-harmonic # Or latest Ignition release

# Verify installation
ign gazebo --version
```

### 2. Install Unity Hub and Project
```bash
# Download Unity Hub from Unity website
# Install Unity 2022.3 LTS through Unity Hub
# In Unity Hub, install URP package through Package Manager
```

### 3. Install ROS 2 Integration
```bash
# Install ROS 2 Humble and required packages
sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-ros-gz
sudo apt install python3-colcon-common-extensions

# Install Unity ROS-TCP-Connector (if not using Robotics Hub)
git clone https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

## Getting Started with the Simulation

### 1. Physics Simulation (Gazebo)

Clone and build the simulation environment:

```bash
# Navigate to your workspace
cd ~/ros2_workspace/src
git clone [repository-url]
cd ~/ros2_workspace

# Build the packages
colcon build --packages-select gazebo_worlds sensors_simulation ros_integration
source install/setup.bash
```

#### Launch a Basic Physics Simulation:
```bash
# Launch a humanoid robot in a basic environment
ros2 launch ros_integration basic_humanoid_simulation.launch.py
```

### 2. Sensor Simulation

The simulation includes various sensor types:

#### LiDAR Configuration:
```bash
# Launch with LiDAR sensor
ros2 launch ros_integration lidar_sensor_simulation.launch.py

# View LiDAR data in RViz
ros2 run rviz2 rviz2 -d ~/ros2_workspace/src/gazebo_worlds/config/lidar_view.rviz
```

#### Depth Camera Configuration:
```bash
# Launch with depth camera
ros2 launch ros_integration depth_camera_simulation.launch.py

# View depth data
ros2 run image_view image_view _image_topic:=/depth_camera/image_raw
```

#### IMU Configuration:
```bash
# Launch with IMU sensors
ros2 launch ros_integration imu_sensor_simulation.launch.py

# View IMU data
ros2 topic echo /imu/data
```

### 3. Unity Visualization

#### Setting up the Unity Project:
1. Open the Unity project from `unity_project/` directory
2. Install required packages via Window → Package Manager
   - Unity Transport
   - Unity Robotics Simulator
   - ROS TCP Connector
3. Open the main scene from `Assets/Scenes/MainSimulation.unity`
4. Configure the ROS TCP Connector settings (typically localhost:10000)

#### Running the Visualization:
1. Start the Gazebo simulation first:
   ```bash
   ros2 launch ros_integration complete_simulation.launch.py
   ```
2. Click "Play" in Unity to start visualization
3. The Unity environment will mirror the Gazebo physics simulation in real-time

#### Human-Robot Interaction:
- **Teleoperation**: Use WASD keys to move the robot, mouse to look around
- **Gesture Recognition**: Available through the interaction interface
- **Sensor Data Overlay**: Press 'S' to toggle sensor data visualization

## Understanding the Digital Twin Architecture

### Gazebo Physics Engine
Gazebo provides the physics-accurate simulation backend:

**Core Components**:
- Physics engine (ODE/Bullet) for realistic movement
- Collision detection for object interactions
- Joint constraints for robot kinematics
- Sensor plugins for realistic data generation

**Creating a New World**:
```xml
<!-- Example SDF world file -->
<sdf version='1.7'>
  <world name='basic_world'>
    <physics type='ode'>
      <gravity>0 0 -9.8</gravity>
    </physics>
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>
  </world>
</sdf>
```

### Unity High-Fidelity Rendering
Unity provides the visualization layer:

**Key Components**:
- URP pipeline for efficient rendering
- 3D models matching the Gazebo simulation
- Real-time rendering of sensor data
- Interactive elements for human-robot interaction

**Example Unity C# Script for Robot Control**:
```csharp
using UnityEngine;

public class RobotController : MonoBehaviour
{
    public float moveSpeed = 5.0f;
    public float turnSpeed = 100.0f;
    
    void Update()
    {
        // Get input
        float moveInput = Input.GetAxis("Vertical");
        float turnInput = Input.GetAxis("Horizontal");
        
        // Move the robot
        transform.Translate(Vector3.forward * moveInput * moveSpeed * Time.deltaTime);
        transform.Rotate(Vector3.up, turnInput * turnSpeed * Time.deltaTime);
    }
}
```

## Performance and Validation

### Physics Accuracy Validation
To validate physics behavior:

```bash
# Monitor physics deviation
ros2 run gazebo_worlds validate_physics_accuracy.py

# Check robot movement accuracy against expected physics
ros2 run ros_integration compare_trajectory.py
```

### Unity Rendering Performance
- Monitor frame rate in Unity (Window → Analysis → Profiler)
- Target: 60 FPS minimum for smooth visualization
- Reduce draw distance if performance is low
- Use occlusion culling for large environments

## Troubleshooting

### Common Issues

1. **Simulation running slowly**:
   ```bash
   # Check real-time factor
   ign gazebo -v 4
   
   # Optimize physics parameters
   # Reduce update rates for sensors if needed
   ```

2. **Unity visualization not syncing with Gazebo**:
   - Verify ROS TCP Connector is running on both ends
   - Check network settings and firewall
   - Confirm same coordinate systems in both environments

3. **Sensor data not publishing**:
   ```bash
   # List active topics
   ros2 topic list
   
   # Check sensor status
   ros2 run rqt_plot rqt_plot
   ```

4. **Robot models not loading**:
   - Verify URDF/SDF files are valid
   - Check model paths in Gazebo
   - Ensure ROS 2 packages are sourced

## Next Steps

1. Explore the complete tutorials in the `/specs/003-digital-twin-gazebo-unity/` directory
2. Modify the example worlds and robot models to experiment with different physics scenarios
3. Create your own sensor configurations for specific use cases
4. Extend the Unity visualization with custom interfaces
5. Connect to real hardware for physical-digital twin applications

## Resources

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [ROS 2 with Gazebo Guide](https://classic.gazebosim.org/tutorials?tut=ros2_integration)
- [Unity URP Documentation](https://docs.unity3d.com/Packages/com.unity.render-pipelines.universal@latest)