# Research: Digital Twin with Gazebo & Unity

## Key Decisions Made

### 1. Gazebo Classic vs Ignition (Now called Garden)

**Decision**: Gazebo Harmonic (formerly Ignition) for new development

**Rationale**:
- Gazebo Classic is in maintenance mode and will be deprecated
- Gazebo Harmonic (the latest Ignition release) has better performance and modern architecture
- Better support for real-time physics simulation with improved collision detection
- Active development and community support

**Alternatives considered**:
- Gazebo Classic: Stable but deprecated, no new feature development
- Gazebo Fortress: LTS version but older than Harmonic

### 2. URDF vs SDF for Robot Models

**Decision**: URDF for robot models with SDF for world environments

**Rationale**:
- URDF is the standard format for robot models in ROS ecosystem
- Better support for articulated robots with joint definitions
- Extensive tooling and visualization support in RViz/Gazebo
- SDF is better for world description with complex environments
- Can convert between formats as needed

**Alternatives considered**:
- Pure SDF approach: Would work but lose ROS tooling benefits
- Custom format: Would require developing new tooling

### 3. Unity Rendering Pipeline (HDRP vs URP vs Built-in)

**Decision**: URP (Universal Render Pipeline) for educational purposes

**Rationale**:
- Lower hardware requirements making it accessible to more students
- Good performance-to-quality balance for educational simulations
- Easier to learn and implement for educational content
- Better performance for real-time simulation visualization
- Good support for VR if needed for immersive learning

**Alternatives considered**:
- HDRP: Higher quality but requires high-end hardware
- Built-in: Simpler but being phased out by Unity

### 4. ROS-Unity Bridge Options

**Decision**: Unity Robotics Hub (ROS-TCP-Connector + Simulation Assets)

**Rationale**:
- Official ROS-Unity integration maintained by Unity
- Good documentation and examples for robotics simulation
- Supports both ROS 1 and ROS 2
- Includes pre-built assets for common robotics tasks
- Active development and community support

**Alternatives considered**:
- Custom TCP/UDP bridge: More control but more development work
- ROS# library: Good but less actively maintained
- Unity ML-Agents with ROS integration: More complex than needed

## Technical Research Findings

### Gazebo Physics Engine Overview

**Gazebo Harmonic (Ignition)** provides:
- Modern physics engine with ODE, Bullet, Simbody support
- Realistic gravity, collision detection, and joint constraints
- Plugin architecture for custom sensors and controllers
- Integration with ROS 2 via gazebo_ros_pkgs

**Physics Parameters**:
- Gravity: Configurable (default 9.8 m/s²)
- Collision detection: Supports various shapes (box, sphere, cylinder, mesh)
- Joint types: Fixed, revolute, prismatic, continuous, etc.

### World and Robot Spawning

**World Management**:
- SDF files define complete simulation environments
- Can include multiple robots in the same world
- Physics engine parameters configurable per world
- Support for environmental effects (wind, friction)

**Robot Spawning**:
- URDF models converted to SDF for simulation
- Custom plugins can be attached to robots
- Multiple spawn points and configurations possible

### Sensor Simulation

**LiDAR Simulation**:
- Ray-based sensor model with configurable resolution
- Support for various LiDAR types (2D, 3D)
- Noise modeling for realistic data simulation
- Output formats compatible with ROS 2 sensor_msgs

**Depth Camera Simulation**:
- Realistic depth estimation with configurable noise
- RGB-D output with point cloud generation
- Support for various field-of-view settings
- Performance considerations for real-time applications

**IMU Simulation**:
- 6-axis simulation (acceleration and angular velocity)
- Configurable noise models matching real sensors
- Orientation estimation using magnetometer integration
- Support for various IMU types and configurations

### Unity High-Fidelity Rendering

**URP Pipeline Capabilities**:
- Real-time rendering with configurable quality settings
- Support for various lighting models and effects
- Good performance for educational simulations
- VR/AR support for immersive experiences

**Asset Integration**:
- Import of 3D models from various formats
- Material and texture management
- Animation and kinematic visualization
- Real-time rendering of sensor data overlays

## Validation Approach

### Quality Validation via Physics Accuracy

**Testing Strategy**:
1. Physics behavior validation: Compare simulated robot movements with theoretical physics
2. Sensor output validation: Verify sensor data matches expected real-world values
3. Unity scene consistency: Ensure visualization matches simulation state
4. Performance validation: Monitor frame rates and simulation accuracy

**Accuracy Metrics**:
- Physics deviation: ≤5% from expected behavior (as specified in success criteria)
- Frame rate: 60 FPS minimum for smooth visualization
- Sensor noise: Realistic models matching actual sensor characteristics

## Architecture Sketch

### Digital Twin Architecture: Gazebo Physics → Unity Visualization

```
Gazebo Harmonic
├── Physics Engine (ODE/Bullet)
├── Robot Models (URDF → SDF)
├── Sensor Plugins (LiDAR, Depth Camera, IMU)
├── ROS Integration (gazebo_ros_pkgs)
└── World Environments (SDF)

ROS 2 Bridge
├── Sensor Data Publishing
├── Robot State Broadcasting
└── Command Interface

Unity URP Project
├── 3D Model Import/Visualization
├── Real-time Rendering
├── Sensor Data Visualization
└── Human-Robot Interaction Interface
```

### Simulation Workflow

```
1. Create Gazebo World (SDF) → Define physics properties
2. Load Robot Model (URDF) → Convert to SDF for simulation
3. Attach Sensors (LiDAR, Depth, IMU) → Configure sensor plugins
4. Start Simulation → Run physics engine with real-time constraints
5. Bridge to Unity → Send state via ROS-Unity connector
6. Visualize in Unity → Render high-fidelity representation
7. Human Interaction → Allow teleoperation and gesture recognition
```

## Research Tasks Completed

1. **Gazebo Harmonic Installation and Setup**: Verified installation process and basic physics simulation
2. **Unity URP Pipeline Configuration**: Researched optimal settings for educational simulations
3. **URDF/SDF Conversion Workflow**: Researched tools for converting between formats
4. **ROS-Unity Bridge Implementation**: Researched Unity Robotics Hub integration
5. **Sensor Simulation Best Practices**: Researched realistic sensor noise models
6. **Performance Optimization**: Researched techniques for maintaining 60 FPS in Unity
7. **Human-Robot Interaction Patterns**: Researched teleoperation and gesture recognition implementations

## Next Steps for Implementation

1. Create foundational Gazebo worlds with physics-accurate environments
2. Develop robot models with appropriate joint configurations
3. Integrate sensor plugins for LiDAR, depth cameras, and IMUs
4. Implement ROS-Unity bridge for real-time visualization
5. Create Unity visualization with URP pipeline
6. Develop human-robot interaction interfaces for teleoperation
7. Validate physics accuracy with ≤5% deviation requirement