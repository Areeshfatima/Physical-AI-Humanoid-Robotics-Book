# Data Model: Digital Twin with Gazebo & Unity

## Key Entities

### 1. Physics-Accurate Simulation Environment
- **Description**: Physics-accurate simulation space with configurable gravity, collision models, and joint constraints where robot behavior is tested
- **Fields**:
  - name: String (unique environment name)
  - gravity: Vector3 (gravity vector in m/s², default 0,0,-9.8)
  - collision_engine: String (physics engine type: ODE, Bullet, Simbody)
  - time_step: Float (simulation time step in seconds)
  - real_time_factor: Float (ratio of simulation time to real time)
  - models: List<RobotModel/StaticModel> (models present in the environment)
  - physics_properties: Map<String, Any> (custom physics parameters)
- **Validation rules**:
  - Gravity magnitude must be reasonable (typically 9-10 m/s² on Earth)
  - Time step must be small enough for stability (typically < 0.01s)
  - Environment name must be unique
- **Relationships**: Contains multiple Robot Models and Static Models

### 2. Robot Model (Humanoid)
- **Description**: Robot with human-like physical characteristics including limbs, joints, and sensor configurations that respond to physics simulation
- **Fields**:
  - robot_name: String (unique robot identifier)
  - urdf_path: String (path to URDF description file)
  - joint_configurations: List<Joint> (all joints in the robot)
  - sensor_attachments: List<Sensor> (sensors attached to the robot)
  - base_link: String (name of the base link in the kinematic tree)
  - mass_properties: Map<String, Float> (mass, center of mass, inertia)
  - joint_limits: Map<String, JointLimit> (position, velocity, effort limits)
- **Validation rules**:
  - URDF must be valid and parseable
  - All joint names must be unique
  - Joint limits must be physically plausible
  - Center of mass must be within physical bounds
- **State transitions**: Model loaded → Model spawned → Model simulated → Model destroyed
- **Relationships**: Belongs to a Simulation Environment, contains multiple Joints and Sensors

### 3. Joint
- **Description**: Connection between two links in the robot model with specific kinematic properties
- **Fields**:
  - joint_name: String (unique identifier for the joint)
  - joint_type: String (fixed, revolute, prismatic, continuous, etc.)
  - parent_link: String (name of parent link)
  - child_link: String (name of child link)
  - origin: Pose (position and orientation relative to parent)
  - axis: Vector3 (rotation axis for revolute joints)
  - limits: JointLimit (position, velocity, and effort limits)
  - dynamics: JointDynamics (damping, friction parameters)
- **Validation rules**:
  - Joint type must be supported by the physics engine
  - Position limits must have min < max
  - Parent and child links must exist
- **Relationships**: Belongs to a Robot Model, connects two Links

### 4. Sensor (Simulation)
- **Description**: Virtual sensors that produce realistic data streams for robot perception and navigation
- **Fields**:
  - sensor_name: String (unique identifier for the sensor)
  - sensor_type: String (lidar, depth_camera, imu, etc.)
  - parent_link: String (link to which sensor is attached)
  - pose: Pose (position and orientation relative to parent link)
  - noise: NoiseModel (parameters for sensor noise simulation)
  - update_rate: Float (frequency of sensor updates in Hz)
  - range_min: Float (minimum sensor range, m)
  - range_max: Float (maximum sensor range, m)
  - topic_name: String (ROS topic for sensor data)
- **Validation rules**:
  - Sensor type must be supported by simulation
  - Update rate must be reasonable for real-time performance
  - Range values must be physically plausible
  - Parent link must exist on the robot
- **State transitions**: Disabled → Enabled → Streaming → Disabled
- **Relationships**: Attached to a Robot Model Link, publishes to ROS topic

### 5. Visualization Environment (Unity)
- **Description**: High-fidelity visual environment where exported assets are rendered with enhanced graphics for visualization and demonstration
- **Fields**:
  - scene_name: String (unique scene identifier)
  - rendering_pipeline: String (URP, HDRP, Built-in)
  - lighting_settings: LightingConfig (environmental lighting parameters)
  - materials: List<Material> (materials for visualization)
  - robot_models: List<UnityRobotModel> (3D models for visualization)
  - sensor_visualizers: List<SensorVisualizer> (visualization for sensor data)
  - interaction_modes: List<String> (teleoperation, gesture recognition, etc.)
- **Validation rules**:
  - Rendering pipeline must be compatible with target hardware
  - Scene name must be unique
  - All referenced assets must exist
- **Relationships**: Mirrors a Simulation Environment, contains Unity-specific representations

### 6. Custom Environment
- **Description**: User-defined simulation spaces with specific physical properties, obstacles, and terrain characteristics for testing robot capabilities
- **Fields**:
  - environment_name: String (unique environment identifier)
  - terrain_properties: TerrainConfig (friction, restitution, etc.)
  - obstacles: List<StaticModel> (obstacles in the environment)
  - lighting: LightConfig (environmental lighting for visualization)
  - weather_effects: List<String> (optional environmental effects)
  - boundaries: BoundaryConfig (limits of the simulation space)
- **Validation rules**:
  - Environment name must be unique
  - All obstacles must have valid SDF descriptions
  - Boundaries must be physically plausible
- **Relationships**: Contains multiple Static Models, serves as container for Robot Models

## Relationship Diagram

```
Simulation Environment (1) ─── contains ─── (many) Robot Model
         │                                           │
         │                                    belongs to
         │                                           │
         └────── contains ────── (many) Static Model  │
                                    │                 │
                                    │                 │
                                    └─── has ─── (many) Joint
                                    │                 │
                                    │                 └─── attached to ─── (many) Sensor
                                    │
         │                         │
         │                         │
         └─── mirrored by ─── Visualization Environment (1)
                                    │
                                    └─── contains ─── (many) Unity Robot Model
                                    │
                                    └─── contains ─── (many) Sensor Visualizer
```

## State Transition Diagrams

### Robot Model Lifecycle
```
Model Loaded ── load_urdf() ──→ Model Spawned ── start_simulation() ──→ Model Simulated
      ↑                              │                                         │
      │                              │ destroy_model()                         │ stop_simulation()
      └───────── unload_model() ←────┴─────────────────────────────────────────┘
```

### Sensor State Transitions
```
Disabled ── enable_sensor() ──→ Enabled ── start_streaming() ──→ Streaming
   ↑                              │                                   │
   │                              │ disable_sensor()                  │ stop_streaming()
   └──────── disable_sensor() ←───┴───────────────────────────────────┘
```

## Validation Rules Summary

- Physics deviation must be ≤5% from expected behavior
- Simulation must maintain real-time performance (60 FPS in Unity)
- All URDF/SDF files must be valid and parseable
- Joint limits must be physically plausible
- Sensor noise models must match real-world characteristics
- Coordinate systems must be consistent between Gazebo and Unity
- All referenced assets must exist and be accessible