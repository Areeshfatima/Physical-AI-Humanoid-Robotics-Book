# Data Model: ROS 2 Fundamentals for Physical AI & Humanoid Robotics

## Key Entities

### 1. ROS 2 Package
- **Description**: Container for ROS 2 functionality in ROS 2 Humble Hawksbill distribution including nodes, libraries, and other resources
- **Fields**:
  - name: String (package name following ROS 2 conventions)
  - version: String (semantic versioning)
  - description: String (brief description of package purpose)
  - maintainers: List<String> (maintainer names and emails)
  - dependencies: List<String> (other ROS 2 packages this package depends on)
  - license: String (software license)
  - build_type: String (ament_python, ament_cmake, etc.)
- **Validation rules**: 
  - Name must follow ROS 2 naming conventions (lowercase, underscores)
  - Version must follow semantic versioning (X.Y.Z)
  - Dependencies must be valid ROS 2 packages
- **Relationships**: Contains multiple ROS 2 Nodes, may depend on other Packages

### 2. ROS 2 Node
- **Description**: Process that performs computation and communicates with other nodes using DDS protocols, with automatic failure recovery for 99.9% uptime and detailed logging, metrics, and tracing
- **Fields**:
  - node_name: String (unique name within the ROS graph)
  - namespace: String (optional namespace for organization)
  - publishers: List<Publisher> (list of topic publishers)
  - subscribers: List<Subscriber> (list of topic subscribers)
  - services: List<Service> (list of service servers)
  - clients: List<Client> (list of service clients)
  - actions: List<Action> (list of action servers/clients)
  - parameters: Map<String, Any> (configurable parameters)
- **Validation rules**:
  - Node name must be unique within the namespace
  - Publishers/subscribers must have valid topic names
  - Services/clients must have valid service names
- **State transitions**: Uninitialized → Activated → Deactivated → Finalized
- **Relationships**: Belongs to a Package, communicates with other Nodes

### 3. Topic (Message Bus)
- **Description**: Named bus over which nodes exchange messages in a publish/subscribe pattern with ≤50ms latency for real-time control and with authentication/encryption for security using DDS protocols
- **Fields**:
  - name: String (topic name with proper namespace)
  - data_type: String (message type, e.g., std_msgs/String)
  - qos_profile: QoSProfile (quality of service settings)
  - message_count: Integer (number of published messages)
  - publishers: List<Node> (nodes publishing to this topic)
  - subscribers: List<Node> (nodes subscribed to this topic)
- **Validation rules**:
  - Topic name must follow ROS 2 naming conventions
  - Data type must be a valid ROS 2 message type
  - QoS settings must be compatible between publishers and subscribers
- **Relationships**: Connected to multiple Publishers and Subscribers

### 4. Service
- **Description**: Request/response communication pattern between nodes with acceptable response time and security authentication/encryption, with failure recovery mechanisms and comprehensive observability using DDS protocols
- **Fields**:
  - name: String (service name with proper namespace)
  - data_type: String (service type, e.g., std_srvs/SetBool)
  - qos_profile: QoSProfile (quality of service settings)
  - server: Node (the node providing the service)
  - clients: List<Node> (nodes using the service)
  - response_time: Float (measured response time in seconds)
- **Validation rules**:
  - Service name must follow ROS 2 naming conventions
  - Data type must be a valid ROS 2 service type
  - Response time must meet performance requirements
- **Relationships**: Connected to one Server and multiple Clients

### 5. Action
- **Description**: Goal-oriented communication pattern with feedback and status updates for real-time tasks with security authentication/encryption, with failure recovery mechanisms and comprehensive observability using DDS protocols
- **Fields**:
  - name: String (action name with proper namespace)
  - data_type: String (action type, e.g., example_interfaces/Fibonacci)
  - server: Node (the node providing the action)
  - clients: List<Node> (nodes using the action)
  - goals_in_progress: List<Goal> (currently executing goals)
  - goal_result_queue: List<Result> (completed goal results)
- **Validation rules**:
  - Action name must follow ROS 2 naming conventions
  - Data type must be a valid ROS 2 action type
  - Must handle goal preemption appropriately
- **State transitions**: PENDING → ACTIVE → (SUCCEEDED/CANCELED/ABORTED)
- **Relationships**: Connected to one Server and multiple Clients

### 6. URDF Model (Unified Robot Description Format)
- **Description**: XML-based description of robot physical and kinematic properties
- **Fields**:
  - name: String (robot name)
  - links: List<Link> (rigid parts of the robot)
  - joints: List<Joint> (connections between links)
  - materials: List<Material> (visual materials)
  - gazebo_config: GazeboConfig (simulation-specific configuration)
  - transmissions: List<Transmission> (actuator interface definitions)
- **Validation rules**:
  - Must have a single root link
  - All joints must connect existing links
  - Joint limits must be physically plausible
  - Mass properties must be positive
- **Relationships**: Used by Robot Controllers and Simulation environments

### 7. Robot Controller
- **Description**: Software component that interfaces between high-level commands and hardware with ≤50ms response time and security authentication/encryption, with automatic failure recovery and detailed logging, metrics, and tracing using DDS protocols
- **Fields**:
  - controller_name: String (name of the controller)
  - type: String (controller type, e.g., joint_state_controller, position_controllers/JointGroupPositionController)
  - joints: List<String> (list of controlled joint names)
  - state_interface: List<String> (interface types for state (e.g., position, velocity))
  - command_interface: List<String> (interface types for commands (e.g., position, velocity, effort))
  - state_topic: String (topic for state publishing)
  - command_topic: String (topic for command subscription)
  - response_time: Float (measured response time in seconds)
- **Validation rules**:
  - Controller name must be unique
  - Joints must exist in the robot's URDF model
  - Response time must meet ≤50ms requirement
  - Interface types must be supported by hardware
- **Relationships**: Connected to URDF Model, communicates via Topics and Services

## Relationship Diagram

```
Package (1) ────── contains ────── (many) Node
    │                                    │
    │                             publishes/subscribes
    │                                    │
    │                         ┌──────── Topic (1) ─────── (many) Publisher/Subscriber
    │                         │              │
    │                         │              └───────── (many) Node
    │                         │
    │                         ├──────── Service (1) ─────── (1) Server + (many) Client
    │                         │              │
    │                         │              └───────── (2) Node
    │                         │
    │                         └──────── Action (1) ─────── (1) Server + (many) Client
    │                                        │
    │                                        └───────── (2) Node
    │
    │
    │    contains
    └──────────────── (many) URDF Model (1) ──── uses ──── (many) Robot Controller
                            │                                    │
                            │                                    │
                            └───────── (many) Link/Joint ────────┘
```

## State Transition Diagrams

### ROS 2 Node Lifecycle
```
Uninitialized ── initialize() ──→ Activated ── deactivate() ──→ Deactivated
      ↑                              │                              │
      │                              │ destroy()                    │ destroy()
      └────────── finalize() ←───────┴──────────────────────────────┘
```

### Action State Transitions
```
PENDING ── accept_new_goal() ──→ ACTIVE
   ↑                              │
   │                              │ cancel_goal()
   │                              ▼
CANCEL_REQUESTED ←──────── CANCELING
   │                              │
   │                              │
   └─── (result ready) ←──────────┘

ACTIVE ── (goal succeeds) ──→ SUCCEEDED
   │
   ├── (goal fails) ────────→ ABORTED
   │
   └── (goal canceled) ──────→ CANCELED
```

## Validation Rules Summary

- All ROS 2 communication must maintain ≤50ms latency
- All communication must implement security authentication/encryption
- System must maintain 99.9% uptime with automatic failure recovery
- All components must implement comprehensive observability (logging, metrics, tracing)
- All ROS 2 naming conventions must be followed
- All URDF models must have physically valid properties
- All message types must be compatible between communicating nodes