# ROS 2 Fundamentals for Humanoid Robotics - Student Guide

## Overview

Welcome to the ROS 2 Fundamentals for Humanoid Robotics module! This comprehensive guide will walk you through the essential concepts of ROS 2 as applied to humanoid robot control systems.

This module covers:
- ROS 2 nodes, topics, services, and actions
- URDF (Unified Robot Description Format) for humanoid robots
- Robot controllers and joint management
- Real-time communication with ≤50ms latency

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Installation](#installation)
3. [Basic Concepts](#basic-concepts)
4. [Publisher/Subscriber Tutorial](#publishersubscriber-tutorial)
5. [Services and Actions Tutorial](#services-and-actions-tutorial)
6. [URDF and Controllers Tutorial](#urdf-and-controllers-tutorial)
7. [Performance and Security](#performance-and-security)
8. [Troubleshooting](#troubleshooting)
9. [Next Steps](#next-steps)

## Prerequisites

Before starting this module, ensure you have:

- A Linux Ubuntu 22.04 LTS system (recommended)
- At least 4GB RAM and 10GB free disk space
- Basic Python programming knowledge
- Understanding of fundamental robotics concepts

## Installation

### Install ROS 2 Humble Hawksbill

```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros Signing.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble packages
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Install additional dependencies for this project
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-controller-manager ros-humble-joint-trajectory-controller
```

### Environment Setup

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your bashrc to auto-source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Basic Concepts

### ROS 2 Architecture

ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

Key concepts:
- **Nodes**: Processes that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Request/response communication pattern
- **Actions**: Goal-oriented communication with feedback
- **Packages**: Containers for ROS 2 functionality

### Communication Patterns

#### Topics (Publishers/Subscribers)
- Asynchronous, decoupled communication
- One-to-many or many-to-one relationships
- Used for continuous data streams (sensors, actuator commands)

#### Services (Request/Response)
- Synchronous communication
- One-to-one relationships
- Used for simple queries and commands

#### Actions (Goals/Feedback/Results)
- Asynchronous communication for long-running tasks
- Provides feedback during execution
- Used for complex tasks requiring progress tracking

## Publisher/Subscriber Tutorial

In this tutorial, you'll learn how to create nodes that communicate using the publish/subscribe pattern.

### Running the Example

1. Build the workspace:
```bash
cd /path/to/physical-ai-humanoid-robotics
colcon build --packages-select publisher_subscriber_tutorial
source install/setup.bash
```

2. Run the publisher in one terminal:
```bash
ros2 run publisher_subscriber_tutorial talker
```

3. Run the subscriber in another terminal:
```bash
source /opt/ros/humble/setup.bash
source /path/to/physical-ai-humanoid-robotics/install/setup.bash
ros2 run publisher_subscriber_tutorial listener
```

### Key Code Elements

The publisher node creates a publisher and timer to send messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1
```

The subscriber node creates a subscription to receive messages:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')
```

## Services and Actions Tutorial

This tutorial demonstrates request/response communication (services) and goal-oriented communication (actions).

### Running the Services Example

1. Run the service server:
```bash
ros2 run services_actions_tutorial add_two_ints_server
```

2. Run the service client:
```bash
ros2 run services_actions_tutorial add_two_ints_client 5 10
```

### Running the Actions Example

1. Run the action server:
```bash
ros2 run services_actions_tutorial fibonacci_action_server
```

2. Run the action client:
```bash
ros2 run services_actions_tutorial fibonacci_action_client
```

### Key Code Elements

Service server:

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response
```

Action server:

```python
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer
import rclpy
from rclpy.node import Node

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal was canceled')
                return Fibonacci.Result()

            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        return result
```

## URDF and Controllers Tutorial

This tutorial covers creating and loading URDF models and connecting Python agents to ROS controllers.

### Running the URDF Example

```bash
ros2 launch urdf_tutorial display.launch.py
```

### Running the Controller Example

```bash
ros2 launch robot_controller controller.launch.py
```

### Key Concepts

#### URDF (Unified Robot Description Format)
URDF is an XML format for representing robot models. Key elements include:

- **Links**: Rigid parts of the robot (base, head, arms, legs)
- **Joints**: Connections between links
- **Visual**: How the robot looks in visualization
- **Collision**: Collision properties for physics simulation
- **Inertial**: Mass properties for physics simulation

#### Robot Controllers
Controllers manage robot joints and actuators. The controller system includes:

- **Joint State Publisher**: Publishes joint position information
- **Controller Manager**: Handles high-level robot commands
- **Robot State Publisher**: Broadcasts transforms for visualization

## Performance and Security

### Performance Requirements
- All messages must be exchanged with ≤50ms latency for real-time control
- System must maintain 99.9% uptime with automatic recovery
- Use Quality of Service (QoS) settings appropriate for your application

### Security Implementation
For safety-critical robot control, implement authentication and encryption:

```bash
# Set security environment variables
export ROS_SECURITY_ROOT_DIRECTORY=/path/to/security/files
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

### Monitoring Tools
Use the provided tools to monitor performance:

1. **Latency Monitor**: Run `python3 src/robot_controller/scripts/performance_monitor.py`
2. **Security Validator**: Run `python3 src/robot_controller/scripts/security_validator.py`
3. **Integration Test**: Run `python3 src/robot_controller/scripts/integration_test.py`

## Troubleshooting

### Common Issues

1. **Nodes not communicating**:
   - Check that nodes are on the same ROS domain ID
   ```bash
   export ROS_DOMAIN_ID=42
   ```

2. **Package not found**:
   - Make sure to source the workspace after building
   ```bash
   cd ~/ros2_workspace
   colcon build
   source install/setup.bash
   ```

3. **Python import errors**:
   - Check that the package is in PYTHONPATH
   ```bash
   python3 -c "import rclpy"  # Test rclpy import
   ```

4. **URDF loading issues**:
   - Verify the URDF syntax
   - Check that all referenced files exist

## Next Steps

1. Explore the complete tutorials in the `/specs/001-ros2-fundamentals/` directory
2. Modify the example code to experiment with different ROS 2 concepts
3. Create your own ROS 2 packages for specific robot applications
4. Integrate with simulation environments like Gazebo
5. Connect to real hardware for physical robot control

### Additional Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS 2 Quality of Service Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)

## Summary

This module has covered the fundamental concepts of ROS 2 for humanoid robotics:

1. **Nodes and Communication**: You learned to create nodes and use topics, services, and actions for communication
2. **URDF**: You created robot models using URDF for visualization and control
3. **Controllers**: You implemented controllers to manage robot joints
4. **Performance**: You ensured ≤50ms latency for real-time control
5. **Security**: You implemented authentication and encryption for safety-critical control

You now have the foundation to build more complex humanoid robot applications using ROS 2!