# Quickstart Guide: ROS 2 Fundamentals for Physical AI & Humanoid Robotics

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS (recommended)
- At least 4GB RAM
- 10GB free disk space
- ROS 2 Humble Hawksbill installed with DDS packages

### Installation

#### Install ROS 2 Humble Hawksbill

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
```

#### Environment Setup

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Add to your bashrc to auto-source
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## Getting Started with the Tutorials

### 1. Publisher/Subscriber Tutorial

This tutorial demonstrates the basic publish/subscribe communication pattern in ROS 2.

#### Create the package:
```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace/src
ros2 pkg create --build-type ament_python publisher_subscriber_tutorial
cd publisher_subscriber_tutorial
```

#### Run the publisher:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 run publisher_subscriber_tutorial talker
```

#### Run the subscriber in a new terminal:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 run publisher_subscriber_tutorial listener
```

### 2. Services and Actions Tutorial

This tutorial demonstrates request/response communication (services) and goal-oriented communication (actions).

#### Run the service server:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 run services_actions_tutorial add_two_ints_server
```

#### Run the service client:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 run services_actions_tutorial add_two_ints_client
```

#### Run the action server:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 run services_actions_tutorial fibonacci_action_server
```

#### Run the action client:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 run services_actions_tutorial fibonacci_action_client
```

### 3. URDF Tutorial

This tutorial shows how to create and visualize robot models using URDF.

#### Launch the URDF viewer:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 launch urdf_tutorial display.launch.py model:=install/urdf_tutorial/share/urdf_tutorial/urdf/simple_humanoid.urdf
```

### 4. Robot Controller Tutorial

This tutorial demonstrates how to control robot joints using ROS 2 controllers.

#### Launch the controller:
```bash
cd ~/ros2_workspace
source install/setup.bash
ros2 launch robot_controller controller.launch.py
```

## Understanding ROS 2 Concepts

### Nodes, Topics, Services, and Actions

#### Nodes
A node is a process that performs computation. In ROS 2, nodes are designed to be lightweight and modular.

**Creating a node in Python:**
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Add publishers, subscribers, services, etc. here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Topics (Publishers and Subscribers)
Topics enable publish/subscribe communication. Publishers send data, and subscribers receive data.

```python
# Publisher
publisher = self.create_publisher(String, 'topic_name', 10)
msg = String()
msg.data = 'Hello World'
publisher.publish(msg)

# Subscriber
def topic_callback(self, msg):
    self.get_logger().info('Received: %s' % msg.data)

subscription = self.create_subscription(
    String,  # Message type
    'topic_name',  # Topic name
    topic_callback,  # Callback function
    10  # QoS profile
)
```

#### Services (Request/Response)
Services provide request/response communication pattern.

```python
# Service Server
from example_interfaces.srv import AddTwoInts

def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
    return response

service = self.create_service(AddTwoInts, 'add_two_ints', add_two_ints_callback)

# Service Client
client = self.create_client(AddTwoInts, 'add_two_ints')
```

#### Actions (Goal/Feedback/Result)
Actions provide goal-oriented communication with feedback and status updates.

```python
# Action Server
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer

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

## URDF Basics

URDF (Unified Robot Description Format) is an XML format for representing robot models:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.5"/>
      </geometry>
    </visual>
  </link>

  <!-- Head link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Joint connecting base to head -->
  <joint name="base_to_head" type="fixed">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.4"/>
  </joint>
</robot>
```

## Performance and Security

### Performance Monitoring
To monitor message latency and performance:
```bash
# Record topics to analyze timing
ros2 bag record /topic_name

# Monitor node status
ros2 node list
ros2 lifecycle list <node_name>
```

### Security
For security implementation, ROS 2 provides security enclaves:
```bash
# Set security environment variables
export ROS_SECURITY_ROOT_DIRECTORY=/path/to/security/files
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

## Troubleshooting

### Common Issues

1. **Nodes not communicating**: Check that nodes are on the same ROS domain ID
   ```bash
   # Set domain ID explicitly
   export ROS_DOMAIN_ID=42
   ```

2. **Package not found**: Make sure to source the workspace after building
   ```bash
   cd ~/ros2_workspace
   colcon build
   source install/setup.bash
   ```

3. **Python import errors**: Check that the package is in PYTHONPATH
   ```bash
   python3 -c "import rclpy"  # Test rclpy import
   ```

## Next Steps

1. Explore the complete tutorials in the `/specs/001-ros2-fundamentals/` directory
2. Modify the example code to experiment with different ROS 2 concepts
3. Create your own ROS 2 packages for specific robot applications
4. Integrate with simulation environments like Gazebo
5. Connect to real hardware for physical robot control

## Resources

- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Python Client Library (rclpy)](https://docs.ros.org/en/humble/p/rclpy/)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ROS 2 Quality of Service Settings](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)