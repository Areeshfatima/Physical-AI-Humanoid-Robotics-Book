#!/bin/bash
# ROS 2 Package Creation Script for Humanoid Robot Control
# This script will create a basic ROS 2 package for humanoid robot control

echo "Creating a basic ROS 2 package for humanoid robot control..."

# Navigate to the workspace
cd ~/humanoid_ws/src

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create the humanoid robot control package
ros2 pkg create --license Apache-2.0 --build-type ament_python humanoid_robot_control

# Create the main control node file
cat << EOF > ~/humanoid_ws/src/humanoid_robot_control/humanoid_robot_control/main_controller.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')
        
        # Publisher for joint commands
        self.joint_cmd_publisher = self.create_publisher(Float32MultiArray, 'joint_commands', 10)
        
        # Subscriber for control commands
        self.command_subscriber = self.create_subscription(
            String,
            'control_commands',
            self.command_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)  # 10Hz control loop
        
        self.get_logger().info('Humanoid Controller initialized')


    def command_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.data}')
        # Process the command and update internal state for control loop


    def control_loop(self):
        # This is where the main control logic would go
        # For now, just publish dummy joint commands
        joints_msg = Float32MultiArray()
        joints_msg.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Placeholder values
        self.joint_cmd_publisher.publish(joints_msg)


def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Humanoid Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF

# Make the main controller executable
chmod +x ~/humanoid_ws/src/humanoid_robot_control/humanoid_robot_control/main_controller.py

# Create setup.py
cat << EOF > ~/humanoid_ws/src/humanoid_robot_control/setup.py
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_robot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[py|xml|yaml]')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='openhack',
    maintainer_email='openhack@todo.todo',
    description='Basic package for humanoid robot control',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_controller = humanoid_robot_control.main_controller:main',
        ],
    },
)
EOF

# Create package.xml
cat << EOF > ~/humanoid_ws/src/humanoid_robot_control/package.xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_robot_control</name>
  <version>0.0.0</version>
  <description>Basic package for humanoid robot control</description>
  <maintainer email="openhack@todo.todo">openhack</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <exec_depend>ros2launch</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF

echo "Basic ROS 2 package for humanoid robot control created!"
echo "Package location: ~/humanoid_ws/src/humanoid_robot_control"
echo "Main controller file: ~/humanoid_ws/src/humanoid_robot_control/humanoid_robot_control/main_controller.py"