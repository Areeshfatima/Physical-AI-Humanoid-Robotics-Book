---
title: ROS2 Launch Systems and Package Management
sidebar_position: 5
description: Managing complex systems and organizing ROS2 projects effectively
keywords:
  - ROS2
  - launch
  - packages
  - management
  - system
  - configuration
id: chapter-4
---






# ROS2 Launch Systems and Package Management

## Learning Objectives

After completing this chapter, you should be able to:
- Organize ROS2 projects using proper package structure
- Create and configure launch files for complex systems
- Implement node composition for efficient resource usage
- Apply best practices for system configuration and deployment

## Introduction

As robotic systems become more complex, managing multiple nodes, their configurations, and their interactions becomes increasingly challenging. ROS2 provides sophisticated tools for system management and package organization that enable the development of robust, scalable robotic applications.

## Package Management

### Package Structure and Organization

ROS2 packages are the basic building units of ROS2 applications. Each package should have a clear, single purpose and contain related functionality.

#### Key Components of a ROS2 Package

1. **package.xml**: Manifest file containing package metadata
2. **CMakeLists.txt**: Build instructions for C++ packages
3. **setup.py**: Build instructions for Python packages
4. **src/**: Source code files
5. **include/**: Header files (for C++)
6. **launch/**: Launch files for starting nodes
7. **config/**: Configuration files
8. **test/**: Test files
9. **scripts/**: Executable scripts

### Creating a Package

Using the `ros2 pkg create` command:

```bash
ros2 pkg create --build-type ament_python my_robot_package
# or
ros2 pkg create --build-type ament_cmake my_robot_package
```

### Package.xml Manifest

The package.xml file contains important metadata:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.0</version>
  <description>Package for my robot functionality</description>
  <maintainer email="user@example.com">User Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Dependencies Management

Dependencies are declared in both package.xml and CMakeLists.txt (or setup.py for Python packages):

**In package.xml:**
- `buildtool_depend`: Build system dependencies
- `depend`: Runtime dependencies
- `test_depend`: Testing dependencies

**In CMakeLists.txt:**
```cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_export_dependencies(rclcpp std_msgs)
```

## Launch Systems

### Introduction to Launch Files

Launch files allow you to start multiple nodes with specific configurations in a coordinated manner. ROS2 uses Python-based launch files that offer powerful features for complex system management.

### Basic Launch File Structure

A simple launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='teleop'
        )
    ])
```

![Launch System Workflow Diagram](./images/launch-workflow.png)
*Figure 4.1: ROS2 launch system workflow showing how nodes are started and configured*

### Advanced Launch Concepts

#### Launch Arguments

Launch files can accept arguments to customize behavior:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    turtlesim_ns = DeclareLaunchArgument(
        'turtlesim_ns',
        default_value='turtlesim1',
        description='Namespace for turtlesim node'
    )
    
    # Use launch configuration in node definition
    launch_args = LaunchConfiguration('turtlesim_ns')
    
    return LaunchDescription([
        turtlesim_ns,
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            namespace=launch_args
        )
    ])
```

#### Conditional Launch

Execute nodes conditionally based on arguments:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    return LaunchDescription([
        sim_time,
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        )
    ])
```

#### Including Other Launch Files

Modularize systems by including other launch files:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_bringup'),
                '/launch/turtlebot3_remote.launch.py'
            ])
        )
    ])
```

### Composition

ROS2 supports component-based composition to run multiple nodes in a single process, reducing overhead and improving performance:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='image_tools',
                plugin='image_tools::Cam2Image',
                name='cam2image'
            ),
            ComposableNode(
                package='image_tools',
                plugin='image_tools::ShowImage',
                name='showimage'
            )
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

## Managing Complex Systems

### Parameter Management

Organize configuration using parameter files:

**config/params.yaml**
```yaml
robot_controller:
  ros__parameters:
    kp: 1.0
    ki: 0.1
    kd: 0.05
    max_velocity: 1.0
    max_acceleration: 0.5
```

Load in launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_package'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='controller_node',
            name='robot_controller',
            parameters=[config]
        )
    ])
```

### Namespacing and Remapping

Organize complex systems with namespaces and remappings:

```python
Node(
    package='navigation2',
    executable='nav2_map_server',
    name='map_server',
    namespace='robot1',
    remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
)
```

## Workspaces and Development

### Creating a Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### Build Systems

ROS2 supports multiple build systems:

1. **ament_cmake**: For C++ packages using CMake
2. **ament_python**: For Python packages
3. **ament_cmake_python**: For packages containing both C++ and Python code

### Build Commands

```bash
# Build specific packages
colcon build --packages-select my_package

# Build with verbose output
colcon build --event-handlers console_direct+

# Build with tests
colcon build --packages-select my_package --cmake-args -DBUILD_TESTING=ON
```

## Best Practices

### Package Development

1. **Modularity**: Keep packages focused and single-purpose
2. **Dependencies**: Minimize dependencies to reduce complexity
3. **Documentation**: Document packages, nodes, and interfaces thoroughly
4. **Testing**: Include tests for all packages

### Launch File Best Practices

1. **Organization**: Group related functionality in logical launch files
2. **Flexibility**: Use arguments to make launch files configurable
3. **Clarity**: Use descriptive names for nodes and parameters
4. **Modularity**: Break complex systems into smaller, reusable launch files

### System Management

1. **Configuration**: Separate configuration from code using parameter files
2. **Namespacing**: Use namespaces to avoid naming conflicts in multi-robot systems
3. **Monitoring**: Include monitoring and debugging tools in launch files
4. **Resource Management**: Consider resource usage when designing system architecture

## Practical Example: Navigation Stack Launch

Here's a practical example combining concepts for a navigation system:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration('params_file')
    
    # Set global parameters
    ld = LaunchDescription([
        SetParameter('use_sim_time', use_sim_time),
    ])
    
    # Navigation launch file
    nav2_bringup_launch_dir = get_package_share_directory('nav2_bringup')
    
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file
        }.items()
    )
    
    ld.add_action(nav_bringup)
    
    return ld
```

## Summary

ROS2 launch systems and package management provide powerful tools for organizing and managing complex robotic applications. Understanding these concepts is essential for developing maintainable, scalable robotic systems.

Package management allows for modular, well-organized code, while launch systems enable coordinated startup of complex systems with configurable parameters. Together, these tools form the foundation for professional robotic application development that builds upon the [communication patterns](./chapter-3.md) and [node architecture](./chapter-2.md) learned in previous chapters.

With this knowledge of ROS2 fundamentals, you're now equipped to develop sophisticated robotic applications using the ROS2 framework.

## Exercises

1. Create a launch file that starts multiple nodes from the ROS2 tutorials (e.g., turtlesim nodes).
2. Modify the launch file to accept parameters that change node behavior.
3. Research and implement a simple node composition example using the component container.

[Next: Module 2](../../module-2/chapter-1.md) | [Previous: ROS2 Services and Actions](./chapter-3.md)