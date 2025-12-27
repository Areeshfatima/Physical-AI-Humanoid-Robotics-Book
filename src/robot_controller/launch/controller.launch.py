from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Create launch description
    ld = LaunchDescription()
    
    # Joint state publisher node
    joint_publisher_node = Node(
        package='robot_controller',
        executable='joint_publisher',
        name='joint_publisher',
        output='screen'
    )
    
    # Controller manager node
    controller_manager_node = Node(
        package='robot_controller',
        executable='controller_manager',
        name='controller_manager',
        output='screen'
    )
    
    # Robot state publisher node to broadcast transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('urdf_tutorial'),
                    'urdf',
                    'humanoid_with_joints.urdf'
                )
            ).read()
        }]
    )
    
    # Add nodes to launch description
    ld.add_action(joint_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(robot_state_publisher_node)
    
    return ld