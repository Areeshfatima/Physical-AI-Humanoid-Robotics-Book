from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import EmitEvent
from launch.events import Shutdown
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='humanoid_lab.sdf',
        description='Choose one of the world files from `/isaac_worlds/worlds`'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )

    # Get world path
    world_file = PathJoinSubstitution([
        FindPackageShare('isaac_worlds'),
        'scenes',  # Changed from 'worlds' to 'scenes' to match our structure
        LaunchConfiguration('world')
    ])

    # Get package directories
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # Gazebo launch - using Isaac Sim launch files
    # Note: This is a simplified example as Isaac Sim has its own launch mechanisms
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'pause': 'false'
        }.items()
    )

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('isaac_worlds'),
                    'models',
                    'humanoid_robot.urdf'  # Using the URDF we created
                )
            ).read()
        }]
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Physics accuracy validator
    physics_validator = Node(
        package='isaac_ros_integration',  # Changed to match our directory structure
        executable='physics_accuracy_validator',
        name='physics_validator',
        output='screen'
    )

    # RViz2 node (conditionally launched)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('isaac_worlds'),
        'rviz',
        'humanoid_navigation.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Return the launch description
    return LaunchDescription([
        world_arg,
        use_rviz_arg,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        physics_validator,
        rviz
    ])