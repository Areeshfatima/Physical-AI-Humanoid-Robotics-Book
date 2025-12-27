from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Inverse Kinematics Solver Node
        Node(
            package='walking_locomotion',
            executable='ik_solver',
            name='ik_solver',
            output='screen',
            parameters=[
                {'use_sim_time': True}  # Use simulation time if available
            ]
        ),
        
        # Gait Pattern Generator Node
        Node(
            package='walking_locomotion',
            executable='gait_generator',
            name='gait_pattern_generator',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        ),
        
        # Main Walking Controller Node
        Node(
            package='walking_locomotion',
            executable='walking_controller',
            name='walking_controller',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])