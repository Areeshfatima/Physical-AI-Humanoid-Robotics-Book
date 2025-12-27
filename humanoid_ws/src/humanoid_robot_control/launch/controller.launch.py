from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_robot_control',
            executable='main_controller',
            name='humanoid_controller',
            output='screen',
            parameters=[]
        )
    ])
