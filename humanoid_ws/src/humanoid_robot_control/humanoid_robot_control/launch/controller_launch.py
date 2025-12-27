import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='humanoid_robot_control',
            executable='main_controller',
            name='humanoid_controller',
            output='screen',
            parameters=[]
        )
    ])