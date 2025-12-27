import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = 'urdf_tutorial'
    share_dir = get_package_share_directory(package_name)

    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(share_dir, 'urdf', 'simple_humanoid.urdf'),
        description='Path to the URDF file'
    )

    # URDF file path
    model_path = LaunchConfiguration('model')

    # Read the URDF file
    urdf_file = open(os.path.join(share_dir, 'urdf', 'simple_humanoid.urdf')).read()

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}]
    )

    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    # Joint state publisher GUI node (for interactive joint control)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])