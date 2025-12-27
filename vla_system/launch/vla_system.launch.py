from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('vla_system')
    
    # Load parameters file
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')
    
    return LaunchDescription([
        # Whisper Node for speech recognition
        Node(
            package='vla_system',
            executable='whisper_node',
            name='whisper_node',
            output='screen',
            parameters=[params_file] if os.path.exists(params_file) else []
        ),
        
        # LLM Planner Node for cognitive planning
        Node(
            package='vla_system',
            executable='llm_planner',
            name='llm_planner_node',
            output='screen',
            parameters=[params_file] if os.path.exists(params_file) else []
        ),
        
        # Navigation Node for path planning and execution
        Node(
            package='vla_system',
            executable='navigation_node',
            name='navigation_node',
            output='screen',
            parameters=[params_file] if os.path.exists(params_file) else []
        ),
        
        # Perception Node for object detection and localization
        Node(
            package='vla_system',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[params_file] if os.path.exists(params_file) else []
        ),
        
        # VLA Manager Node for coordinating the overall system
        Node(
            package='vla_system',
            executable='vla_manager',
            name='vla_manager',
            output='screen',
            parameters=[params_file] if os.path.exists(params_file) else []
        ),
        
        # Action Executor Node for executing action plans
        Node(
            package='vla_system',
            executable='action_executor',
            name='action_executor',
            output='screen',
            parameters=[params_file] if os.path.exists(params_file) else []
        )
    ])