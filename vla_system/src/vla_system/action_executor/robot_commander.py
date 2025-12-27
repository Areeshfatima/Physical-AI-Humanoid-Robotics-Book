"""Robot commander for the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from action_msgs.msg import GoalStatus
import json
from enum import Enum
from typing import Dict, Any, Optional


class RobotCommander(Node):
    """Commander class that interfaces with robot hardware/simulation."""
    
    def __init__(self, node_name: str = 'robot_commander'):
        super().__init__(node_name)
        
        # Get parameters
        self.declare_parameter('action_execution_timeout', 30.0)
        self.declare_parameter('action_retry_attempts', 3)
        
        self.action_timeout = self.get_parameter('action_execution_timeout').value
        self.max_retry_attempts = self.get_parameter('action_retry_attempts').value
        
        # Track active commands
        self.active_goals = {}
        self.command_history = []
        
        # Create publishers for robot commands
        qos_profile = QoSProfile(depth=10)
        
        # Publisher for sending commands to robot
        self.command_publisher = self.create_publisher(
            String,
            'robot_commands',
            qos_profile
        )
        
        # Publisher for navigation goals
        self.nav_goal_publisher = self.create_publisher(
            PoseStamped,
            'navigation_goals',
            qos_profile
        )
        
        # Subscriber for command responses
        self.command_response_subscriber = self.create_subscription(
            String,
            'command_responses',
            self.command_response_callback,
            qos_profile
        )
        
        self.get_logger().info(
            f'Robot Commander initialized with timeout: {self.action_timeout}s, '
            f'max retries: {self.max_retry_attempts}'
        )
    
    def send_navigation_command(self, target_position: Dict[str, float], relative: bool = False):
        """Send a navigation command to the robot."""
        try:
            cmd_data = {
                'type': 'navigation',
                'target_position': target_position,
                'relative': relative,
                'timestamp': self.get_clock().now().to_msg().nanosec
            }
            
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.command_publisher.publish(cmd_msg)
            
            cmd_id = str(cmd_data['timestamp'])
            self.active_goals[cmd_id] = {
                'type': 'navigation',
                'target': target_position,
                'start_time': self.get_clock().now().nanoseconds / 1e9,
                'retries': 0
            }
            
            self.get_logger().info(f'Sent navigation command to {target_position}')
            
            return cmd_id
            
        except Exception as e:
            self.get_logger().error(f'Error sending navigation command: {str(e)}')
            return None
    
    def send_perception_command(self, object_type: str = 'any', search_area: str = 'forward_cone'):
        """Send a perception command to the robot."""
        try:
            cmd_data = {
                'type': 'perception',
                'object_type': object_type,
                'search_area': search_area,
                'timestamp': self.get_clock().now().to_msg().nanosec
            }
            
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.command_publisher.publish(cmd_msg)
            
            cmd_id = str(cmd_data['timestamp'])
            self.active_goals[cmd_id] = {
                'type': 'perception',
                'object_type': object_type,
                'start_time': self.get_clock().now().nanoseconds / 1e9,
                'retries': 0
            }
            
            self.get_logger().info(f'Sent perception command for {object_type} objects')
            
            return cmd_id
            
        except Exception as e:
            self.get_logger().error(f'Error sending perception command: {str(e)}')
            return None
    
    def send_manipulation_command(self, action: str, target_object: Optional[Dict[str, Any]] = None):
        """Send a manipulation command to the robot."""
        try:
            cmd_data = {
                'type': 'manipulation',
                'action': action,
                'target_object': target_object or {},
                'timestamp': self.get_clock().now().to_msg().nanosec
            }
            
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.command_publisher.publish(cmd_msg)
            
            cmd_id = str(cmd_data['timestamp'])
            self.active_goals[cmd_id] = {
                'type': 'manipulation',
                'action': action,
                'target_object': target_object,
                'start_time': self.get_clock().now().nanoseconds / 1e9,
                'retries': 0
            }
            
            self.get_logger().info(f'Sent manipulation command: {action}')
            
            return cmd_id
            
        except Exception as e:
            self.get_logger().error(f'Error sending manipulation command: {str(e)}')
            return None
    
    def send_wait_command(self, duration: float):
        """Send a wait command to the robot."""
        try:
            cmd_data = {
                'type': 'wait',
                'duration': duration,
                'timestamp': self.get_clock().now().to_msg().nanosec
            }
            
            cmd_msg = String()
            cmd_msg.data = json.dumps(cmd_data)
            self.command_publisher.publish(cmd_msg)
            
            cmd_id = str(cmd_data['timestamp'])
            self.active_goals[cmd_id] = {
                'type': 'wait',
                'duration': duration,
                'start_time': self.get_clock().now().nanoseconds / 1e9,
                'retries': 0
            }
            
            self.get_logger().info(f'Sent wait command for {duration}s')
            
            return cmd_id
            
        except Exception as e:
            self.get_logger().error(f'Error sending wait command: {str(e)}')
            return None
    
    def command_response_callback(self, msg: String):
        """Callback for handling command responses from the robot."""
        try:
            response_data = json.loads(msg.data)
            
            cmd_id = response_data.get('command_id')
            status = response_data.get('status')
            result = response_data.get('result')
            
            # Update active goal status
            if cmd_id in self.active_goals:
                goal = self.active_goals[cmd_id]
                
                if status == 'completed':
                    self.get_logger().info(f'Command {cmd_id} completed successfully')
                    # Remove from active goals
                    del self.active_goals[cmd_id]
                    
                    # Add to command history
                    self.command_history.append({
                        'id': cmd_id,
                        'type': goal['type'],
                        'result': 'success',
                        'timestamp': self.get_clock().now().nanoseconds / 1e9
                    })
                    
                elif status == 'failed':
                    self.get_logger().warn(f'Command {cmd_id} failed: {result}')
                    
                    # Handle retries
                    goal['retries'] += 1
                    if goal['retries'] < self.max_retry_attempts:
                        self.get_logger().info(f'Retrying command {cmd_id}, attempt {goal["retries"] + 1}')
                        # In a real implementation, you'd resend the command here
                    else:
                        self.get_logger().error(f'Command {cmd_id} failed after {self.max_retry_attempts} attempts')
                        del self.active_goals[cmd_id]
                        
                        # Add to command history as failed
                        self.command_history.append({
                            'id': cmd_id,
                            'type': goal['type'],
                            'result': 'failed',
                            'timestamp': self.get_clock().now().nanoseconds / 1e9
                        })
                
                elif status == 'in_progress':
                    # Update command status
                    goal['status'] = 'in_progress'
                    
        except Exception as e:
            self.get_logger().error(f'Error processing command response: {str(e)}')
    
    def check_active_goals_timeout(self):
        """Check for active goals that have timed out."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        expired_goals = []
        
        for cmd_id, goal_data in self.active_goals.items():
            elapsed = current_time - goal_data['start_time']
            if elapsed > self.action_timeout:
                expired_goals.append(cmd_id)
                self.get_logger().warn(f'Command {cmd_id} timed out after {elapsed:.2f}s')
        
        # Remove expired goals
        for cmd_id in expired_goals:
            del self.active_goals[cmd_id]
    
    def is_command_active(self, cmd_type: str = None) -> bool:
        """Check if any commands of a specific type are currently active."""
        if cmd_type:
            return any(goal_data['type'] == cmd_type for goal_data in self.active_goals.values())
        else:
            return len(self.active_goals) > 0
    
    def get_command_status(self, cmd_id: str) -> Optional[Dict[str, Any]]:
        """Get the status of a specific command."""
        return self.active_goals.get(cmd_id)