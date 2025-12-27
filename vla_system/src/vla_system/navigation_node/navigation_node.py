"""ROS 2 node for robot navigation in the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Point, PoseStamped
from ..models.navigation_plan import NavigationPlan
from ..models.path_point import PathPoint
from .navigation_planner import NavigationPlanner
import json
import uuid
from datetime import datetime


class NavigationNode(Node):
    """ROS 2 node that handles robot navigation planning and execution."""
    
    def __init__(self):
        super().__init__('navigation_node')
        
        # Get parameters
        self.declare_parameter('navigation_timeout', 30.0)
        self.declare_parameter('min_distance_to_goal', 0.5)
        self.declare_parameter('max_linear_velocity', 0.5)
        self.declare_parameter('max_angular_velocity', 1.0)
        
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.min_distance_to_goal = self.get_parameter('min_distance_to_goal').value
        self.max_linear_velocity = self.get_parameter('max_linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        
        # Initialize navigation planner
        self.navigation_planner = NavigationPlanner()
        
        # Track active navigation plans
        self.active_plans = {}
        
        # Create subscribers
        qos_profile = QoSProfile(depth=10)
        
        # Subscribe to navigation requests
        self.navigation_request_subscriber = self.create_subscription(
            String,
            'navigation_requests',
            self.navigation_request_callback,
            qos_profile
        )
        
        # Subscribe to action plans that involve navigation
        self.action_plan_subscriber = self.create_subscription(
            String,
            'action_plans',
            self.action_plan_callback,
            qos_profile
        )
        
        # Subscribe to robot position updates
        self.robot_position_subscriber = self.create_subscription(
            Point,
            'robot_position',
            self.robot_position_callback,
            qos_profile
        )
        
        # Create publishers
        self.navigation_plan_publisher = self.create_publisher(
            String,  # Using String for now - in a complete implementation we'd have a custom message
            'navigation_plans',
            qos_profile
        )
        
        self.navigation_status_publisher = self.create_publisher(
            String,
            'navigation_status',
            qos_profile
        )
        
        # Store current robot position
        self.current_position = Point(x=0.0, y=0.0, z=0.0)
        
        self.get_logger().info(
            f'Navigation Node initialized with timeout: {self.navigation_timeout}s, '
            f'min goal distance: {self.min_distance_to_goal}m'
        )
    
    def navigation_request_callback(self, msg: String):
        """Callback to handle navigation requests."""
        try:
            # Parse navigation request (format: "x,y,z" for destination)
            try:
                coords = [float(x.strip()) for x in msg.data.split(',')]
                if len(coords) != 3:
                    raise ValueError("Invalid coordinate format, expected 'x,y,z'")
                    
                goal_x, goal_y, goal_z = coords
            except ValueError as e:
                self.get_logger().error(f'Invalid navigation request format: {msg.data}, error: {str(e)}')
                return
            
            self.get_logger().info(f'Received navigation request to ({goal_x}, {goal_y}, {goal_z})')
            
            # Plan the path
            nav_plan = self.navigation_planner.plan_path(
                start_position=(self.current_position.x, self.current_position.y, self.current_position.z),
                goal_position=(goal_x, goal_y, goal_z)
            )
            
            if nav_plan:
                # Publish the navigation plan
                plan_msg = String()
                plan_msg.data = json.dumps({
                    'id': nav_plan.id,
                    'destination_x': nav_plan.destination_x,
                    'destination_y': nav_plan.destination_y,
                    'destination_z': nav_plan.destination_z,
                    'path': [
                        {
                            'id': point.id,
                            'x': point.x,
                            'y': point.y,
                            'z': point.z,
                            'sequence_number': point.sequence_number
                        } for point in nav_plan.path
                    ],
                    'estimated_duration': nav_plan.estimated_duration,
                    'status': nav_plan.status
                })
                
                self.navigation_plan_publisher.publish(plan_msg)
                self.get_logger().info(f'Published navigation plan with {len(nav_plan.path)} waypoints')
                
                # Store the active plan
                self.active_plans[nav_plan.id] = nav_plan
            else:
                self.get_logger().error(f'Failed to plan navigation to ({goal_x}, {goal_y}, {goal_z})')
                
        except Exception as e:
            self.get_logger().error(f'Error processing navigation request: {str(e)}')
    
    def action_plan_callback(self, msg: String):
        """Callback to handle action plans that contain navigation steps."""
        try:
            plan_data = json.loads(msg.data)
            
            # Check if this plan contains navigation steps
            has_navigation = any(
                step.get('action_type') == 'navigate' 
                for step in plan_data.get('plan_steps', [])
            )
            
            if has_navigation:
                # Find the navigation step(s) and extract destination
                for step in plan_data.get('plan_steps', []):
                    if step.get('action_type') == 'navigate':
                        params = step.get('parameters', {})
                        
                        if params.get('relative', False):
                            # Calculate absolute coordinates based on current position
                            target_pos = params.get('target_position', {})
                            goal_x = self.current_position.x + target_pos.get('x', 0.0)
                            goal_y = self.current_position.y + target_pos.get('y', 0.0)
                            goal_z = self.current_position.z + target_pos.get('z', 0.0)
                        else:
                            # Use absolute coordinates directly
                            target_pos = params.get('target_position', {})
                            goal_x = target_pos.get('x', self.current_position.x)
                            goal_y = target_pos.get('y', self.current_position.y)
                            goal_z = target_pos.get('z', self.current_position.z)
                        
                        self.get_logger().info(
                            f'Received navigation action in plan {plan_data["id"]} to ({goal_x}, {goal_y}, {goal_z})'
                        )
                        
                        # Plan the navigation
                        nav_plan = self.navigation_planner.plan_path(
                            start_position=(self.current_position.x, self.current_position.y, self.current_position.z),
                            goal_position=(goal_x, goal_y, goal_z)
                        )
                        
                        if nav_plan:
                            # Publish the navigation plan
                            plan_msg = String()
                            plan_msg.data = json.dumps({
                                'id': nav_plan.id,
                                'destination_x': nav_plan.destination_x,
                                'destination_y': nav_plan.destination_y,
                                'destination_z': nav_plan.destination_z,
                                'path': [
                                    {
                                        'id': point.id,
                                        'x': point.x,
                                        'y': point.y,
                                        'z': point.z,
                                        'sequence_number': point.sequence_number
                                    } for point in nav_plan.path
                                ],
                                'estimated_duration': nav_plan.estimated_duration,
                                'status': nav_plan.status,
                                'source_action_plan_id': plan_data['id']  # Link back to original action plan
                            })
                            
                            self.navigation_plan_publisher.publish(plan_msg)
                            self.get_logger().info(f'Published navigation plan for action step')
                            
                            # Store the active plan
                            self.active_plans[nav_plan.id] = nav_plan
                        else:
                            self.get_logger().error(f'Failed to plan navigation for action step to ({goal_x}, {goal_y}, {goal_z})')
            
        except Exception as e:
            self.get_logger().error(f'Error processing action plan for navigation: {str(e)}')
    
    def robot_position_callback(self, msg: Point):
        """Callback to update robot position."""
        self.current_position = msg
        self.get_logger().debug(f'Robot position updated to ({msg.x}, {msg.y}, {msg.z})')
        
        # Check if any active navigation plans are completed
        self._check_navigation_status()
    
    def _check_navigation_status(self):
        """Check if any active navigation plans have been completed."""
        # In a real system, this would be more sophisticated
        # For now, we'll just publish a status update
        status_msg = String()
        status_msg.data = json.dumps({
            'current_position': {
                'x': self.current_position.x,
                'y': self.current_position.y,
                'z': self.current_position.z
            },
            'active_plans_count': len(self.active_plans)
        })
        
        self.navigation_status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Navigation Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()