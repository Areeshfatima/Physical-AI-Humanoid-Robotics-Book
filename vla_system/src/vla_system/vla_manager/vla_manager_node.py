"""ROS 2 node for coordinating the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Point
import json
import uuid
from datetime import datetime
from ..models.action_plan import ActionPlan
from ..models.plan_step import PlanStep, ActionType
from .state_machine import StateMachine, VLAState
from ..utils.ros_helpers import get_current_ros_time


class VLAManagerNode(Node):
    """ROS 2 node that coordinates the overall VLA system."""
    
    def __init__(self):
        super().__init__('vla_manager')
        
        # Get parameters
        self.declare_parameter('system_state_update_rate', 10.0)
        self.declare_parameter('command_timeout', 60.0)
        self.declare_parameter('max_command_history', 100)
        
        self.state_update_rate = self.get_parameter('system_state_update_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_command_history = self.get_parameter('max_command_history').value
        
        # Initialize state machine
        self.state_machine = StateMachine()

        # Track active plans and commands
        self.active_plans = {}
        self.command_history = []
        self.current_robot_position = Point(x=0.0, y=0.0, z=0.0)

        # Security measures: Initialize command validation
        self._initialize_security_measures()
        
        # Create subscribers
        qos_profile = QoSProfile(depth=10)
        
        # Subscribe to voice commands
        self.voice_command_subscriber = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            qos_profile
        )
        
        # Subscribe to action plans from the LLM planner
        self.action_plan_subscriber = self.create_subscription(
            String,
            'action_plans',
            self.action_plan_callback,
            qos_profile
        )
        
        # Subscribe to navigation status updates
        self.navigation_status_subscriber = self.create_subscription(
            String,
            'navigation_status',
            self.navigation_status_callback,
            qos_profile
        )
        
        # Subscribe to robot position updates
        self.robot_position_subscriber = self.create_subscription(
            Point,
            'robot_position',
            self.robot_position_callback,
            qos_profile
        )
        
        # Subscribe to perception data
        self.perception_data_subscriber = self.create_subscription(
            String,
            'perception_data',
            self.perception_data_callback,
            qos_profile
        )
        
        # Create publishers
        self.system_status_publisher = self.create_publisher(
            String,
            'system_status',
            qos_profile
        )
        
        self.current_plan_publisher = self.create_publisher(
            String,
            'current_plan',
            qos_profile
        )
        
        self.command_publisher = self.create_publisher(
            String,
            'robot_commands',  # Commands to execute on the robot
            qos_profile
        )
        
        # Create timer for state updates
        self.state_update_timer = self.create_timer(1.0/self.state_update_rate, self.state_update_callback)
        
        self.get_logger().info(
            f'VLA Manager Node initialized with state update rate: {self.state_update_rate}Hz, '
            f'command timeout: {self.command_timeout}s'
        )

    def _initialize_security_measures(self):
        """Initialize security measures for input validation and command authorization."""
        # Define allowed commands (command patterns that are considered safe)
        self.allowed_commands = [
            'move', 'go', 'navigate', 'walk', 'step', 'turn', 'rotate',
            'find', 'look', 'search', 'detect', 'see', 'perceive',
            'wait', 'stop', 'pause', 'continue',
            'pick', 'grasp', 'take', 'drop', 'release'  # Basic manipulation commands
        ]

        # Define dangerous command patterns to block
        self.blocked_patterns = [
            'self-destruct', 'shutdown', 'terminate', 'kill', 'destroy',
            'format', 'delete', 'rm', 'reboot', 'restart system',
            'access', 'modify', 'change setting', 'override safety'
        ]

        # Initialize command rate limiting
        self.command_timestamps = {}
        self.max_commands_per_minute = 30  # Limit commands to prevent flooding
        self.command_window = 60  # Time window in seconds

    def _is_command_safe(self, command_text: str) -> bool:
        """Check if a command is safe to execute."""
        # Convert to lowercase for comparison
        cmd_lower = command_text.lower()

        # Check for blocked patterns
        for pattern in self.blocked_patterns:
            if pattern in cmd_lower:
                self.get_logger().warn(f'Blocked potentially dangerous command: {command_text}')
                return False

        # Basic command safety check - does it contain allowed keywords?
        # This is a simple heuristic - in a real system you'd have more sophisticated validation
        contains_allowed = any(keyword in cmd_lower for keyword in self.allowed_commands)

        if not contains_allowed and len(cmd_lower.split()) > 0:
            # For now, we'll allow commands that don't match our simple patterns
            # In a real system, you'd want more comprehensive validation
            pass

        # Check for command injection attempts
        dangerous_chars = [';', '&', '|', '`', '$(', '${', '>', '<', '>>']
        for char in dangerous_chars:
            if char in command_text:
                self.get_logger().warn(f'Command contains potentially dangerous character: {char}')
                return False

        return True

    def _is_rate_limited(self, client_id: str = "default") -> bool:
        """Check if a client is sending commands too frequently."""
        current_time = datetime.now().timestamp()

        # Clean old timestamps outside the window
        self.command_timestamps[client_id] = [
            ts for ts in self.command_timestamps.get(client_id, [])
            if current_time - ts < self.command_window
        ]

        # Add current timestamp
        self.command_timestamps[client_id].append(current_time)

        # Check if rate limit exceeded
        if len(self.command_timestamps[client_id]) > self.max_commands_per_minute:
            self.get_logger().warn(f'Rate limit exceeded for client {client_id}')
            return True

        return False
    
    def voice_command_callback(self, msg: String):
        """Callback for incoming voice commands."""
        try:
            # Parse the voice command
            parts = msg.data.split(':', 1)
            if len(parts) != 2:
                self.get_logger().error(f'Invalid command format: {msg.data}')
                return

            cmd_id, command_text = parts[0], parts[1]

            # Apply security checks
            if not self._is_command_safe(command_text):
                self.get_logger().error(f'Unsafe command blocked: "{command_text}"')
                return

            # Check rate limiting (use a default client ID, in a real system you'd identify the actual client)
            if self._is_rate_limited("default"):
                self.get_logger().warn(f'Command rate limit exceeded for command: "{command_text}"')
                return

            self.get_logger().info(f'Received voice command: "{command_text}" (ID: {cmd_id})')

            # Add to command history
            self._add_to_command_history(cmd_id, command_text)

            # Transition to processing state
            self.state_machine.transition_to(VLAState.PROCESSING_COMMAND, {
                'command_id': cmd_id,
                'command_text': command_text
            })

        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {str(e)}')
            self.state_machine.transition_to(VLAState.ERROR, {'error': str(e)})
    
    def action_plan_callback(self, msg: String):
        """Callback for incoming action plans."""
        try:
            plan_data = json.loads(msg.data)
            
            # Create ActionPlan object from the data
            plan_steps = []
            for step_data in plan_data.get('plan_steps', []):
                step = PlanStep(
                    id=step_data['id'],
                    action_type=step_data['action_type'],
                    parameters=step_data['parameters'],
                    sequence_number=step_data['sequence_number'],
                    estimated_duration=step_data['estimated_duration']
                )
                plan_steps.append(step)
            
            action_plan = ActionPlan(
                id=plan_data['id'],
                command_id=plan_data['command_id'],
                plan_steps=plan_steps,
                created_at=datetime.fromisoformat(plan_data['created_at']),
                status=plan_data['status'],
                execution_progress=plan_data['execution_progress']
            )
            
            # Add to active plans
            self.active_plans[action_plan.id] = action_plan
            
            self.get_logger().info(
                f'Received action plan with {len(action_plan.plan_steps)} steps (ID: {action_plan.id})'
            )
            
            # If we're waiting for a plan, start execution
            if self.state_machine.get_current_state() in [VLAState.PROCESSING_COMMAND, VLAState.PLANNING_ACTIONS]:
                self._start_plan_execution(action_plan)
            
        except Exception as e:
            self.get_logger().error(f'Error processing action plan: {str(e)}')
            self.state_machine.transition_to(VLAState.ERROR, {'error': str(e)})
    
    def navigation_status_callback(self, msg: String):
        """Callback for navigation status updates."""
        try:
            status_data = json.loads(msg.data)
            
            # Update robot position from navigation status if available
            if 'current_position' in status_data:
                pos = status_data['current_position']
                self.current_robot_position = Point(x=pos['x'], y=pos['y'], z=pos['z'])
            
            # Check if navigation is complete when in NAVIGATING state
            if self.state_machine.get_current_state() == VLAState.NAVIGATING:
                # In a real implementation, we would check if the navigation is complete
                # For now, we'll just log the status
                self.get_logger().debug(f'Navigation status: {status_data}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing navigation status: {str(e)}')
    
    def robot_position_callback(self, msg: Point):
        """Callback to update robot position."""
        self.current_robot_position = msg
        self.get_logger().debug(f'Robot position updated to ({msg.x}, {msg.y}, {msg.z})')
    
    def perception_data_callback(self, msg: String):
        """Callback for perception data updates."""
        try:
            perception_data = json.loads(msg.data)
            self.get_logger().debug(f'Received perception data with {len(perception_data.get("detected_objects", []))} objects')
            
            # In a real system, perception data might trigger plan updates
            # For now, we'll just log it
            
        except Exception as e:
            self.get_logger().error(f'Error processing perception data: {str(e)}')
    
    def state_update_callback(self):
        """Periodic callback to update system state and publish status."""
        try:
            # Publish system status
            status_msg = String()
            status_data = {
                'state': self.state_machine.get_current_state().value,
                'state_duration': self.state_machine.get_state_duration(),
                'active_plans_count': len(self.active_plans),
                'command_history_count': len(self.command_history),
                'robot_position': {
                    'x': self.current_robot_position.x,
                    'y': self.current_robot_position.y,
                    'z': self.current_robot_position.z
                },
                'timestamp': datetime.now().isoformat()
            }
            status_msg.data = json.dumps(status_data)
            self.system_status_publisher.publish(status_msg)
            
            # Publish current plan if one exists
            if self.state_machine.active_action_plan_id:
                active_plan = self.active_plans.get(self.state_machine.active_action_plan_id)
                if active_plan:
                    plan_msg = String()
                    plan_msg.data = json.dumps({
                        'id': active_plan.id,
                        'command_id': active_plan.command_id,
                        'status': active_plan.status,
                        'execution_progress': active_plan.execution_progress,
                        'total_steps': len(active_plan.plan_steps),
                        'current_step': active_plan.execution_progress if active_plan.plan_steps else None
                    })
                    self.current_plan_publisher.publish(plan_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in state update: {str(e)}')
    
    def _add_to_command_history(self, cmd_id: str, command_text: str):
        """Add a command to the history, maintaining the maximum length."""
        self.command_history.append({
            'id': cmd_id,
            'text': command_text,
            'timestamp': datetime.now(),
            'status': 'received'
        })
        
        # Trim history if it exceeds max length
        if len(self.command_history) > self.max_command_history:
            self.command_history = self.command_history[-self.max_command_history:]
    
    def _start_plan_execution(self, action_plan: ActionPlan):
        """Start executing an action plan."""
        try:
            # Update state machine
            self.state_machine.active_action_plan_id = action_plan.id
            self.state_machine.active_action_step = 0
            self.state_machine.transition_to(VLAState.EXECUTING_ACTIONS, {
                'plan_id': action_plan.id,
                'total_steps': len(action_plan.plan_steps)
            })
            
            # Update plan status
            action_plan.status = 'executing'
            
            # Execute the first step
            if action_plan.plan_steps:
                self._execute_plan_step(action_plan, 0)
            
        except Exception as e:
            self.get_logger().error(f'Error starting plan execution: {str(e)}')
            self.state_machine.transition_to(VLAState.ERROR, {'error': str(e)})
    
    def _execute_plan_step(self, action_plan: ActionPlan, step_index: int):
        """Execute a specific step in the action plan."""
        try:
            if step_index >= len(action_plan.plan_steps):
                # Plan completed
                action_plan.status = 'completed'
                self.state_machine.transition_to(VLAState.COMPLETED)
                self.get_logger().info(f'Action plan {action_plan.id} completed')
                return
            
            step = action_plan.plan_steps[step_index]
            action_plan.execution_progress = step_index
            
            self.get_logger().info(f'Executing plan step {step_index}: {step.action_type}')
            
            # Update active step in state machine
            self.state_machine.active_action_step = step_index
            
            # Handle different action types
            if step.action_type == ActionType.NAVIGATE.value:
                self.state_machine.transition_to(VLAState.NAVIGATING, {
                    'destination': step.parameters.get('target_position', {}),
                    'step_id': step.id
                })
                
                # Publish navigation command
                nav_cmd = json.dumps({
                    'type': 'navigate',
                    'target': step.parameters.get('target_position', {}),
                    'relative': step.parameters.get('relative', False)
                })
                cmd_msg = String()
                cmd_msg.data = nav_cmd
                self.command_publisher.publish(cmd_msg)
                
            elif step.action_type == ActionType.PERCEIVE.value:
                self.state_machine.transition_to(VLAState.PERCEIVING, {
                    'object_type': step.parameters.get('object_type'),
                    'step_id': step.id
                })
                
                # Publish perception command
                per_cmd = json.dumps({
                    'type': 'perceive',
                    'object_type': step.parameters.get('object_type', 'any'),
                    'search_area': step.parameters.get('search_area', 'forward_cone')
                })
                cmd_msg = String()
                cmd_msg.data = per_cmd
                self.command_publisher.publish(cmd_msg)
                
            elif step.action_type == ActionType.WAIT.value:
                self.state_machine.transition_to(VLAState.WAITING, {
                    'duration': step.parameters.get('duration', 1.0),
                    'step_id': step.id
                })
                
                # Publish wait command
                wait_cmd = json.dumps({
                    'type': 'wait',
                    'duration': step.parameters.get('duration', 1.0)
                })
                cmd_msg = String()
                cmd_msg.data = wait_cmd
                self.command_publisher.publish(cmd_msg)
                
                # For now, immediately move to next step after "waiting"
                # In a real implementation, this would be timed
                self._execute_next_step(action_plan)
                
            elif step.action_type == ActionType.MANIPULATE.value:
                # For this implementation, manipulation is treated as a high-level action
                # In a real system, this would require more sophisticated handling
                self.state_machine.transition_to(VLAState.MANIPULATING, {
                    'action': step.parameters.get('action'),
                    'target_object': step.parameters.get('target_object'),
                    'step_id': step.id
                })
                
                # Publish manipulation command
                manip_cmd = json.dumps({
                    'type': 'manipulate',
                    'action': step.parameters.get('action', 'grasp'),
                    'target_object': step.parameters.get('target_object')
                })
                cmd_msg = String()
                cmd_msg.data = manip_cmd
                self.command_publisher.publish(cmd_msg)
                
                # For now, immediately move to next step after "manipulation"
                self._execute_next_step(action_plan)
                
        except Exception as e:
            self.get_logger().error(f'Error executing plan step: {str(e)}')
            self.state_machine.transition_to(VLAState.ERROR, {'error': str(e)})
    
    def _execute_next_step(self, action_plan: ActionPlan):
        """Execute the next step in the action plan."""
        next_step_index = action_plan.execution_progress + 1
        self._execute_plan_step(action_plan, next_step_index)


def main(args=None):
    rclpy.init(args=args)
    
    node = VLAManagerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('VLA Manager Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()