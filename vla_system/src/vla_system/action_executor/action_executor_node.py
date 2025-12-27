"""ROS 2 node for executing action plans in the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
import json
import uuid
from datetime import datetime
from ..models.action_plan import ActionPlan
from ..models.plan_step import PlanStep, ActionType
from .robot_commander import RobotCommander


class ActionExecutorNode(Node):
    """ROS 2 node that executes action plans on the robot."""
    
    def __init__(self):
        super().__init__('action_executor')
        
        # Get parameters
        self.declare_parameter('action_execution_timeout', 30.0)
        self.declare_parameter('action_retry_attempts', 3)
        
        self.action_timeout = self.get_parameter('action_execution_timeout').value
        self.max_retry_attempts = self.get_parameter('action_retry_attempts').value
        
        # Initialize robot commander
        self.robot_commander = RobotCommander("action_executor_commander")
        
        # Track active plans and steps
        self.active_plans = {}
        self.current_step_executions = {}
        
        # Create subscribers
        qos_profile = QoSProfile(depth=10)
        
        # Subscribe to action plans to execute
        self.action_plan_subscriber = self.create_subscription(
            String,
            'action_plans',
            self.action_plan_callback,
            qos_profile
        )
        
        # Subscribe to command responses from robot
        self.command_response_subscriber = self.create_subscription(
            String,
            'command_responses',
            self.command_response_callback,
            qos_profile
        )
        
        # Create publishers
        self.plan_status_publisher = self.create_publisher(
            String,
            'plan_status',
            qos_profile
        )
        
        self.step_completion_publisher = self.create_publisher(
            String,
            'step_completion',
            qos_profile
        )
        
        self.get_logger().info(
            f'Action Executor Node initialized with timeout: {self.action_timeout}s, '
            f'max retries: {self.max_retry_attempts}'
        )
    
    def action_plan_callback(self, msg: String):
        """Callback to handle incoming action plans for execution."""
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
                f'Received action plan with {len(action_plan.plan_steps)} steps for execution (ID: {action_plan.id})'
            )
            
            # Start executing the plan immediately
            self._execute_plan(action_plan)
            
        except Exception as e:
            self.get_logger().error(f'Error processing action plan: {str(e)}')
    
    def _execute_plan(self, action_plan: ActionPlan):
        """Execute an action plan by running each step in sequence."""
        try:
            # Update plan status
            action_plan.status = 'executing'
            
            self.get_logger().info(f'Starting execution of plan {action_plan.id}')
            
            # Execute the first step
            if action_plan.plan_steps:
                self._execute_step(action_plan, 0)
            else:
                # Empty plan - mark as completed
                self._mark_plan_completed(action_plan)
            
        except Exception as e:
            self.get_logger().error(f'Error starting plan execution: {str(e)}')
            self._mark_plan_failed(action_plan, str(e))
    
    def _execute_step(self, action_plan: ActionPlan, step_index: int):
        """Execute a specific step in the action plan."""
        try:
            if step_index >= len(action_plan.plan_steps):
                # All steps completed
                self._mark_plan_completed(action_plan)
                return
            
            step = action_plan.plan_steps[step_index]
            action_plan.execution_progress = step_index
            
            self.get_logger().info(
                f'Executing step {step_index} of plan {action_plan.id}: {step.action_type}'
            )
            
            # Store the current execution in progress
            execution_id = str(uuid.uuid4())
            self.current_step_executions[execution_id] = {
                'plan_id': action_plan.id,
                'step_index': step_index,
                'step_id': step.id,
                'start_time': datetime.now()
            }
            
            # Execute different types of actions
            if step.action_type == ActionType.NAVIGATE.value:
                target_pos = step.parameters.get('target_position', {})
                relative = step.parameters.get('relative', False)
                
                cmd_id = self.robot_commander.send_navigation_command(target_pos, relative)
                if cmd_id:
                    # Track this command with the execution
                    self.current_step_executions[execution_id]['command_id'] = cmd_id
                else:
                    self.get_logger().error(f'Failed to send navigation command for step {step_index}')
                    self._mark_plan_failed(action_plan, f'Navigation command failed for step {step_index}')
                    return
            
            elif step.action_type == ActionType.PERCEIVE.value:
                obj_type = step.parameters.get('object_type', 'any')
                search_area = step.parameters.get('search_area', 'forward_cone')
                
                cmd_id = self.robot_commander.send_perception_command(obj_type, search_area)
                if cmd_id:
                    self.current_step_executions[execution_id]['command_id'] = cmd_id
                else:
                    self.get_logger().error(f'Failed to send perception command for step {step_index}')
                    self._mark_plan_failed(action_plan, f'Perception command failed for step {step_index}')
                    return
            
            elif step.action_type == ActionType.WAIT.value:
                duration = step.parameters.get('duration', 1.0)
                
                cmd_id = self.robot_commander.send_wait_command(duration)
                if cmd_id:
                    self.current_step_executions[execution_id]['command_id'] = cmd_id
                    # For wait commands, we can immediately proceed to next step since 
                    # the wait is handled at a lower level
                    self._execute_next_step(action_plan)
                else:
                    self.get_logger().error(f'Failed to send wait command for step {step_index}')
                    self._mark_plan_failed(action_plan, f'Wait command failed for step {step_index}')
                    return
            
            elif step.action_type == ActionType.MANIPULATE.value:
                action = step.parameters.get('action', 'grasp')
                target_obj = step.parameters.get('target_object')
                
                cmd_id = self.robot_commander.send_manipulation_command(action, target_obj)
                if cmd_id:
                    self.current_step_executions[execution_id]['command_id'] = cmd_id
                else:
                    self.get_logger().error(f'Failed to send manipulation command for step {step_index}')
                    self._mark_plan_failed(action_plan, f'Manipulation command failed for step {step_index}')
                    return
            
            # Publish plan status update
            self._publish_plan_status(action_plan)
            
        except Exception as e:
            self.get_logger().error(f'Error executing step: {str(e)}')
            self._mark_plan_failed(action_plan, str(e))
    
    def _execute_next_step(self, action_plan: ActionPlan):
        """Execute the next step in the action plan."""
        next_step_index = action_plan.execution_progress + 1
        self._execute_step(action_plan, next_step_index)
    
    def command_response_callback(self, msg: String):
        """Callback for command responses from the robot."""
        try:
            response_data = json.loads(msg.data)
            
            cmd_id = response_data.get('command_id')
            status = response_data.get('status')
            
            # Find the execution associated with this command
            execution_id = None
            for ex_id, ex_data in self.current_step_executions.items():
                if ex_data.get('command_id') == cmd_id:
                    execution_id = ex_id
                    break
            
            if not execution_id:
                return  # Unknown command response
            
            # Get the plan and step associated with this execution
            ex_data = self.current_step_executions[execution_id]
            plan_id = ex_data['plan_id']
            step_index = ex_data['step_index']
            
            action_plan = self.active_plans.get(plan_id)
            if not action_plan:
                self.get_logger().error(f'Unknown plan ID in command response: {plan_id}')
                return
            
            if status == 'completed':
                self.get_logger().info(f'Step {step_index} of plan {plan_id} completed successfully')
                
                # Publish step completion
                completion_msg = String()
                completion_msg.data = json.dumps({
                    'plan_id': plan_id,
                    'step_index': step_index,
                    'status': 'completed',
                    'timestamp': datetime.now().isoformat()
                })
                self.step_completion_publisher.publish(completion_msg)
                
                # Remove the completed execution
                del self.current_step_executions[execution_id]
                
                # Execute the next step
                self._execute_next_step(action_plan)
                
            elif status == 'failed':
                error_msg = response_data.get('error', 'Unknown error')
                self.get_logger().error(f'Step {step_index} of plan {plan_id} failed: {error_msg}')
                
                # For now, we'll mark the entire plan as failed
                # In a more sophisticated system, you might have recovery strategies
                self._mark_plan_failed(action_plan, f'Step {step_index} failed: {error_msg}')
                
                # Remove the failed execution
                del self.current_step_executions[execution_id]
                
            elif status == 'in_progress':
                # Command is still executing, continue monitoring
                pass
                
        except Exception as e:
            self.get_logger().error(f'Error processing command response: {str(e)}')
    
    def _mark_plan_completed(self, action_plan: ActionPlan):
        """Mark an action plan as completed."""
        action_plan.status = 'completed'
        
        self.get_logger().info(f'Plan {action_plan.id} completed successfully')
        
        # Remove from active plans
        if action_plan.id in self.active_plans:
            del self.active_plans[action_plan.id]
        
        # Publish final status
        self._publish_plan_status(action_plan)
    
    def _mark_plan_failed(self, action_plan: ActionPlan, error: str):
        """Mark an action plan as failed."""
        action_plan.status = 'failed'
        
        self.get_logger().error(f'Plan {action_plan.id} failed: {error}')
        
        # Remove from active plans
        if action_plan.id in self.active_plans:
            del self.active_plans[action_plan.id]
        
        # Publish final status
        self._publish_plan_status(action_plan)
    
    def _publish_plan_status(self, action_plan: ActionPlan):
        """Publish the current status of an action plan."""
        status_msg = String()
        status_msg.data = json.dumps({
            'id': action_plan.id,
            'command_id': action_plan.command_id,
            'status': action_plan.status,
            'execution_progress': action_plan.execution_progress,
            'total_steps': len(action_plan.plan_steps),
            'timestamp': datetime.now().isoformat()
        })
        self.plan_status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = ActionExecutorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Action Executor Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()