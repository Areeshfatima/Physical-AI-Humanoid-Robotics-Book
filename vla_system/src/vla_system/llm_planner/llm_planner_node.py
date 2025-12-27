"""ROS 2 node for LLM-based cognitive planning in the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import json
import uuid
from datetime import datetime
from std_msgs.msg import String
from ..models.action_plan import ActionPlan
from ..models.plan_step import PlanStep, ActionType
from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
import torch
import os


class LLMPlannerNode(Node):
    """ROS 2 node that translates natural language commands into robot action plans using LLM."""
    
    def __init__(self):
        super().__init__('llm_planner_node')
        
        # Get parameters
        self.declare_parameter('llm_model_name', 'microsoft/DialoGPT-medium')
        self.declare_parameter('llm_max_length', 100)
        self.declare_parameter('planning_timeout', 5.0)
        
        model_name = self.get_parameter('llm_model_name').value
        self.max_length = self.get_parameter('llm_max_length').value
        self.timeout = self.get_parameter('planning_timeout').value
        
        # Initialize the LLM model
        self.get_logger().info(f'Loading LLM model: {model_name}')
        
        try:
            # Using Hugging Face transformers for local LLM execution
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.model = AutoModelForCausalLM.from_pretrained(model_name)
            
            # Add padding token if it doesn't exist
            if self.tokenizer.pad_token is None:
                self.tokenizer.pad_token = self.tokenizer.eos_token
            
            # Check if CUDA is available and use GPU if possible
            if torch.cuda.is_available():
                self.get_logger().info("Using GPU for LLM processing")
                self.model = self.model.cuda()
            else:
                self.get_logger().info("Using CPU for LLM processing")
                
            self.get_logger().info(f'LLM model {model_name} loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load LLM model {model_name}: {str(e)}')
            # Fallback to a simple pipeline if the specific model fails
            self.get_logger().info('Using fallback pipeline approach')
            self.tokenizer = None
            self.model = None
        
        # Define the prompt template for action planning
        self.planning_prompt_template = self._load_prompt_template()
        
        # Create subscriber for voice commands
        qos_profile = QoSProfile(depth=10)
        self.voice_command_subscriber = self.create_subscription(
            String,
            'voice_commands',
            self.voice_command_callback,
            qos_profile
        )
        
        # Create publisher for action plans
        self.action_plan_publisher = self.create_publisher(
            String,  # Using String for now - in a complete implementation we'd have a custom message
            'action_plans',
            qos_profile
        )
        
        # Create subscriber for requesting action plans with text commands
        self.text_command_subscriber = self.create_subscription(
            String,
            'text_commands',
            self.text_command_callback,
            qos_profile
        )
        
        self.get_logger().info(f'LLM Planner Node initialized with model {model_name}')
    
    def _load_prompt_template(self):
        """Load the action planning prompt template."""
        # Try to load from file first
        prompt_file = os.path.join(
            os.path.dirname(__file__), 
            'prompt_templates', 
            'action_planning_prompt.txt'
        )
        
        if os.path.exists(prompt_file):
            with open(prompt_file, 'r') as f:
                return f.read()
        else:
            # Fallback to a basic template
            return ("Translate this command to robot actions: {command}. "
                    "Respond in JSON format with action sequence.")
    
    def voice_command_callback(self, msg: String):
        """Callback to handle voice commands and generate action plans."""
        try:
            # Parse the command (format: "id:text")
            parts = msg.data.split(':', 1)
            if len(parts) != 2:
                self.get_logger().error(f'Invalid command format: {msg.data}')
                return
                
            cmd_id, command_text = parts[0], parts[1]
            
            self.get_logger().info(f'Received voice command: "{command_text}"')
            
            # Generate action plan from the command
            action_plan = self.generate_action_plan(command_text, cmd_id)
            
            if action_plan:
                # Publish the action plan
                plan_msg = String()
                plan_msg.data = json.dumps({
                    'id': action_plan.id,
                    'command_id': action_plan.command_id,
                    'plan_steps': [
                        {
                            'id': step.id,
                            'action_type': step.action_type,
                            'parameters': step.parameters,
                            'sequence_number': step.sequence_number,
                            'estimated_duration': step.estimated_duration
                        } for step in action_plan.plan_steps
                    ],
                    'created_at': action_plan.created_at.isoformat(),
                    'status': action_plan.status,
                    'execution_progress': action_plan.execution_progress
                })
                
                self.action_plan_publisher.publish(plan_msg)
                self.get_logger().info(f'Published action plan with {len(action_plan.plan_steps)} steps')
            else:
                self.get_logger().error(f'Failed to generate action plan for command: {command_text}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing voice command: {str(e)}')
    
    def text_command_callback(self, msg: String):
        """Callback to handle text commands directly."""
        try:
            command_text = msg.data
            self.get_logger().info(f'Received text command: "{command_text}"')
            
            # Generate action plan from the command
            # Use a random ID for text commands
            cmd_id = str(uuid.uuid4())
            action_plan = self.generate_action_plan(command_text, cmd_id)
            
            if action_plan:
                # Publish the action plan
                plan_msg = String()
                plan_msg.data = json.dumps({
                    'id': action_plan.id,
                    'command_id': action_plan.command_id,
                    'plan_steps': [
                        {
                            'id': step.id,
                            'action_type': step.action_type,
                            'parameters': step.parameters,
                            'sequence_number': step.sequence_number,
                            'estimated_duration': step.estimated_duration
                        } for step in action_plan.plan_steps
                    ],
                    'created_at': action_plan.created_at.isoformat(),
                    'status': action_plan.status,
                    'execution_progress': action_plan.execution_progress
                })
                
                self.action_plan_publisher.publish(plan_msg)
                self.get_logger().info(f'Published action plan with {len(action_plan.plan_steps)} steps')
            else:
                self.get_logger().error(f'Failed to generate action plan for command: {command_text}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing text command: {str(e)}')
    
    def generate_action_plan(self, command_text: str, command_id: str) -> 'Optional[ActionPlan]':
        """Generate an action plan from a natural language command."""
        try:
            # Format the prompt with the command
            prompt = self.planning_prompt_template.format(command=command_text)
            
            # Generate action plan using the LLM
            action_steps = self._generate_with_llm(prompt)
            
            if not action_steps:
                self.get_logger().warn(f'No action steps generated for command: {command_text}')
                # Handle ambiguous or unclear commands
                self._handle_ambiguous_command(command_text)
                return None
            
            # Create PlanStep objects from the generated steps
            plan_steps = []
            for i, step_data in enumerate(action_steps):
                try:
                    # Validate and create PlanStep
                    step = PlanStep(
                        id=step_data.get('id', f'step_{i}_{uuid.uuid4()}'),
                        action_type=step_data['action_type'],
                        parameters=step_data.get('parameters', {}),
                        sequence_number=step_data.get('sequence_number', i),
                        estimated_duration=step_data.get('estimated_duration', 1.0)
                    )
                    plan_steps.append(step)
                except Exception as e:
                    self.get_logger().error(f'Error creating PlanStep: {str(e)}')
                    # Handle error by trying to continue with other steps
                    continue

            if not plan_steps:
                self.get_logger().error('No valid PlanSteps could be created')
                # Try to generate a clarification request as a default response
                clarification_plan = self._generate_clarification_plan(command_text)
                if clarification_plan:
                    return clarification_plan
                return None
            
            # Create the ActionPlan
            plan_id = str(uuid.uuid4())
            action_plan = ActionPlan(
                id=plan_id,
                command_id=command_id,
                plan_steps=plan_steps,
                created_at=datetime.now(),
                status='pending',  # Will be updated by the executor
                execution_progress=0
            )
            
            return action_plan
            
        except Exception as e:
            self.get_logger().error(f'Error generating action plan: {str(e)}')
            return None
    
    def _generate_with_llm(self, prompt: str):
        """Generate action steps using the LLM."""
        try:
            # In a full implementation, we would use the actual LLM here
            # For now, we'll implement a simple rule-based approach as fallback
            # and include the LLM implementation if models are properly loaded
            if self.model and self.tokenizer:
                return self._generate_with_transformer_model(prompt)
            else:
                # Fallback rule-based approach for demonstration
                return self._generate_with_rules(prompt)
        except Exception as e:
            self.get_logger().error(f'Error in LLM generation: {str(e)}')
            # Return rule-based result as fallback
            return self._generate_with_rules(prompt)

    def _handle_ambiguous_command(self, command_text: str):
        """Handle ambiguous or unclear commands by publishing a clarification request."""
        try:
            self.get_logger().info(f'Handling ambiguous command: {command_text}')

            # In a complete system, this might trigger a request back to the user
            # for clarification. For now, we'll log the issue.
            # In a real implementation, you might publish to a clarification topic

            # For demonstration, we'll also try to generate a clarification plan
            clarification_plan = self._generate_clarification_plan(command_text)
            if clarification_plan:
                # Publish the clarification plan
                plan_msg = String()
                plan_msg.data = json.dumps({
                    'id': clarification_plan.id,
                    'command_id': clarification_plan.command_id,
                    'plan_steps': [
                        {
                            'id': step.id,
                            'action_type': step.action_type,
                            'parameters': step.parameters,
                            'sequence_number': step.sequence_number,
                            'estimated_duration': step.estimated_duration
                        } for step in clarification_plan.plan_steps
                    ],
                    'created_at': clarification_plan.created_at.isoformat(),
                    'status': clarification_plan.status,
                    'execution_progress': clarification_plan.execution_progress
                })

                # Note: Clarification plans might go to a different topic
                self.action_plan_publisher.publish(plan_msg)
                self.get_logger().info('Published clarification request plan')

        except Exception as e:
            self.get_logger().error(f'Error handling ambiguous command: {str(e)}')

    def _generate_clarification_plan(self, command_text: str) -> 'Optional[ActionPlan]':
        """Generate a plan to request clarification for an ambiguous command."""
        try:
            import uuid
            from datetime import datetime

            # Create a simple plan for clarification
            clarification_step = PlanStep(
                id=f'clarify_{uuid.uuid4()}',
                action_type='wait',  # Wait for user input
                parameters={
                    'clarification_request': f'Could you please clarify your command: "{command_text}"?',
                    'timeout': 30.0  # Wait 30 seconds for response
                },
                sequence_number=0,
                estimated_duration=30.0
            )

            plan_id = str(uuid.uuid4())
            command_id = str(uuid.uuid4())  # Create a new command ID for clarification

            clarification_plan = ActionPlan(
                id=plan_id,
                command_id=command_id,
                plan_steps=[clarification_step],
                created_at=datetime.now(),
                status='pending',
                execution_progress=0
            )

            return clarification_plan

        except Exception as e:
            self.get_logger().error(f'Error generating clarification plan: {str(e)}')
            return None

    def _generate_with_transformer_model(self, prompt: str):
        """Generate action steps using a transformer model."""
        try:
            inputs = self.tokenizer.encode(prompt, return_tensors='pt')

            if torch.cuda.is_available():
                inputs = inputs.cuda()

            # Generate text with the model
            with torch.no_grad():
                outputs = self.model.generate(
                    inputs,
                    max_length=len(inputs[0]) + self.max_length,
                    num_return_sequences=1,
                    do_sample=True,
                    temperature=0.7,
                    pad_token_id=self.tokenizer.eos_token_id
                )

            # Decode the generated text
            generated_text = self.tokenizer.decode(outputs[0], skip_special_tokens=True)

            # Extract the response part (after the prompt)
            response_start = generated_text.find(prompt.strip()) + len(prompt.strip())
            response_text = generated_text[response_start:].strip()

            # Try to parse the JSON response
            try:
                # Look for JSON array in the response
                import re
                json_match = re.search(r'\[.*\]', response_text, re.DOTALL)
                if json_match:
                    json_str = json_match.group()
                    action_steps = json.loads(json_str)

                    # Validate the action sequence before returning
                    if self._validate_action_sequence(action_steps):
                        return action_steps
                    else:
                        self.get_logger().warn('Generated action sequence failed validation')
                        return []

            except json.JSONDecodeError:
                self.get_logger().warn(f'Could not parse JSON from LLM response: {response_text}')

            # If JSON parsing failed, use rule-based fallback
            return self._generate_with_rules(prompt)

        except Exception as e:
            self.get_logger().error(f'Error generating with transformer model: {str(e)}')
            return self._generate_with_rules(prompt)
    
    def _generate_with_rules(self, prompt: str):
        """Generate action steps using simple rules as fallback."""
        try:
            # Simple rule-based parsing for demonstration
            # In a real implementation, you'd use more sophisticated NLP
            command = prompt.split('Now, translate this command into a sequence of actions:')[1].strip()

            # Simple command parsing
            command_lower = command.lower()

            if 'move' in command_lower or 'go' in command_lower or 'navigate' in command_lower:
                if 'forward' in command_lower or 'ahead' in command_lower:
                    # Extract distance if mentioned
                    import re
                    distance_match = re.search(r'(\d+(?:\.\d+)?)\s*(meters?|m)', command_lower)
                    distance = float(distance_match.group(1)) if distance_match else 1.0

                    action_steps = [{
                        'id': f'nav_{uuid.uuid4()}',
                        'action_type': 'navigate',
                        'parameters': {
                            'target_position': {'x': distance, 'y': 0.0, 'z': 0.0},
                            'relative': True
                        },
                        'sequence_number': 0,
                        'estimated_duration': distance * 2.0  # Assume 0.5 m/s
                    }]

                    # Validate the action sequence
                    if self._validate_action_sequence(action_steps):
                        return action_steps
                    else:
                        self.get_logger().warn('Generated navigation action failed validation')
                        return []

                elif 'kitchen' in command_lower or 'room' in command_lower:
                    action_steps = [{
                        'id': f'nav_{uuid.uuid4()}',
                        'action_type': 'navigate',
                        'parameters': {
                            'target_position': {'x': 5.0, 'y': 3.0, 'z': 0.0},
                            'relative': False
                        },
                        'sequence_number': 0,
                        'estimated_duration': 10.0
                    }]

                    # Validate the action sequence
                    if self._validate_action_sequence(action_steps):
                        return action_steps
                    else:
                        self.get_logger().warn('Generated navigation action failed validation')
                        return []

            elif 'find' in command_lower or 'look' in command_lower or 'detect' in command_lower:
                # Identify object to find
                obj_keywords = ['block', 'object', 'person', 'door', 'table', 'chair']
                target_obj = None
                for keyword in obj_keywords:
                    if keyword in command_lower:
                        target_obj = keyword
                        break

                if target_obj:
                    action_steps = [{
                        'id': f'per_{uuid.uuid4()}',
                        'action_type': 'perceive',
                        'parameters': {
                            'object_type': target_obj,
                            'search_area': 'forward_cone'
                        },
                        'sequence_number': 0,
                        'estimated_duration': 3.0
                    }]

                    # Validate the action sequence
                    if self._validate_action_sequence(action_steps):
                        return action_steps
                    else:
                        self.get_logger().warn('Generated perception action failed validation')
                        return []

            elif 'wait' in command_lower or 'stop' in command_lower:
                import re
                time_match = re.search(r'(\d+(?:\.\d+)?)\s*(seconds?|secs?|s)', command_lower)
                duration = float(time_match.group(1)) if time_match else 5.0

                action_steps = [{
                    'id': f'wait_{uuid.uuid4()}',
                    'action_type': 'wait',
                    'parameters': {'duration': duration},
                    'sequence_number': 0,
                    'estimated_duration': duration
                }]

                # Validate the action sequence
                if self._validate_action_sequence(action_steps):
                    return action_steps
                else:
                    self.get_logger().warn('Generated wait action failed validation')
                    return []

            # Default: unknown command
            self.get_logger().warn(f'Unknown command type for: {command}')
            return []

        except Exception as e:
            self.get_logger().error(f'Error in rule-based generation: {str(e)}')
            return []

    def _validate_action_sequence(self, action_steps):
        """Validate an action sequence to ensure it follows logical rules."""
        try:
            if not action_steps:
                return False  # Empty sequence is invalid

            # Check that sequence numbers are consecutive starting from 0
            sequence_numbers = [step.get('sequence_number', 0) for step in action_steps]
            expected_sequence = list(range(len(action_steps)))

            if sorted(sequence_numbers) != expected_sequence:
                self.get_logger().warn(f'Invalid sequence numbers: {sequence_numbers}')
                return False

            # Check that all required fields are present
            for step in action_steps:
                required_fields = ['id', 'action_type', 'parameters', 'sequence_number', 'estimated_duration']
                for field in required_fields:
                    if field not in step:
                        self.get_logger().warn(f'Missing required field "{field}" in step: {step}')
                        return False

                # Validate action type
                valid_action_types = ['navigate', 'perceive', 'manipulate', 'wait']
                if step['action_type'] not in valid_action_types:
                    self.get_logger().warn(f'Invalid action type "{step["action_type"]}" in step: {step}')
                    return False

                # Validate estimated duration is positive
                if not isinstance(step['estimated_duration'], (int, float)) or step['estimated_duration'] <= 0:
                    self.get_logger().warn(f'Invalid duration "{step["estimated_duration"]}" in step: {step}')
                    return False

            # All validations passed
            return True

        except Exception as e:
            self.get_logger().error(f'Error validating action sequence: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    node = LLMPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('LLM Planner Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()