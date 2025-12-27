import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import numpy as np
import math
from builtin_interfaces.msg import Time


class WalkingController(Node):
    """
    Main walking controller node that orchestrates gait generation and inverse kinematics
    """
    
    def __init__(self):
        super().__init__('walking_controller')
        
        # Robot parameters
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint'
        ]
        
        # Publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_group_position_controller/commands',  # Standard ROS2 control interface
            10
        )
        
        # Publisher for joint states (for visualization)
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Subscriber for walking commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscriber for gait parameters
        self.gait_param_sub = self.create_subscription(
            Float64MultiArray,
            'gait_parameters',
            self.gait_param_callback,
            10
        )
        
        # Initialize joint positions
        self.joint_positions = [0.0] * len(self.joint_names)
        self.target_walk_speed = 0.0
        self.target_walk_direction = 0.0
        
        # Walking parameters
        self.step_height = 0.05  # Maximum foot lift height
        self.step_length = 0.15  # Step length
        self.step_duration = 1.0  # Time for one step
        self.stride_width = 0.2   # Distance between feet
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        # Joint state message template
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = self.joint_names
        
        self.get_logger().info('Walking Controller initialized')
        self.get_logger().info(f'Controlling joints: {self.joint_names}')
    
    def cmd_vel_callback(self, msg):
        """
        Callback for velocity commands
        """
        # Extract linear and angular velocities
        linear_x = msg.linear.x  # Forward/backward movement
        linear_y = msg.linear.y  # Lateral movement  
        angular_z = msg.angular.z  # Rotation
        
        # Calculate target walking speed and direction
        self.target_walk_speed = math.sqrt(linear_x**2 + linear_y**2)
        self.target_walk_direction = math.atan2(linear_y, linear_x)
        
        # Log command if it changes significantly
        if abs(self.target_walk_speed) > 0.01 or abs(angular_z) > 0.01:
            self.get_logger().info(
                f'Command received: v={self.target_walk_speed:.2f} m/s, '
                f'direction={math.degrees(self.target_walk_direction):.1f}°, '
                f'rotation={angular_z:.2f} rad/s'
            )
    
    def gait_param_callback(self, msg):
        """
        Callback for gait parameter adjustments
        msg.data format: [step_height, step_length, step_duration, stride_width]
        """
        if len(msg.data) >= 4:
            self.step_height = msg.data[0]
            self.step_length = msg.data[1]
            self.step_duration = msg.data[2]
            self.stride_width = msg.data[3]
            
            self.get_logger().info(
                f'Gait parameters updated: height={self.step_height:.2f}, '
                f'length={self.step_length:.2f}, duration={self.step_duration:.2f}, '
                f'stride={self.stride_width:.2f}'
            )
    
    def control_loop(self):
        """
        Main control loop that generates walking motion
        """
        # Generate walking pattern based on current command
        hip_angles, knee_angles, ankle_angles = self.generate_walking_pattern()
        
        # Update joint positions
        self.joint_positions[0] = hip_angles[0]    # left hip
        self.joint_positions[1] = knee_angles[0]   # left knee
        self.joint_positions[2] = ankle_angles[0]  # left ankle
        self.joint_positions[3] = hip_angles[1]    # right hip
        self.joint_positions[4] = knee_angles[1]   # right knee
        self.joint_positions[5] = ankle_angles[1]  # right ankle
        
        # Publish joint commands
        self.publish_joint_commands()
        
        # Publish joint states for visualization
        self.publish_joint_states()
    
    def generate_walking_pattern(self):
        """
        Generate coordinated joint angles for walking motion
        Returns: (hip_angles, knee_angles, ankle_angles) for left and right legs
        """
        # Calculate gait phase based on time and walking speed
        current_time = self.get_clock().now().nanoseconds / 1e9
        gait_phase = (current_time / self.step_duration) % 2.0  # 2.0 for full step cycle (left+right)
        
        # Determine which leg is in stance vs swing phase
        left_stance = (0.0 <= gait_phase < 1.0)
        right_stance = (1.0 <= gait_phase < 2.0)
        
        # Default angles (standing position)
        hip_default = 0.0
        knee_default = 0.0
        ankle_default = 0.0
        
        # Calculate walking-related angles based on speed and phase
        hip_amplitude = 0.1 if abs(self.target_walk_speed) > 0.01 else 0.0
        knee_amplitude = 0.05 if abs(self.target_walk_speed) > 0.01 else 0.0
        ankle_amplitude = 0.02 if abs(self.target_walk_speed) > 0.01 else 0.0
        
        # Calculate phase-specific angles for each leg
        if self.target_walk_speed > 0.01:  # Moving forward
            if left_stance:
                # Left leg in stance phase - push back
                left_hip = hip_default - hip_amplitude * math.sin(gait_phase * math.pi)
                left_knee = knee_default - knee_amplitude * math.sin(gait_phase * math.pi)
                left_ankle = ankle_default - ankle_amplitude * math.sin(gait_phase * math.pi)
                
                # Right leg in swing phase - move forward
                right_hip = hip_default + hip_amplitude * math.sin((gait_phase - 1.0) * math.pi)
                right_knee = knee_default + min(knee_amplitude * 1.5, 
                    knee_amplitude * 2 * math.sin((gait_phase - 1.0) * math.pi)**2)
                right_ankle = ankle_default + ankle_amplitude * math.sin((gait_phase - 1.0) * math.pi)
            else:
                # Right leg in stance phase - push back
                right_hip = hip_default - hip_amplitude * math.sin((gait_phase - 1.0) * math.pi)
                right_knee = knee_default - knee_amplitude * math.sin((gait_phase - 1.0) * math.pi)
                right_ankle = ankle_default - ankle_amplitude * math.sin((gait_phase - 1.0) * math.pi)
                
                # Left leg in swing phase - move forward
                left_hip = hip_default + hip_amplitude * math.sin((gait_phase - 2.0) * math.pi)
                left_knee = knee_default + min(knee_amplitude * 1.5, 
                    knee_amplitude * 2 * math.sin((gait_phase - 2.0) * math.pi)**2)
                left_ankle = ankle_default + ankle_amplitude * math.sin((gait_phase - 2.0) * math.pi)
        else:
            # Standing still - minimal movement
            left_hip = hip_default
            left_knee = knee_default
            left_ankle = ankle_default
            right_hip = hip_default
            right_knee = knee_default
            right_ankle = ankle_default
        
        # Apply direction adjustments (for turning)
        if abs(self.target_walk_direction) > 0.1:  # Turning
            # Adjust for turning by differentially moving legs
            turn_factor = abs(self.target_walk_direction) / math.pi * 0.5
            if self.target_walk_direction > 0:  # Turning left
                left_hip -= turn_factor * hip_amplitude
                right_hip += turn_factor * hip_amplitude
            else:  # Turning right
                left_hip += turn_factor * hip_amplitude
                right_hip -= turn_factor * hip_amplitude
        
        # Ensure joint limits are respected
        joint_limits = {
            'hip': (-0.5, 0.5),
            'knee': (0, 0.7),
            'ankle': (-0.3, 0.3)
        }
        
        def apply_joint_limits(angle, joint_type):
            min_val, max_val = joint_limits[joint_type]
            return max(min_val, min(max_val, angle))
        
        # Apply limits
        left_hip = apply_joint_limits(left_hip, 'hip')
        right_hip = apply_joint_limits(right_hip, 'hip')
        left_knee = apply_joint_limits(left_knee, 'knee')
        right_knee = apply_joint_limits(right_knee, 'knee')
        left_ankle = apply_joint_limits(left_ankle, 'ankle')
        right_ankle = apply_joint_limits(right_ankle, 'ankle')
        
        return (
            [left_hip, right_hip],
            [left_knee, right_knee],
            [left_ankle, right_ankle]
        )
    
    def publish_joint_commands(self):
        """
        Publish joint commands to the robot controllers
        """
        cmd_msg = Float64MultiArray()
        cmd_msg.data = self.joint_positions
        self.joint_cmd_pub.publish(cmd_msg)
    
    def publish_joint_states(self):
        """
        Publish joint states for visualization in RViz
        """
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_state_msg.position = self.joint_positions
        self.joint_state_msg.velocity = [0.0] * len(self.joint_positions)
        self.joint_state_msg.effort = [0.0] * len(self.joint_positions)
        
        self.joint_state_pub.publish(self.joint_state_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = WalkingController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Walking Controller node stopped by user')
    except Exception as e:
        node.get_logger().error(f'Error in walking controller: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()