import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import time


class GaitPatternGenerator(Node):
    """
    Gait Pattern Generator for humanoid walking locomotion
    Implements various walking patterns including static and dynamic gaits
    """
    
    def __init__(self):
        super().__init__('gait_pattern_generator')
        
        # Gait parameters
        self.step_width = 0.15  # Distance between feet (meters)
        self.step_length = 0.15  # Forward step length (meters)
        self.step_height = 0.05  # Maximum foot lift height (meters)
        self.step_duration = 1.0  # Time for one complete step (seconds)
        self.zmp_offset = 0.05   # Zero Moment Point offset for stability
        
        # Walking state
        self.current_phase = 0.0  # Current phase of gait cycle (0.0 to 1.0)
        self.is_walking = False
        self.walk_speed = 0.0  # m/s
        self.walk_direction = 0.0  # radians (0 = forward, pi/2 = left, etc)
        
        # Publishers
        self.foot_target_pub = self.create_publisher(
            Float64MultiArray,
            'foot_target_positions',
            10
        )
        
        # Timer for gait pattern generation
        self.timer = self.create_timer(0.05, self.generate_gait_pattern)  # 20 Hz update rate
        
        # Subscriber for walk commands
        self.walk_cmd_sub = self.create_subscription(
            Float64MultiArray,
            'walk_command',
            self.walk_command_callback,
            10
        )
        
        self.get_logger().info('Gait Pattern Generator initialized')
        
    def walk_command_callback(self, msg):
        """
        Callback to handle walk commands
        msg.data format: [speed, direction] where:
        - speed: forward speed in m/s
        - direction: direction in radians (0 = forward, pi/2 = left, etc)
        """
        if len(msg.data) >= 2:
            self.walk_speed = msg.data[0]
            self.walk_direction = msg.data[1]
            
            # Start walking if speed is non-zero, stop otherwise
            self.is_walking = abs(self.walk_speed) > 0.01  # Small threshold to avoid noise
            
            if self.is_walking:
                self.get_logger().info(f'Starting to walk at {self.walk_speed:.2f} m/s, direction {math.degrees(self.walk_direction):.1f}°')
            else:
                self.get_logger().info('Stopping walk')
    
    def generate_gait_pattern(self):
        """
        Main function to generate gait patterns based on walking state
        """
        if not self.is_walking:
            # When not walking, keep feet in neutral position
            neutral_positions = self.get_neutral_foot_positions()
            self.publish_foot_targets(neutral_positions[0], neutral_positions[1])
            return
        
        # Update the gait phase based on time and desired speed
        dt = 0.05  # Timer period (seconds)
        step_progress = (self.walk_speed * dt) / self.step_length  # Based on current speed
        self.current_phase = (self.current_phase + step_progress) % 1.0
        
        # Generate foot positions based on current gait phase
        left_foot_pos, right_foot_pos = self.calculate_foot_positions()
        
        # Publish the calculated foot positions
        self.publish_foot_targets(left_foot_pos, right_foot_pos)
        
        # Log current gait state periodically
        if int(self.current_phase * 20) % 10 == 0:  # Every 10% of cycle
            self.get_logger().info(
                f'Gait phase: {self.current_phase:.2f}, '
                f'Left: ({left_foot_pos[0]:.2f}, {left_foot_pos[1]:.2f}), '
                f'Right: ({right_foot_pos[0]:.2f}, {right_foot_pos[1]:.2f})'
            )
    
    def get_neutral_foot_positions(self):
        """
        Get neutral foot positions when not walking
        """
        # Neutral positions: feet shoulder-width apart, directly below hips
        left_foot = [-self.step_width/2, 0.0]  # x, z (x is forward/back, z is up/down)
        right_foot = [self.step_width/2, 0.0]
        return left_foot, right_foot
    
    def calculate_foot_positions(self):
        """
        Calculate foot positions based on current gait phase
        Implements a simple alternating gait pattern
        """
        # Calculate support and swing phases for each leg
        left_support = self.calculate_support_phase(self.current_phase, 0.0)
        right_support = self.calculate_support_phase(self.current_phase, 0.5)
        
        # Calculate swing motion for each leg
        left_swing = self.calculate_swing_trajectory(self.current_phase, 0.5)
        right_swing = self.calculate_swing_trajectory(self.current_phase, 0.0)
        
        # Combine support and swing to get final positions
        left_foot_x = left_support[0] + left_swing[0]
        left_foot_z = left_support[1] + left_swing[1]
        
        right_foot_x = right_support[0] + right_swing[0]
        right_foot_z = right_support[1] + right_swing[1]
        
        return [left_foot_x, left_foot_z], [right_foot_x, right_foot_z]
    
    def calculate_support_phase(self, phase, offset):
        """
        Calculate support phase trajectory for a foot
        offset: Phase offset for alternating legs
        """
        # Adjust phase with offset and wrap around
        adjusted_phase = (phase + offset) % 1.0
        
        # Support phase is when the foot is on the ground moving forward
        if 0.3 < adjusted_phase < 0.7:  # Support phase (30%-70% of gait cycle)
            # Move foot forward based on walking speed and direction
            forward_progress = (adjusted_phase - 0.3) / 0.4  # Normalize to 0-1
            x_offset = self.step_length * (forward_progress - 0.5)  # From -0.5*step_length to +0.5*step_length
            z_offset = 0.0  # Foot remains on ground
            
            # Apply walking direction
            x_final = x_offset * math.cos(self.walk_direction)
            z_final = -self.step_width/2 + x_offset * math.sin(self.walk_direction)  # For left foot
        else:
            # When not in support phase, return to neutral position
            x_final = 0.0
            z_final = -self.step_width/2 if "left" in locals() else self.step_width/2
            
        return x_final, z_final
    
    def calculate_swing_trajectory(self, phase, offset):
        """
        Calculate swing phase trajectory for a foot
        offset: Phase offset for alternating legs
        """
        # Adjust phase with offset and wrap around
        adjusted_phase = (phase + offset) % 1.0
        
        # Swing phase is when the foot is lifted and moved forward
        if adjusted_phase < 0.3 or adjusted_phase > 0.7:  # Swing phase (0-30% and 70-100% of gait cycle)
            if adjusted_phase < 0.3:
                swing_phase = adjusted_phase / 0.3  # Normalize to 0-1
            else:
                swing_phase = (adjusted_phase - 0.7) / 0.3  # Normalize to 0-1
            
            # Forward movement during swing
            x_swing = self.step_length * (swing_phase - 0.5)  # From -0.5*step_length to +0.5*step_length
            
            # Vertical movement during swing (parabolic trajectory)
            # Lift foot at start and end of swing, lower in middle
            height_factor = 4 * swing_phase * (1 - swing_phase)  # Parabolic curve
            z_swing = self.step_height * height_factor
            
            # Apply walking direction
            x_final = x_swing * math.cos(self.walk_direction)
            z_final = x_swing * math.sin(self.walk_direction)
        else:
            # When not in swing phase, no additional movement
            x_final = 0.0
            z_final = 0.0
            
        return x_final, z_final
    
    def publish_foot_targets(self, left_foot_pos, right_foot_pos):
        """
        Publish the calculated foot target positions
        Format: [left_foot_x, left_foot_z_relative, right_foot_x, right_foot_z_relative]
        """
        msg = Float64MultiArray()
        msg.data = [
            left_foot_pos[0],   # left foot x position
            left_foot_pos[1],   # left foot z position
            right_foot_pos[0],  # right foot x position
            right_foot_pos[1]   # right foot z position
        ]
        
        self.foot_target_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = GaitPatternGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Gait Pattern Generator node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()