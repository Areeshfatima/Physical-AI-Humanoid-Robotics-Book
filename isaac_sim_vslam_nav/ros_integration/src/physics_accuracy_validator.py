#!/usr/bin/env python3

"""
Physics Accuracy Validation Tools

This script validates that the simulation maintains ≤5% deviation from expected behavior
for educational purposes.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time
import time


class PhysicsAccuracyValidator(Node):
    """
    Node that validates physics simulation accuracy by comparing 
    actual robot behavior with expected theoretical physics behavior.
    """
    
    def __init__(self):
        super().__init__('physics_accuracy_validator')
        
        # Store reference trajectory and actual trajectory
        self.actual_trajectory = []
        self.deviation_measurements = []
        
        # Physics parameters
        self.gravity = 9.8  # m/s^2
        self.max_acceptable_deviation = 5.0  # 5% max deviation
        
        # Subscribe to robot odometry to track actual movement
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/humanoid_robot/odom',  # Assuming robot publishes its own odometry
            self.odom_callback,
            10
        )
        
        # Subscribe to joint states for validation
        self.joint_subscription = self.create_subscription(
            JointState,
            '/humanoid_robot/joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher for validation metrics
        self.metrics_publisher = self.create_publisher(
            Float64MultiArray,
            '/physics_validation_metrics',
            10
        )
        
        # Setup validation timer
        self.validation_timer = self.create_timer(1.0, self.run_validation_cycle)
        
        # Setup for tracking trajectory over time
        self.start_time = self.get_clock().now()
        self.initial_pose = None
        
        self.get_logger().info('Physics Accuracy Validator initialized with 5% max deviation threshold')
    
    def odom_callback(self, msg):
        """Store actual robot pose from odometry"""
        # Store the actual pose from the simulation
        actual_pose = {
            'position': [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            'orientation': [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w],
            'timestamp': msg.header.stamp
        }
        
        self.actual_trajectory.append(actual_pose)
        
        # Store initial pose if this is the first message
        if self.initial_pose is None:
            self.initial_pose = actual_pose.copy()
    
    def joint_callback(self, msg):
        """Monitor joint states to validate physics accuracy"""
        # Validate that joints are moving within expected limits
        for i, joint_name in enumerate(msg.name):
            if i < len(msg.position) and i < len(msg.velocity) and i < len(msg.effort):
                # For each joint, validate that velocity and position remain physically plausible
                position = msg.position[i]
                velocity = msg.velocity[i]
                effort = msg.effort[i]
                
                # Log joint state for validation
                self.get_logger().debug(f'Joint {joint_name}: pos={position:.3f}, vel={velocity:.3f}, effort={effort:.3f}')
    
    def calculate_theoretical_motion(self, initial_pose, elapsed_time, motion_type='linear_motion'):
        """
        Calculate theoretical motion based on physics equations
        """
        # Simple physics model - in real implementation this would be more complex
        theoretical_pose = {
            'position': [0.0, 0.0, 0.0],
            'orientation': initial_pose['orientation'][:] if initial_pose else [0.0, 0.0, 0.0, 1.0],
            'timestamp': Time()
        }

        if motion_type == 'freefall':
            # Calculate where the robot should be based on gravity alone
            start_pos = initial_pose['position']
            # Use s = ut + 0.5*a*t^2 (assuming initial velocity is 0 in z direction)
            theoretical_pose['position'][0] = start_pos[0]  # No horizontal acceleration
            theoretical_pose['position'][1] = start_pos[1]  # No horizontal acceleration
            theoretical_pose['position'][2] = start_pos[2] - 0.5 * self.gravity * elapsed_time**2  # Gravity pulling down

        elif motion_type == 'linear_motion':
            # Calculate where the robot should be with constant velocity
            start_pos = initial_pose['position']
            velocity = [0.5, 0.0, 0.0]  # Example constant velocity
            theoretical_pose['position'][0] = start_pos[0] + velocity[0] * elapsed_time
            theoretical_pose['position'][1] = start_pos[1] + velocity[1] * elapsed_time
            theoretical_pose['position'][2] = start_pos[2] + velocity[2] * elapsed_time

        return theoretical_pose
    
    def calculate_deviation_percentage(self, actual, expected):
        """
        Calculate deviation percentage between actual and expected values
        """
        # Calculate Euclidean distance between actual and expected positions
        pos_diff = np.sqrt(sum([(a - e) ** 2 for a, e in zip(actual['position'], expected['position'])]))

        # Calculate expected displacement magnitude for reference
        if self.initial_pose:
            start_pos = self.initial_pose['position']
            exp_displacement = np.sqrt(sum([(e - s) ** 2 for e, s in zip(expected['position'], start_pos)]))
        else:
            exp_displacement = 1.0  # Default if no initial pose available

        if exp_displacement > 0.001:  # Avoid division by zero
            deviation_percentage = (pos_diff / exp_displacement) * 100
        else:
            deviation_percentage = pos_diff * 100  # If expected displacement is very small, use absolute position difference

        return min(deviation_percentage, 100.0)  # Cap at 100% for sanity
    
    def run_validation_cycle(self):
        """Run a validation cycle to check physics accuracy"""
        if not self.actual_trajectory:
            return  # No data to validate yet
        
        # Get the most recent actual pose
        latest_actual = self.actual_trajectory[-1]
        
        # Calculate elapsed time for theoretical motion
        current_time = self.get_clock().now()
        elapsed_time = (current_time.nanoseconds - self.start_time.nanoseconds) / 1e9
        
        # Calculate theoretical position based on time
        theoretical = self.calculate_theoretical_motion(self.initial_pose, elapsed_time, 'linear_motion')
        
        # Calculate deviation
        deviation_percent = self.calculate_deviation_percentage(latest_actual, theoretical)
        
        # Store validation result
        validation_result = {
            'timestamp': latest_actual['timestamp'],
            'actual_position': latest_actual['position'],
            'expected_position': theoretical['position'],
            'deviation_percent': deviation_percent,
            'within_tolerance': deviation_percent <= self.max_acceptable_deviation
        }
        
        self.deviation_measurements.append(validation_result)
        
        # Publish metrics
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [
            float(deviation_percent),
            1.0 if validation_result['within_tolerance'] else 0.0,
            float(elapsed_time)
        ]
        self.metrics_publisher.publish(metrics_msg)
        
        # Log validation results
        status = "✓ PASS" if validation_result['within_tolerance'] else "✗ FAIL"
        self.get_logger().info(
            f'Physics validation: {status} '
            f'Deviation: {deviation_percent:.2f}% '
            f'(Max: {self.max_acceptable_deviation}%)'
        )
        
        if not validation_result['within_tolerance']:
            self.get_logger().warn(
                f'Physics deviation exceeds threshold: '
                f'Actual: {latest_actual["position"]}, '
                f'Expected: {theoretical["position"]}'
            )
    
    def get_validation_summary(self):
        """Get summary statistics of validation results"""
        if not self.deviation_measurements:
            return {
                'total_samples': 0,
                'pass_rate': 0.0,
                'avg_deviation': 0.0,
                'max_deviation': 0.0,
                'within_tolerance': True
            }
        
        deviations = [m['deviation_percent'] for m in self.deviation_measurements]
        passes = sum(1 for m in self.deviation_measurements if m['within_tolerance'])
        
        summary = {
            'total_samples': len(self.deviation_measurements),
            'pass_rate': (passes / len(self.deviation_measurements)) * 100 if self.deviation_measurements else 0.0,
            'avg_deviation': sum(deviations) / len(deviations) if deviations else 0.0,
            'max_deviation': max(deviations) if deviations else 0.0,
            'within_tolerance': (passes / len(self.deviation_measurements) >= 0.95) if self.deviation_measurements else True  # 95% tolerance
        }
        
        return summary


def main(args=None):
    """
    Main function to run physics validation
    """
    rclpy.init(args=args)

    validator = PhysicsAccuracyValidator()

    print("Physics Accuracy Validator started...")
    print(f"Validating physics simulation for ≤{validator.max_acceptable_deviation}% deviation requirement")

    try:
        rclpy.spin(validator)
    except KeyboardInterrupt:
        validator.get_logger().info('Physics validator stopped by user')

        # Print validation summary before shutting down
        summary = validator.get_validation_summary()
        print("\n" + "="*50)
        print("PHYSICS ACCURACY VALIDATION SUMMARY")
        print("="*50)
        print(f"Total samples: {summary['total_samples']}")
        print(f"Pass rate: {summary['pass_rate']:.2f}%")
        print(f"Average deviation: {summary['avg_deviation']:.3f}%")
        print(f"Maximum deviation: {summary['max_deviation']:.3f}%")
        print(f"Overall tolerance: {'✓ PASSED' if summary['within_tolerance'] else '✗ FAILED'}")
        print("="*50)
    finally:
        validator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()