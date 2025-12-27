#!/usr/bin/env python3
"""
Test script for walking locomotion implementation
This script sends test commands to the walking controller to verify functionality
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import time


class WalkingTest(Node):
    def __init__(self):
        super().__init__('walking_test')
        
        # Publisher for sending walk commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Publisher for gait parameters
        self.gait_param_pub = self.create_publisher(Float64MultiArray, 'gait_parameters', 10)
        
        # Timer to send test commands
        self.timer = self.create_timer(0.1, self.run_test_sequence)
        self.test_step = 0
        self.test_start_time = self.get_clock().now().nanoseconds / 1e9
        
        self.get_logger().info('Walking test node initialized')
    
    def run_test_sequence(self):
        """
        Run a sequence of walking tests
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.test_start_time
        
        cmd = Twist()
        
        if elapsed < 2.0:
            # Initial period - no movement
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Test 0: Initial state (standing still)')
        elif elapsed < 4.0:
            # First test - move forward slowly
            cmd.linear.x = 0.1
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Test 1: Moving forward slowly')
        elif elapsed < 6.0:
            # Second test - move forward faster
            cmd.linear.x = 0.2
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Test 2: Moving forward faster')
        elif elapsed < 8.0:
            # Third test - move backward
            cmd.linear.x = -0.1
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('Test 3: Moving backward')
        elif elapsed < 10.0:
            # Fourth test - turn in place
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.2
            self.get_logger().info('Test 4: Turning in place')
        elif elapsed < 12.0:
            # Fifth test - diagonal movement
            cmd.linear.x = 0.1
            cmd.linear.y = 0.1
            cmd.angular.z = 0.0
            self.get_logger().info('Test 5: Diagonal movement')
        else:
            # Stop after all tests
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info('All tests completed. Stopping.')
            
            # Stop the timer to end the test
            self.timer.cancel()
        
        # Publish the command
        self.cmd_vel_pub.publish(cmd)
        
        # Optionally adjust gait parameters during testing
        if int(elapsed) % 4 == 0 and int(elapsed) > 0:
            self.adjust_gait_parameters()
    
    def adjust_gait_parameters(self):
        """
        Adjust gait parameters to test the adaptation
        """
        gait_msg = Float64MultiArray()
        # Format: [step_height, step_length, step_duration, stride_width]
        gait_msg.data = [0.05, 0.15, 1.0, 0.2]
        self.gait_param_pub.publish(gait_msg)
        self.get_logger().info('Gait parameters updated')


def main(args=None):
    rclpy.init(args=args)
    
    node = WalkingTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Walking test stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()