#!/usr/bin/env python3

"""
Comprehensive Logging, Metrics, and Tracing for Isaac Sim Components

This module provides comprehensive logging, metrics collection, and tracing
for all Isaac Sim components as required by the observability requirements.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import Log
import json
import time
from datetime import datetime
import psutil  # For system resource monitoring
from collections import deque
import os
from cv_bridge import CvBridge
import numpy as np
import math
from geometry_msgs.msg import Vector3, Quaternion


class IsaacSimObserver(Node):
    """
    Node that provides comprehensive logging, metrics, and tracing for Isaac Sim components
    following observability requirements.
    """
    
    def __init__(self):
        super().__init__('isaac_sim_observer')
        
        # Initialize data collection
        self.metrics_buffer = deque(maxlen=100)  # Keep last 100 metrics entries
        self.log_entries = deque(maxlen=500)     # Keep last 500 log entries
        self.tracing_buffer = deque(maxlen=200)  # Keep last 200 trace entries
        
        # Setup publishers for different types of data
        self.metrics_pub = self.create_publisher(Float64, '/system_metrics', 10)
        self.status_pub = self.create_publisher(String, '/system_status', 10)
        
        # Subscribe to critical topics for monitoring
        self.odom_sub = self.create_subscription(Odometry, '/humanoid_robot/odom', self.odom_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar_scan', self.lidar_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_camera/depth/image_raw', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # CV Bridge for image processing
        self.cv_bridge = CvBridge()
        
        # Timer for periodic system metrics collection
        self.metrics_timer = self.create_timer(1.0, self.collect_system_metrics)
        
        # Timer for publishing aggregated metrics
        self.publish_timer = self.create_timer(0.5, self.publish_aggregated_metrics)
        
        # Resource monitoring setup
        self.process = psutil.Process()
        
        # Performance tracking variables
        self.start_time = time.time()
        self.message_counts = {
            'odom': 0,
            'lidar': 0,
            'depth': 0,
            'imu': 0,
            'total': 0
        }
        
        # Performance goals
        self.target_fps = 30  # Target from spec (≥30 FPS)
        self.max_cpu_usage = 80.0  # Percentage
        self.max_memory_usage = 80.0  # Percentage

        self.get_logger().info('Isaac Sim Observer initialized with comprehensive logging and metrics')
        self.get_logger().info(f'Target FPS: {self.target_fps}, Max CPU: {self.max_cpu_usage}%, Max Memory: {self.max_memory_usage}%')
    
    def odom_callback(self, msg):
        """Monitor odometry messages for metrics and logging"""
        self.message_counts['odom'] += 1
        self.message_counts['total'] += 1
        
        # Log position for debugging
        self.get_logger().debug(
            f'ODOM: pos=({msg.pose.pose.position.x:.3f}, {msg.pose.pose.position.y:.3f}, {msg.pose.pose.position.z:.3f}), '
            f'rot=({msg.pose.pose.orientation.x:.3f}, {msg.pose.pose.orientation.y:.3f}, '
            f'{msg.pose.pose.orientation.z:.3f}, {msg.pose.pose.orientation.w:.3f})'
        )
    
    def lidar_callback(self, msg):
        """Monitor LiDAR messages for metrics and logging"""
        self.message_counts['lidar'] += 1
        self.message_counts['total'] += 1
        
        # Calculate scan rate
        timestamp = self.get_clock().now()
        
        # Log scan statistics
        valid_ranges = [r for r in msg.ranges if r <= msg.range_max and r >= msg.range_min]
        self.get_logger().debug(f'LIDAR: {len(valid_ranges)}/{len(msg.ranges)} valid ranges, min: {min(valid_ranges) if valid_ranges else 0:.2f}m')
        
        # Validate data quality
        if len(valid_ranges) == 0:
            self.get_logger().warn('LiDAR: No valid ranges detected - possible sensor malfunction')
    
    def depth_callback(self, msg):
        """Monitor depth camera messages for metrics and logging"""
        self.message_counts['depth'] += 1
        self.message_counts['total'] += 1
        
        # Convert ROS image to OpenCV format for validation
        try:
            depth_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            
            # Validate image dimensions match expected resolution
            if depth_image.shape[0] != 480 or depth_image.shape[1] != 640:
                self.get_logger().warn(f'DEPTH: Unexpected resolution {depth_image.shape[1]}x{depth_image.shape[0]}, expected 640x480')
            
            # Get valid depths
            valid_depths = depth_image[np.isfinite(depth_image)]
            if len(valid_depths) == 0:
                self.get_logger().warn('DEPTH: No valid depth values in image - possible sensor malfunction')
            else:
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                self.get_logger().debug(f'DEPTH: Resolution {msg.width}x{msg.height}, '
                                      f'Min depth: {min_depth:.2f}m, Max depth: {max_depth:.2f}m')
        except Exception as e:
            self.get_logger().error(f'DEPTH: Error processing image: {e}')
    
    def imu_callback(self, msg):
        """Monitor IMU messages for metrics and logging"""
        self.message_counts['imu'] += 1
        self.message_counts['total'] += 1
        
        # Validate IMU values are within expected ranges
        lin_acc_mag = math.sqrt(msg.linear_acceleration.x**2 + 
                               msg.linear_acceleration.y**2 + 
                               msg.linear_acceleration.z**2)
        ang_vel_mag = math.sqrt(msg.angular_velocity.x**2 + 
                               msg.angular_velocity.y**2 + 
                               msg.angular_velocity.z**2)
        
        if lin_acc_mag > 50.0:  # More than 5g - unusual unless aggressive movement
            self.get_logger().warn(f'IMU: High linear acceleration detected: {lin_acc_mag:.2f} m/s^2')
        
        if ang_vel_mag > 10.0:  # Very high angular velocity
            self.get_logger().warn(f'IMU: High angular velocity detected: {ang_vel_mag:.2f} rad/s')
        
        # Log IMU data
        self.get_logger().debug(
            f'IMU: accel=({msg.linear_acceleration.x:.3f}, {msg.linear_acceleration.y:.3f}, {msg.linear_acceleration.z:.3f}), '
            f'gyro=({msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, {msg.angular_velocity.z:.3f}), '
            f'mag={lin_acc_mag:.2f} m/s^2'
        )
    
    def collect_system_metrics(self):
        """Collect comprehensive system metrics"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Collect system metrics
        metrics = {
            'timestamp': current_time,
            'elapsed_time': elapsed_time,
            'cpu_percent': psutil.cpu_percent(interval=None),
            'memory_percent': psutil.virtual_memory().percent,
            'process_memory_mb': self.process.memory_info().rss / 1024 / 1024,
            'process_cpu_percent': self.process.cpu_percent(),
            'message_counts': self.message_counts.copy(),
            'fps_actual': self.calculate_current_fps(),
            'fps_target': self.target_fps  # Store target for reference
        }
        
        # Add metrics to buffer
        self.metrics_buffer.append(metrics)
    
    def calculate_current_fps(self):
        """Calculate approximate FPS based on message rates"""
        # For Isaac Sim, we approximate FPS based on sensor data rates
        if len(self.metrics_buffer) >= 2:
            time_diff = self.metrics_buffer[-1]['elapsed_time'] - self.metrics_buffer[-2]['elapsed_time']
            if time_diff > 0:
                # Estimate FPS as the rate of depth camera messages (typically published at FPS rate)
                recent_count = self.metrics_buffer[-1]['message_counts']['depth'] - self.metrics_buffer[-2]['message_counts']['depth']
                approx_fps = recent_count / time_diff if time_diff > 0 else 0
                return min(approx_fps, 100)  # Cap at 100 FPS for sanity
            else:
                return 0  # Avoid division by zero
        else:
            return 0  # Insufficient data
    
    def publish_aggregated_metrics(self):
        """Publish aggregated system metrics"""
        if not self.metrics_buffer:
            return
        
        latest_metrics = self.metrics_buffer[-1]
        
        # Create status message
        status_msg = String()
        status_data = {
            'cpu_percent': latest_metrics['cpu_percent'],
            'memory_percent': latest_metrics['memory_percent'],
            'process_memory_mb': latest_metrics['process_memory_mb'],
            'approx_fps': latest_metrics['fps_actual'],
            'target_fps': latest_metrics['fps_target'],
            'message_rates': {
                'odom_hz': latest_metrics['message_counts']['odom'] / latest_metrics['elapsed_time'] if latest_metrics['elapsed_time'] > 0 else 0,
                'lidar_hz': latest_metrics['message_counts']['lidar'] / latest_metrics['elapsed_time'] if latest_metrics['elapsed_time'] > 0 else 0,
                'depth_hz': latest_metrics['message_counts']['depth'] / latest_metrics['elapsed_time'] if latest_metrics['elapsed_time'] > 0 else 0,
                'imu_hz': latest_metrics['message_counts']['imu'] / latest_metrics['elapsed_time'] if latest_metrics['elapsed_time'] > 0 else 0
            },
            'system_status': self.evaluate_system_health(latest_metrics),
            'timestamp': latest_metrics['timestamp']
        }
        
        status_msg.data = json.dumps(status_data, indent=2)
        self.status_pub.publish(status_msg)
        
        # Check for any performance issues and log warnings
        self.check_performance_alerts(latest_metrics)
    
    def evaluate_system_health(self, metrics):
        """Evaluate system health based on collected metrics"""
        issues = []
        
        if metrics['cpu_percent'] > self.max_cpu_usage:
            issues.append(f"CPU usage high: {metrics['cpu_percent']:.1f}%")
        
        if metrics['memory_percent'] > self.max_memory_usage:
            issues.append(f"Memory usage high: {metrics['memory_percent']:.1f}%")
        
        if metrics['fps_actual'] < self.target_fps * 0.8:  # Alert if below 80% of target
            issues.append(f"FPS below target: {metrics['fps_actual']:.1f} vs {self.target_fps}")
        
        if not issues:
            return "OPTIMAL"
        elif len(issues) <= 2:
            return "DEGRADED"
        else:
            return "CRITICAL"
    
    def check_performance_alerts(self, metrics):
        """Check for performance issues and log alerts"""
        if metrics['cpu_percent'] > self.max_cpu_usage * 0.9:
            self.get_logger().warn(f'High CPU usage detected: {metrics["cpu_percent"]:.1f}% (threshold: {self.max_cpu_usage}%)')
        
        if metrics['memory_percent'] > self.max_memory_usage * 0.9:
            self.get_logger().warn(f'High memory usage detected: {metrics["memory_percent"]:.1f}% (threshold: {self.max_memory_usage}%)')
        
        if metrics['fps_actual'] < self.target_fps * 0.7:  # Alert if below 70% of target
            self.get_logger().warn(f'Low FPS detected: {metrics["fps_actual"]:.1f} vs target {self.target_fps}')
        
        # Log periodic summary
        if int(metrics['elapsed_time']) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(
                f'System Health: CPU={metrics["cpu_percent"]:.1f}%, '
                f'Memory={metrics["memory_percent"]:.1f}%, '
                f'Approx FPS={metrics["fps_actual"]:.1f}, '
                f'Messages={metrics["message_counts"]["total"]}, '
                f'Status={self.evaluate_system_health(metrics)}'
            )


def main(args=None):
    """
    Main function to run the logging and metrics system
    """
    rclpy.init(args=args)
    
    observer_node = IsaacSimObserver()
    
    print("Comprehensive Logging and Metrics System initialized...")
    print("Monitoring Isaac Sim components for observability requirements")
    
    try:
        rclpy.spin(observer_node)
    except KeyboardInterrupt:
        observer_node.get_logger().info('Logging and metrics system stopped by user')
        
        # Print final status summary if there are metrics
        if observer_node.metrics_buffer:
            final_metrics = observer_node.metrics_buffer[-1]
            print("\n" + "="*60)
            print("FINAL SYSTEM METRICS SUMMARY")
            print("="*60)
            print(f"Runtime: {final_metrics['elapsed_time']:.2f} seconds")
            print(f"Messages processed: {final_metrics['message_counts']['total']}")
            print(f"Current CPU usage: {final_metrics['cpu_percent']:.1f}%")
            print(f"Current memory usage: {final_metrics['memory_percent']:.1f}%")
            print(f"Approximate FPS: {final_metrics['fps_actual']:.2f}")
            print(f"Target FPS: {final_metrics['fps_target']}")
            print(f"System health status: {observer_node.evaluate_system_health(final_metrics)}")
            print("="*60)
    finally:
        observer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()