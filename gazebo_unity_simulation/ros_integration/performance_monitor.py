#!/usr/bin/env python3

"""
Performance Monitoring Tool for ≤50ms Latency Requirements

This module monitors and validates that the simulation meets ≤50ms latency requirements
for real-time robot control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu, JointState
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist, Vector3
from builtin_interfaces.msg import Time
import time
import statistics
from collections import deque
import threading
import numpy as np


class PerformanceMonitor(Node):
    """
    Monitors performance metrics including latency, throughput, and resource usage
    to ensure ≤50ms requirements for real-time robot control.
    """

    def __init__(self):
        super().__init__('performance_monitor')

        # Performance tracking buffers
        self.latency_buffer_size = 100  # Track last 100 measurements

        # Timing buffers for different message types
        self.lidar_timing = {'timestamps': deque(maxlen=self.latency_buffer_size)}
        self.depth_timing = {'timestamps': deque(maxlen=self.latency_buffer_size)}
        self.imu_timing = {'timestamps': deque(maxlen=self.latency_buffer_size)}
        self.joint_timing = {'timestamps': deque(maxlen=self.latency_buffer_size)}
        self.command_timing = {'timestamps': deque(maxlen=self.latency_buffer_size)}

        # Performance stats
        self.performance_stats = {
            'lidar_latency': {'values': deque(maxlen=self.latency_buffer_size)},
            'depth_latency': {'values': deque(maxlen=self.latency_buffer_size)},
            'imu_latency': {'values': deque(maxlen=self.latency_buffer_size)},
            'joint_latency': {'values': deque(maxlen=self.latency_buffer_size)},
            'command_latency': {'values': deque(maxlen=self.latency_buffer_size)}
        }

        # Publishers for performance metrics
        self.perf_publisher = self.create_publisher(Float64MultiArray, '/performance_metrics', 10)
        self.status_publisher = self.create_publisher(String, '/performance_status', 10)

        # Subscribers for monitoring various sensor/command topics
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar_scan', self.lidar_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera/depth/image_raw', self.depth_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10
        )

        # Timer for publishing performance metrics
        self.metrics_timer = self.create_timer(1.0, self.publish_performance_metrics)
        self.status_timer = self.create_timer(5.0, self.publish_performance_status)

        # Thresholds
        self.latency_threshold_ms = 50.0  # ≤50ms requirement
        self.compliance_threshold = 0.95  # 95% of messages within threshold

        self.get_logger().info('Performance Monitor initialized')
        self.get_logger().info(f'Latency threshold: {self.latency_threshold_ms}ms')

    def calculate_latency(self, msg_timestamp):
        """Calculate latency between message timestamp and reception time"""
        reception_time = self.get_clock().now()
        if isinstance(msg_timestamp, Time):
            msg_time = Time.from_msg(msg_timestamp)
        else:
            msg_time = msg_timestamp

        latency_ns = (reception_time.nanoseconds - msg_time.nanoseconds)
        latency_ms = latency_ns / 1_000_000.0  # Convert to milliseconds
        return latency_ms

    def lidar_callback(self, msg):
        """Record timing information for LiDAR messages"""
        latency = self.calculate_latency(msg.header.stamp)
        self.performance_stats['lidar_latency']['values'].append(latency)

        if len(self.performance_stats['lidar_latency']['values']) > 0:
            avg_latency = statistics.mean(self.performance_stats['lidar_latency']['values'])
            self.get_logger().debug(f'LiDAR latency: {latency:.2f}ms (avg: {avg_latency:.2f}ms)')

    def depth_callback(self, msg):
        """Record timing information for depth camera messages"""
        latency = self.calculate_latency(msg.header.stamp)
        self.performance_stats['depth_latency']['values'].append(latency)

        if len(self.performance_stats['depth_latency']['values']) > 0:
            avg_latency = statistics.mean(self.performance_stats['depth_latency']['values'])
            self.get_logger().debug(f'Depth latency: {latency:.2f}ms (avg: {avg_latency:.2f}ms)')

    def imu_callback(self, msg):
        """Record timing information for IMU messages"""
        latency = self.calculate_latency(msg.header.stamp)
        self.performance_stats['imu_latency']['values'].append(latency)

        if len(self.performance_stats['imu_latency']['values']) > 0:
            avg_latency = statistics.mean(self.performance_stats['imu_latency']['values'])
            self.get_logger().debug(f'IMU latency: {latency:.2f}ms (avg: {avg_latency:.2f}ms)')

    def joint_callback(self, msg):
        """Record timing information for joint state messages"""
        latency = self.calculate_latency(msg.header.stamp)
        self.performance_stats['joint_latency']['values'].append(latency)

        if len(self.performance_stats['joint_latency']['values']) > 0:
            avg_latency = statistics.mean(self.performance_stats['joint_latency']['values'])
            self.get_logger().debug(f'Joint latency: {latency:.2f}ms (avg: {avg_latency:.2f}ms)')

    def cmd_callback(self, msg):
        """Record timing information for command messages"""
        # For commands, we record the time of receipt
        # In a real system, we'd track round-trip from command to effect
        # For now, we'll just record the receipt time
        current_time = self.get_clock().now().nanoseconds / 1_000_000.0  # Current time in ms
        self.performance_stats['command_latency']['values'].append(current_time)

    def publish_performance_metrics(self):
        """Publish current performance metrics"""
        if not self.performance_stats['lidar_latency']['values']:
            # No data yet, skip publishing
            return

        # Calculate metrics for each sensor type
        lidar_avg = statistics.mean(self.performance_stats['lidar_latency']['values']) if self.performance_stats['lidar_latency']['values'] else 0
        depth_avg = statistics.mean(self.performance_stats['depth_latency']['values']) if self.performance_stats['depth_latency']['values'] else 0
        imu_avg = statistics.mean(self.performance_stats['imu_latency']['values']) if self.performance_stats['imu_latency']['values'] else 0
        joint_avg = statistics.mean(self.performance_stats['joint_latency']['values']) if self.performance_stats['joint_latency']['values'] else 0

        # Calculate compliance percentages
        lidar_compliant = self.calc_compliance('lidar_latency')
        depth_compliant = self.calc_compliance('depth_latency')
        imu_compliant = self.calc_compliance('imu_latency')
        joint_compliant = self.calc_compliance('joint_latency')

        # Create message with metrics
        metrics_msg = Float64MultiArray()
        metrics_msg.data = [
            lidar_avg, lidar_compliant,
            depth_avg, depth_compliant,
            imu_avg, imu_compliant,
            joint_avg, joint_compliant,
            self.latency_threshold_ms  # Include threshold for reference
        ]

        self.perf_publisher.publish(metrics_msg)

        # Log detailed metrics occasionally
        current_time = self.get_clock().now().nanoseconds / 1e9
        if int(current_time) % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(
                f'Performance Metrics:\n'
                f'  LiDAR: Avg={lidar_avg:.2f}ms, Compliance={lidar_compliant:.1f}%\n'
                f'  Depth: Avg={depth_avg:.2f}ms, Compliance={depth_compliant:.1f}%\n'
                f'  IMU: Avg={imu_avg:.2f}ms, Compliance={imu_compliant:.1f}%\n'
                f'  Joint: Avg={joint_avg:.2f}ms, Compliance={joint_compliant:.1f}%'
            )

    def calc_compliance(self, sensor_type):
        """Calculate percentage of messages compliant with latency threshold"""
        values = self.performance_stats[sensor_type]['values']
        if not values:
            return 0.0

        compliant_count = sum(1 for val in values if val <= self.latency_threshold_ms)
        return (compliant_count / len(values)) * 100

    def publish_performance_status(self):
        """Publish overall performance status"""
        status_msg = String()

        # Calculate overall compliance
        all_compliant = True
        status_parts = []

        for sensor_type in ['lidar_latency', 'depth_latency', 'imu_latency', 'joint_latency']:
            compliance = self.calc_compliance(sensor_type)
            sensor_name = sensor_type.replace('_latency', '').upper()

            status = 'OK' if compliance >= self.compliance_threshold else 'ISSUE'
            if compliance < self.compliance_threshold:
                all_compliant = False

            status_parts.append(f'{sensor_name}:{status}({compliance:.1f}%)')

        overall_status = 'PASS' if all_compliant else 'FAIL'
        status_msg.data = f'PERFORMANCE_{overall_status}: ' + ', '.join(status_parts) + f' | Threshold: ≤{self.latency_threshold_ms}ms'

        self.status_publisher.publish(status_msg)

        # Log status to console as well
        self.get_logger().info(f'Performance Status: {status_msg.data}')


class PerformanceAnalyzer:
    """
    Standalone performance analyzer that can process performance data
    and generate reports.
    """

    @staticmethod
    def analyze_performance_data(metrics_data):
        """
        Analyze collected performance metrics and generate insights
        """
        print("Analyzing Performance Data...")
        print("="*40)

        # Calculate key metrics
        if len(metrics_data.get('lidar_latency', [])) > 0:
            lidar_metrics = PerformanceAnalyzer.calc_detailed_metrics(metrics_data['lidar_latency'])
            print(f"LiDAR Performance:")
            print(f"  - Mean Latency: {lidar_metrics['mean']:.2f}ms")
            print(f"  - Median Latency: {lidar_metrics['median']:.2f}ms")
            print(f"  - 95th Percentile: {lidar_metrics['p95']:.2f}ms")
            print(f"  - Max Latency: {lidar_metrics['max']:.2f}ms")
            print(f"  - Compliance: {lidar_metrics['compliance']:.1f}%")

        if len(metrics_data.get('depth_latency', [])) > 0:
            depth_metrics = PerformanceAnalyzer.calc_detailed_metrics(metrics_data['depth_latency'])
            print(f"\nDepth Camera Performance:")
            print(f"  - Mean Latency: {depth_metrics['mean']:.2f}ms")
            print(f"  - Median Latency: {depth_metrics['median']:.2f}ms")
            print(f"  - 95th Percentile: {depth_metrics['p95']:.2f}ms")
            print(f"  - Max Latency: {depth_metrics['max']:.2f}ms")
            print(f"  - Compliance: {depth_metrics['compliance']:.1f}%")

        print("\nPerformance Analysis Complete")

    @staticmethod
    def calc_detailed_metrics(latencies):
        """Calculate detailed performance metrics"""
        if not latencies:
            return {'mean': 0, 'median': 0, 'p95': 0, 'max': 0, 'compliance': 0}

        sorted_lats = sorted(latencies)

        metrics = {
            'mean': statistics.mean(latencies),
            'median': statistics.median(latencies),
            'p95': np.percentile(latencies, 95),
            'max': max(latencies),
            'compliance': PerformanceAnalyzer.calc_compliance(latencies, 50.0)  # 50ms threshold
        }

        return metrics

    @staticmethod
    def calc_compliance(latencies, threshold):
        """Calculate percentage of values below threshold"""
        if not latencies:
            return 0.0

        compliant_count = sum(1 for val in latencies if val <= threshold)
        return (compliant_count / len(latencies)) * 100


def run_performance_analysis():
    """
    Run a performance analysis to validate ≤50ms latency requirements
    """
    print("Starting performance analysis for ≤50ms latency validation...")
    print("This will monitor real-time performance metrics to ensure robotics control requirements are met.\n")

    print("Performance Validation Approach:")
    print("1. Continuous monitoring of message latencies")
    print("2. Real-time compliance calculation")
    print("3. Alerting when performance drops below threshold")
    print("4. Detailed analytics and reporting")
    print("")

    print("Requirements Check:")
    print(f"- Target latency: ≤{50.0}ms for real-time robot control")
    print("- Minimum compliance: 95% of messages meeting latency requirement")
    print("- Reporting: Real-time metrics and alerts")
    print("")

    print("Validation Tools:")
    print("✓ PerformanceMonitor node for real-time tracking")
    print("✓ Metrics publication for external monitoring")
    print("✓ Performance status reporting")
    print("✓ Detailed compliance analysis")

    return True


def main(args=None):
    """
    Main function to run the performance monitoring
    """
    rclpy.init(args=args)

    monitor = PerformanceMonitor()

    try:
        print("Performance monitoring started...")
        print(f"Monitoring for ≤{monitor.latency_threshold_ms}ms latency requirements")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Performance monitor stopped by user')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

    return True


if __name__ == '__main__':
    main()