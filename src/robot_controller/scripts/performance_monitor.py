#!/usr/bin/env python3

"""
Performance monitoring tool for measuring ROS 2 message latency.
This tool measures the latency of messages between publisher and subscriber nodes
to ensure ≤50ms requirements for real-time robot control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String, Header
import time
import statistics
from collections import deque


class LatencyMonitor(Node):
    """
    Node that monitors message latency by sending timestamped messages
    and calculating the round-trip time.
    """
    
    def __init__(self):
        super().__init__('latency_monitor')
        
        # Set up QoS profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        # Publisher and subscriber for latency testing
        self.publisher = self.create_publisher(String, 'latency_test', qos_profile)
        self.subscription = self.create_subscription(
            String,
            'latency_test_result',
            self.result_callback,
            qos_profile
        )
        
        # Track latency measurements
        self.sent_times = {}
        self.latencies = deque(maxlen=100)  # Keep last 100 measurements
        self.message_counter = 0
        
        # Timer to send test messages periodically
        self.timer = self.create_timer(0.1, self.send_test_message)  # 10Hz test messages
        
        self.get_logger().info('Latency Monitor initialized')
        self.get_logger().info('Monitoring message latency for ≤50ms requirement')

    def send_test_message(self):
        """Send a test message with timestamp"""
        current_time = self.get_clock().now().nanoseconds / 1e9  # Convert to seconds
        self.message_counter += 1
        
        # Create message with timestamp and ID
        msg = String()
        msg.data = f"latency_test|{self.message_counter}|{current_time}"
        
        # Store sent time for this message ID
        self.sent_times[self.message_counter] = current_time
        
        self.publisher.publish(msg)
        
        if self.message_counter % 10 == 0:  # Log every 10 messages
            self.get_logger().info(f'Sent latency test message #{self.message_counter}')

    def result_callback(self, msg):
        """Process response from echo node"""
        try:
            # Parse message: "latency_test|id|timestamp|echo"
            parts = msg.data.split('|')
            if len(parts) >= 4 and parts[0] == 'latency_test_echo':
                msg_id = int(parts[1])
                sent_time = float(parts[2])
                
                # Calculate round-trip time
                receive_time = self.get_clock().now().nanoseconds / 1e9
                rtt = receive_time - sent_time
                
                # Store latency (half of round-trip time)
                latency = rtt / 2.0
                self.latencies.append(latency)
                
                # Check if latency meets ≤50ms requirement
                if latency <= 0.050:  # ≤50ms
                    self.get_logger().info(
                        f'Latency test #{msg_id}: {latency*1000:.2f}ms ✓ (≤50ms requirement met)',
                        throttle_duration_sec=1.0
                    )
                else:
                    self.get_logger().error(
                        f'Latency test #{msg_id}: {latency*1000:.2f}ms ✗ (exceeds 50ms requirement!)',
                        throttle_duration_sec=1.0
                    )
                    
        except Exception as e:
            self.get_logger().error(f'Error processing result: {str(e)}')

    def get_statistics(self):
        """Get latency statistics"""
        if not self.latencies:
            return None
            
        return {
            'count': len(self.latencies),
            'mean': statistics.mean(self.latencies) * 1000,  # Convert to ms
            'median': statistics.median(self.latencies) * 1000,  # Convert to ms
            'min': min(self.latencies) * 1000,  # Convert to ms
            'max': max(self.latencies) * 1000,  # Convert to ms
            'p95': sorted(self.latencies)[int(0.95 * len(self.latencies))] * 1000,  # 95th percentile, convert to ms
            'violations': sum(1 for lat in self.latencies if lat > 0.050)  # Count violations of ≤50ms
        }


class LatencyEcho(Node):
    """
    Simple echo node that receives latency test messages and sends them back.
    This helps measure round-trip time.
    """
    
    def __init__(self):
        super().__init__('latency_echo')
        
        # Set up QoS profile
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        
        # Publisher and subscriber
        self.publisher = self.create_publisher(String, 'latency_test_result', qos_profile)
        self.subscription = self.create_subscription(
            String,
            'latency_test',
            self.test_callback,
            qos_profile
        )
        
        self.get_logger().info('Latency Echo initialized')

    def test_callback(self, msg):
        """Echo back the received message"""
        try:
            # Parse message: "latency_test|id|timestamp"
            parts = msg.data.split('|')
            if len(parts) >= 3 and parts[0] == 'latency_test':
                msg_id = parts[1]
                original_timestamp = parts[2]
                
                # Create echo message: "latency_test_echo|id|timestamp"
                echo_msg = String()
                echo_msg.data = f"latency_test_echo|{msg_id}|{original_timestamp}"
                
                self.publisher.publish(echo_msg)
        except Exception as e:
            self.get_logger().error(f'Error in test_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Create both nodes
    monitor_node = LatencyMonitor()
    echo_node = LatencyEcho()
    
    # Use a MultiThreadedExecutor to run both nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(monitor_node)
    executor.add_node(echo_node)
    
    try:
        # Run for a certain period or until interrupted
        print("Starting latency monitoring... Press Ctrl+C to stop and view statistics")
        executor.spin()
    except KeyboardInterrupt:
        print("\nStopping latency monitoring...")
        
        # Print final statistics
        stats = monitor_node.get_statistics()
        if stats:
            print(f"\n=== Latency Statistics ===")
            print(f"Total measurements: {stats['count']}")
            print(f"Mean latency: {stats['mean']:.3f} ms")
            print(f"Median latency: {stats['median']:.3f} ms")
            print(f"Min latency: {stats['min']:.3f} ms")
            print(f"Max latency: {stats['max']:.3f} ms")
            print(f"95th percentile: {stats['p95']:.3f} ms")
            print(f"Violations of ≤50ms requirement: {stats['violations']}")
            
            if stats['max'] <= 50.0:
                print("✓ All messages met the ≤50ms latency requirement")
            else:
                print("✗ Some messages exceeded the ≤50ms latency requirement")
        else:
            print("No measurements collected")
    
    # Cleanup
    monitor_node.destroy_node()
    echo_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()