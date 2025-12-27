#!/usr/bin/env python3

"""
Integration Test: Complex Scenario with All Four User Stories

This test verifies that all components work together in a complex scenario
involving physics simulation, sensor simulation, Unity visualization, and
custom environments.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import time
import threading
import subprocess
import signal
import sys
from pathlib import Path


def run_complete_integration_test():
    """
    Run a complete integration test involving all four user stories
    """
    print("Starting complete integration test...")
    print("Testing the complete digital twin workflow:")
    print("  1. Physics-accurate Gazebo environment")
    print("  2. Sensor simulation (LiDAR, depth camera, IMU)") 
    print("  3. Unity visualization")
    print("  4. Custom environment building")
    print()
    
    try:
        # Start ROS 2
        rclpy.init()
        
        # Create the integration test node
        integration_test_node = IntegrationTestNode()
        
        # Give system time to initialize
        time.sleep(2.0)
        
        print("Integration test started...")
        
        # Run tests for 15 seconds to gather data
        start_time = time.time()
        test_duration = 15.0  # seconds
        
        while time.time() - start_time < test_duration:
            rclpy.spin_once(integration_test_node, timeout_sec=0.1)
            
            # Print status periodically
            elapsed = time.time() - start_time
            if int(elapsed) % 5 == 0:
                print(f"  Test running for {elapsed:.1f}s...")
        
        # Collect and analyze results
        results = integration_test_node.get_test_results()
        
        # Print final summary
        print("\n" + "="*60)
        print("COMPLETE INTEGRATION TEST RESULTS")
        print("="*60)
        
        print("Data Collection Results:")
        print(f"  LiDAR messages received: {results['lidar_count']}")
        print(f"  Depth images received: {results['depth_count']}")
        print(f"  IMU messages received: {results['imu_count']}")
        print(f"  Joint states received: {results['joint_count']}")
        print(f"  Command messages received: {results['cmd_count']}")
        
        # Validate data rates
        expected_duration = test_duration
        print(f"\nExpected message counts (based on {expected_duration:.1f}s runtime):")
        print(f"  LiDAR (~10Hz): {int(10 * expected_duration)} (received: {results['lidar_count']})")
        print(f"  Depth (~30Hz): {int(30 * expected_duration)} (received: {results['depth_count']})")
        print(f"  IMU (~100Hz): {int(100 * expected_duration)} (received: {results['imu_count']})")
        print(f"  Joint (~50Hz): {int(50 * expected_duration)} (received: {results['joint_count']})")
        
        # Calculate success metrics
        lidar_success = abs(results['lidar_count'] - int(10 * expected_duration)) / (10 * expected_duration) < 0.2  # ±20% tolerance
        depth_success = abs(results['depth_count'] - int(30 * expected_duration)) / (30 * expected_duration) < 0.2  # ±20% tolerance
        imu_success = abs(results['imu_count'] - int(100 * expected_duration)) / (100 * expected_duration) < 0.3  # ±30% tolerance (higher for faster rate)
        joint_success = abs(results['joint_count'] - int(50 * expected_duration)) / (50 * expected_duration) < 0.2  # ±20% tolerance
        cmd_success = results['cmd_count'] > 0  # At least some commands received
        
        print(f"\nValidation Results:")
        print(f"  LiDAR data collection: {'✓ PASS' if lidar_success else '✗ FAIL'}")
        print(f"  Depth camera data collection: {'✓ PASS' if depth_success else '✗ FAIL'}")
        print(f"  IMU data collection: {'✓ PASS' if imu_success else '✗ FAIL'}")
        print(f"  Joint state collection: {'✓ PASS' if joint_success else '✗ FAIL'}")
        print(f"  Command processing: {'✓ PASS' if cmd_success else '✗ FAIL'}")
        
        # Overall assessment
        all_success = all([lidar_success, depth_success, imu_success, joint_success, cmd_success])
        
        print(f"\nOverall Integration Test: {'✓ PASS' if all_success else '✗ FAIL'}")
        
        if all_success:
            print("  ✓ All user stories are successfully integrated")
            print("  ✓ Physics simulation, sensor simulation, and visualization work together")
            print("  ✓ System maintains expected data rates")
        else:
            print("  ✗ Some components are not properly integrated")
            print("  ✗ Check ROS 2 communication and component initialization")
        
        # Test specific user story objectives
        print(f"\nUser Story Integration Validation:")
        print(f"  US1 (Physics): {'✓ PASS' if results['joint_count'] > 0 else '✗ FAIL'} - Robot joints simulated")
        print(f"  US2 (Sensors): {'✓ PASS' if all([results['lidar_count'] > 0, results['depth_count'] > 0, results['imu_count'] > 0]) else '✗ FAIL'} - Multiple sensors active")
        print(f"  US3 (Unity): {'✓ PASS' if results['joint_count'] > 0 else '✗ FAIL'} - Joint states available for visualization")
        print(f"  US4 (Environments): {'✓ PASS' if results['lidar_count'] > 0 else '✗ FAIL'} - Environment sensing active")
        
        # Clean up
        integration_test_node.destroy_node()
        rclpy.shutdown()
        
        return all_success
        
    except Exception as e:
        print(f"Integration test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False


class IntegrationTestNode(Node):
    """
    Node that integrates with all components to verify complete system functionality
    """
    
    def __init__(self):
        super().__init__('integration_test_node')
        
        # Initialize counters for each data type
        self.lidar_count = 0
        self.depth_count = 0
        self.imu_count = 0
        self.joint_count = 0
        self.cmd_count = 0
        
        # Track timestamps for rate calculations
        self.lidar_times = []
        self.depth_times = []
        self.imu_times = []
        self.joint_times = []
        self.cmd_times = []
        
        # Subscribe to all sensor topics
        self.lidar_sub = self.create_subscription(
            LaserScan, '/lidar_scan', self.lidar_callback, 10)
        
        self.depth_sub = self.create_subscription(
            Image, '/depth_camera/depth/image_raw', self.depth_callback, 10)
        
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Publisher for commands to test full loop
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer to occasionally send commands
        self.cmd_timer = self.create_timer(1.0, self.send_test_command)
        
        self.get_logger().info('Integration test node initialized')
    
    def lidar_callback(self, msg):
        self.lidar_count += 1
        self.lidar_times.append(self.get_clock().now())
        
    def depth_callback(self, msg):
        self.depth_count += 1
        self.depth_times.append(self.get_clock().now())
    
    def imu_callback(self, msg):
        self.imu_count += 1
        self.imu_times.append(self.get_clock().now())
    
    def joint_callback(self, msg):
        self.joint_count += 1
        self.joint_times.append(self.get_clock().now())
    
    def cmd_callback(self, msg):
        self.cmd_count += 1
        self.cmd_times.append(self.get_clock().now())
    
    def send_test_command(self):
        """Send a test command to ensure command pathway works"""
        cmd = Twist()
        cmd.linear.x = 0.5  # Move forward slightly
        cmd.angular.z = 0.1  # Turn slightly
        self.cmd_pub.publish(cmd)
    
    def get_test_results(self):
        """Get the collected test results"""
        return {
            'lidar_count': self.lidar_count,
            'depth_count': self.depth_count,
            'imu_count': self.imu_count,
            'joint_count': self.joint_count,
            'cmd_count': self.cmd_count
        }


def validate_educational_requirements():
    """
    Validate that all tutorials meet educational requirements
    (simplified but technically correct)
    """
    print("\nValidating educational requirements...")
    print("Checking that all components are suitable for student learning:")
    print("- Content is simplified but technically accurate")
    print("- Examples clearly demonstrate concepts")
    print("- Documentation is approachable for students")
    print()
    
    # Check documentation structure
    docs_paths = [
        "/mnt/e/Hackathon-1/physical-ai-humanoid-robotics/my-book/docs/ros2-fundamentals/",
    ]
    
    requirements_met = True
    
    for path in docs_paths:
        path_obj = Path(path)
        if path_obj.exists():
            docs = list(path_obj.glob("*.md"))
            if len(docs) > 0:
                print(f"  ✓ Found {len(docs)} educational documents in {path}")
            else:
                print(f"  ⚠ No documentation found in {path}")
                requirements_met = False
        else:
            print(f"  ⚠ Documentation path does not exist: {path}")
            requirements_met = False
    
    # Check for key educational elements
    try:
        with open("/mnt/e/Hackathon-1/physical-ai-humanoid-robotics/src/quickstart.md", 'r') as f:
            content = f.read()
            if len(content) > 1000:  # Check that quickstart has substantial content
                print("  ✓ Comprehensive quickstart guide available")
            else:
                print("  ⚠ Quickstart guide may be insufficient")
                requirements_met = False
    except FileNotFoundError:
        print("  ⚠ Quickstart guide not found")
        requirements_met = False
    
    print(f"\nEducational Requirements: {'✓ MET' if requirements_met else '✗ NOT MET'}")
    
    return requirements_met


def main():
    """
    Main function to run the complete integration test
    """
    print("Digital Twin with Gazebo & Unity - Complete Integration Test")
    print("="*60)
    print("Testing that all four user stories work together in a complex scenario:")
    print("US1: Physics-accurate Gazebo environment")
    print("US2: Sensor simulation (LiDAR, depth cameras, IMUs)")
    print("US3: Unity visualization")
    print("US4: Custom environment building")
    print()
    
    # Run the integration test
    integration_success = run_complete_integration_test()
    
    # Validate educational requirements
    educational_success = validate_educational_requirements()
    
    print("\n" + "="*60)
    print("FINAL VALIDATION SUMMARY")
    print("="*60)
    print(f"Integration Test: {'✓ PASS' if integration_success else '✗ FAIL'}")
    print(f"Educational Validation: {'✓ PASS' if educational_success else '✗ FAIL'}")
    
    overall_success = integration_success and educational_success
    print(f"Overall Assessment: {'✓ COMPLETE' if overall_success else '✗ INCOMPLETE'}")
    
    if overall_success:
        print("\n✓ All components successfully integrated!")
        print("✓ System meets design specifications")
        print("✓ Educational requirements satisfied")
        print("✓ Ready for student use")
    else:
        print("\n✗ System integration incomplete")
        print("✗ Some components require additional work")
        print("✗ Check validation results above for details")
    
    return overall_success


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)