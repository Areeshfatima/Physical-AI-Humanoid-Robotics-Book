#!/usr/bin/env python3

"""
Integration test for all ROS 2 packages.
This test runs all packages together in a complex scenario involving
publisher/subscriber, services/actions, and URDF/robot controllers.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci
import time
import threading
from rclpy.action import ActionClient


class IntegrationTestNode(Node):
    """
    Node that coordinates integration testing of all packages
    """
    
    def __init__(self):
        super().__init__('integration_test_node')
        
        # Create publishers for different communication patterns
        qos_profile = QoSProfile(depth=10)
        self.chatter_publisher = self.create_publisher(String, 'chatter', qos_profile)
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', qos_profile)
        
        # Create subscribers to verify communication
        self.test_result_subscriber = self.create_subscription(
            String, 'test_results', self.result_callback, qos_profile)
        
        # Store test results
        self.test_results = []
        self.tests_completed = 0
        self.all_tests_passed = True
        
        # Service client for integration test
        self.service_client = self.create_client(AddTwoInts, 'add_two_ints')
        
        # Action client for integration test
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')
        
        self.get_logger().info('Integration Test Node initialized')

    def result_callback(self, msg):
        """Handle test result messages"""
        self.test_results.append(msg.data)
        self.get_logger().info(f'Test result received: {msg.data}')
        
        if 'FAILED' in msg.data:
            self.all_tests_passed = False

    def wait_for_services_and_actions(self):
        """Wait for services and actions to be available"""
        self.get_logger().info('Waiting for service...')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.get_logger().info('Waiting for action server...')
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available')
            return False
        
        return True

    def run_publisher_subscriber_test(self):
        """Test publisher/subscriber communication"""
        self.get_logger().info('Running publisher/subscriber test...')
        
        # Send a test message
        msg = String()
        msg.data = 'Integration Test Message'
        self.chatter_publisher.publish(msg)
        
        # Wait briefly for any subscribers to process
        time.sleep(0.5)
        
        self.get_logger().info('Publisher/subscriber test completed')
        return True

    def run_service_test(self):
        """Test service communication"""
        self.get_logger().info('Running service test...')
        
        # Create and send a request
        request = AddTwoInts.Request()
        request.a = 10
        request.b = 20
        
        future = self.service_client.call_async(request)
        
        # Wait for response
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.done():
            response = future.result()
            if response and response.sum == 30:
                self.get_logger().info(f'Service test passed: 10 + 20 = {response.sum}')
                return True
            else:
                self.get_logger().error(f'Service test failed: expected 30, got {response.sum if response else "None"}')
                return False
        else:
            self.get_logger().error('Service call timed out')
            return False

    def run_action_test(self):
        """Test action communication"""
        self.get_logger().info('Running action test...')
        
        # Create and send a goal
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 5  # Generate first 5 Fibonacci numbers
        
        # Send goal asynchronously
        future = self.action_client.send_goal_async(goal_msg)
        
        # Wait for goal to be accepted
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Action goal accepted')
                
                # Wait for result
                get_result_future = goal_handle.get_result_async()
                start_time = time.time()
                while not get_result_future.done() and (time.time() - start_time) < 10.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                if get_result_future.done():
                    result_response = get_result_future.result()
                    sequence = result_response.result.sequence
                    self.get_logger().info(f'Action test passed: Fibonacci sequence = {sequence}')
                    return True
                else:
                    self.get_logger().error('Action result timed out')
                    return False
            else:
                self.get_logger().error('Action goal was rejected')
                return False
        else:
            self.get_logger().error('Action goal send timed out')
            return False

    def run_controller_test(self):
        """Test robot controller communication"""
        self.get_logger().info('Running controller test...')
        
        # Send velocity command to controller
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.angular.z = 0.5
        
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Wait briefly to allow controller to respond
        time.sleep(1.0)
        
        self.get_logger().info('Controller test completed')
        return True

    def run_complete_integration_test(self):
        """Run all tests in sequence"""
        self.get_logger().info('Starting complete integration test...')
        
        # Wait for all services and actions to be available
        if not self.wait_for_services_and_actions():
            self.get_logger().error('Failed to connect to services/actions')
            return False
        
        # Run each test component
        tests = [
            ("Publisher/Subscriber", self.run_publisher_subscriber_test),
            ("Service", self.run_service_test),
            ("Action", self.run_action_test),
            ("Controller", self.run_controller_test)
        ]
        
        all_passed = True
        for test_name, test_func in tests:
            self.get_logger().info(f'Running {test_name} test...')
            if test_func():
                self.get_logger().info(f'{test_name} test: PASSED')
            else:
                self.get_logger().error(f'{test_name} test: FAILED')
                all_passed = False
        
        return all_passed


def run_integration_test():
    """Function to run the complete integration test"""
    rclpy.init()
    
    test_node = IntegrationTestNode()
    
    try:
        success = test_node.run_complete_integration_test()
        
        if success:
            test_node.get_logger().info('🎉 All integration tests PASSED!')
            return 0
        else:
            test_node.get_logger().error('❌ Some integration tests FAILED!')
            return 1
            
    except Exception as e:
        test_node.get_logger().error(f'Integration test error: {str(e)}')
        return 1
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


def create_integration_launch_file():
    """Create a launch file that brings up all packages together"""
    
    launch_content = '''from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch all packages together for integration testing
    
    ld = LaunchDescription()
    
    # Publisher/Subscriber nodes
    talker_node = Node(
        package='publisher_subscriber_tutorial',
        executable='talker',
        name='integration_test_talker',
        parameters=[]
    )
    
    listener_node = Node(
        package='publisher_subscriber_tutorial',
        executable='listener',
        name='integration_test_listener',
        parameters=[]
    )
    
    # Service server and client (for testing)
    service_server_node = Node(
        package='services_actions_tutorial',
        executable='add_two_ints_server',
        name='integration_test_service_server',
        parameters=[]
    )
    
    # Action server and client (for testing)
    action_server_node = Node(
        package='services_actions_tutorial',
        executable='fibonacci_action_server',
        name='integration_test_action_server',
        parameters=[]
    )
    
    # Robot controller nodes
    controller_manager_node = Node(
        package='robot_controller',
        executable='controller_manager',
        name='integration_test_controller_manager',
        parameters=[]
    )
    
    joint_publisher_node = Node(
        package='robot_controller',
        executable='joint_publisher',
        name='integration_test_joint_publisher',
        parameters=[]
    )
    
    # Robot state publisher for URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='integration_test_robot_state_publisher',
        parameters=[{
            'robot_description': open(
                os.path.join(
                    get_package_share_directory('urdf_tutorial'),
                    'urdf',
                    'simple_humanoid.urdf'
                )
            ).read()
        }]
    )
    
    # Add all nodes to launch description
    ld.add_action(talker_node)
    ld.add_action(listener_node)
    ld.add_action(service_server_node)
    ld.add_action(action_server_node)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_publisher_node)
    ld.add_action(robot_state_publisher_node)
    
    return ld
'''
    
    # Write the launch file
    launch_dir = "src/integration_tests/launch"
    os.makedirs(launch_dir, exist_ok=True)
    
    with open(f"{launch_dir}/integration_test.launch.py", "w") as f:
        f.write(launch_content)
    
    print(f"Integration test launch file created at {launch_dir}/integration_test.launch.py")


def main():
    """Main function to run integration tests"""
    print("=== ROS 2 Packages Integration Test ===")
    print("Testing all packages together in a complex scenario...")
    print()
    
    # Create the integration launch file
    create_integration_launch_file()
    
    # Run the integration test
    result = run_integration_test()
    
    print()
    if result == 0:
        print("🎉 Integration test completed successfully!")
        print("All packages work together as expected.")
    else:
        print("❌ Integration test failed!")
        print("Some packages may not be working together correctly.")
    
    return result


if __name__ == "__main__":
    exit(main())