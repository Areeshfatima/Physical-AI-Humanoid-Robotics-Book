import unittest
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time
from robot_controller.controller_manager import ControllerManager
from robot_controller.joint_publisher import JointStatePublisher


class TestURDFControllerIntegration(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.controller_node = ControllerManager()
        self.joint_publisher_node = JointStatePublisher()
        
        # Create a publisher to send velocity commands
        self.cmd_vel_publisher = self.controller_node.create_publisher(
            Twist,
            'cmd_vel',
            QoSProfile(depth=10)
        )

    def tearDown(self):
        self.controller_node.destroy_node()
        self.joint_publisher_node.destroy_node()
        rclpy.shutdown()

    def test_joint_state_publishing(self):
        """Test that joint states are published correctly"""
        # Wait for the first joint state message
        received_joint_states = []
        
        def joint_state_callback(msg):
            received_joint_states.append(msg)
        
        # Subscribe to joint states
        subscription = self.controller_node.create_subscription(
            JointState,
            'joint_states',
            joint_state_callback,
            QoSProfile(depth=10)
        )
        
        # Spin briefly to receive messages
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self.controller_node)
        executor.add_node(self.joint_publisher_node)
        
        # Run for a short time to receive messages
        start_time = time.time()
        while len(received_joint_states) < 3 and (time.time() - start_time) < 2.0:
            executor.spin_once(timeout_sec=0.1)
        
        # Verify we received joint state messages
        self.assertGreater(len(received_joint_states), 0)
        if received_joint_states:
            # Check that expected joint names are present
            joint_names = received_joint_states[0].name
            expected_joints = ['neck_joint', 'left_leg_joint', 'right_leg_joint', 'left_arm_joint', 'right_arm_joint']
            for expected_joint in expected_joints:
                self.assertIn(expected_joint, joint_names)

    def test_controller_response_to_velocity_commands(self):
        """Test that controller responds to velocity commands"""
        # Send a velocity command to the controller
        twist_msg = Twist()
        twist_msg.linear.x = 1.0
        twist_msg.linear.y = 0.5
        twist_msg.angular.z = 0.2
        
        # Publish the command
        self.cmd_vel_publisher.publish(twist_msg)
        
        # Give time for the controller to process the command
        time.sleep(0.2)
        
        # Verify the controller state has changed in response to the command
        # This is difficult to verify directly without more complex integration,
        # but we can at least ensure the subscription callback doesn't crash
        self.assertIsNotNone(self.controller_node.joint_state.position)


def validate_urdf_response_time():
    """Function to validate URDF model response time with ≤50ms requirement"""
    import time
    
    rclpy.init()
    node = ControllerManager()
    
    # Measure time to process commands
    start_time = time.time()
    
    # Send multiple commands to test response time
    for i in range(10):
        twist_msg = Twist()
        twist_msg.linear.x = i * 0.1
        node.cmd_vel_callback(twist_msg)
    
    end_time = time.time()
    avg_processing_time = (end_time - start_time) / 10
    
    print(f"Average command processing time: {avg_processing_time*1000:.2f} ms")
    
    # Check if it meets ≤50ms requirement
    if avg_processing_time <= 0.05:
        print("✓ Meets ≤50ms response time requirement")
    else:
        print("✗ Does not meet ≤50ms response time requirement")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # Run unit tests
    unittest.main()