import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import math
import time


class JointStatePublisher(Node):

    def __init__(self):
        super().__init__('joint_state_publisher')

        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Message for joint states
        self.joint_state = JointState()
        
        # Initialize joint names
        self.joint_state.name = [
            'neck_joint', 
            'left_leg_joint', 
            'right_leg_joint', 
            'left_arm_joint', 
            'right_arm_joint'
        ]
        
        # Set initial positions
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set initial velocities and efforts to zero
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Timer for publishing joint states
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Initialize time for animation
        self.start_time = time.time()

    def timer_callback(self):
        # Update joint states with animated values
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # Animate joints with different frequencies
        self.joint_state.position[0] = 0.2 * math.sin(elapsed_time)  # neck joint
        self.joint_state.position[1] = 0.1 * math.sin(elapsed_time * 0.5)  # left leg
        self.joint_state.position[2] = 0.1 * math.sin(elapsed_time * 0.5)  # right leg
        self.joint_state.position[3] = 0.3 * math.sin(elapsed_time * 0.7)  # left arm
        self.joint_state.position[4] = 0.3 * math.sin(elapsed_time * 0.7)  # right arm
        
        # Update header
        self.joint_state.header = Header()
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.joint_state.header.frame_id = 'base_link'
        
        # Publish joint states
        self.joint_pub.publish(self.joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()