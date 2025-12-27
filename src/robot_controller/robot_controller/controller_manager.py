import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import math
import traceback
import time
from rclpy.exceptions import ROSInterruptException


class ControllerManager(Node):

    def __init__(self):
        super().__init__('controller_manager')

        # Create subscribers for different command types
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )

        # Create publisher for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            qos_profile
        )

        # Create publisher for joint states (for visualization)
        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            qos_profile
        )

        # Initialize joint states
        self.joint_state = JointState()
        self.joint_state.name = [
            'neck_joint',
            'left_leg_joint',
            'right_leg_joint',
            'left_arm_joint',
            'right_arm_joint'
        ]
        self.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.velocity = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_state.effort = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Timer for publishing joint states
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Add failure recovery tracking
        self.last_cmd_time = self.get_clock().now()
        self.error_count = 0
        self.max_error_count = 5  # Reset after this many errors

        # Add comprehensive logging
        self.get_logger().info('Controller Manager initialized with enhanced logging and recovery')
        self.get_logger().info(f'Controlling joints: {self.joint_state.name}')

    def cmd_vel_callback(self, msg):
        try:
            # Process velocity commands and convert to joint movements
            linear_x = msg.linear.x
            linear_y = msg.linear.y
            angular_z = msg.angular.z

            # Update last command time for failure detection
            self.last_cmd_time = self.get_clock().now()

            # Example: Map linear and angular velocities to joint movements
            # This is a simplified mapping for demonstration
            self.joint_state.position[0] += angular_z * 0.01  # neck follows angular rotation
            self.joint_state.position[1] += linear_x * 0.005  # legs move with forward/backward motion
            self.joint_state.position[2] += linear_x * 0.005
            self.joint_state.position[3] += linear_y * 0.01   # arms move with lateral motion
            self.joint_state.position[4] -= linear_y * 0.01

            # Apply joint limits
            for i in range(len(self.joint_state.position)):
                self.joint_state.position[i] = max(-1.0, min(1.0, self.joint_state.position[i]))

            self.get_logger().info(
                f'Command received: linear=({linear_x}, {linear_y}), angular={angular_z}',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error processing cmd_vel: {str(e)}')
            self.get_logger().error(traceback.format_exc())

            # Attempt recovery if too many errors
            if self.error_count >= self.max_error_count:
                self.attempt_recovery()

    def attempt_recovery(self):
        """Attempt to recover from errors by resetting state"""
        self.get_logger().warn('Attempting recovery from errors...')
        self.joint_state.position = [0.0] * len(self.joint_state.position)
        self.error_count = 0
        self.get_logger().info('State reset, recovery attempt complete')

    def timer_callback(self):
        try:
            # Check for communication failures (no commands received for a while)
            time_since_last_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
            if time_since_last_cmd > 5.0:  # 5 seconds without command
                self.get_logger().warn(f'No commands received for {time_since_last_cmd:.2f}s, potential communication failure')

            # Update header for joint states
            self.joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state.header.frame_id = 'base_link'

            # Publish joint states
            self.joint_pub.publish(self.joint_state)

        except Exception as e:
            self.error_count += 1
            self.get_logger().error(f'Error in timer_callback: {str(e)}')
            self.get_logger().error(traceback.format_exc())

            # Attempt recovery if too many errors
            if self.error_count >= self.max_error_count:
                self.attempt_recovery()


def main(args=None):
    rclpy.init(args=args)

    node = ControllerManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal')
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {str(e)}')
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()