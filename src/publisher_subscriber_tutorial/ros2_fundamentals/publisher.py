import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import time
import traceback


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        # Set up QoS profile with appropriate settings for real-time performance
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.publisher_ = self.create_publisher(String, 'topic', qos_profile)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Add more comprehensive logging
        self.get_logger().info('MinimalPublisher initialized')
        self.get_logger().info(f'Publishing to topic: topic with QoS depth: {qos_profile.depth}')

    def timer_callback(self):
        try:
            msg = String()
            msg.data = f'Hello World: {self.i}'
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: "{msg.data}"', throttle_duration_sec=1.0)
            self.i += 1
        except Exception as e:
            self.get_logger().error(f'Error in timer_callback: {str(e)}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        minimal_publisher.get_logger().info('Received interrupt signal')
    except Exception as e:
        minimal_publisher.get_logger().error(f'Unhandled exception: {str(e)}')
        minimal_publisher.get_logger().error(traceback.format_exc())
    finally:
        # Destroy the node explicitly
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()