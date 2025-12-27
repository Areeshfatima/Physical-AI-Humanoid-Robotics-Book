import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from std_msgs.msg import String
import traceback
import time


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Set up QoS profile with appropriate settings for real-time performance
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning

        # Add more comprehensive logging
        self.get_logger().info('MinimalSubscriber initialized')
        self.get_logger().info(f'Subscribing to topic: topic with QoS depth: {qos_profile.depth}')

    def listener_callback(self, msg):
        try:
            self.get_logger().info(f'I heard: "{msg.data}"')
        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {str(e)}')
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    try:
        rclpy.spin(minimal_subscriber)
    except KeyboardInterrupt:
        minimal_subscriber.get_logger().info('Received interrupt signal')
    except Exception as e:
        minimal_subscriber.get_logger().error(f'Unhandled exception: {str(e)}')
        minimal_subscriber.get_logger().error(traceback.format_exc())
    finally:
        # Destroy the node explicitly
        minimal_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()