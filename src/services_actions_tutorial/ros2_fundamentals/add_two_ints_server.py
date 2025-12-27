import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import traceback
from rclpy.qos import QoSProfile


class AddTwoIntsServer(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service with QoS profile
        qos_profile = QoSProfile(depth=10)
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback,
            qos_profile=qos_profile
        )

        # Add more comprehensive logging
        self.get_logger().info('AddTwoIntsServer initialized')
        self.get_logger().info('Ready to receive addition requests')

    def add_two_ints_callback(self, request, response):
        try:
            response.sum = request.a + request.b
            self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}, sum: {response.sum}')
            return response
        except Exception as e:
            self.get_logger().error(f'Error in add_two_ints_callback: {str(e)}')
            self.get_logger().error(traceback.format_exc())
            # Return error response if needed
            response.sum = 0  # Default error value
            return response


def main(args=None):
    rclpy.init(args=args)

    add_two_ints_server = AddTwoIntsServer()

    try:
        rclpy.spin(add_two_ints_server)
    except KeyboardInterrupt:
        add_two_ints_server.get_logger().info('Received interrupt signal')
    except Exception as e:
        add_two_ints_server.get_logger().error(f'Unhandled exception: {str(e)}')
        add_two_ints_server.get_logger().error(traceback.format_exc())
    finally:
        add_two_ints_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()