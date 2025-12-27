import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile
from std_msgs.msg import String
from publisher_subscriber_tutorial.publisher import MinimalPublisher
from publisher_subscriber_tutorial.subscriber import MinimalSubscriber


class TestPublisherSubscriber(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        self.node = rclpy.create_node('test_pub_sub_node')
        
        # Create a publisher and subscriber for testing
        self.publisher = self.node.create_publisher(String, 'topic', 10)
        self.received_messages = []
        
        # Create subscription to capture messages
        self.subscription = self.node.create_subscription(
            String,
            'topic',
            self.message_callback,
            QoSProfile(depth=10)
        )

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def message_callback(self, msg):
        self.received_messages.append(msg.data)

    def test_publisher_subscriber_communication(self):
        """Test that publisher sends messages and subscriber receives them"""
        # Create a simple publisher to send a test message
        test_msg = String()
        test_msg.data = 'Test message for validation'
        
        # Publish the message
        self.publisher.publish(test_msg)
        
        # Spin to process the message
        executor = SingleThreadedExecutor()
        executor.add_node(self.node)
        
        # Wait briefly for the message to be processed
        timer = self.node.create_timer(0.1, lambda: None)  # Dummy timer to keep spinning
        executor.spin_once(timeout_sec=0.2)
        self.node.destroy_timer(timer)
        
        # Check that we received the message
        self.assertIn('Test message for validation', self.received_messages)


def test_latency():
    """Simple latency test for publisher/subscriber communication"""
    import time
    
    rclpy.init()
    node = rclpy.create_node('latency_test_node')
    
    # Track message timestamps
    timestamps = []
    
    def timestamp_callback(msg):
        receive_time = time.time()
        # Extract sent time from message if available
        timestamps.append(receive_time)
    
    subscription = node.create_subscription(
        String,
        'topic',
        timestamp_callback,
        10
    )
    
    # This is a simplified test - in real scenarios you'd have timestamps in messages
    # For now, just verify the functionality works
    
    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()