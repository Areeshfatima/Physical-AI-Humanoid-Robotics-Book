import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from example_interfaces.srv import AddTwoInts
from example_interfaces.action import Fibonacci
from services_actions_tutorial.add_two_ints_client import AddTwoIntsClient
from services_actions_tutorial.add_two_ints_server import AddTwoIntsServer


class TestServiceCommunication(unittest.TestCase):

    def setUp(self):
        rclpy.init()
        
        # Create server and client nodes
        self.server_node = AddTwoIntsServer()
        self.client_node = AddTwoIntsClient()
        
        # Use a single-threaded executor to manage both nodes
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.server_node)
        self.executor.add_node(self.client_node)

    def tearDown(self):
        self.server_node.destroy_node()
        self.client_node.destroy_node()
        rclpy.shutdown()

    def test_service_request_response(self):
        """Test that service client can send request and server responds"""
        # Wait for service to be available
        if not self.client_node.cli.wait_for_service(timeout_sec=5.0):
            self.fail("Service not available")
        
        # Send a request
        future = self.client_node.cli.call_async(AddTwoInts.Request(a=2, b=3))
        
        # Spin until the future is complete
        self.executor.spin_until_future_complete(future, timeout_sec=5.0)
        
        # Check response
        response = future.result()
        self.assertIsNotNone(response)
        self.assertEqual(response.sum, 5)


def performance_validation():
    """Function to validate performance of communication patterns"""
    import time
    
    rclpy.init()
    
    # Test service call timing
    start_time = time.time()
    
    # Create server and client
    server_node = AddTwoIntsServer()
    client_node = AddTwoIntsClient()
    
    executor = SingleThreadedExecutor()
    executor.add_node(server_node)
    executor.add_node(client_node)
    
    if client_node.cli.wait_for_service(timeout_sec=1.0):
        future = client_node.cli.call_async(AddTwoInts.Request(a=5, b=7))
        executor.spin_until_future_complete(future, timeout_sec=5.0)
        
        end_time = time.time()
        response = future.result()
        
        print(f"Service call took: {end_time - start_time:.4f} seconds")
        print(f"Response: {response.sum}")
    
    server_node.destroy_node()
    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()