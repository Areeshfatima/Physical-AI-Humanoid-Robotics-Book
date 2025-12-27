import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import math
import cv2
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class DepthCameraNode(Node):
    """
    Simulated depth camera sensor node that publishes depth images
    mimicking a real depth camera's behavior in the simulation environment.
    """
    
    def __init__(self):
        super().__init__('depth_camera_node')
        
        # Depth camera configuration parameters
        self.width = 640
        self.height = 480
        self.focal_length = 320.0  # pixels (assumes ~60° FOV for 640px width)
        self.center_x = self.width / 2.0
        self.center_y = self.height / 2.0
        
        # Field of view
        self.hfov = math.radians(60.0)  # Horizontal field of view in radians
        self.vfov = self.hfov * self.height / self.width  # Vertical FOV adjusted for aspect ratio
        
        # Depth range
        self.min_depth = 0.1  # meters
        self.max_depth = 10.0  # meters
        
        # Noise parameters for realistic simulation
        self.depth_noise_std = 0.01  # 1cm standard deviation at 1m
        
        # Publisher for depth images
        self.depth_pub = self.create_publisher(Image, '/depth_camera/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/depth_camera/camera_info', 10)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Timer for publishing depth images
        self.image_timer = self.create_timer(1/30.0, self.publish_depth_image)  # 30Hz
        
        # Initialize camera info message
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.width = self.width
        self.camera_info_msg.height = self.height
        self.camera_info_msg.k = [self.focal_length, 0.0, self.center_x,
                                  0.0, self.focal_length, self.center_y,
                                  0.0, 0.0, 1.0]
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
        self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info_msg.p = [self.focal_length, 0.0, self.center_x, 0.0,
                                  0.0, self.focal_length, self.center_y, 0.0,
                                  0.0, 0.0, 1.0, 0.0]
        self.camera_info_msg.distortion_model = 'plumb_bob'
        
        self.get_logger().info(f'Depth camera initialized with resolution {self.width}x{self.height}, '
                              f'FOV: {math.degrees(self.hfov):.1f}°x{math.degrees(self.vfov):.1f}°, '
                              f'Depth range: {self.min_depth}m - {self.max_depth}m')
    
    def generate_sample_depth_image(self):
        """
        Generate a simulated depth image based on a simple environment layout.
        In a real implementation, this would interface with the Gazebo simulation.
        """
        # Create a depth image as a numpy array (height x width)
        depth_array = np.full((self.height, self.width), self.max_depth, dtype=np.float32)
        
        # Create some objects in the scene
        # Add a "wall" in the distance
        wall_distance = 2.5  # meters
        y_wall_start = int(self.height * 0.3)  # Start at 30% height
        depth_array[y_wall_start:, :] = wall_distance
        
        # Add a "box" closer to the camera
        box_distance = 1.2  # meters
        box_width = int(self.width * 0.3)  # 30% of image width
        box_height = int(self.height * 0.2)  # 20% of image height
        box_start_x = int(self.width/2 - box_width/2)  # Center horizontally
        box_start_y = int(self.height * 0.4)  # Positioned lower in the view
        depth_array[box_start_y:box_start_y+box_height, box_start_x:box_start_x+box_width] = box_distance
        
        # Add some "furniture" to the left
        furn_distance = 3.0  # meters
        furn_width = int(self.width * 0.15)  # 15% of image width
        furn_start_x = int(self.width * 0.1)  # Near left edge
        furn_start_y = int(self.height * 0.6)  # Bottom half
        depth_array[furn_start_y:furn_start_y+furn_height, furn_start_x:furn_start_x+furn_width] = furn_distance
        
        # Add realistic noise
        noise = np.random.normal(0, self.depth_noise_std, depth_array.shape).astype(np.float32)
        # Noise magnitude should increase with depth (noise scales with distance)
        depth_with_noise = depth_array + noise * (depth_array / self.min_depth)
        depth_with_noise = np.clip(depth_with_noise, self.min_depth, self.max_depth)
        
        # Convert to 32FC1 OpenCV format
        depth_image = np.array(depth_with_noise, dtype=np.float32)
        
        return depth_image
    
    def publish_depth_image(self):
        """
        Publish a depth image with current simulated data
        """
        # Generate the current depth image
        depth_img = self.generate_sample_depth_image()
        
        # Create the Image message
        try:
            depth_msg = self.bridge.cv2_to_imgmsg(depth_img, encoding='32FC1')
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = 'depth_camera_link'  # Frame ID for the depth camera
            
            # Publish the depth image
            self.depth_pub.publish(depth_msg)
            
            # Update and publish camera info
            self.camera_info_msg.header.stamp = depth_msg.header.stamp
            self.camera_info_msg.header.frame_id = depth_msg.header.frame_id
            self.camera_info_pub.publish(self.camera_info_msg)
            
            # Log some statistics
            valid_depths = depth_img[(depth_img >= self.min_depth) & (depth_img <= self.max_depth)]
            if len(valid_depths) > 0:
                self.get_logger().debug(f'Published depth image: Min={np.min(valid_depths):.2f}m, '
                                       f'Max={np.max(valid_depths):.2f}m, '
                                       f'Avg={np.mean(valid_depths):.2f}m')
            else:
                self.get_logger().debug('Published depth image: all pixels at max range')
        
        except Exception as e:
            self.get_logger().error(f'Error publishing depth image: {e}')


def main(args=None):
    """
    Main function to run the depth camera node
    """
    rclpy.init(args=args)
    
    depth_camera_node = DepthCameraNode()
    
    try:
        rclpy.spin(depth_camera_node)
    except KeyboardInterrupt:
        depth_camera_node.get_logger().info('Depth camera node stopped by user')
    finally:
        depth_camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()