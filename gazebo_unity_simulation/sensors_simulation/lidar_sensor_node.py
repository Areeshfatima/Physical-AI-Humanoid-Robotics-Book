import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import SetJointProperties
from gazebo_msgs.srv import GetJointProperties
import numpy as np
import math


class LidarSensorNode(Node):
    """
    Simulated LiDAR sensor node that publishes LaserScan messages
    mimicking a real LiDAR sensor's behavior in the simulation environment.
    """
    
    def __init__(self):
        super().__init__('lidar_sensor_node')
        
        # LiDAR configuration parameters
        self.angle_min = -math.pi  # -180 degrees
        self.angle_max = math.pi   # 180 degrees
        self.angle_increment = math.radians(0.25)  # 0.25 degree resolution
        self.scan_time = 0.05  # 50ms per scan (20Hz)
        self.range_min = 0.1   # Minimum range 0.1m
        self.range_max = 30.0  # Maximum range 30m
        
        # Calculate number of rays
        self.num_rays = int((self.angle_max - self.angle_min) / self.angle_increment)
        
        # Add some noise parameters for realistic simulation
        self.range_noise_std = 0.01  # 1cm standard deviation for range noise
        
        # Publisher for LiDAR data
        self.laser_pub = self.create_publisher(LaserScan, '/lidar_scan', 10)
        
        # Timer for publishing scans
        self.scan_timer = self.create_timer(self.scan_time, self.publish_scan)
        
        # Initialize scan message
        self.scan_msg = LaserScan()
        self.scan_msg.angle_min = self.angle_min
        self.scan_msg.angle_max = self.angle_max
        self.scan_msg.angle_increment = self.angle_increment
        self.scan_msg.time_increment = 0.0  # Not using time increment per ray in this sim
        self.scan_msg.scan_time = self.scan_time
        self.scan_msg.range_min = self.range_min
        self.scan_msg.range_max = self.range_max
        
        self.get_logger().info(f'LiDAR sensor initialized with {self.num_rays} rays, range {self.range_min}m - {self.range_max}m')
    
    def generate_sample_scan(self):
        """
        Generate a simulated scan based on a simple environment layout.
        In a real implementation, this would interface with the Gazebo simulation.
        """
        # Create sample environment - some objects at different distances
        # This is a simplified representation of what the LiDAR might "see"
        ranges = []
        
        # Generate sample distances based on a simple environment
        for i in range(self.num_rays):
            angle = self.angle_min + i * self.angle_increment
            
            # Create a simple environment with walls and obstacles
            # In real implementation, this would come from raycasting in Gazebo
            range_val = self.range_max  # Default to max range (free space)
            
            # Create a "box" in front of the robot
            if abs(math.sin(angle)) < 0.1:  # Approximately straight ahead
                if abs(math.cos(angle)) > 0.9:  # And actually facing forward
                    range_val = 2.0  # Object 2m ahead
            
            # Create a "wall" to the right
            if math.cos(angle) < 0 and math.sin(angle) > 0.7:  # Right side
                range_val = 1.5  # Wall 1.5m away
            
            # Add some realistic noise
            range_val += np.random.normal(0, self.range_noise_std)
            range_val = max(self.range_min, min(self.range_max, range_val))  # Clamp to valid range
            
            ranges.append(range_val)
        
        return ranges
    
    def publish_scan(self):
        """
        Publish a laser scan message with current simulated data
        """
        # Generate the current scan
        current_ranges = self.generate_sample_scan()
        
        # Update message timestamp
        self.scan_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_msg.header.frame_id = 'lidar_link'  # Usually matches the sensor frame
        
        # Set the ranges
        self.scan_msg.ranges = current_ranges
        # Clear intensities if not used (some sensors don't publish intensity)
        self.scan_msg.intensities = []  # Empty if using a sensor that doesn't report intensity
        
        # Publish the scan
        self.laser_pub.publish(self.scan_msg)
        
        self.get_logger().debug(f'Published LiDAR scan with {len(current_ranges)} readings, range: {min(current_ranges):.2f}m - {max(current_ranges):.2f}m')


def main(args=None):
    """
    Main function to run the LiDAR sensor node
    """
    rclpy.init(args=args)
    
    lidar_node = LidarSensorNode()
    
    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        lidar_node.get_logger().info('LiDAR sensor node stopped by user')
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()