import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header
import numpy as np
import math
import time
from scipy.spatial.transform import Rotation as R


class ImuSensorNode(Node):
    """
    Simulated IMU sensor node that publishes IMU messages
    with realistic acceleration, angular velocity, and orientation data.
    """
    
    def __init__(self):
        super().__init__('imu_sensor_node')
        
        # IMU configuration parameters
        self.linear_acceleration_std = 0.017  # ~0.1 m/s^2 (typical for MEMS IMU)
        self.angular_velocity_std = 0.0015    # ~0.1 deg/s resolution in rad/s
        self.magnetic_field_std = 0.1         # microTesla
        
        # Bias parameters (typical for low-cost IMUs)
        self.accel_bias = np.array([0.01, -0.02, 0.03])  # m/s^2
        self.gyro_bias = np.array([0.0001, -0.0002, 0.0003])  # rad/s (bias equivalent to ~0.005-0.01 deg/s)
        
        # Publisher for IMU data
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Timer for publishing IMU data (typically 100-200Hz for IMUs)
        self.imu_timer = self.create_timer(0.01, self.publish_imu_data)  # 100Hz
        
        # Initialize reference states
        self.start_time = time.time()
        self.reference_orientation = R.from_quat([0, 0, 0, 1])  # Start at identity
        self.reference_linear_acceleration = np.array([0, 0, 0], dtype=float)  # Acceleration in body frame
        self.reference_angular_velocity = np.array([0, 0, 0], dtype=float)    # Angular velocity in body frame
        
        # Store previous orientation for integration
        self.current_orientation = self.reference_orientation
        self.previous_time = None
        
        self.get_logger().info('IMU sensor initialized with realistic noise models')
    
    def get_simulated_motion(self, dt):
        """
        Generate simulated motion based on a simple movement pattern
        In a real implementation, this would come from the physics simulation
        """
        current_time = time.time() - self.start_time
        
        # Simulate some simple motion - perhaps the robot standing with small movements
        # These are in the IMU's body frame
        
        # Linear accelerations (in m/s^2)
        # Include gravity (which will be transformed to body frame)
        gravity = np.array([0, 0, 9.81])
        
        # Add some small oscillatory motions to simulate breathing, small tremors, etc
        freq = 0.5  # Hz, slow motion
        amp = 0.05  # m/s^2
        linear_acc_body = np.array([
            amp * math.sin(2 * math.pi * freq * current_time),
            amp * math.cos(2 * math.pi * freq * current_time),
            0  # Z component will be influenced by gravity
        ])
        
        # Angular velocities (in rad/s)
        # Small random rotations
        ang_vel_body = np.array([
            0.01 * math.sin(0.3 * 2 * math.pi * current_time),
            0.01 * math.cos(0.4 * 2 * math.pi * current_time),
            0.02 * math.sin(0.5 * 2 * math.pi * current_time)
        ])
        
        return linear_acc_body + gravity, ang_vel_body
    
    def publish_imu_data(self):
        """
        Publish IMU data with realistic noise and bias characteristics
        """
        # Calculate time difference
        current_ros_time = self.get_clock().now()
        current_time_float = current_ros_time.nanoseconds / 1e9
        
        # Calculate dt
        dt = 0.01  # Default to 0.01s if no previous time
        if self.previous_time is not None:
            dt = current_time_float - self.previous_time
        self.previous_time = current_time_float
        
        # Get simulated motion
        linear_acc_true, ang_vel_true = self.get_simulated_motion(dt)
        
        # Simulate IMU measurements with noise and bias
        
        # Linear acceleration with noise and bias
        accel_noise = np.random.normal(0, self.linear_acceleration_std, size=3)
        linear_acc_measured = linear_acc_true + self.accel_bias + accel_noise
        
        # Angular velocity with noise and bias
        gyro_noise = np.random.normal(0, self.angular_velocity_std, size=3)
        angular_vel_measured = ang_vel_true + self.gyro_bias + gyro_noise
        
        # Update orientation by integrating angular velocity
        # Use the average angular velocity over the timestep
        avg_ang_vel = angular_vel_measured
        delta_angle = avg_ang_vel * dt
        
        # Convert to axis-angle representation and then to quaternion
        angle = np.linalg.norm(delta_angle)
        if angle > 1e-6:  # Avoid division by zero
            axis = delta_angle / angle
            # Create small rotation quaternion
            cos_half_angle = math.cos(angle / 2)
            sin_half_angle = math.sin(angle / 2)
            dq = np.array([axis[0] * sin_half_angle, 
                          axis[1] * sin_half_angle, 
                          axis[2] * sin_half_angle, cos_half_angle])
            
            # Apply rotation
            new_rotation = R.from_quat(dq) * self.current_orientation
            self.current_orientation = new_rotation
        else:
            # No rotation, keep current orientation
            pass
        
        # Get current orientation as quaternion
        orientation_quat = self.current_orientation.as_quat()
        
        # Add noise to orientation (only for uncertainty estimate, not actual orientation)
        orientation_quat_noisy = orientation_quat.copy()
        # For now, just use the noise-free orientation; orientation noise would be added differently in practice
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_ros_time.to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # Set orientation (with zero covariance since we're simulating perfect integration)
        imu_msg.orientation = Quaternion(x=orientation_quat[0], y=orientation_quat[1], 
                                       z=orientation_quat[2], w=orientation_quat[3])
        # For sim, set orientation covariance to zero (perfect knowledge in simulation)
        imu_msg.orientation_covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Set angular velocity
        imu_msg.angular_velocity = Vector3(x=angular_vel_measured[0], 
                                          y=angular_vel_measured[1], 
                                          z=angular_vel_measured[2])
        # Angular velocity covariance (based on sensor noise characteristics)
        ang_vel_cov = self.angular_velocity_std**2
        imu_msg.angular_velocity_covariance = [ang_vel_cov, 0.0, 0.0, 0.0, ang_vel_cov, 0.0, 0.0, 0.0, ang_vel_cov]
        
        # Set linear acceleration
        imu_msg.linear_acceleration = Vector3(x=linear_acc_measured[0], 
                                             y=linear_acc_measured[1], 
                                             z=linear_acc_measured[2])
        # Linear acceleration covariance (based on sensor noise characteristics)
        lin_acc_cov = self.linear_acceleration_std**2
        imu_msg.linear_acceleration_covariance = [lin_acc_cov, 0.0, 0.0, 0.0, lin_acc_cov, 0.0, 0.0, 0.0, lin_acc_cov]
        
        # Publish the IMU data
        self.imu_pub.publish(imu_msg)
        
        # Log some statistics (only once in a while to avoid spamming)
        if int(current_time_float) % 5 == 0:
            self.get_logger().debug(f'IMU data published - Acc: [{linear_acc_measured[0]:.3f}, {linear_acc_measured[1]:.3f}, {linear_acc_measured[2]:.3f}], '
                                   f'Gyro: [{angular_vel_measured[0]:.4f}, {angular_vel_measured[1]:.4f}, {angular_vel_measured[2]:.4f}]')


def main(args=None):
    """
    Main function to run the IMU sensor node
    """
    rclpy.init(args=args)
    
    imu_node = ImuSensorNode()
    
    try:
        rclpy.spin(imu_node)
    except KeyboardInterrupt:
        imu_node.get_logger().info('IMU sensor node stopped by user')
    finally:
        imu_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()