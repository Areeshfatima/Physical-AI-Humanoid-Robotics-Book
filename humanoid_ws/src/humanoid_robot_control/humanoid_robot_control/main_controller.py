#!/usr/bin/env python3

"""
Main controller for the humanoid robot supporting Vision-Language-Action (VLA) systems
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState, Image, CameraInfo, PointCloud2, Imu, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math
import json
import threading
from collections import deque


class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Publishers
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory',
            10
        )

        # Navigation publisher
        self.nav_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Publishers for various sensors
        self.imu_publisher = self.create_publisher(Imu, '/imu/data', 10)
        self.laser_publisher = self.create_publisher(LaserScan, '/scan', 10)
        self.camera_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Subscribers for control commands
        self.cmd_subscriber = self.create_subscription(
            String,
            'control_commands',
            self.command_callback,
            10
        )

        # Subscribers for sensor data
        self.vision_subscriber = self.create_subscription(
            String,
            'vision_commands',
            self.vision_callback,
            10
        )

        # Timer for publishing joint commands
        self.timer = self.create_timer(0.05, self.publish_joint_commands)  # 50ms for real-time performance
        self.nav_timer = self.create_timer(0.1, self.publish_navigation_commands)  # 100ms for navigation
        self.sensor_timer = self.create_timer(0.02, self.publish_sensor_data)  # 20ms for sensors

        # Initialize joint positions
        self.joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint',
            'head_pan_joint', 'head_tilt_joint'
        ]

        self.current_positions = [0.0] * len(self.joint_names)
        self.target_positions = [0.0] * len(self.joint_names)

        # Robot state tracking
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # For vision-language-action pipeline
        self.perception_data = {
            'detected_objects': [],
            'object_positions': {}
        }

        # Command queue for processing multiple commands
        self.command_queue = deque()
        self.command_lock = threading.Lock()

        self.get_logger().info('Humanoid Controller initialized with VLA support')

    def command_callback(self, msg):
        """Process incoming control commands from LLM-based cognitive planning"""
        command = msg.data.lower().strip()

        with self.command_lock:
            self.command_queue.append(command)

        self.get_logger().info(f'Queued command: {command}')

    def vision_callback(self, msg):
        """Process incoming visual perception data"""
        try:
            vision_data = json.loads(msg.data)
            self.perception_data = vision_data
            self.get_logger().info(f'Updated perception data: {len(self.perception_data["detected_objects"])} objects')
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode vision data: {e}')

    def process_commands(self):
        """Process commands from the queue"""
        with self.command_lock:
            if not self.command_queue:
                return

            command = self.command_queue.popleft()

        # Parse command for navigation, manipulation, etc.
        if 'walk' in command or 'move forward' in command:
            self.get_logger().info('Starting walk sequence')
            self.walk_gait()
        elif 'wave' in command:
            self.get_logger().info('Starting wave sequence')
            self.wave_motion()
        elif 'reset' in command:
            self.get_logger().info('Resetting joints to home position')
            self.reset_joints()
        elif 'turn left' in command:
            self.get_logger().info('Turning left')
            self.linear_velocity = 0.0
            self.angular_velocity = 0.5  # radians per second
        elif 'turn right' in command:
            self.get_logger().info('Turning right')
            self.linear_velocity = 0.0
            self.angular_velocity = -0.5  # radians per second
        elif 'move forward' in command:
            self.get_logger().info('Moving forward')
            self.linear_velocity = 0.5  # meters per second
            self.angular_velocity = 0.0
        elif 'stop' in command:
            self.get_logger().info('Stopping movement')
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
        elif 'look at' in command or 'turn head' in command:
            self.get_logger().info('Adjusting head orientation')
            self.look_at_object()
        else:
            self.get_logger().warn(f'Unknown command: {command}')

    def publish_joint_commands(self):
        """Publish joint trajectory commands"""
        # Process any queued commands
        self.process_commands()

        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        duration = Duration()
        duration.sec = 0
        duration.nanosec = 50000000  # 50ms
        point.time_from_start = duration

        msg.points.append(point)
        self.joint_cmd_publisher.publish(msg)

    def publish_navigation_commands(self):
        """Publish navigation commands"""
        msg = Twist()
        msg.linear.x = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.nav_publisher.publish(msg)

        # Update robot position based on velocities
        dt = 0.1  # 100ms based on timer
        self.robot_x += self.linear_velocity * dt * math.cos(self.robot_theta)
        self.robot_y += self.linear_velocity * dt * math.sin(self.robot_theta)
        self.robot_theta += self.angular_velocity * dt

        # Publish odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        # Convert theta to quaternion using standard math
        cos_half_theta = math.cos(self.robot_theta / 2.0)
        sin_half_theta = math.sin(self.robot_theta / 2.0)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = sin_half_theta
        odom_msg.pose.pose.orientation.w = cos_half_theta
        odom_msg.twist.twist.linear.x = self.linear_velocity
        odom_msg.twist.twist.angular.z = self.angular_velocity

        self.odom_publisher.publish(odom_msg)

    def publish_sensor_data(self):
        """Publish simulated sensor data"""
        # Publish IMU data
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        # Simulate slight movement
        imu_msg.linear_acceleration.x = 0.1 * math.sin(self.get_clock().now().nanoseconds / 1e9)
        imu_msg.linear_acceleration.y = 0.1 * math.cos(self.get_clock().now().nanoseconds / 1e9)
        imu_msg.linear_acceleration.z = 9.81  # gravity
        self.imu_publisher.publish(imu_msg)

        # Publish LaserScan data
        laser_msg = LaserScan()
        laser_msg.header.stamp = self.get_clock().now().to_msg()
        laser_msg.header.frame_id = 'laser_frame'
        laser_msg.angle_min = -math.pi / 2
        laser_msg.angle_max = math.pi / 2
        laser_msg.angle_increment = math.pi / 180  # 1 degree
        laser_msg.range_min = 0.1
        laser_msg.range_max = 10.0
        laser_msg.ranges = [5.0 + 2.0 * math.sin(i * math.pi / 180) for i in range(181)]
        self.laser_publisher.publish(laser_msg)

    def walk_gait(self):
        """Implement a basic walking gait pattern with smooth transitions"""
        # Simple oscillatory pattern for walking
        time_sec = self.get_clock().now().nanoseconds / 1e9
        amplitude = 0.5
        frequency = 1.0

        # Hip joints (opposite phase)
        self.target_positions[0] = amplitude * math.sin(2 * math.pi * frequency * time_sec)  # left hip
        self.target_positions[3] = amplitude * math.sin(2 * math.pi * frequency * time_sec + math.pi)  # right hip

        # Knee joints (delayed phase)
        self.target_positions[1] = amplitude * 0.5 * math.sin(2 * math.pi * frequency * time_sec - math.pi/2)
        self.target_positions[4] = amplitude * 0.5 * math.sin(2 * math.pi * frequency * time_sec + math.pi/2)

        # Shoulder joints (opposite to hips for balance)
        self.target_positions[6] = amplitude * 0.3 * math.sin(2 * math.pi * frequency * time_sec + math.pi)
        self.target_positions[8] = amplitude * 0.3 * math.sin(2 * math.pi * frequency * time_sec)

        # Keep other joints stable
        self.target_positions[2] = 0.0  # left ankle
        self.target_positions[5] = 0.0  # right ankle
        self.target_positions[7] = -0.5  # left elbow
        self.target_positions[9] = -0.5  # right elbow

    def wave_motion(self):
        """Implement waving motion with right arm"""
        time_sec = self.get_clock().now().nanoseconds / 1e9
        amplitude = 0.8
        frequency = 1.5

        # Right shoulder and elbow for waving
        self.target_positions[8] = amplitude * math.sin(2 * math.pi * frequency * time_sec)  # right shoulder
        self.target_positions[9] = amplitude * 0.7 * math.sin(2 * math.pi * frequency * time_sec + math.pi/4)  # right elbow

    def look_at_object(self):
        """Adjust head position to look at an object"""
        # In a real system, this would use perception data to position the head
        # For simulation, we'll just move the head in a scanning pattern
        time_sec = self.get_clock().now().nanoseconds / 1e9
        self.target_positions[10] = 0.5 * math.sin(0.5 * time_sec)  # head pan
        self.target_positions[11] = 0.3 * math.sin(0.3 * time_sec)  # head tilt

    def reset_joints(self):
        """Reset all joints to zero position"""
        self.target_positions = [0.0] * len(self.joint_names)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0


def main(args=None):
    rclpy.init(args=args)

    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('Shutting down Humanoid Controller')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()