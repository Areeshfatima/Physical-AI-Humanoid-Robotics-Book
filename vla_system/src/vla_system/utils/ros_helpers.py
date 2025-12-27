"""Shared ROS 2 utilities for the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


def create_qos_profile(depth=10):
    """Create a standard QoS profile for the VLA system."""
    return QoSProfile(
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST
    )


def get_current_ros_time(node: Node) -> Time:
    """Get the current ROS time from a node's clock."""
    return node.get_clock().now().to_msg()


def normalize_quaternion_orientation(roll: float, pitch: float, yaw: float):
    """
    Normalize orientation angles to standard ranges.
    This is a simplified function - in practice, you'd want a more comprehensive orientation library
    """
    import math
    
    # Normalize to [-π, π] range
    roll = math.atan2(math.sin(roll), math.cos(roll))
    pitch = math.atan2(math.sin(pitch), math.cos(pitch))
    yaw = math.atan2(math.sin(yaw), math.cos(yaw))
    
    return roll, pitch, yaw


def distance_3d(x1: float, y1: float, z1: float, x2: float, y2: float, z2: float) -> float:
    """Calculate 3D Euclidean distance between two points."""
    import math
    
    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1
    
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def wait_for_service_client(client, timeout_sec=10.0):
    """Wait for a service client to be ready."""
    import time
    
    start_time = time.time()
    while not client.wait_for_service(timeout_sec=1.0):
        if time.time() - start_time > timeout_sec:
            raise TimeoutError(f"Service not available after {timeout_sec} seconds")
        rclpy.spin_once(client._node, timeout_sec=0.1)