"""
Common Simulation Utilities and Helper Functions

This module contains common utilities and helper functions for physics validation
and other foundational simulation tasks.
"""

import numpy as np
import math
from geometry_msgs.msg import Vector3, Point, Quaternion
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


def normalize_vector(v):
    """Normalize a 3D vector"""
    norm = np.linalg.norm(v)
    if norm == 0:
        return np.array([0.0, 0.0, 0.0])
    return np.array(v) / norm


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions
    q1, q2: [x, y, z, w] format
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    
    return [x, y, z, w]


def rotation_matrix_from_quaternion(q):
    """
    Convert quaternion to rotation matrix
    q: [x, y, z, w] format
    """
    x, y, z, w = q
    
    # Rotation matrix
    rot_matrix = np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
    ])
    
    return rot_matrix


def calculate_distance_3d(point1, point2):
    """Calculate Euclidean distance between two 3D points"""
    dx = point1.x - point2.x
    dy = point1.y - point2.y
    dz = point1.z - point2.z
    return math.sqrt(dx*dx + dy*dy + dz*dz)


def calculate_deviation_percentage(actual, expected):
    """
    Calculate deviation percentage between actual and expected values
    Handles both scalar and vector values
    """
    if isinstance(actual, (int, float)) and isinstance(expected, (int, float)):
        if expected != 0:
            return abs((actual - expected) / expected) * 100
        else:
            return abs(actual) * 100  # If expected is 0, just use absolute value
    elif hasattr(actual, '__iter__') and hasattr(expected, '__iter__'):
        # For vectors/lists, calculate magnitude-based deviation
        actual_mag = np.linalg.norm(actual)
        expected_mag = np.linalg.norm(expected)
        if expected_mag != 0:
            return abs((actual_mag - expected_mag) / expected_mag) * 100
        else:
            return actual_mag * 100
    else:
        return 0.0  # Default case


def validate_joint_limits(joint_positions, joint_limits_dict):
    """
    Validate that joint positions are within their limits
    joint_positions: dict with joint names as keys and positions as values
    joint_limits_dict: dict with joint names as keys and {'lower': val, 'upper': val} as values
    """
    invalid_joints = []
    
    for joint_name, position in joint_positions.items():
        if joint_name not in joint_limits_dict:
            continue  # Skip joints without defined limits
        
        limits = joint_limits_dict[joint_name]
        if position < limits['lower'] or position > limits['upper']:
            invalid_joints.append({
                'joint': joint_name,
                'position': position,
                'limits': limits
            })
    
    return invalid_joints


def calculate_physics_metrics(actual_positions, expected_positions, timestamps):
    """
    Calculate physics validation metrics
    """
    if len(actual_positions) != len(expected_positions) or len(actual_positions) != len(timestamps):
        raise ValueError("All arrays must have the same length")
    
    if len(actual_positions) < 2:
        return 0.0, 0.0, 0.0  # Need at least 2 points to calculate metrics
    
    # Calculate error metrics
    position_errors = []
    for actual, expected in zip(actual_positions, expected_positions):
        error = calculate_distance_3d(actual, expected)
        position_errors.append(error)
    
    # Calculate mean error
    mean_error = sum(position_errors) / len(position_errors)
    
    # Calculate maximum error
    max_error = max(position_errors) if position_errors else 0.0
    
    # Calculate RMSE (Root Mean Square Error)
    rmse = math.sqrt(sum(e*e for e in position_errors) / len(position_errors)) if position_errors else 0.0
    
    return mean_error, max_error, rmse


def load_yaml_config(config_path):
    """
    Load configuration from YAML file
    """
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        return config
    except Exception as e:
        print(f"Error loading config from {config_path}: {e}")
        return {}


def get_gravitational_force(mass, gravity=9.81):
    """
    Calculate gravitational force (weight)
    """
    return mass * gravity


def integrate_velocity(initial_velocity, acceleration, time_delta):
    """
    Integrate velocity using acceleration over time
    """
    return initial_velocity + acceleration * time_delta


def integrate_position(initial_position, velocity, time_delta):
    """
    Integrate position using velocity over time
    """
    # For simplicity, using basic Euler integration
    # In real applications, you might want more sophisticated methods
    dx = velocity.x * time_delta
    dy = velocity.y * time_delta
    dz = velocity.z * time_delta
    
    new_position = Point()
    new_position.x = initial_position.x + dx
    new_position.y = initial_position.y + dy
    new_position.z = initial_position.z + dz
    
    return new_position


class PhysicsMathUtils:
    """
    Utility class for physics calculations and mathematical operations
    """
    
    @staticmethod
    def calculate_kinetic_energy(mass, velocity_vector):
        """
        Calculate kinetic energy: KE = 0.5 * m * v²
        """
        speed_squared = velocity_vector.x**2 + velocity_vector.y**2 + velocity_vector.z**2
        return 0.5 * mass * speed_squared
    
    @staticmethod
    def calculate_potential_energy(mass, height, gravity=9.81):
        """
        Calculate potential energy: PE = m * g * h
        """
        return mass * gravity * height
    
    @staticmethod
    def calculate_work(force_vector, displacement_vector):
        """
        Calculate work done by a force: W = F · d
        """
        return (force_vector.x * displacement_vector.x + 
                force_vector.y * displacement_vector.y + 
                force_vector.z * displacement_vector.z)
    
    @staticmethod
    def calculate_power(work, time_delta):
        """
        Calculate power: P = W / t
        """
        if time_delta != 0:
            return work / time_delta
        return 0.0
    
    @staticmethod
    def calculate_acceleration(velocity_initial, velocity_final, time_delta):
        """
        Calculate acceleration: a = (v_f - v_i) / t
        Returns a Vector3 with acceleration components
        """
        if time_delta == 0:
            return Vector3(x=0.0, y=0.0, z=0.0)
        
        ax = (velocity_final.x - velocity_initial.x) / time_delta
        ay = (velocity_final.y - velocity_initial.y) / time_delta
        az = (velocity_final.z - velocity_initial.z) / time_delta
        
        return Vector3(x=ax, y=ay, z=az)


class SimulationValidator:
    """
    Class for validating simulation outputs against expected behavior
    """
    
    def __init__(self, max_deviation_percent=5.0):
        self.max_deviation_percent = max_deviation_percent
        
    def validate_trajectory(self, expected_poses, actual_poses):
        """
        Validate that actual trajectory matches expected trajectory
        """
        if len(expected_poses) != len(actual_poses):
            raise ValueError("Expected and actual poses must have the same length")
        
        validation_results = []
        
        for exp_pose, act_pose in zip(expected_poses, actual_poses):
            # Calculate positional deviation
            pos_deviation = calculate_distance_3d(
                exp_pose.position, 
                act_pose.position
            )
            
            # Calculate positional deviation percentage
            exp_mag = math.sqrt(exp_pose.position.x**2 + exp_pose.position.y**2 + exp_pose.position.z**2)
            if exp_mag > 1e-6:  # Avoid division by zero
                pos_deviation_pct = (pos_deviation / exp_mag) * 100
            else:
                pos_deviation_pct = pos_deviation * 100  # If expected is near zero, use absolute error
            
            # Calculate orientation deviation (dot product method)
            dot_product = (exp_pose.orientation.x * act_pose.orientation.x + 
                          exp_pose.orientation.y * act_pose.orientation.y + 
                          exp_pose.orientation.z * act_pose.orientation.z + 
                          exp_pose.orientation.w * act_pose.orientation.w)
            # Clamp to [-1, 1] to avoid numerical errors
            dot_product = max(-1.0, min(1.0, dot_product))
            angle_deviation_rad = 2 * math.acos(abs(dot_product))
            angle_deviation_deg = math.degrees(angle_deviation_rad)
            
            result = {
                'pos_deviation': pos_deviation,
                'pos_deviation_percent': pos_deviation_pct,
                'angle_deviation_deg': angle_deviation_deg,
                'within_tolerance': pos_deviation_pct <= self.max_deviation_percent and angle_deviation_deg <= 5.0
            }
            
            validation_results.append(result)
        
        return validation_results
    
    def calculate_validation_score(self, validation_results):
        """
        Calculate overall validation score based on results
        """
        if not validation_results:
            return 0.0
        
        valid_count = sum(1 for result in validation_results if result['within_tolerance'])
        return (valid_count / len(validation_results)) * 100


# Example usage and test function
def test_utilities():
    """
    Test the utility functions
    """
    print("Testing common simulation utilities...")
    
    # Test vector normalization
    v = [3, 4, 0]
    normalized = normalize_vector(v)
    print(f"Normalize {v}: {normalized}, magnitude: {np.linalg.norm(normalized)}")
    
    # Test distance calculation
    p1 = Point(x=0.0, y=0.0, z=0.0)
    p2 = Point(x=3.0, y=4.0, z=0.0)
    dist = calculate_distance_3d(p1, p2)
    print(f"Distance between {p1} and {p2}: {dist}")
    
    # Test deviation calculation
    dev = calculate_deviation_percentage(1.05, 1.0)
    print(f"Deviation between 1.05 and 1.0: {dev}%")
    
    # Test physics math utilities
    utils = PhysicsMathUtils()
    velocity = Vector3(x=2.0, y=0.0, z=0.0)
    ke = utils.calculate_kinetic_energy(5.0, velocity)
    print(f"Kinetic energy of 5kg object moving at 2m/s: {ke} J")
    
    # Test validator
    validator = SimulationValidator(max_deviation_percent=5.0)
    print(f"Created validator with {validator.max_deviation_percent}% tolerance")
    
    print("Utility tests completed successfully!")


if __name__ == "__main__":
    test_utilities()