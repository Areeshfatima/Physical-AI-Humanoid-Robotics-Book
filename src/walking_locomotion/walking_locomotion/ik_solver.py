import rclpy
from rclpy.node import Node
import numpy as np
import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class InverseKinematicsSolver(Node):
    """
    Inverse Kinematics Solver for humanoid robot leg control
    Implements a geometric inverse kinematics solver for a 2-DOF leg model
    """
    
    def __init__(self):
        super().__init__('ik_solver')
        
        # Leg parameters based on the humanoid URDF
        self.femur_length = 0.2  # Upper leg length
        self.tibia_length = 0.2  # Lower leg length
        
        # Initialize joint angle values
        self.left_leg_angles = [0.0, 0.0]  # [hip_angle, knee_angle]
        self.right_leg_angles = [0.0, 0.0]  # [hip_angle, knee_angle]
        
        # Publishers for joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            'joint_commands',
            10
        )
        
        # Subscriber for target foot positions
        self.target_sub = self.create_subscription(
            Float64MultiArray,
            'foot_target_positions',
            self.foot_target_callback,
            10
        )
        
        self.get_logger().info('Inverse Kinematics Solver initialized')
        
    def calculate_leg_ik(self, x, z, is_left_leg=True):
        """
        Calculate inverse kinematics for a 2-DOF leg
        x: Forward/backward position relative to hip
        z: Height position (negative = below hip)
        is_left_leg: True for left leg, False for right leg
        Returns: [hip_angle, knee_angle] in radians
        """
        # Ensure the target position is reachable
        target_distance = math.sqrt(x**2 + z**2)
        max_reach = self.femur_length + self.tibia_length
        
        if target_distance > max_reach:
            # Scale down the target to the maximum reach
            scale = max_reach / target_distance
            x *= scale
            z *= scale
        
        # Calculate knee angle using law of cosines
        # a, b: femur and tibia lengths
        # c: distance from hip to target
        a = self.femur_length
        b = self.tibia_length
        c = math.sqrt(x**2 + z**2)
        
        # Angle at knee
        knee_angle = math.pi - math.acos((a**2 + b**2 - c**2) / (2*a*b))
        
        # Calculate hip angle
        # First angle between hip-target line and vertical
        alpha = math.atan2(x, abs(z))
        
        # Angle between femur and hip-target line
        beta = math.acos((a**2 + c**2 - b**2) / (2*a*c))
        
        # Total hip angle (positive for forward steps, negative for backward)
        if z >= 0:  # Target is above hip
            hip_angle = alpha + beta
        else:  # Target is below hip
            hip_angle = alpha + beta - math.pi/2
            
        # Adjust angles for the robot's joint configuration
        # In the URDF, the joints are configured differently, so we may need to adjust signs
        hip_angle = -hip_angle  # Adjust based on URDF joint axis
        
        return [hip_angle, knee_angle]
    
    def foot_target_callback(self, msg):
        """
        Callback to handle target foot positions
        msg.data format: [left_foot_x, left_foot_z, right_foot_x, right_foot_z]
        """
        if len(msg.data) >= 4:
            # Calculate IK for left leg
            left_x, left_z = msg.data[0], msg.data[1]
            self.left_leg_angles = self.calculate_leg_ik(left_x, left_z, is_left_leg=True)
            
            # Calculate IK for right leg
            right_x, right_z = msg.data[2], msg.data[3]
            self.right_leg_angles = self.calculate_leg_ik(right_x, right_z, is_left_leg=False)
            
            # Publish the calculated joint angles
            self.publish_joint_commands()
            
            self.get_logger().info(
                f'Left leg: hip={self.left_leg_angles[0]:.3f}, knee={self.left_leg_angles[1]:.3f} | '
                f'Right leg: hip={self.right_leg_angles[0]:.3f}, knee={self.right_leg_angles[1]:.3f}'
            )
    
    def publish_joint_commands(self):
        """
        Publish the calculated joint angles to control the robot
        """
        cmd_msg = Float64MultiArray()
        
        # Assuming joint ordering: [left_hip, left_knee, right_hip, right_knee]
        cmd_msg.data = [
            self.left_leg_angles[0],  # left hip
            self.left_leg_angles[1],  # left knee
            self.right_leg_angles[0], # right hip
            self.right_leg_angles[1]  # right knee
        ]
        
        self.joint_cmd_pub.publish(cmd_msg)
    
    def get_current_joint_angles(self):
        """
        Get current joint angles for both legs
        """
        return {
            'left_leg': self.left_leg_angles,
            'right_leg': self.right_leg_angles
        }


def main(args=None):
    rclpy.init(args=args)
    
    node = InverseKinematicsSolver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('IK Solver node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()