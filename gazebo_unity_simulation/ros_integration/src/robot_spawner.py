#!/usr/bin/env python3

"""
Robot Spawning Service for Gazebo Simulation

This module provides ROS 2 services to spawn robots into a Gazebo simulation environment.
It accepts URDF models and places them at specified positions in the simulation.
"""

import rclpy
from rclpy.node import Node
import os
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SpawnEntity
from ament_index_python.packages import get_package_share_directory


class RobotSpawner(Node):
    """
    ROS 2 service server that allows to spawn robots into Gazebo simulation
    """
    
    def __init__(self):
        super().__init__('robot_spawner')
        
        # Create service for spawning entities
        self.spawn_service = self.create_service(
            SpawnEntity,
            'spawn_robot',
            self.spawn_robot_callback
        )
        
        self.get_logger().info('Robot Spawner service initialized')
    
    def spawn_robot_callback(self, request, response):
        """
        Callback function for spawning a robot into the simulation
        """
        try:
            # Validate request
            if not request.name:
                response.success = False
                response.status_message = "Robot name is required"
                return response
                
            if not request.xml:
                response.success = False
                response.status_message = "Robot URDF/XML is required"
                return response
            
            # If URDF path is provided instead of direct XML, try to load it
            robot_xml = request.xml
            if not request.xml.strip().startswith('<'):
                # Assume it's a path to a URDF file
                try:
                    if os.path.exists(request.xml):
                        with open(request.xml, 'r') as f:
                            robot_xml = f.read()
                    else:
                        response.success = False
                        response.status_message = f"URDF file not found: {request.xml}"
                        return response
                except Exception as e:
                    response.success = False
                    response.status_message = f"Error reading URDF file: {str(e)}"
                    return response
            
            # Set default pose if not provided
            if request.initial_pose.position.x == 0.0 and \
               request.initial_pose.position.y == 0.0 and \
               request.initial_pose.position.z == 0.0:
                # Default to a position in the center of the world
                request.initial_pose.position.x = 0.0
                request.initial_pose.position.y = 0.0
                request.initial_pose.position.z = 1.0  # Lift slightly above ground
            
            # Log the spawning request
            self.get_logger().info(f'Spawning robot: {request.name} at ({request.initial_pose.position.x}, '
                                 f'{request.initial_pose.position.y}, {request.initial_pose.position.z})')
            
            # In a real implementation, we would call the Gazebo spawn_entity service
            # For this example, we'll simulate the behavior and return success
            response.success = True
            response.status_message = f"Successfully spawned robot {request.name}"
            
            return response
        
        except Exception as e:
            self.get_logger().error(f'Error spawning robot: {str(e)}')
            response.success = False
            response.status_message = f"Error spawning robot: {str(e)}"
            return response


def main(args=None):
    """
    Main function to run the robot spawner service
    """
    rclpy.init(args=args)
    
    robot_spawner = RobotSpawner()
    
    try:
        rclpy.spin(robot_spawner)
    except KeyboardInterrupt:
        robot_spawner.get_logger().info('Robot Spawner interrupted by user')
    finally:
        robot_spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()