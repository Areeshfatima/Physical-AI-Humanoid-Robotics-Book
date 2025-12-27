#!/usr/bin/env python3

"""
Basic ROS 2 Node Template for Simulation Components

This module provides a template for creating ROS 2 nodes that interact with
the Gazebo physics simulation and other simulation components.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
import time
import threading
from enum import Enum
from typing import Any, Dict, List, Optional


class SimulationNodeState(Enum):
    """Enumeration for node states"""
    INITIALIZING = "initializing"
    RUNNING = "running"
    PAUSED = "paused"
    SHUTTING_DOWN = "shutting_down"
    ERROR = "error"


class SimulationComponentNode(Node):
    """
    Base class for simulation components that provides common functionality
    and structure for nodes interacting with Gazebo physics simulation.
    """
    
    def __init__(self, node_name: str, **kwargs: Any):
        super().__init__(node_name, **kwargs)
        
        # Initialize node state
        self.state = SimulationNodeState.INITIALIZING
        self.state_lock = threading.Lock()
        
        # Set up common QoS profiles for simulation
        self.default_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(String, f'/{node_name}/status', self.default_qos)
        self.health_publisher = self.create_publisher(String, f'/{node_name}/health', self.default_qos)
        
        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.health_timer = self.create_timer(2.0, self.publish_health)
        
        # Internal state tracking
        self.start_time = time.time()
        self.last_update_time = self.get_clock().now()
        self.errors_count = 0
        self.warnings_count = 0
        
        # Setup complete
        self.set_state(SimulationNodeState.RUNNING)
        self.get_logger().info(f'{node_name} initialized and running')
    
    def set_state(self, new_state: SimulationNodeState) -> None:
        """Safely update the node state"""
        with self.state_lock:
            old_state = self.state
            self.state = new_state
            self.get_logger().debug(f'State transition: {old_state.value} -> {new_state.value}')
    
    def get_state(self) -> SimulationNodeState:
        """Get the current node state"""
        with self.state_lock:
            return self.state
    
    def publish_status(self) -> None:
        """Publish periodic status message"""
        if self.get_state() != SimulationNodeState.SHUTTING_DOWN:
            status_msg = String()
            status_msg.data = f"State: {self.get_state().value}, Runtime: {time.time() - self.start_time:.1f}s"
            self.status_publisher.publish(status_msg)
    
    def publish_health(self) -> None:
        """Publish periodic health message"""
        if self.get_state() != SimulationNodeState.SHUTTING_DOWN:
            health_msg = String()
            health_status = "HEALTHY" if self.errors_count == 0 else "ERRORS_DETECTED"
            health_msg.data = f"{health_status} - Errors: {self.errors_count}, Warnings: {self.warnings_count}"
            self.health_publisher.publish(health_msg)
    
    def register_shutdown_hook(self, hook_func) -> None:
        """Register a function to be called during shutdown"""
        import atexit
        atexit.register(hook_func)
    
    def safe_execution(self, func, *args, error_msg: str = "", **kwargs) -> Optional[Any]:
        """Safely execute a function with error handling"""
        try:
            return func(*args, **kwargs)
        except Exception as e:
            self.errors_count += 1
            error_str = f"{error_msg}: {str(e)}" if error_msg else str(e)
            self.get_logger().error(error_str)
            self.set_state(SimulationNodeState.ERROR)
            return None


class PhysicsSimulationNode(SimulationComponentNode):
    """
    Specialized node for physics simulation components.
    Inherits from SimulationComponentNode and adds physics-specific functionality.
    """
    
    def __init__(self, node_name: str = "physics_simulation_node", **kwargs: Any):
        super().__init__(node_name, **kwargs)
        
        # Physics-specific parameters
        self.gravity = [0.0, 0.0, -9.81]  # Default Earth gravity
        self.simulation_time = 0.0
        self.real_time_factor = 1.0
        self.time_step = 0.001  # 1ms default time step
        
        # Physics publishers
        self.physics_param_publisher = self.create_publisher(
            String, f'/{node_name}/physics_parameters', self.default_qos
        )
        
        # Physics timers
        self.physics_param_timer = self.create_timer(5.0, self.publish_physics_params)
        
        self.get_logger().info(f'Physics simulation node {node_name} initialized')
    
    def publish_physics_params(self) -> None:
        """Publish current physics parameters"""
        params_msg = String()
        params_msg.data = (f"Gravity: {self.gravity}, TimeStep: {self.time_step}, "
                          f"RTF: {self.real_time_factor}, SimTime: {self.simulation_time:.3f}")
        self.physics_param_publisher.publish(params_msg)
    
    def update_simulation_time(self, new_time: float) -> None:
        """Update the simulation time"""
        self.simulation_time = new_time
        self.last_update_time = self.get_clock().now()
    
    def set_gravity(self, gravity_vector: List[float]) -> bool:
        """Set the gravity vector for the simulation"""
        if len(gravity_vector) != 3:
            self.get_logger().error("Gravity vector must have 3 components [x, y, z]")
            return False
        
        self.gravity = gravity_vector
        self.get_logger().info(f"Gravity set to: {self.gravity}")
        return True
    
    def set_real_time_factor(self, factor: float) -> bool:
        """Set the real-time factor for the simulation"""
        if factor <= 0:
            self.get_logger().error("Real-time factor must be positive")
            return False
        
        self.real_time_factor = factor
        self.get_logger().info(f"Real-time factor set to: {factor}")
        return True
    
    def set_time_step(self, time_step: float) -> bool:
        """Set the time step for the simulation"""
        if time_step <= 0:
            self.get_logger().error("Time step must be positive")
            return False
        
        self.time_step = time_step
        self.get_logger().info(f"Time step set to: {time_step}")
        return True


class SensorSimulationNode(SimulationComponentNode):
    """
    Specialized node for sensor simulation components.
    Inherits from SimulationComponentNode and adds sensor-specific functionality.
    """
    
    def __init__(self, node_name: str = "sensor_simulation_node", sensor_types: List[str] = None, **kwargs: Any):
        super().__init__(node_name, **kwargs)
        
        # Sensor-specific parameters
        self.sensor_types = sensor_types or []
        self.sensors_active = {stype: True for stype in self.sensor_types}
        self.sensor_data_buffers = {stype: [] for stype in self.sensor_types}
        self.sensor_frequency = 30.0  # Default 30Hz for most sensors
        
        # Sensor publishers and subscribers
        self.sensor_status_publisher = self.create_publisher(
            String, f'/{node_name}/sensor_status', self.default_qos
        )
        
        # Sensor timers
        self.sensor_status_timer = self.create_timer(2.0, self.publish_sensor_status)
        
        self.get_logger().info(f'Sensor simulation node {node_name} initialized for {len(self.sensor_types)} sensor types')
    
    def publish_sensor_status(self) -> None:
        """Publish current sensor status"""
        status_parts = []
        for sensor_type, active in self.sensors_active.items():
            buffer_size = len(self.sensor_data_buffers[sensor_type])
            status_parts.append(f"{sensor_type}: {'ON' if active else 'OFF'} ({buffer_size} samples)")
        
        status_msg = String()
        status_msg.data = "|".join(status_parts)
        self.sensor_status_publisher.publish(status_msg)
    
    def activate_sensor(self, sensor_type: str) -> bool:
        """Activate a specific sensor"""
        if sensor_type not in self.sensors_active:
            self.get_logger().warn(f"Unknown sensor type: {sensor_type}")
            return False
        
        self.sensors_active[sensor_type] = True
        self.get_logger().info(f"Activated sensor: {sensor_type}")
        return True
    
    def deactivate_sensor(self, sensor_type: str) -> bool:
        """Deactivate a specific sensor"""
        if sensor_type not in self.sensors_active:
            self.get_logger().warn(f"Unknown sensor type: {sensor_type}")
            return False
        
        self.sensors_active[sensor_type] = False
        self.get_logger().info(f"Deactivated sensor: {sensor_type}")
        return True
    
    def add_sensor_data(self, sensor_type: str, data: Any) -> bool:
        """Add data to a sensor's buffer"""
        if sensor_type not in self.sensor_data_buffers:
            self.get_logger().warn(f"Unknown sensor type: {sensor_type}")
            return False
        
        # Add data to buffer, keeping only the last 100 entries
        self.sensor_data_buffers[sensor_type].append(data)
        if len(self.sensor_data_buffers[sensor_type]) > 100:
            self.sensor_data_buffers[sensor_type].pop(0)
        
        return True
    
    def get_sensor_data(self, sensor_type: str, num_samples: int = 1) -> Optional[List[Any]]:
        """Get recent sensor data"""
        if sensor_type not in self.sensor_data_buffers:
            return None
        
        buffer = self.sensor_data_buffers[sensor_type]
        if not buffer:
            return None
        
        # Return the last num_samples or all available samples
        return buffer[-min(num_samples, len(buffer)):]


def main(args=None):
    """
    Main function demonstrating the simulation node templates
    """
    rclpy.init(args=args)
    
    # Example 1: Basic simulation component node
    basic_node = SimulationComponentNode("demo_basic_node")
    
    # Example 2: Physics simulation node
    physics_node = PhysicsSimulationNode("demo_physics_node")
    physics_node.set_gravity([0.0, 0.0, -8.0])  # Lower gravity for demo
    physics_node.update_simulation_time(1.5)
    
    # Example 3: Sensor simulation node
    sensor_node = SensorSimulationNode("demo_sensor_node", ["lidar", "camera", "imu"])
    sensor_node.activate_sensor("lidar")
    sensor_node.add_sensor_data("lidar", [1.0, 1.1, 1.2])
    
    try:
        # Run nodes briefly to show they work
        start_time = time.time()
        timeout = 5.0  # Run for 5 seconds
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(basic_node, timeout_sec=0.1)
            rclpy.spin_once(physics_node, timeout_sec=0.1)
            rclpy.spin_once(sensor_node, timeout_sec=0.1)
    
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        basic_node.destroy_node()
        physics_node.destroy_node()
        sensor_node.destroy_node()
        rclpy.shutdown()
        
        print("Simulation node templates demo completed!")


if __name__ == '__main__':
    main()