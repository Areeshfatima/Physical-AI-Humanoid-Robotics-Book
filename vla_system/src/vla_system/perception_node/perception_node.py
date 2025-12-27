"""ROS 2 node for visual perception in the VLA system."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
# Note: In a real implementation, you would use cv_bridge to convert ROS images to OpenCV format
# For this implementation, we'll simulate the conversion
try:
    from cv_bridge import CvBridge
except ImportError:
    # Mock CvBridge if not available
    class CvBridge:
        def imgmsg_to_cv2(self, img_msg, desired_encoding="passthrough"):
            # This is a simplified mock - in reality, you'd properly decode the image
            # For this example, we'll return a dummy image
            import numpy as np
            # Create a dummy image - in a real implementation, you'd properly decode img_msg
            return np.zeros((480, 640, 3), dtype=np.uint8)  # Default 640x480 RGB image
    CvBridge = CvBridge()
import numpy as np
import json
import uuid
from datetime import datetime
from ..models.perception_data import PerceptionData
from ..models.detected_object import DetectedObject
from .object_detector import ObjectDetector


class PerceptionNode(Node):
    """ROS 2 node that handles visual perception and object detection."""
    
    def __init__(self):
        super().__init__('perception_node')
        
        # Get parameters
        self.declare_parameter('detection_confidence_threshold', 0.7)
        self.declare_parameter('max_detection_range', 5.0)
        self.declare_parameter('object_classes', ['block', 'obstacle', 'target'])
        
        self.confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        self.max_detection_range = self.get_parameter('max_detection_range').value
        self.object_classes = self.get_parameter('object_classes').value
        
        # Initialize object detector
        self.object_detector = ObjectDetector()
        
        # Initialize CvBridge for image conversion
        self.cv_bridge = CvBridge()
        
        # Store current robot position to calculate object positions relative to robot
        self.current_robot_position = Point(x=0.0, y=0.0, z=0.0)
        
        # Create subscribers
        qos_profile = QoSProfile(depth=10)
        
        # Subscribe to camera image data
        self.image_subscriber = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            qos_profile
        )
        
        # Subscribe to robot position updates
        self.robot_position_subscriber = self.create_subscription(
            Point,
            'robot_position',
            self.robot_position_callback,
            qos_profile
        )
        
        # Subscribe to perception requests
        self.perception_request_subscriber = self.create_subscription(
            String,
            'perception_requests',
            self.perception_request_callback,
            qos_profile
        )
        
        # Create publishers
        self.perception_data_publisher = self.create_publisher(
            String,  # Using String for now - in a complete implementation we'd have a custom message
            'perception_data',
            qos_profile
        )
        
        self.detected_objects_publisher = self.create_publisher(
            String,
            'detected_objects',
            qos_profile
        )
        
        self.get_logger().info(
            f'Perception Node initialized with confidence threshold: {self.confidence_threshold}, '
            f'max detection range: {self.max_detection_range}m'
        )
    
    def image_callback(self, msg: Image):
        """Callback to process incoming camera images."""
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect objects in the image
            detected_objects = self.object_detector.detect_objects(cv_image)
            
            # Filter objects by confidence
            filtered_objects = self.object_detector.filter_by_confidence(
                detected_objects, 
                self.confidence_threshold
            )
            
            # Create perception data
            perception_id = str(uuid.uuid4())
            perception_data = PerceptionData(
                id=perception_id,
                timestamp=datetime.now(),
                detected_objects=filtered_objects,
                environment_map={},  # This would be populated with more complex mapping in a real system
                confidence_threshold=self.confidence_threshold
            )
            
            # Publish perception data
            perception_msg = String()
            perception_msg.data = json.dumps({
                'id': perception_data.id,
                'timestamp': perception_data.timestamp.isoformat(),
                'detected_objects': [
                    {
                        'id': obj.id,
                        'object_type': obj.object_type,
                        'position_x': obj.position_x,
                        'position_y': obj.position_y,
                        'position_z': obj.position_z,
                        'confidence': obj.confidence,
                        'properties': obj.properties
                    } for obj in perception_data.detected_objects
                ],
                'confidence_threshold': perception_data.confidence_threshold
            })
            
            self.perception_data_publisher.publish(perception_msg)
            
            # Also publish just the detected objects separately
            objects_msg = String()
            objects_msg.data = json.dumps([
                {
                    'id': obj.id,
                    'object_type': obj.object_type,
                    'position_x': obj.position_x,
                    'position_y': obj.position_y,
                    'position_z': obj.position_z,
                    'confidence': obj.confidence,
                    'properties': obj.properties
                } for obj in filtered_objects
            ])
            
            self.detected_objects_publisher.publish(objects_msg)
            
            self.get_logger().info(f'Detected {len(filtered_objects)} objects with confidence >= {self.confidence_threshold}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def robot_position_callback(self, msg: Point):
        """Callback to update robot position."""
        self.current_robot_position = msg
        self.get_logger().debug(f'Robot position updated to ({msg.x}, {msg.y}, {msg.z})')
    
    def perception_request_callback(self, msg: String):
        """Callback to handle perception requests."""
        try:
            # Parse the request - format could be "detect X" or "find Y"
            request = msg.data.strip().lower()
            
            # For now, we'll just trigger a detection cycle
            # In a real system, this might trigger specific detection based on the request
            self.get_logger().info(f'Received perception request: {request}')
            
            # This would normally trigger camera capture and analysis
            # For this implementation, we'll just log the request
            # In a real implementation, you'd want to capture a recent image and process it
            
        except Exception as e:
            self.get_logger().error(f'Error processing perception request: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = PerceptionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Perception Node stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()