"""Object detection module for the VLA system."""

import cv2
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from ..models.detected_object import DetectedObject
import uuid


class ObjectDetector:
    """Class to handle object detection and recognition."""
    
    def __init__(self):
        """Initialize the object detector."""
        # For this implementation, we'll use OpenCV's built-in methods
        # In a real implementation, you might use YOLO, SSD, or other advanced models
        
        # Define common colors to detect (in HSV format)
        self.color_ranges = {
            'red': [(0, 50, 50), (10, 255, 255)],      # Lower red range
            'red2': [(170, 50, 50), (180, 255, 255)],  # Upper red range
            'blue': [(100, 50, 50), (130, 255, 255)],
            'green': [(40, 50, 50), (80, 255, 255)],
            'yellow': [(20, 50, 50), (30, 255, 255)],
            'orange': [(10, 50, 50), (20, 255, 255)],
        }
        
        # Define minimum area for objects to be considered valid
        self.min_object_area = 500  # pixels
    
    def detect_objects(self, image: np.ndarray, camera_matrix: Optional[np.ndarray] = None) -> List[DetectedObject]:
        """
        Detect objects in the given image.
        
        Args:
            image: Input image as numpy array (BGR format)
            camera_matrix: Optional camera matrix for 3D position estimation
            
        Returns:
            List of DetectedObject instances
        """
        detected_objects = []
        
        try:
            # Convert image to HSV for better color detection
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            
            # Detect objects by color
            for color_name, (lower, upper) in self.color_ranges.items():
                # Create mask for the current color
                mask = cv2.inRange(hsv, lower, upper)
                
                # Apply morphological operations to clean up the mask
                kernel = np.ones((5,5), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                
                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                for contour in contours:
                    # Calculate contour area and filter small objects
                    area = cv2.contourArea(contour)
                    if area < self.min_object_area:
                        continue
                    
                    # Get bounding box for the contour
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calculate center position
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Estimate 3D position if camera matrix is provided
                    # For this simple implementation, we'll use 2D coordinates directly
                    # In a real system, you'd use depth information and camera calibration
                    position_x = float(center_x)
                    position_y = float(center_y)
                    position_z = float(area) * 0.001  # Use area as a proxy for distance (simplified)
                    
                    # Create detected object
                    obj = DetectedObject(
                        id=f"obj_{color_name}_{uuid.uuid4()}",
                        object_type='block',  # For color-based detection, assume block shape
                        position_x=position_x,
                        position_y=position_y,
                        position_z=position_z,
                        confidence=min(0.9, area / 10000.0),  # Scale confidence based on size
                        properties={
                            'color': color_name,
                            'area': area,
                            'bbox': {'x': x, 'y': y, 'width': w, 'height': h},
                            'center': {'x': center_x, 'y': center_y}
                        }
                    )
                    
                    detected_objects.append(obj)
            
            # Additionally, we can look for specific shapes (squares, circles) for other object types
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect circles using HoughCircles (for detecting round objects like balls)
            circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=50,
                param1=50,
                param2=30,
                minRadius=10,
                maxRadius=100
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                for (x, y, r) in circles:
                    # Check if this circle was already detected as a colored object
                    already_detected = False
                    for obj in detected_objects:
                        # Simple overlap check
                        dx = abs(obj.position_x - x)
                        dy = abs(obj.position_y - y)
                        if dx < r and dy < r:
                            already_detected = True
                            break
                    
                    if not already_detected:
                        obj = DetectedObject(
                            id=f"circle_{uuid.uuid4()}",
                            object_type='ball',  # Identified as round/circular
                            position_x=float(x),
                            position_y=float(y),
                            position_z=float(r) * 0.1,  # Use radius as a proxy for distance
                            confidence=min(0.8, (r * 2) / 100.0),  # Confidence based on size
                            properties={
                                'shape': 'circular',
                                'radius': r,
                                'center': {'x': x, 'y': y}
                            }
                        )
                        detected_objects.append(obj)
        
        except Exception as e:
            print(f"Error in object detection: {str(e)}")
        
        return detected_objects
    
    def detect_specific_objects(self, image: np.ndarray, target_object: str = "any") -> List[DetectedObject]:
        """
        Detect specific types of objects in the image.
        
        Args:
            image: Input image as numpy array (BGR format)
            target_object: Specific object type to detect (e.g., 'block', 'ball', 'person', 'any')
            
        Returns:
            List of DetectedObject instances matching the target type
        """
        all_objects = self.detect_objects(image)
        
        if target_object == "any":
            return all_objects
        else:
            # Filter objects by type if possible
            # Note: In this basic implementation, we're identifying object type by color and shape
            # rather than specific object types like 'person', so we'll return all objects
            # In a real implementation, you would use more sophisticated detection methods
            return [obj for obj in all_objects]
    
    def filter_by_confidence(self, objects: List[DetectedObject], min_confidence: float = 0.5) -> List[DetectedObject]:
        """Filter detected objects by minimum confidence threshold."""
        return [obj for obj in objects if obj.confidence >= min_confidence]
    
    def filter_by_object_type(self, objects: List[DetectedObject], object_type: str) -> List[DetectedObject]:
        """Filter detected objects by specific object type."""
        return [obj for obj in objects if obj.object_type == object_type]