"""Detected Object model for the VLA system."""

from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class DetectedObject:
    """Represents an object detected by the robot's perception system."""
    
    id: str
    object_type: str  # "block", "obstacle", "target", etc.
    position_x: float  # X coordinate relative to robot
    position_y: float  # Y coordinate relative to robot
    position_z: float  # Z coordinate relative to robot
    confidence: float  # Detection confidence (0.0-1.0)
    properties: Dict[str, Any]  # Additional object characteristics
    
    def __post_init__(self):
        """Validate the DetectedObject after initialization."""
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Confidence must be between 0.0 and 1.0")
        
        # Define valid object types (this can be extended based on requirements)
        valid_object_types = ["block", "obstacle", "target", "person", "furniture", "unknown"]
        if self.object_type not in valid_object_types:
            raise ValueError(f"Object type must be one of {valid_object_types}")