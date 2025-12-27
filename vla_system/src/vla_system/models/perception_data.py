"""Perception Data model for the VLA system."""

from dataclasses import dataclass
from typing import List
from datetime import datetime
from .detected_object import DetectedObject


@dataclass
class PerceptionData:
    """Represents perception data captured by the robot."""
    
    id: str
    timestamp: datetime
    detected_objects: List[DetectedObject]  # List of objects detected in environment
    environment_map: dict  # Representation of environment (structure to be defined)
    confidence_threshold: float  # Minimum confidence for object detection (0.0-1.0)
    
    def __post_init__(self):
        """Validate the PerceptionData after initialization."""
        if self.timestamp > datetime.now():
            raise ValueError("Timestamp must be in the past")
        
        if not 0.0 <= self.confidence_threshold <= 1.0:
            raise ValueError("Confidence threshold must be between 0.0 and 1.0")