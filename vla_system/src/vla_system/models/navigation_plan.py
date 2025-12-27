"""Navigation Plan model for the VLA system."""

from dataclasses import dataclass
from typing import List
from .path_point import PathPoint


@dataclass
class NavigationPlan:
    """Represents a navigation plan for the robot."""
    
    id: str
    destination_x: float  # Target X coordinate
    destination_y: float  # Target Y coordinate
    destination_z: float  # Target Z coordinate
    path: List[PathPoint]  # Ordered list of waypoints
    estimated_duration: float  # Estimated time to reach destination in seconds
    status: str  # "planning", "ready", "executing", "completed", "failed"
    
    def __post_init__(self):
        """Validate the NavigationPlan after initialization."""
        valid_statuses = ["planning", "ready", "executing", "completed", "failed"]
        if self.status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")
        
        # Validate that path contains at least one point for non-trivial destinations
        if (self.destination_x != 0.0 or self.destination_y != 0.0 or self.destination_z != 0.0) and not self.path:
            raise ValueError("Path must contain at least one point for non-trivial destinations")