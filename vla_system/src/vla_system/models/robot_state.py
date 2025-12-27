"""Robot State model for the VLA system."""

from dataclasses import dataclass
from datetime import datetime
from enum import Enum


class RobotStatus(Enum):
    """Enumeration of possible robot statuses."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    EXECUTING_ACTION = "executing_action"
    ERROR = "error"


@dataclass
class RobotState:
    """Represents the current state of the robot."""
    
    position_x: float  # X coordinate in 3D space
    position_y: float  # Y coordinate in 3D space
    position_z: float  # Z coordinate in 3D space
    orientation_roll: float  # Roll in Euler angles
    orientation_pitch: float  # Pitch in Euler angles
    orientation_yaw: float  # Yaw in Euler angles
    status: str  # Current robot status
    battery_level: float  # Battery level (0.0-1.0)
    last_updated: datetime
    
    def __post_init__(self):
        """Validate the RobotState after initialization."""
        valid_statuses = [status.value for status in RobotStatus]
        if self.status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")
        
        if not 0.0 <= self.battery_level <= 1.0:
            raise ValueError("Battery level must be between 0.0 and 1.0")
        
        if self.last_updated > datetime.now():
            raise ValueError("Last updated must be in the past")