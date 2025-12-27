"""Path Point model for the VLA system."""

from dataclasses import dataclass


@dataclass
class PathPoint:
    """Represents a single point in a navigation path."""
    
    id: str
    x: float
    y: float
    z: float
    sequence_number: int  # Order in navigation path
    
    def __post_init__(self):
        """Validate the PathPoint after initialization."""
        if self.sequence_number < 0:
            raise ValueError("Sequence number must be non-negative")