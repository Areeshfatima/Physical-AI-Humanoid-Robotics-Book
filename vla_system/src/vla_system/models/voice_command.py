"""Voice Command model for the VLA system."""

from dataclasses import dataclass
from typing import Optional
from datetime import datetime


@dataclass
class VoiceCommand:
    """Represents a voice command from a user."""
    
    id: str
    text: str
    timestamp: datetime
    confidence: float  # Speech recognition confidence score (0.0-1.0)
    source: str  # Audio source identifier
    processed: bool = False
    
    def __post_init__(self):
        """Validate the VoiceCommand after initialization."""
        if not self.text.strip():
            raise ValueError("Text must not be empty")
        
        if not 0.0 <= self.confidence <= 1.0:
            raise ValueError("Confidence must be between 0.0 and 1.0")
        
        if self.timestamp > datetime.now():
            raise ValueError("Timestamp must be in the past")