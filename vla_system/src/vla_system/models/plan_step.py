"""Plan Step model for the VLA system."""

from dataclasses import dataclass
from typing import Dict, Any
from enum import Enum


class ActionType(Enum):
    """Enumeration of possible action types."""
    NAVIGATE = "navigate"
    PERCEIVE = "perceive"
    MANIPULATE = "manipulate"
    WAIT = "wait"


@dataclass
class PlanStep:
    """Represents a single step in an action plan."""
    
    id: str
    action_type: str  # Should be one of the values in ActionType
    parameters: Dict[str, Any]  # Action-specific parameters
    sequence_number: int  # Order of this step in the plan
    estimated_duration: float  # Estimated time to complete this step in seconds
    executed: bool = False  # Whether this step has been executed
    execution_result: str = "pending"  # "success", "failed", "skipped", or "pending"
    
    def __post_init__(self):
        """Validate the PlanStep after initialization."""
        valid_action_types = [action_type.value for action_type in ActionType]
        if self.action_type not in valid_action_types:
            raise ValueError(f"Action type must be one of {valid_action_types}")
        
        if self.sequence_number < 0:
            raise ValueError("Sequence number must be non-negative")
        
        valid_execution_results = ["success", "failed", "skipped", "pending"]
        if self.execution_result not in valid_execution_results:
            raise ValueError(f"Execution result must be one of {valid_execution_results}")