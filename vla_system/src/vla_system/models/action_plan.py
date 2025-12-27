"""Action Plan model for the VLA system."""

from dataclasses import dataclass
from typing import List
from datetime import datetime
from .plan_step import PlanStep


@dataclass
class ActionPlan:
    """Represents a plan of actions to be executed by the robot."""
    
    id: str
    command_id: str  # Reference to the original voice command
    plan_steps: List[PlanStep]
    created_at: datetime
    status: str  # "pending", "executing", "completed", "failed"
    execution_progress: int = 0  # Index of currently executing step
    
    def __post_init__(self):
        """Validate the ActionPlan after initialization."""
        if not self.command_id:
            raise ValueError("Command ID must not be empty")
        
        if not self.plan_steps:
            raise ValueError("Plan steps must contain at least one step")
        
        valid_statuses = ["pending", "executing", "completed", "failed"]
        if self.status not in valid_statuses:
            raise ValueError(f"Status must be one of {valid_statuses}")
        
        if self.execution_progress < 0:
            raise ValueError("Execution progress must be non-negative")