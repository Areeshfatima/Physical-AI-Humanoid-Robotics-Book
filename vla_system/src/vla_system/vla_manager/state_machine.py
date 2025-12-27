"""State machine for the VLA system."""

from enum import Enum
from typing import Dict, Any, Optional
from datetime import datetime
import uuid


class VLAState(Enum):
    """Enumeration of possible VLA system states."""
    IDLE = "idle"
    PROCESSING_COMMAND = "processing_command"
    PLANNING_ACTIONS = "planning_actions"
    EXECUTING_ACTIONS = "executing_actions"
    NAVIGATING = "navigating"
    PERCEIVING = "perceiving"
    MANIPULATING = "manipulating"
    WAITING = "waiting"
    ERROR = "error"
    COMPLETED = "completed"


class StateMachine:
    """State machine to manage the execution states of the VLA system."""
    
    def __init__(self):
        """Initialize the state machine."""
        self.current_state = VLAState.IDLE
        self.state_history = []
        self.last_state_change = datetime.now()
        self.active_action_plan_id = None
        self.active_action_step = None
        
        # Store state transition callbacks
        self.state_callbacks: Dict[str, callable] = {}
    
    def get_current_state(self) -> VLAState:
        """Get the current state of the system."""
        return self.current_state
    
    def set_state(self, new_state: VLAState, context: Optional[Dict[str, Any]] = None):
        """Set a new state and record the transition."""
        old_state = self.current_state
        self.current_state = new_state
        
        # Record state transition
        transition = {
            'id': str(uuid.uuid4()),
            'from_state': old_state.value,
            'to_state': new_state.value,
            'timestamp': datetime.now(),
            'context': context or {}
        }
        self.state_history.append(transition)
        self.last_state_change = datetime.now()
        
        # Execute any registered callback for this state transition
        callback_key = f"{old_state.value}_to_{new_state.value}"
        if callback_key in self.state_callbacks:
            self.state_callbacks[callback_key](context)
        
        # Execute general state callback
        if new_state.value in self.state_callbacks:
            self.state_callbacks[new_state.value](context)
    
    def register_callback(self, state_or_transition: str, callback: callable):
        """Register a callback for a specific state or state transition."""
        self.state_callbacks[state_or_transition] = callback
    
    def can_transition_to(self, target_state: VLAState) -> bool:
        """Check if the system can transition to the target state."""
        # Define valid state transitions
        valid_transitions = {
            VLAState.IDLE: [
                VLAState.PROCESSING_COMMAND,
                VLAState.ERROR
            ],
            VLAState.PROCESSING_COMMAND: [
                VLAState.PLANNING_ACTIONS,
                VLAState.ERROR,
                VLAState.IDLE
            ],
            VLAState.PLANNING_ACTIONS: [
                VLAState.EXECUTING_ACTIONS,
                VLAState.ERROR,
                VLAState.IDLE
            ],
            VLAState.EXECUTING_ACTIONS: [
                VLAState.NAVIGATING,
                VLAState.PERCEIVING,
                VLAState.MANIPULATING,
                VLAState.WAITING,
                VLAState.COMPLETED,
                VLAState.ERROR
            ],
            VLAState.NAVIGATING: [
                VLAState.EXECUTING_ACTIONS,
                VLAState.PERCEIVING,
                VLAState.COMPLETED,
                VLAState.ERROR
            ],
            VLAState.PERCEIVING: [
                VLAState.EXECUTING_ACTIONS,
                VLAState.NAVIGATING,
                VLAState.COMPLETED,
                VLAState.ERROR
            ],
            VLAState.MANIPULATING: [
                VLAState.EXECUTING_ACTIONS,
                VLAState.COMPLETED,
                VLAState.ERROR
            ],
            VLAState.WAITING: [
                VLAState.EXECUTING_ACTIONS,
                VLAState.COMPLETED,
                VLAState.IDLE
            ],
            VLAState.ERROR: [
                VLAState.IDLE,
                VLAState.PROCESSING_COMMAND
            ],
            VLAState.COMPLETED: [
                VLAState.IDLE
            ]
        }
        
        current_state_transitions = valid_transitions.get(self.current_state, [])
        return target_state in current_state_transitions
    
    def transition_to(self, target_state: VLAState, context: Optional[Dict[str, Any]] = None) -> bool:
        """Attempt to transition to a target state, returning success status."""
        if self.can_transition_to(target_state):
            self.set_state(target_state, context)
            return True
        else:
            print(f"Invalid state transition from {self.current_state.value} to {target_state.value}")
            return False
    
    def get_state_duration(self) -> float:
        """Get the duration of the current state in seconds."""
        now = datetime.now()
        duration = now - self.last_state_change
        return duration.total_seconds()
    
    def reset(self):
        """Reset the state machine to IDLE state."""
        self.set_state(VLAState.IDLE)
        self.state_history = []
        self.active_action_plan_id = None
        self.active_action_step = None
    
    def get_state_info(self) -> Dict[str, Any]:
        """Get detailed information about the current state."""
        return {
            'current_state': self.current_state.value,
            'state_duration': self.get_state_duration(),
            'total_transitions': len(self.state_history),
            'active_action_plan_id': self.active_action_plan_id,
            'active_action_step': self.active_action_step
        }