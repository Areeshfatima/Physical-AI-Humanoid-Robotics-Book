# Data Model for Vision-Language-Action Systems

## Core Entities

### VoiceCommand
- **Fields**:
  - id: String (unique identifier)
  - text: String (transcribed speech)
  - timestamp: DateTime (when command was received)
  - confidence: Float (speech recognition confidence score 0.0-1.0)
  - source: String (audio source identifier)
  - processed: Boolean (whether command has been processed)
- **Relationships**:
  - One-to-Many with ActionPlan (a voice command can generate multiple action plans)
- **Validation**:
  - text must not be empty
  - confidence must be between 0.0 and 1.0
  - timestamp must be in the past

### ActionPlan
- **Fields**:
  - id: String (unique identifier)
  - command_id: String (reference to VoiceCommand)
  - plan_steps: Array[PlanStep] (ordered list of actions)
  - created_at: DateTime
  - status: String (e.g., "pending", "executing", "completed", "failed")
  - execution_progress: Integer (index of currently executing step)
- **Relationships**:
  - Many-to-One with VoiceCommand (many action plans can come from one voice command)
  - One-to-Many with PlanStep (contains multiple steps)
- **Validation**:
  - command_id must reference a valid VoiceCommand
  - plan_steps must contain at least one step
  - status must be one of defined values

### PlanStep
- **Fields**:
  - id: String (unique identifier)
  - action_type: String (e.g., "navigate", "perceive", "manipulate", "wait")
  - parameters: Object (action-specific parameters)
  - sequence_number: Integer (order in the plan)
  - estimated_duration: Float (in seconds)
  - executed: Boolean (whether step has been executed)
  - execution_result: String (e.g., "success", "failed", "skipped")
- **Relationships**:
  - Many-to-One with ActionPlan (belongs to one action plan)
- **Validation**:
  - action_type must be one of defined values
  - sequence_number must be non-negative
  - parameters must match the expected structure for action_type

### RobotState
- **Fields**:
  - position_x: Float (x coordinate in 3D space)
  - position_y: Float (y coordinate in 3D space)
  - position_z: Float (z coordinate in 3D space)
  - orientation_roll: Float (roll in Euler angles)
  - orientation_pitch: Float (pitch in Euler angles)
  - orientation_yaw: Float (yaw in Euler angles)
  - status: String (e.g., "idle", "navigating", "executing_action", "error")
  - battery_level: Float (0.0-1.0)
  - last_updated: DateTime
- **Validation**:
  - All position and orientation values should be valid numbers
  - battery_level must be between 0.0 and 1.0
  - status must be one of defined values

### PerceptionData
- **Fields**:
  - id: String (unique identifier)
  - timestamp: DateTime
  - detected_objects: Array[DetectedObject] (list of objects detected in environment)
  - environment_map: Object (representation of environment)
  - confidence_threshold: Float (minimum confidence for object detection)
- **Relationships**:
  - One-to-Many with DetectedObject (contains multiple detected objects)
- **Validation**:
  - timestamp must be in the past
  - confidence_threshold must be between 0.0 and 1.0

### DetectedObject
- **Fields**:
  - id: String (unique identifier)
  - object_type: String (e.g., "block", "obstacle", "target")
  - position_x: Float (x coordinate relative to robot)
  - position_y: Float (y coordinate relative to robot)
  - position_z: Float (z coordinate relative to robot)
  - confidence: Float (detection confidence 0.0-1.0)
  - properties: Object (additional characteristics)
- **Relationships**:
  - Many-to-One with PerceptionData (belongs to one perception data record)
- **Validation**:
  - confidence must be between 0.0 and 1.0
  - position values should be valid numbers
  - object_type must be from a defined set of types

### NavigationPlan
- **Fields**:
  - id: String (unique identifier)
  - destination_x: Float (target x coordinate)
  - destination_y: Float (target y coordinate)
  - destination_z: Float (target z coordinate)
  - path: Array[PathPoint] (ordered list of waypoints)
  - estimated_duration: Float (in seconds)
  - status: String (e.g., "planning", "ready", "executing", "completed", "failed")
- **Relationships**:
  - One-to-Many with PathPoint (contains multiple path points)
- **Validation**:
  - destination coordinates must be valid numbers
  - status must be one of defined values
  - path must contain at least one point for non-trivial destinations

### PathPoint
- **Fields**:
  - id: String (unique identifier)
  - x: Float
  - y: Float
  - z: Float
  - sequence_number: Integer (order in path)
- **Relationships**:
  - Many-to-One with NavigationPlan (belongs to one navigation plan)
- **Validation**:
  - All coordinate values must be valid numbers
  - sequence_number must be non-negative

## State Transitions

### ActionPlan States
- pending → executing: When plan execution begins
- executing → completed: When all steps are successfully executed
- executing → failed: When a step fails and no recovery is possible
- executing → pending: When plan needs to be updated based on new perception data

### RobotState Statuses
- idle → navigating: When navigation command is received
- navigating → executing_action: When navigation is complete and action execution begins
- executing_action → idle: When action execution is complete
- [any] → error: When an error condition occurs
- error → idle: When error is resolved

## Validation Rules from Requirements

1. **FR-001**: Speech recognition accuracy should be at least 85% in normal conditions and 80% in noisy conditions
2. **FR-002**: Action plans must correctly translate natural language commands to robot actions with 90% accuracy for simple commands
3. **FR-003**: Robot action sequences must execute correctly on the humanoid robot or its simulation
4. **FR-007**: System must provide feedback about execution status of voice commands
5. **FR-009**: Perceived objects must be correctly identified with position information
6. **FR-010**: Navigation plans must successfully reach target locations when possible
7. **FR-011**: Error conditions must be handled gracefully with logging
8. **FR-012**: Input validation must prevent malicious commands from executing