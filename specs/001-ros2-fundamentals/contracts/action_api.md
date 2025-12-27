# ROS 2 Action API Contract

## Overview
This contract specifies the communication interface for action client/server patterns in the ROS 2 fundamentals tutorial for humanoid robotics, specifically for Fibonacci sequence generation.

## Action Server: Fibonacci Action Server

### Node Name
`fibonacci_action_server`

### Provided Actions
- **Action Name**: `fibonacci`
- **Action Type**: `example_interfaces/action/Fibonacci`
- **Communication Pattern**: Goal-oriented with feedback and result

### Goal Schema
```
example_interfaces/action/Fibonacci Goal
- order: int32 (number of Fibonacci numbers to generate)
```

### Feedback Schema
```
example_interfaces/action/Fibonacci Feedback
- sequence: int32[] (current sequence of Fibonacci numbers generated)
```

### Result Schema
```
example_interfaces/action/Fibonacci Result
- sequence: int32[] (final sequence of Fibonacci numbers)
```

### Performance Requirements
- Goal processing: ≤50ms per Fibonacci number computation
- Feedback transmission: ≤50ms for real-time updates
- Action availability: 99.9% uptime with auto-recovery

### Security Requirements
- Authentication and encryption as per ROS 2 security configuration
- Action server identification in ROS 2 graph

## Action Client: Fibonacci Action Client

### Node Name
`fibonacci_action_client`

### Consumed Actions
- **Action Name**: `fibonacci`
- **Action Type**: `example_interfaces/action/Fibonacci`

### Goal Interface
```
send_goal(goal: Fibonacci.Goal) -> ClientGoalHandle
- goal.order: int32 (number of Fibonacci numbers to generate)
```

### Feedback Callback Interface
```
feedback_callback(feedback_msg: Fibonacci.Feedback)
- feedback_msg.sequence: int32[] (partial sequence during computation)
```

### Result Handling Interface
```
result_callback(future: Future)
- future.result().result.sequence: int32[] (final sequence)
- future.result().status: int8 (action completion status)
```

### Performance Requirements
- Goal transmission: ≤50ms
- Feedback processing: ≤50ms from reception to callback execution
- Result processing: ≤50ms from reception to callback execution

### Security Requirements
- Authentication and encryption as per ROS 2 security configuration
- Client identification in ROS 2 graph

## Usage Example
```python
# Server execution implementation
def execute_callback(self, goal_handle):
    feedback_msg = Fibonacci.Feedback()
    feedback_msg.sequence = [0, 1]
    
    for i in range(1, goal_handle.request.order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            return Fibonacci.Result()
            
        feedback_msg.sequence.append(
            feedback_msg.sequence[i] + feedback_msg.sequence[i-1])
        goal_handle.publish_feedback(feedback_msg)
    
    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = feedback_msg.sequence
    return result

# Client goal request
goal_msg = Fibonacci.Goal()
goal_msg.order = 10
self._send_goal_future = self._action_client.send_goal_async(
    goal_msg,
    feedback_callback=self.feedback_callback)
```

## Action States
- `PENDING`: Goal accepted but not yet started
- `ACTIVE`: Goal is currently being processed
- `CANCELING`: Goal is being canceled
- `SUCCEEDED`: Goal completed successfully
- `CANCELED`: Goal was canceled
- `ABORTED`: Goal failed to complete

## Error Handling
- Invalid goal parameters: Server validates input and rejects invalid goals
- Goal preemption: Server can handle new goals replacing current ones
- Execution failures: Server reports appropriate status for failed executions
- Network interruptions: Built-in action call retry and reconnection mechanisms