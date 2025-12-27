---
title: ROS2 Services and Actions
sidebar_position: 4
description: >-
  Advanced communication patterns for request-response and goal-oriented
  interactions
keywords:
  - ROS2
  - services
  - actions
  - request-response
  - goal-oriented
  - communication
id: chapter-3
---






# ROS2 Services and Actions

## Learning Objectives

After completing this chapter, you should be able to:
- Implement service-server communication patterns
- Design and execute action-based goal-oriented tasks
- Select appropriate communication patterns for different use cases
- Apply best practices for asynchronous and synchronous communication

## Introduction to Advanced Communication Patterns

While topics provide an excellent mechanism for continuous data streaming, many robotic applications require more sophisticated communication patterns. ROS2 provides two additional communication paradigms: **services** for request-response interactions and **actions** for goal-oriented tasks with feedback.

## Services

### Service Concept

Services provide **synchronous request-response** communication between nodes. This pattern is ideal for:

- Requesting immediate computation results
- Querying the state of a system
- Triggering specific actions with guaranteed completion
- Configuration changes

### Service Implementation

A service consists of three parts:

1. **Service Definition** (.srv file)
2. **Service Server** (implements the service)
3. **Service Client** (calls the service)

![ROS2 Service Communication Diagram](./images/service-communication.png)
*Figure 3.1: ROS2 service communication showing the request-response pattern*

#### Service Definition

The service definition specifies both the request and response data structures:

```text
# Request message
string name
int64 goal
---
# Response message
bool success
string message
int64 result
```

### Service Example

Here's a complete service implementation in Python:

**Service Definition** (srv/AddTwoInts.srv):
```text
int64 a
int64 b
---
int64 sum
```

**Service Server**:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(
            AddTwoInts, 
            'add_two_ints', 
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request received: {request.a} + {request.b} = {response.sum}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Service Client**:
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        f'Result: {response.sum}'
    )
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Actions

### Action Concept

Actions provide **asynchronous goal-oriented** communication with feedback. This pattern is ideal for:

- Long-running tasks (navigation, manipulation)
- Tasks requiring continuous feedback
- Operations that can be preempted or canceled
- Operations with intermediate results

### Action Structure

Actions have three message types:

1. **Goal**: What to do
2. **Feedback**: Progress updates during execution
3. **Result**: Final outcome of the action

### Action Implementation

#### Action Definition

Action definitions use .action files:

```text
# Goal: Define what the action should do
int32 order
string command
---
# Result: Define what the action returns
int32 complete
string sequence
---
# Feedback: Define progress updates during execution
int32 remaining
string status
```

### Action Example

Here's a complete action implementation in Python:

**Action Definition** (action/Fibonacci.action):
```text
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

**Action Server**:
```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]
        
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1]
            )
            
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info(f'Result: {result.sequence}')
        
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = FibonacciActionServer()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)
    
    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Action Client**:
```python
import time

from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci')

    def send_goal(self, order=10):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()

    action_client.send_goal()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()
```

## When to Use Each Communication Pattern

### Topics vs Services vs Actions

| Pattern | Use Case | Response Type | Asynchronous | Cancelable | Feedback |
|---------|----------|---------------|--------------|------------|----------|
| Topics | Continuous data streaming | None | Yes | No | No |
| Services | Request-response | Immediate | No | No | No |
| Actions | Long-running goals | Deferred | Yes | Yes | Yes |

### Choosing the Right Pattern

1. **Use Topics** for:
   - Sensor data streaming
   - Continuous state updates
   - Broadcasting information

2. **Use Services** for:
   - Simple request-response operations
   - Quick computations
   - Configuration changes

3. **Use Actions** for:
   - Long-running operations
   - Tasks with progress feedback
   - Operations that might need cancellation

## Best Practices

### Services

1. **Timeout Handling**: Always implement timeout handling in clients
2. **Error Responses**: Use appropriate response codes for different error conditions
3. **Synchronous Design**: Remember services are synchronous and can block execution
4. **Simple Operations**: Use for simple, quick operations only

### Actions

1. **Feedback Frequency**: Don't overwhelm the system with frequent feedback
2. **Goal Preemption**: Consider whether your action should support preemption
3. **Result Validation**: Validate results before completing the action
4. **Resource Management**: Properly clean up resources when an action is canceled

## Summary

Services and actions provide powerful communication patterns that complement [topics](./chapter-2.md) in ROS2. Services enable synchronous request-response interactions, while actions provide asynchronous goal-oriented communication with feedback and cancellation support.

Understanding when and how to use each communication pattern is crucial for designing robust and efficient robotic systems. The next chapter will explore system management with [ROS2 launch systems and package management](./chapter-4.md), which are essential for deploying complex robotic applications.

## Exercises

1. Create a service that performs a mathematical operation (e.g., addition) on two numbers received in the request.
2. Implement an action server that simulates a navigation task with feedback on progress.
3. Design a scenario where you would use services vs. actions vs. topics and justify your choice.

[Next: ROS2 Launch Systems and Package Management](./chapter-4.md) | [Previous: ROS2 Nodes and Topics](./chapter-2.md)