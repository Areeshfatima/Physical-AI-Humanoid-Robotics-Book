# ROS 2 Publisher/Subscriber API Contract

## Overview
This contract specifies the communication interface for publisher/subscriber patterns in the ROS 2 fundamentals tutorial for humanoid robotics.

## Publisher: Talker Node

### Node Name
`talker`

### Published Topics
- **Topic**: `chatter`
- **Message Type**: `std_msgs/String`
- **QoS Profile**: Reliable, queue size 10
- **Frequency**: 2 Hz (every 0.5 seconds)

### Message Schema
```
std_msgs/String
- data: string (the message content)
```

### Performance Requirements
- Message latency: ≤50ms
- Message delivery: Best-effort with 99.9% availability

### Security Requirements
- Authentication and encryption as per ROS 2 security configuration
- Publisher identification in ROS 2 graph

## Subscriber: Listener Node

### Node Name
`listener`

### Subscribed Topics
- **Topic**: `chatter`
- **Message Type**: `std_msgs/String`
- **QoS Profile**: Reliable, queue size 10

### Callback Interface
```
callback(msg: std_msgs/String)
- msg.data: string (the received message content)
```

### Performance Requirements
- Message processing: ≤50ms from receipt to callback execution
- Message retention: Maintain queue of up to 10 messages

### Security Requirements
- Authentication and encryption as per ROS 2 security configuration
- Subscriber identification in ROS 2 graph

## Usage Example
```python
# Publisher sends messages
publisher.publish(String(data='Hello World: ' + str(count)))

# Subscriber receives messages
def chatter_callback(msg):
    print(f'I heard: [{msg.data}]')
```

## Error Handling
- Network interruptions: Automatic reconnection attempts
- Message corruption: ROS 2 middleware handles at transport layer
- Node disconnection: Graceful handling with status reporting