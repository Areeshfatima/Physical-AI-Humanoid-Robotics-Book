# ROS 2 Service API Contract

## Overview
This contract specifies the communication interface for service client/server patterns in the ROS 2 fundamentals tutorial for humanoid robotics.

## Service Server: Add Two Ints Server

### Node Name
`add_two_ints_server`

### Provided Services
- **Service Name**: `add_two_ints`
- **Service Type**: `example_interfaces/srv/AddTwoInts`
- **QoS Profile**: Services use reliable communication by default

### Request Schema
```
example_interfaces/srv/AddTwoInts Request
- a: int64 (first integer operand)
- b: int64 (second integer operand)
```

### Response Schema
```
example_interfaces/srv/AddTwoInts Response
- sum: int64 (result of adding a + b)
```

### Performance Requirements
- Request processing: ≤100ms response time
- Service availability: 99.9% uptime with auto-recovery

### Security Requirements
- Authentication and encryption as per ROS 2 security configuration
- Service identification in ROS 2 graph

## Service Client: Add Two Ints Client

### Node Name
`add_two_ints_client`

### Consumed Services
- **Service Name**: `add_two_ints`
- **Service Type**: `example_interfaces/srv/AddTwoInts`

### Request Interface
```
send_request(a: int64, b: int64) -> Future
```

### Response Handling
```
response_callback(future: Future)
- future.result().sum: int64 (the result of the addition)
```

### Performance Requirements
- Request transmission: ≤50ms
- Response handling: ≤50ms from reception to callback execution

### Security Requirements
- Authentication and encryption as per ROS 2 security configuration
- Client identification in ROS 2 graph

## Usage Example
```python
# Server callback implementation
def add_two_ints_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Request: {request.a} + {request.b}')
    return response

# Client request
request = AddTwoInts.Request()
request.a = 2
request.b = 3
future = self.cli.call_async(request)
```

## Error Handling
- Invalid parameters: Server validates input and returns appropriate errors
- Service unavailability: Client handles service not available gracefully
- Network interruptions: Built-in service call retry mechanisms