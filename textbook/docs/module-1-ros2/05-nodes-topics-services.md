---
sidebar_position: 5
title: "Chapter 5: Nodes, Topics, Services, and Actions"
description: "Deep dive into ROS 2 communication patterns"
---

# Nodes, Topics, Services, and Actions

## Learning Objectives

By the end of this chapter, you will be able to:

1. Create ROS 2 nodes in Python using rclpy
2. Implement publishers and subscribers for topic communication
3. Create and call services for synchronous operations
4. Implement action servers and clients for long-running tasks

## Prerequisites

- [Chapter 4: ROS 2 Architecture](/module-1-ros2/ros2-architecture)
- ROS 2 Humble installed and configured
- Python 3.10+ programming knowledge

## ROS 2 Nodes

A **node** is the fundamental computational unit in ROS 2. Good node design follows the single responsibility principle.

### Creating a Basic Node

```python
#!/usr/bin/env python3
"""Basic ROS 2 node example."""

import rclpy
from rclpy.node import Node


class MinimalNode(Node):
    """A minimal ROS 2 node."""
    
    def __init__(self):
        # Initialize the node with a name
        super().__init__('minimal_node')
        
        # Log that the node has started
        self.get_logger().info('Minimal node has been started!')
    
    def run(self):
        """Main execution loop."""
        self.get_logger().info('Node is running...')


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create the node
    node = MinimalNode()
    
    try:
        # Spin the node so callbacks are called
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Node Lifecycle

```
rclpy.init()
    │
    ▼
Node.__init__()
    │
    ▼
rclpy.spin(node)  ◄─── Callbacks executed here
    │
    ▼
node.destroy_node()
    │
    ▼
rclpy.shutdown()
```

## Topics: Publish/Subscribe Communication

Topics enable **many-to-many** asynchronous communication.

### Publisher Example

```python
#!/usr/bin/env python3
"""ROS 2 Publisher example."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """Publishes messages to a topic."""
    
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Create a publisher
        # Args: message_type, topic_name, queue_size
        self.publisher = self.create_publisher(
            String,
            'chatter',
            10
        )
        
        # Create a timer to publish periodically
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
    
    def timer_callback(self):
        """Called periodically by the timer."""
        msg = String()
        msg.data = f'Hello, ROS 2! Count: {self.count}'
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Subscriber Example

```python
#!/usr/bin/env python3
"""ROS 2 Subscriber example."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """Subscribes to messages from a topic."""
    
    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Create a subscription
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
    
    def listener_callback(self, msg):
        """Called when a message is received."""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Custom Message Types

Messages are defined in `.msg` files:

```
# my_package/msg/RobotStatus.msg
string robot_name
float64 battery_level
bool is_moving
geometry_msgs/Pose current_pose
```

Using custom messages:

```python
from my_package.msg import RobotStatus

msg = RobotStatus()
msg.robot_name = "Atlas"
msg.battery_level = 85.5
msg.is_moving = True
```

## Services: Request/Response Communication

Services provide **synchronous** request/response communication.

### Service Definition

```
# my_package/srv/AddTwoInts.srv
int64 a
int64 b
---
int64 sum
```

### Service Server

```python
#!/usr/bin/env python3
"""ROS 2 Service Server example."""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service server that adds two integers."""
    
    def __init__(self):
        super().__init__('add_two_ints_server')
        
        # Create a service
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        
        self.get_logger().info('Service ready!')
    
    def add_callback(self, request, response):
        """Handle service requests."""
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Service Client

```python
#!/usr/bin/env python3
"""ROS 2 Service Client example."""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Service client that calls the add service."""
    
    def __init__(self):
        super().__init__('add_two_ints_client')
        
        # Create a client
        self.client = self.create_client(
            AddTwoInts,
            'add_two_ints'
        )
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
    
    def send_request(self, a, b):
        """Send a request to the service."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        
        # Call service asynchronously
        future = self.client.call_async(request)
        return future


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    
    # Send request
    future = node.send_request(3, 5)
    
    # Wait for response
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result()
    node.get_logger().info(f'Result: {result.sum}')
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Actions: Long-Running Tasks with Feedback

Actions are ideal for tasks that:
- Take a long time to complete
- Need to provide progress feedback
- May need to be canceled

### Action Definition

```
# my_package/action/Fibonacci.action
# Goal
int32 order
---
# Result
int32[] sequence
---
# Feedback
int32[] partial_sequence
```

### Action Server

```python
#!/usr/bin/env python3
"""ROS 2 Action Server example."""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """Action server that computes Fibonacci sequence."""
    
    def __init__(self):
        super().__init__('fibonacci_action_server')
        
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )
        
        self.get_logger().info('Action server ready!')
    
    def execute_callback(self, goal_handle):
        """Execute the action goal."""
        self.get_logger().info('Executing goal...')
        
        # Initialize sequence
        sequence = [0, 1]
        feedback_msg = Fibonacci.Feedback()
        
        for i in range(1, goal_handle.request.order):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()
            
            # Compute next number
            sequence.append(sequence[i] + sequence[i-1])
            
            # Send feedback
            feedback_msg.partial_sequence = sequence
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {sequence}')
            
            # Simulate work
            time.sleep(1)
        
        # Goal succeeded
        goal_handle.succeed()
        
        result = Fibonacci.Result()
        result.sequence = sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Action Client

```python
#!/usr/bin/env python3
"""ROS 2 Action Client example."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """Action client for Fibonacci computation."""
    
    def __init__(self):
        super().__init__('fibonacci_action_client')
        
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )
    
    def send_goal(self, order):
        """Send a goal to the action server."""
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        
        self._action_client.wait_for_server()
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        
        self.get_logger().info('Goal accepted')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Handle the final result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """Handle feedback from the server."""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Received feedback: {feedback.partial_sequence}'
        )


def main(args=None):
    rclpy.init(args=args)
    action_client = FibonacciActionClient()
    action_client.send_goal(10)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

## Comparison: When to Use What

| Pattern | Use Case | Example |
|---------|----------|---------|
| **Topic** | Continuous data streams | Sensor data, robot pose |
| **Service** | Quick request/response | Get parameter, compute value |
| **Action** | Long-running tasks | Navigate to goal, pick object |

```
             ┌─────────────────────────────────────────┐
             │           Communication Patterns         │
             └─────────────────────────────────────────┘
             
Topic        ●─────────────────────────────────────────●
             One-way, continuous, fire-and-forget
             
Service      ●─────────────●  Quick round-trip
             Request → Response
             
Action       ●─────────●─────────●─────────●─────────●
             Goal → Feedback → Feedback → Feedback → Result
```

## Summary

- **Nodes** are single-purpose computational units
- **Topics** provide asynchronous publish/subscribe communication
- **Services** enable synchronous request/response
- **Actions** handle long-running tasks with feedback and cancellation
- Choose the right pattern based on your communication needs

## Exercises

### Exercise 1: Publisher/Subscriber
Create a publisher that sends `geometry_msgs/Twist` messages and a subscriber that logs the received velocity commands.

### Exercise 2: Service
Create a service that takes a robot name (string) and returns its battery level (float).

### Exercise 3: Action
Create an action that simulates a robot moving to a goal position, sending distance-remaining as feedback.

<details>
<summary>Exercise 1 Solution</summary>

```python
from geometry_msgs.msg import Twist

# In publisher
self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
msg = Twist()
msg.linear.x = 0.5
msg.angular.z = 0.1
self.publisher.publish(msg)
```

</details>

## References

1. ROS 2 Tutorials. https://docs.ros.org/en/humble/Tutorials.html
2. rclpy API Documentation. https://docs.ros2.org/humble/api/rclpy/

---

**Next Chapter**: [Building ROS 2 Packages with Python](/module-1-ros2/ros2-python-packages)
