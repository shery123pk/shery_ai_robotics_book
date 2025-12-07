---
sidebar_position: 3
---

# Services and Actions

Learn the request-response communication patterns in ROS 2: Services for synchronous operations and Actions for long-running tasks with feedback.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand when to use services vs. topics
- Create ROS 2 service servers and clients
- Understand when to use actions vs. services
- Create ROS 2 action servers and clients
- Handle action cancellation and feedback
- Choose the right communication pattern for your use case

---

## Communication Patterns Comparison

| Pattern | Use Case | Synchronous | Feedback | Cancellable |
|---------|----------|-------------|----------|-------------|
| **Topics** | Streaming data (sensors, commands) | No | No | N/A |
| **Services** | Quick request-response (status check) | Yes | No | No |
| **Actions** | Long tasks (navigation, manipulation) | No | Yes | Yes |

---

## ROS 2 Services

Services implement a **request-response** pattern:

```
┌─────────┐                          ┌─────────┐
│ Client  │ ─────── Request ──────> │ Server  │
│         │                          │         │
│         │ <───── Response ──────  │         │
└─────────┘                          └─────────┘
```

### When to Use Services

- **Configuration**: Set parameters, change modes
- **State Queries**: Get robot status, battery level
- **One-Time Operations**: Take a photo, reset odometry
- **Quick Computations**: Calculate inverse kinematics

**Don't use services for**:
- High-frequency data (use topics instead)
- Long-running tasks (use actions instead)

---

## Creating a Service Server

Let's create a service that adds two integers.

```python title="add_two_ints_server.py"
#!/usr/bin/env python3
"""
Service Server Example
Provides an addition service
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsServer(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('add_two_ints_server')

        # Create service
        self.srv = self.create_service(
            AddTwoInts,              # Service type
            'add_two_ints',          # Service name
            self.add_two_ints_callback  # Callback function
        )

        self.get_logger().info('Add Two Ints Server ready')

    def add_two_ints_callback(
        self,
        request: AddTwoInts.Request,
        response: AddTwoInts.Response
    ) -> AddTwoInts.Response:
        """Handle service requests."""
        # Perform computation
        response.sum = request.a + request.b

        self.get_logger().info(
            f'Request: {request.a} + {request.b} = {response.sum}'
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Service Server

```bash
python3 add_two_ints_server.py
```

**Output**:
```
[INFO] [add_two_ints_server]: Add Two Ints Server ready
```

---

## Creating a Service Client

Now let's create a client to call this service.

```python title="add_two_ints_client.py"
#!/usr/bin/env python3
"""
Service Client Example
Calls the addition service
"""

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):
    """Service client that requests addition."""

    def __init__(self):
        super().__init__('add_two_ints_client')

        # Create client
        self.cli = self.create_client(
            AddTwoInts,       # Service type
            'add_two_ints'    # Service name
        )

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Connected to service')

    def send_request(self, a: int, b: int):
        """Send addition request."""
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service asynchronously
        self.future = self.cli.call_async(request)
        self.get_logger().info(f'Sent request: {a} + {b}')


def main(args=None):
    rclpy.init(args=args)

    # Get numbers from command line
    if len(sys.argv) != 3:
        print('Usage: add_two_ints_client.py <a> <b>')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    # Create client and send request
    client = AddTwoIntsClient()
    client.send_request(a, b)

    # Wait for response
    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
                client.get_logger().info(f'Result: {response.sum}')
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Calling the Service

**Terminal 1 (Server)**:
```bash
python3 add_two_ints_server.py
```

**Terminal 2 (Client)**:
```bash
python3 add_two_ints_client.py 10 20
```

**Output**:
```
[INFO] [add_two_ints_client]: Connected to service
[INFO] [add_two_ints_client]: Sent request: 10 + 20
[INFO] [add_two_ints_client]: Result: 30
```

---

## Service Command-Line Tools

### List Available Services

```bash
ros2 service list
```

**Output**:
```
/add_two_ints
/add_two_ints_server/describe_parameters
/add_two_ints_server/get_parameter_types
...
```

### Get Service Type

```bash
ros2 service type /add_two_ints
```

**Output**:
```
example_interfaces/srv/AddTwoInts
```

### Call Service from Command Line

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
```

**Output**:
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=7)

response:
example_interfaces.srv.AddTwoInts_Response(sum=12)
```

---

## ROS 2 Actions

Actions are for **long-running tasks** with three components:

1. **Goal**: What to achieve
2. **Feedback**: Progress updates
3. **Result**: Final outcome

```
┌─────────┐                          ┌─────────┐
│ Client  │ ─────── Goal ─────────> │ Server  │
│         │                          │         │
│         │ <───── Feedback ──────  │         │ (periodic)
│         │ <───── Feedback ──────  │         │
│         │ <───── Feedback ──────  │         │
│         │                          │         │
│         │ <───── Result ─────────  │         │
└─────────┘                          └─────────┘
```

### When to Use Actions

- **Navigation**: Move robot to a goal pose
- **Manipulation**: Pick and place objects
- **Long Computations**: Path planning, SLAM
- **Any task where**:
  - Duration > 1 second
  - User needs progress updates
  - Cancellation might be needed

---

## Creating an Action Server

Let's create a Fibonacci action that computes Fibonacci sequences.

```python title="fibonacci_action_server.py"
#!/usr/bin/env python3
"""
Action Server Example
Computes Fibonacci sequence with feedback
"""

import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    """Action server that computes Fibonacci sequences."""

    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Fibonacci Action Server started')

    def execute_callback(self, goal_handle):
        """Execute the Fibonacci action."""
        self.get_logger().info('Executing goal...')

        # Get the goal order
        order = goal_handle.request.order

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, order):
            # Check if cancellation requested
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Compute next number
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] +
                feedback_msg.partial_sequence[i - 1]
            )

            # Publish feedback
            self.get_logger().info(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)

            # Simulate computation time
            time.sleep(1)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence

        self.get_logger().info(f'Result: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Creating an Action Client

```python title="fibonacci_action_client.py"
#!/usr/bin/env python3
"""
Action Client Example
Sends Fibonacci goals and monitors progress
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    """Action client for Fibonacci computation."""

    def __init__(self):
        super().__init__('fibonacci_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            Fibonacci,
            'fibonacci'
        )

        self.get_logger().info('Fibonacci Action Client started')

    def send_goal(self, order: int):
        """Send a Fibonacci goal."""
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create goal message
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        # Send goal with callbacks
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

        # Get result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback messages."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.partial_sequence}')

    def get_result_callback(self, future):
        """Handle final result."""
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')


def main(args=None):
    rclpy.init(args=args)

    # Create client and send goal
    client = FibonacciActionClient()
    client.send_goal(10)

    # Spin until complete
    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Action

**Terminal 1 (Server)**:
```bash
python3 fibonacci_action_server.py
```

**Terminal 2 (Client)**:
```bash
python3 fibonacci_action_client.py
```

**Output (Client)**:
```
[INFO] [fibonacci_action_client]: Fibonacci Action Client started
[INFO] [fibonacci_action_client]: Waiting for action server...
[INFO] [fibonacci_action_client]: Sending goal: order=10
[INFO] [fibonacci_action_client]: Goal accepted
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2]
[INFO] [fibonacci_action_client]: Feedback: [0, 1, 1, 2, 3]
...
[INFO] [fibonacci_action_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

---

## Action Cancellation

Let's add cancellation capability to the client.

```python title="fibonacci_action_client_cancel.py"
#!/usr/bin/env python3
"""
Action Client with Cancellation
Demonstrates cancelling an action after 3 seconds
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClientCancel(Node):
    """Action client with cancellation after 3 seconds."""

    def __init__(self):
        super().__init__('fibonacci_action_client_cancel')

        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self._goal_handle = None

        # Timer to cancel after 3 seconds
        self._cancel_timer = self.create_timer(3.0, self.cancel_goal)

    def send_goal(self, order: int):
        """Send goal."""
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Store goal handle."""
        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted - will cancel in 3 seconds')

        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Log feedback."""
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.partial_sequence}')

    def cancel_goal(self):
        """Cancel the current goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done)

        self._cancel_timer.cancel()

    def cancel_done(self, future):
        """Handle cancellation response."""
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully cancelled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def get_result_callback(self, future):
        """Handle result."""
        result = future.result().result
        status = future.result().status

        if status == 4:  # STATUS_CANCELED
            self.get_logger().info('Goal was canceled')
        elif status == 3:  # STATUS_SUCCEEDED
            self.get_logger().info(f'Goal succeeded with result: {result.sequence}')


def main(args=None):
    rclpy.init(args=args)

    client = FibonacciActionClientCancel()
    client.send_goal(20)  # Long sequence - will be cancelled

    rclpy.spin(client)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Action Command-Line Tools

### List Actions

```bash
ros2 action list
```

**Output**:
```
/fibonacci
```

### Get Action Info

```bash
ros2 action info /fibonacci
```

**Output**:
```
Action: /fibonacci
Action clients: 1
Action servers: 1
```

### Send Goal from Command Line

```bash
ros2 action send_goal /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

**Output**:
```
Waiting for an action server to become available...
Sending goal:
     order: 5

Goal accepted with ID: ...

Feedback:
    partial_sequence: [0, 1, 1]

Feedback:
    partial_sequence: [0, 1, 1, 2]

...

Result:
    sequence: [0, 1, 1, 2, 3, 5]

Goal finished with status: SUCCEEDED
```

### Send Goal with Feedback

```bash
ros2 action send_goal --feedback /fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 10}"
```

---

## Choosing the Right Pattern

### Decision Flowchart

```
Need communication?
│
├─> One-way, continuous data? ──> TOPIC
│
├─> Quick request-response (under 1s)? ──> SERVICE
│
└─> Long task with progress/cancellation? ──> ACTION
```

### Real-World Examples

| Task | Pattern | Why |
|------|---------|-----|
| **Stream camera images** | Topic | Continuous data flow |
| **Get battery level** | Service | Quick status query |
| **Navigate to goal** | Action | Long task, needs feedback |
| **Send velocity commands** | Topic | High-frequency control |
| **Set robot mode** | Service | One-time configuration |
| **Execute pick-and-place** | Action | Long task, can fail/cancel |

---

## Best Practices

1. **Service Timeouts**
   ```python
   # Wait with timeout
   if not client.wait_for_service(timeout_sec=5.0):
       self.get_logger().error('Service not available!')
       return
   ```

2. **Action Feedback Frequency**
   - Publish feedback at reasonable intervals (0.1-1 Hz)
   - Don't spam with every tiny update

3. **Error Handling**
   ```python
   try:
       response = future.result()
   except Exception as e:
       self.get_logger().error(f'Service call failed: {e}')
   ```

4. **Cancellation Handling**
   - Always check `is_cancel_requested` in action servers
   - Clean up resources on cancellation

5. **Action Result**
   - Always return a result, even on cancellation or failure
   - Include error messages in result for debugging

---

## Self-Assessment

### Questions

1. **When should you use a service instead of a topic?**
   <details>
   <summary>Answer</summary>
   Use a service when you need a request-response pattern for one-time operations or status queries. Topics are better for continuous data streaming.
   </details>

2. **What are the three components of an action?**
   <details>
   <summary>Answer</summary>
   Goal (what to achieve), Feedback (progress updates), and Result (final outcome).
   </details>

3. **Why would you cancel an action instead of just stopping the client?**
   <details>
   <summary>Answer</summary>
   Cancellation allows the server to clean up resources gracefully and stop the task safely. Just stopping the client would leave the server still executing the task.
   </details>

### Exercises

1. **Create a Battery Service**: Write a service server that returns a simulated battery level (0-100%). Create a client that queries it every 5 seconds.

2. **Create a Countdown Action**: Write an action server that counts down from N to 0, publishing feedback every second. Create a client that can cancel the countdown.

3. **Service vs Topic**: Convert the temperature publisher/subscriber from Chapter 2 into a service. What are the advantages and disadvantages?

4. **Action with Failure**: Modify the Fibonacci action to randomly fail 20% of the time. Handle the failure in the client.

---

## Next Steps

Now that you understand all ROS 2 communication patterns, proceed to:

👉 [Chapter 4: URDF and Robot Description](./04-urdf-robots.md) to learn how to describe robot kinematics

---

## Additional Resources

- [ROS 2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services.html)
- [ROS 2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Actions.html)
- [Action Design Best Practices](https://design.ros2.org/articles/actions.html)

---

**Key Takeaway**: Services provide synchronous request-response for quick operations, while Actions enable long-running tasks with feedback and cancellation. Choose the right pattern for your use case to build efficient robotic systems.
