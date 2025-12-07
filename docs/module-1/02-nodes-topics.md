---
sidebar_position: 2
---

# Nodes and Topics

Master the fundamental communication pattern in ROS 2: the publisher-subscriber model using nodes and topics.

## Learning Objectives

By the end of this chapter, you will be able to:

- Create ROS 2 publisher nodes in Python
- Create ROS 2 subscriber nodes in Python
- Understand message types and how to use them
- Configure Quality of Service (QoS) settings
- Debug topic communication using command-line tools
- Build a multi-node robotic system

---

## The Publisher-Subscriber Pattern

ROS 2 uses a **publish-subscribe** pattern for asynchronous, many-to-many communication:

```
┌─────────────┐         Topic: /sensor_data         ┌─────────────┐
│  Publisher  │ ────────────────────────────────> │ Subscriber  │
│   (Camera)  │                                     │  (Display)  │
└─────────────┘                                     └─────────────┘
                                                           ▲
                                                           │
                                                    ┌──────┴──────┐
                                                    │ Subscriber  │
                                                    │   (Logger)  │
                                                    └─────────────┘
```

### Key Characteristics

- **Decoupled**: Publishers don't know who's listening
- **Asynchronous**: No blocking - fire and forget
- **Many-to-Many**: Multiple publishers and subscribers per topic
- **Typed**: Topics have a specific message type

---

## Creating a Publisher Node

Let's create a node that publishes a counter value every second.

```python title="publisher_node.py"
#!/usr/bin/env python3
"""
Publisher Node Example
Publishes integer counter values to /counter topic
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class PublisherNode(Node):
    """Publishes counter values at 1 Hz."""

    def __init__(self):
        super().__init__('publisher_node')

        # Create publisher
        self.publisher_ = self.create_publisher(
            Int32,           # Message type
            'counter',       # Topic name
            10               # Queue size
        )

        # Create timer (1 Hz = 1.0 seconds)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info('Publisher node started')

    def timer_callback(self):
        """Publish counter value."""
        msg = Int32()
        msg.data = self.counter

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {self.counter}')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Publisher

```bash
# Make executable
chmod +x publisher_node.py

# Run node
python3 publisher_node.py
```

**Expected Output**:
```
[INFO] [publisher_node]: Publisher node started
[INFO] [publisher_node]: Published: 0
[INFO] [publisher_node]: Published: 1
[INFO] [publisher_node]: Published: 2
...
```

---

## Creating a Subscriber Node

Now let's create a node that receives and processes these counter values.

```python title="subscriber_node.py"
#!/usr/bin/env python3
"""
Subscriber Node Example
Subscribes to /counter topic and displays received values
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SubscriberNode(Node):
    """Subscribes to counter values."""

    def __init__(self):
        super().__init__('subscriber_node')

        # Create subscription
        self.subscription = self.create_subscription(
            Int32,                    # Message type
            'counter',                # Topic name
            self.listener_callback,   # Callback function
            10                        # Queue size
        )

        self.get_logger().info('Subscriber node started')

    def listener_callback(self, msg: Int32):
        """Called when a message is received."""
        self.get_logger().info(f'Received: {msg.data}')

        # Process the data
        if msg.data % 10 == 0:
            self.get_logger().warn(f'Milestone reached: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running Publisher and Subscriber Together

**Terminal 1 (Publisher)**:
```bash
python3 publisher_node.py
```

**Terminal 2 (Subscriber)**:
```bash
python3 subscriber_node.py
```

You should see the subscriber receiving all published messages!

---

## Common ROS 2 Message Types

ROS 2 provides standard message packages for common data:

### std_msgs - Basic Types

```python
from std_msgs.msg import String, Int32, Float64, Bool

# String message
msg = String()
msg.data = "Hello, ROS 2!"

# Integer message
msg = Int32()
msg.data = 42

# Float message
msg = Float64()
msg.data = 3.14159

# Boolean message
msg = Bool()
msg.data = True
```

### geometry_msgs - Spatial Data

```python
from geometry_msgs.msg import Twist, Pose, Point

# Velocity command (for mobile robots)
twist = Twist()
twist.linear.x = 0.5    # Forward velocity (m/s)
twist.angular.z = 0.2   # Rotation velocity (rad/s)

# Position
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 0.0
```

### sensor_msgs - Sensor Data

```python
from sensor_msgs.msg import Image, LaserScan, Imu

# Camera image
image = Image()
image.width = 640
image.height = 480
image.encoding = "rgb8"

# Laser scan
scan = LaserScan()
scan.angle_min = -1.57  # -90 degrees
scan.angle_max = 1.57   # +90 degrees
scan.ranges = [...]     # Distance measurements
```

---

## Practical Example: Robot Velocity Controller

Let's build a practical system with a command publisher and velocity monitor.

```python title="velocity_commander.py"
#!/usr/bin/env python3
"""
Velocity Commander
Publishes velocity commands for a mobile robot
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math


class VelocityCommander(Node):
    """Sends velocity commands in a circular pattern."""

    def __init__(self):
        super().__init__('velocity_commander')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)  # 10 Hz
        self.time = 0.0

        self.get_logger().info('Velocity Commander started')

    def publish_velocity(self):
        """Publish velocity commands."""
        msg = Twist()

        # Create circular motion
        msg.linear.x = 0.5                        # Forward speed
        msg.angular.z = 0.3 * math.sin(self.time) # Sinusoidal rotation

        self.publisher_.publish(msg)
        self.time += 0.1

        self.get_logger().debug(
            f'Cmd: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

```python title="velocity_monitor.py"
#!/usr/bin/env python3
"""
Velocity Monitor
Subscribes to velocity commands and monitors for safety
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class VelocityMonitor(Node):
    """Monitors velocity commands for safety limits."""

    # Safety thresholds
    MAX_LINEAR_SPEED = 1.0   # m/s
    MAX_ANGULAR_SPEED = 1.0  # rad/s

    def __init__(self):
        super().__init__('velocity_monitor')

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10
        )

        self.get_logger().info('Velocity Monitor started')

    def velocity_callback(self, msg: Twist):
        """Check velocity for safety violations."""
        linear = abs(msg.linear.x)
        angular = abs(msg.angular.z)

        # Check limits
        if linear > self.MAX_LINEAR_SPEED:
            self.get_logger().error(
                f'LINEAR SPEED VIOLATION: {linear:.2f} > {self.MAX_LINEAR_SPEED}'
            )

        if angular > self.MAX_ANGULAR_SPEED:
            self.get_logger().error(
                f'ANGULAR SPEED VIOLATION: {angular:.2f} > {self.MAX_ANGULAR_SPEED}'
            )

        # Log normal operation
        if linear <= self.MAX_LINEAR_SPEED and angular <= self.MAX_ANGULAR_SPEED:
            self.get_logger().info(
                f'✓ Safe velocity: linear={linear:.2f}, angular={angular:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VelocityMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Quality of Service (QoS)

QoS settings control the reliability and performance of communication.

### QoS Profiles

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Reliable communication (like TCP)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Best-effort communication (like UDP, faster)
best_effort_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Create publisher with custom QoS
self.publisher_ = self.create_publisher(
    Twist,
    'cmd_vel',
    qos_profile=reliable_qos
)
```

### When to Use Which QoS

| Use Case | Reliability | Why |
|----------|-------------|-----|
| **Sensor Data** | Best Effort | High frequency, old data not useful |
| **Commands** | Reliable | Must not lose critical commands |
| **Telemetry** | Best Effort | Real-time is more important than history |
| **Configuration** | Reliable | Cannot miss configuration updates |

---

## Debugging with ROS 2 Tools

### List Active Topics

```bash
ros2 topic list
```

**Output**:
```
/cmd_vel
/counter
/parameter_events
/rosout
```

### See Topic Info

```bash
ros2 topic info /cmd_vel
```

**Output**:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

### Echo Topic Messages

```bash
ros2 topic echo /cmd_vel
```

**Output**:
```
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.3
---
```

### Check Publishing Rate

```bash
ros2 topic hz /cmd_vel
```

**Output**:
```
average rate: 10.003
  min: 0.099s max: 0.101s std dev: 0.0005s window: 10
```

### Publish from Command Line

```bash
# Publish a single message
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Publish at 1 Hz
ros2 topic pub --rate 1 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

---

## Topic Remapping

Change topic names at runtime without modifying code:

```bash
# Remap /cmd_vel to /robot1/cmd_vel
ros2 run my_package velocity_commander --ros-args \
  --remap cmd_vel:=robot1/cmd_vel

# Remap multiple topics
ros2 run my_package my_node --ros-args \
  --remap old_topic:=new_topic \
  --remap /cmd_vel:=/robot/cmd_vel
```

---

## Best Practices

1. **Naming Conventions**
   - Use `snake_case` for topic names: `/robot/cmd_vel`
   - Use namespaces for multi-robot: `/robot1/cmd_vel`, `/robot2/cmd_vel`

2. **Message Selection**
   - Use standard messages when possible (geometry_msgs, sensor_msgs)
   - Create custom messages only when necessary

3. **Queue Sizes**
   - Start with 10 for most applications
   - Increase for high-frequency data (camera: 100+)
   - Decrease for low-frequency (configuration: 1-5)

4. **Logging Levels**
   - `DEBUG`: Detailed information (verbose)
   - `INFO`: General information (default)
   - `WARN`: Warning messages
   - `ERROR`: Error messages
   - `FATAL`: Critical errors

5. **Performance**
   - Avoid heavy computation in callbacks
   - Use appropriate QoS for your use case
   - Consider message size for bandwidth

---

## Self-Assessment

### Questions

1. **What's the difference between a publisher and a subscriber?**
   <details>
   <summary>Answer</summary>
   A publisher sends messages to a topic, while a subscriber receives messages from a topic. Publishers don't know who's listening, and subscribers don't know who's sending.
   </details>

2. **Why would you use BEST_EFFORT QoS instead of RELIABLE?**
   <details>
   <summary>Answer</summary>
   BEST_EFFORT is faster and better for high-frequency sensor data where the latest value is more important than receiving every single message. RELIABLE ensures delivery but adds latency.
   </details>

3. **What happens if a subscriber starts after messages have already been published?**
   <details>
   <summary>Answer</summary>
   By default, the subscriber will only receive new messages published after it starts. However, you can configure QoS with TRANSIENT_LOCAL durability to receive previously published messages.
   </details>

### Exercises

1. **Create a Temperature Publisher**: Write a node that publishes random temperature values (15-30°C) to `/temperature` every 2 seconds using `Float64` messages

2. **Create a Temperature Monitor**: Write a subscriber that listens to `/temperature` and logs a warning if temperature exceeds 25°C

3. **Multi-Topic Publisher**: Modify the publisher to publish to both `/temperature/celsius` and `/temperature/fahrenheit`

4. **Rate Analysis**: Use `ros2 topic hz` to measure the publishing rate of your temperature node

5. **Topic Remapping**: Run your temperature publisher with the topic remapped to `/sensor/temp`

---

## Next Steps

Now that you understand publishers and subscribers, proceed to:

👉 [Chapter 3: Services and Actions](./03-services-actions.md) to learn about request-response communication

---

## Additional Resources

- [ROS 2 Topics Tutorial](https://docs.ros.org/en/humble/Tutorials/Topics.html)
- [QoS Settings Guide](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)
- [Standard Messages](https://github.com/ros2/common_interfaces)

---

**Key Takeaway**: Topics enable decoupled, asynchronous communication between nodes. Mastering publishers and subscribers is essential for building modular robotic systems.
