---
sidebar_position: 1
---

# Introduction to ROS 2

Learn the foundations of ROS 2 (Robot Operating System 2), the industry-standard middleware for robot software development.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand what ROS 2 is and why it's essential for robotics
- Explain the key differences between ROS 1 and ROS 2
- Install ROS 2 Humble on Ubuntu 22.04
- Set up a ROS 2 workspace
- Run your first ROS 2 node

---

## What is ROS 2?

**ROS 2 (Robot Operating System 2)** is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.

### Key Features

- **Distributed Architecture**: Run nodes on multiple computers
- **Language Agnostic**: Python, C++, and more
- **Real-Time Capable**: Supports real-time operating systems
- **Security**: Built-in encryption and authentication
- **Cross-Platform**: Linux, Windows, macOS support

### Why ROS 2?

ROS 2 was designed to address limitations in ROS 1:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| **Real-Time** | Limited | Full support |
| **Security** | None | DDS security |
| **Multi-Robot** | Difficult | Native support |
| **Production Use** | Research-focused | Production-ready |
| **Communication** | TCP/UDP | DDS (Data Distribution Service) |

---

## Core Concepts

### Nodes

A **node** is a single process that performs computation. Robots typically have many nodes:
- Camera driver node
- Motor controller node
- Path planning node
- SLAM node

### Topics

**Topics** are named buses over which nodes exchange messages. Topics use a publish-subscribe pattern:
- **Publishers** send messages to a topic
- **Subscribers** receive messages from a topic

### Messages

**Messages** are data structures that define the type of information sent over topics. Examples:
- `sensor_msgs/Image` - Camera images
- `geometry_msgs/Twist` - Velocity commands
- `nav_msgs/Odometry` - Robot position

### Services

**Services** provide request-response communication:
- Client sends request → Server processes → Returns response
- Synchronous communication (waits for response)

### Actions

**Actions** are for long-running tasks with feedback:
- Client sends goal → Server provides feedback → Returns result
- Asynchronous communication with cancellation support

---

## Installation (Ubuntu 22.04)

### Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install software-properties-common -y
```

### Add ROS 2 Repository

```bash
# Add ROS 2 GPG key
sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
  sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Humble

```bash
# Update package list
sudo apt update

# Install ROS 2 Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y
```

### Environment Setup

Add to `~/.bashrc`:

```bash
# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Enable command-line completion
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```

Apply changes:

```bash
source ~/.bashrc
```

### Verify Installation

```bash
# Check ROS 2 version
ros2 --version

# List available commands
ros2 --help

# Expected output: ros2 cli version: 0.18.x
```

---

## ROS 2 Workspace Setup

### Create Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace (empty for now)
colcon build

# Source workspace overlay
source install/setup.bash
```

### Workspace Structure

```
ros2_ws/
├── build/       # Build artifacts
├── install/     # Installed packages
├── log/         # Build logs
└── src/         # Source code (your packages go here)
```

---

## Your First ROS 2 Node

### Using Built-in Demo

Test your installation with a talker-listener demo:

**Terminal 1 (Talker)**:
```bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2 (Listener)**:
```bash
ros2 run demo_nodes_cpp listener
```

You should see the talker publishing messages and the listener receiving them!

### Inspecting with Command-Line Tools

**List active nodes**:
```bash
ros2 node list
# Output: /talker, /listener
```

**List active topics**:
```bash
ros2 topic list
# Output: /chatter, /parameter_events, /rosout
```

**See topic messages**:
```bash
ros2 topic echo /chatter
# Shows live messages from talker
```

**Check topic info**:
```bash
ros2 topic info /chatter
# Shows publishers, subscribers, message type
```

---

## Creating a Simple Python Node

Let's create a minimal ROS 2 node in Python:

```python title="my_first_node.py"
#!/usr/bin/env python3
"""
My First ROS 2 Node
A simple node that prints "Hello, ROS 2!" every second
"""

import rclpy
from rclpy.node import Node


class MyFirstNode(Node):
    """A simple ROS 2 node."""

    def __init__(self):
        super().__init__('my_first_node')

        # Create a timer that calls timer_callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

        self.get_logger().info('My First Node has started!')

    def timer_callback(self):
        """Called every second by the timer."""
        self.counter += 1
        self.get_logger().info(f'Hello, ROS 2! Count: {self.counter}')


def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = MyFirstNode()

    # Spin node (keeps it running)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Running the Node

```bash
# Make executable
chmod +x my_first_node.py

# Run node
python3 my_first_node.py
```

**Expected Output**:
```
[INFO] [my_first_node]: My First Node has started!
[INFO] [my_first_node]: Hello, ROS 2! Count: 1
[INFO] [my_first_node]: Hello, ROS 2! Count: 2
[INFO] [my_first_node]: Hello, ROS 2! Count: 3
...
```

---

## Key ROS 2 Commands

| Command | Purpose |
|---------|---------|
| `ros2 run <package> <node>` | Run a node from a package |
| `ros2 node list` | List active nodes |
| `ros2 topic list` | List active topics |
| `ros2 topic echo <topic>` | Display messages on a topic |
| `ros2 topic info <topic>` | Show topic details |
| `ros2 service list` | List available services |
| `ros2 param list` | List node parameters |
| `ros2 bag record <topic>` | Record topic data |
| `colcon build` | Build workspace |
| `colcon test` | Run tests |

---

## Best Practices

1. **Use Descriptive Node Names**: `camera_driver` not `node1`
2. **Follow Naming Conventions**: `snake_case` for topics/nodes
3. **Log Appropriately**: Use `get_logger()` with proper severity levels
4. **Handle Shutdown Gracefully**: Implement cleanup in `destroy_node()`
5. **Use Type Hints**: Make Python code more maintainable

---

## Self-Assessment

### Questions

1. **What is the primary advantage of ROS 2 over ROS 1?**
   <details>
   <summary>Answer</summary>
   Real-time support, built-in security (DDS), production-ready architecture, and native multi-robot support.
   </details>

2. **What is the difference between a topic and a service?**
   <details>
   <summary>Answer</summary>
   Topics use publish-subscribe (asynchronous, many-to-many), while services use request-response (synchronous, one-to-one).
   </details>

3. **What command would you use to see all messages published on the `/cmd_vel` topic?**
   <details>
   <summary>Answer</summary>
   `ros2 topic echo /cmd_vel`
   </details>

### Exercises

1. **Install ROS 2 Humble** on your Ubuntu 22.04 system following the instructions above
2. **Run the talker-listener demo** and observe the messages
3. **Create and run** the `my_first_node.py` example
4. **Modify the node** to print a custom message of your choice
5. **Use `ros2 node info`** to inspect your running node

---

## Next Steps

Now that you understand ROS 2 basics, proceed to:

👉 [Chapter 2: Nodes and Topics](./02-nodes-topics.md) to learn about publishers and subscribers

---

## Additional Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [ROS 2 Design](https://design.ros2.org/)
- [ROS Answers](https://answers.ros.org/) - Community Q&A

---

**Key Takeaway**: ROS 2 provides a powerful middleware framework that simplifies robot software development through modular, distributed architecture. Mastering ROS 2 is essential for modern robotics engineering.
