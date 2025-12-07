# ROS 2 Code Generator Skill

## Description
Generates production-ready ROS 2 Python code with best practices, type hints, and proper error handling.

## Usage
Invoke this skill when you need to create ROS 2 nodes, packages, or components.

## Input
Provide a description of the desired ROS 2 functionality:
- Node type: Publisher / Subscriber / Service / Action / Lifecycle
- Topic/Service name and message type
- Functionality description
- Additional requirements (logging, parameters, etc.)

## Output
Complete Python code package including:

### 1. Python Node File
```python
#!/usr/bin/env python3
"""
[Module docstring with description]
"""

import rclpy
from rclpy.node import Node
from [msg_package].msg import [MessageType]
from typing import Optional

class [NodeName](Node):
    """[Class docstring]"""

    def __init__(self) -> None:
        super().__init__('[node_name]')

        # Parameters
        self.declare_parameter('param_name', 'default_value')

        # Publishers/Subscribers
        self.[publisher/subscription] = self.create_[publisher/subscription](
            [MessageType],
            '[topic_name]',
            10
        )

        # Timers
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('[Node] initialized')

    def timer_callback(self) -> None:
        """[Callback description]"""
        # Implementation
        pass

def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = [NodeName]()
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

### 2. Package Setup Files

**setup.py**:
```python
from setuptools import find_packages, setup

package_name = '[package_name]'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='[Your Name]',
    maintainer_email='[email]',
    description='[Description]',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            '[node_name] = [package_name].[node_name]:main',
        ],
    },
)
```

**package.xml**:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>[package_name]</name>
  <version>0.1.0</version>
  <description>[Description]</description>
  <maintainer email="[email]">[Your Name]</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>[msg_package]</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### 3. Launch File (if applicable)
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='[package_name]',
            executable='[node_name]',
            name='[node_name]',
            output='screen',
            parameters=[{
                'param_name': 'value',
            }]
        ),
    ])
```

### 4. Installation & Usage Instructions
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select [package_name]
source install/setup.bash

# Run the node
ros2 run [package_name] [node_name]

# Or use launch file
ros2 launch [package_name] [launch_file].launch.py
```

## Best Practices Applied
- ✅ Type hints for all functions
- ✅ Docstrings for classes and methods
- ✅ Proper error handling (try/except/finally)
- ✅ Logging with appropriate levels
- ✅ Parameter declarations
- ✅ Resource cleanup in destructor
- ✅ PEP 8 compliant formatting

## Example Invocation
```
"Generate a ROS 2 publisher node that publishes Twist messages to /cmd_vel topic at 10Hz,
with parameters for linear and angular velocity limits."
```

Output: Complete package with publisher node, setup files, launch file, and documentation.
