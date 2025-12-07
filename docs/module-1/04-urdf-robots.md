---
sidebar_position: 4
---

# URDF and Robot Description

Learn how to describe robot kinematics and geometry using URDF (Unified Robot Description Format), the standard for representing robots in ROS 2.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand URDF structure (links and joints)
- Create basic URDF robot descriptions
- Use XACRO for modular and maintainable robot models
- Visualize robots in RViz2
- Add sensors and actuators to robot descriptions
- Debug URDF models using command-line tools

---

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based format for representing robot kinematics and dynamics.

### Key Components

- **Links**: Rigid bodies (chassis, wheels, arms)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Visual**: How the robot looks (meshes, colors)
- **Collision**: Simplified geometry for collision detection
- **Inertial**: Mass and inertia properties for physics simulation

### Why URDF?

- **Standardized**: Universal format across ROS ecosystem
- **Simulation-Ready**: Works with Gazebo, Isaac Sim
- **Visualization**: Compatible with RViz2
- **Planning**: Used by motion planning algorithms (MoveIt)

---

## Basic URDF Structure

Let's start with a simple robot: a box on wheels.

```xml title="simple_robot.urdf"
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base Link (Robot Body) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Left Wheel Link -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting base to left wheel -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.2 0.25 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right Wheel (similar structure) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0"
               iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0.2 -0.25 0" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

</robot>
```

### Understanding the Structure

**Links** define rigid bodies:
- `<visual>`: Appearance (meshes, colors)
- `<collision>`: Simplified shapes for collision detection
- `<inertial>`: Physical properties (mass, inertia)

**Joints** connect links:
- `type`: revolute, continuous, prismatic, fixed
- `<parent>` and `<child>`: Which links are connected
- `<origin>`: Position and orientation (xyz, rpy)
- `<axis>`: Rotation/translation axis

---

## Joint Types

| Type | Movement | Use Case |
|------|----------|----------|
| **fixed** | None | Sensors, static parts |
| **revolute** | Rotation (limited) | Robot arms, grippers |
| **continuous** | Rotation (unlimited) | Wheels, turrets |
| **prismatic** | Linear | Elevators, linear actuators |
| **floating** | 6-DOF | Flying robots (rare in URDF) |
| **planar** | 2D plane | Mobile bases (rare) |

### Examples

```xml
<!-- Fixed joint (sensor mount) -->
<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.3 0 0.2" rpy="0 0 0"/>
</joint>

<!-- Revolute joint (robot arm) -->
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
</joint>

<!-- Prismatic joint (linear actuator) -->
<joint name="elevator_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="platform"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="0" upper="1.0" effort="100" velocity="0.5"/>
</joint>
```

---

## Visualizing in RViz2

### Launch File

Create a launch file to visualize your URDF:

```python title="display_robot.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get URDF file path
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'simple_robot.urdf'
    )

    # Read URDF file
    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Joint State Publisher GUI (for testing)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('my_robot_description'),
                'rviz',
                'display.rviz'
            )]
        )
    ])
```

### Running RViz2

```bash
# Launch visualization
ros2 launch my_robot_description display_robot.launch.py

# Or manually
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat simple_robot.urdf)"

ros2 run rviz2 rviz2
```

In RViz2:
1. Set **Fixed Frame** to `base_link`
2. Add **RobotModel** display
3. Adjust joint positions using Joint State Publisher GUI

---

## XACRO: Modular Robot Descriptions

**XACRO (XML Macros)** makes URDF more maintainable with:
- Constants and variables
- Mathematical expressions
- Macros for reusable components
- Conditional statements

### Converting URDF to XACRO

```xml title="simple_robot.urdf.xacro"
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="simple_robot">

  <!-- Properties (Constants) -->
  <xacro:property name="base_width" value="0.4"/>
  <xacro:property name="base_length" value="0.6"/>
  <xacro:property name="base_height" value="0.2"/>
  <xacro:property name="wheel_radius" value="0.1"/>
  <xacro:property name="wheel_width" value="0.05"/>
  <xacro:property name="wheel_offset_y" value="0.25"/>

  <!-- Macro for wheel -->
  <xacro:macro name="wheel" params="prefix reflect">
    <link name="${prefix}_wheel">
      <visual>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black">
          <color rgba="0 0 0 1"/>
        </material>
      </visual>

      <collision>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>

      <inertial>
        <mass value="1.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0"
                 iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="0.2 ${reflect * wheel_offset_y} 0" rpy="-1.5708 0 0"/>
      <axis xyz="0 0 1"/>
    </joint>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>

    <collision>
      <geometry>
        <box size="${base_length} ${base_width} ${base_height}"/>
      </geometry>
    </collision>

    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Instantiate wheels using macro -->
  <xacro:wheel prefix="left" reflect="1"/>
  <xacro:wheel prefix="right" reflect="-1"/>

</robot>
```

### Processing XACRO Files

```bash
# Convert XACRO to URDF
ros2 run xacro xacro simple_robot.urdf.xacro > simple_robot.urdf

# Check URDF validity
check_urdf simple_robot.urdf
```

**Output**:
```
robot name is: simple_robot
---------- Successfully Parsed XML ---------------
root Link: base_link has 2 child(ren)
    child(1):  left_wheel
    child(2):  right_wheel
```

---

## Adding Sensors

### LiDAR Sensor

```xml
<!-- LiDAR Link -->
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
    <material name="red">
      <color rgba="0.8 0 0 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.07"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.2"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0"
             iyy="0.001" iyz="0.0" izz="0.001"/>
  </inertial>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.3 0 0.15" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for LiDAR simulation -->
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.12</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="scan" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Camera Sensor

```xml
<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.08 0.03"/>
    </geometry>
    <material name="green">
      <color rgba="0 0.8 0 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.02 0.08 0.03"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.0001" ixy="0.0" ixz="0.0"
             iyy="0.0001" iyz="0.0" izz="0.0001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="base_link"/>
  <child link="camera_link"/>
  <origin xyz="0.31 0 0.1" rpy="0 0 0"/>
</joint>
```

---

## URDF Best Practices

1. **Naming Conventions**
   - Use descriptive link names: `base_link`, `left_wheel`, `camera_link`
   - Suffix joints with `_joint`: `shoulder_joint`, `wheel_joint`

2. **Origin Frame**
   - Always define a `base_link` as the root
   - Use consistent axis orientations (X forward, Y left, Z up)

3. **Visual vs Collision**
   - Visual: High-detail meshes for rendering
   - Collision: Simple shapes (box, cylinder) for performance

4. **XACRO Organization**
   ```
   robot_description/
   ├── urdf/
   │   ├── robot.urdf.xacro          # Main file
   │   ├── materials.xacro           # Color definitions
   │   ├── inertias.xacro            # Inertia macros
   │   └── sensors/
   │       ├── lidar.xacro
   │       └── camera.xacro
   ```

5. **Inertia Calculation**
   - Use physics-accurate values for simulation
   - Common shapes: [Inertia formulas](https://en.wikipedia.org/wiki/List_of_moments_of_inertia)

---

## Debugging URDF

### Check URDF Validity

```bash
check_urdf robot.urdf
```

### View URDF Tree

```bash
urdf_to_graphiz robot.urdf
# Generates robot.pdf showing link hierarchy
```

### Common Errors

**Error**: `No root link found`
- **Fix**: Ensure one link has no parent joint

**Error**: `joint axis is zero`
- **Fix**: Define `<axis xyz="..."/>` for revolute/continuous joints

**Error**: `Failed to parse robot model`
- **Fix**: Check XML syntax, close all tags

---

## Publishing Robot State

```python title="robot_state_publisher_node.py"
#!/usr/bin/env python3
"""
Publish robot URDF to /robot_description topic
"""

import rclpy
from rclpy.node import Node


class RobotStatePublisher(Node):
    """Publishes robot URDF description."""

    def __init__(self):
        super().__init__('robot_state_publisher')

        # Read URDF file
        with open('/path/to/robot.urdf', 'r') as f:
            robot_description = f.read()

        # Declare parameter
        self.declare_parameter('robot_description', robot_description)

        self.get_logger().info('Robot description published')


def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Self-Assessment

### Questions

1. **What's the difference between a `visual` and `collision` tag in URDF?**
   <details>
   <summary>Answer</summary>
   Visual defines how the robot appears (meshes, textures), while collision uses simplified geometry for physics simulation and collision detection.
   </details>

2. **When would you use a `fixed` joint vs a `revolute` joint?**
   <details>
   <summary>Answer</summary>
   Fixed joints are for static connections (sensors, camera mounts). Revolute joints allow rotation with limits (robot arms, doors).
   </details>

3. **What's the main advantage of XACRO over plain URDF?**
   <details>
   <summary>Answer</summary>
   XACRO allows macros, variables, and expressions, making robot descriptions more modular, reusable, and maintainable.
   </details>

### Exercises

1. **Create a Mobile Robot**: Design a 4-wheeled robot with a LiDAR sensor on top using URDF

2. **Use XACRO Macros**: Convert your robot to XACRO and create a macro for wheels to avoid duplication

3. **Visualize in RViz2**: Launch your robot in RViz2 and verify all links and joints are correct

4. **Add a Robotic Arm**: Extend your robot with a 2-DOF arm using revolute joints

5. **Calculate Inertia**: Compute accurate inertia values for a cylindrical link (mass=2kg, radius=0.1m, height=0.3m)

---

## Next Steps

Congratulations on completing Module 1! You now understand ROS 2 fundamentals. Proceed to:

👉 [Module 2: Gazebo & Unity](../module-2/01-gazebo-intro.md) to learn physics simulation

---

## Additional Resources

- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [XACRO Documentation](http://wiki.ros.org/xacro)
- [RViz2 User Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/RViz-User-Guide.html)
- [Inertia Calculator](https://github.com/ros/robot_state_publisher)

---

**Key Takeaway**: URDF is the universal language for describing robots in ROS 2. Mastering URDF and XACRO enables you to model any robot for simulation, visualization, and control.
