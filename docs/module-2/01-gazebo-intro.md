---
sidebar_position: 1
---

# Introduction to Gazebo Simulation

Learn to simulate robots in realistic physics environments using Gazebo, the industry-standard robotics simulator integrated with ROS 2.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand what Gazebo is and why simulation is essential
- Install Gazebo Classic and Gazebo Garden
- Spawn robots in Gazebo worlds
- Understand physics engines and simulation parameters
- Use Gazebo plugins for sensors and actuators
- Bridge Gazebo with ROS 2 nodes

---

## What is Gazebo?

**Gazebo** is a 3D robotics simulator that provides:

- **Physics Simulation**: Realistic gravity, friction, collisions
- **Sensor Simulation**: Cameras, LiDAR, IMU, GPS
- **Rendering**: Photorealistic visuals with lighting and shadows
- **ROS Integration**: Seamless communication with ROS 2 nodes
- **Plugin System**: Extensible sensor and actuator models

### Why Simulate?

| Benefit | Description |
|---------|-------------|
| **Safety** | Test dangerous scenarios without risk |
| **Cost** | No hardware damage from mistakes |
| **Speed** | Iterate faster than real-world testing |
| **Reproducibility** | Exact scenario replication |
| **Scaling** | Test multiple robots simultaneously |
| **Edge Cases** | Simulate rare events (sensor failures, collisions) |

---

## Gazebo Versions

### Gazebo Classic (v11)

- **Status**: Stable, widely used
- **ROS 2 Support**: Via `ros_gz_bridge`
- **Physics**: ODE (Open Dynamics Engine)
- **Best For**: ROS 2 Humble users, production systems

### Gazebo Garden/Harmonic (Ignition)

- **Status**: Next-generation (rebranded from Ignition Gazebo)
- **ROS 2 Support**: Native integration
- **Physics**: DART, Bullet, ODE
- **Best For**: Cutting-edge features, photorealistic rendering

**For this course, we'll use Gazebo Classic** due to its stability and compatibility with ROS 2 Humble.

---

## Installation (Ubuntu 22.04)

### Install Gazebo Classic

```bash
# Install Gazebo 11 (Classic)
sudo apt update
sudo apt install gazebo11 libgazebo11-dev -y

# Install ROS 2 Gazebo packages
sudo apt install ros-humble-gazebo-ros-pkgs -y
sudo apt install ros-humble-gazebo-ros2-control -y

# Verify installation
gazebo --version
# Expected: Gazebo multi-robot simulator, version 11.x.x
```

### Install Gazebo Garden (Optional)

```bash
# Add Gazebo Garden repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
  http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | \
  sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Garden
sudo apt update
sudo apt install gz-garden -y

# Verify
gz sim --version
```

---

## Launching Gazebo

### Empty World

```bash
# Launch Gazebo GUI
gazebo

# Launch with specific world
gazebo worlds/empty.world

# Launch without GUI (headless)
gzserver worlds/empty.world
```

### Gazebo Interface

**Main Components**:

1. **World View**: 3D visualization of environment
2. **Insert Tab**: Add models (robots, objects, lights)
3. **Model Editor**: Design custom models
4. **Building Editor**: Create indoor environments
5. **Time Controls**: Play, pause, reset simulation

**Camera Controls**:
- **Rotate**: Left-click + drag
- **Pan**: Middle-click + drag
- **Zoom**: Scroll wheel
- **Orbit**: Shift + left-click + drag

---

## Spawning a Robot

### Using URDF with Gazebo

Update your URDF to include Gazebo-specific tags:

```xml title="robot_gazebo.urdf"
<?xml version="1.0"?>
<robot name="mobile_robot">

  <!-- Base Link -->
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
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.183" ixy="0.0" ixz="0.0"
               iyy="0.333" iyz="0.0" izz="0.5"/>
    </inertial>
  </link>

  <!-- Gazebo-specific properties -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.2</mu1>  <!-- Friction coefficient 1 -->
    <mu2>0.2</mu2>  <!-- Friction coefficient 2 -->
  </gazebo>

  <!-- Differential drive plugin -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <ros>
        <namespace>/mobile_robot</namespace>
        <remapping>cmd_vel:=cmd_vel</remapping>
        <remapping>odom:=odom</remapping>
      </ros>

      <!-- Wheels -->
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>

      <!-- Kinematics -->
      <wheel_separation>0.5</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>

      <!-- Limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <!-- Output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>

      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>

</robot>
```

### Spawn Script

```python title="spawn_robot.launch.py"
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to URDF
    urdf_file = os.path.join(
        get_package_share_directory('my_robot_description'),
        'urdf',
        'robot_gazebo.urdf'
    )

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([
        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'mobile_robot',
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        )
    ])
```

### Launch Simulation

```bash
ros2 launch my_robot_description spawn_robot.launch.py
```

---

## Physics Engines

Gazebo supports multiple physics engines:

| Engine | Strengths | Use Case |
|--------|-----------|----------|
| **ODE** (default) | Fast, stable | General robotics |
| **Bullet** | Rigid body dynamics | Collisions, impacts |
| **DART** | Multi-body dynamics | Humanoids, complex kinematics |
| **Simbody** | Biomechanics | Human-robot interaction |

### Configuring Physics

```xml title="custom.world"
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>       <!-- 1ms time step -->
      <real_time_factor>1.0</real_time_factor>   <!-- Real-time speed -->
      <real_time_update_rate>1000</real_time_update_rate>

      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
          <sor>1.3</sor>
        </solver>
        <constraints>
          <cfm>0.0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun (lighting) -->
    <include>
      <uri>model://sun</uri>
    </include>

  </world>
</sdf>
```

---

## Gazebo Plugins

Plugins add functionality to models:

### Common Plugins

| Plugin | Purpose | ROS 2 Topic |
|--------|---------|-------------|
| `libgazebo_ros_diff_drive.so` | Differential drive | `/cmd_vel`, `/odom` |
| `libgazebo_ros_ray_sensor.so` | LiDAR/Laser scan | `/scan` |
| `libgazebo_ros_camera.so` | Camera | `/camera/image_raw` |
| `libgazebo_ros_imu_sensor.so` | IMU | `/imu` |
| `libgazebo_ros_gps_sensor.so` | GPS | `/gps/fix` |

### LiDAR Plugin Example

```xml
<gazebo reference="lidar_link">
  <sensor name="lidar" type="ray">
    <always_on>true</always_on>
    <update_rate>10</update_rate>
    <visualize>true</visualize>

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
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
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

### Testing Sensor Data

```bash
# Launch robot with LiDAR
ros2 launch my_robot_description spawn_robot.launch.py

# Echo LiDAR data
ros2 topic echo /scan

# Visualize in RViz2
ros2 run rviz2 rviz2
```

---

## Controlling the Robot

### Send Velocity Commands

```bash
# Publish velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.2}}"
```

### Teleop Keyboard

```bash
# Install teleop package
sudo apt install ros-humble-teleop-twist-keyboard -y

# Run teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls**:
- `i`: Forward
- `k`: Stop
- `j`: Turn left
- `l`: Turn right
- `u`, `o`, `m`, `.`: Diagonal movements

---

## Simulation Best Practices

1. **Time Step Optimization**
   - Smaller steps (0.001s): More accurate, slower
   - Larger steps (0.01s): Faster, less accurate
   - Balance based on your needs

2. **Collision Geometry**
   - Use simple shapes (boxes, cylinders) for collision
   - Detailed meshes only for visual

3. **Inertia Values**
   - Accurate inertia prevents unrealistic behavior
   - Use online calculators or CAD software

4. **Real-Time Factor**
   - RTF = 1.0: Real-time
   - RTF < 1.0: Simulation slower than real-time
   - RTF > 1.0: Simulation faster (requires powerful hardware)

5. **Sensor Noise**
   - Always add Gaussian noise to sensors
   - Matches real-world sensor behavior

---

## Debugging Gazebo

### Check Simulation Performance

```bash
# Monitor real-time factor
gz stats

# Expected output:
# Factor[1.00] SimTime[10.50] RealTime[10.52] Paused[F]
```

### Common Issues

**Issue**: Robot falls through ground
- **Fix**: Ensure collision geometry is defined and inertia is correct

**Issue**: Robot doesn't move
- **Fix**: Check plugin configuration, joint types, and friction coefficients

**Issue**: Simulation is very slow
- **Fix**: Simplify collision meshes, increase time step, reduce sensor update rates

---

## Self-Assessment

### Questions

1. **What's the difference between Gazebo Classic and Gazebo Garden?**
   <details>
   <summary>Answer</summary>
   Gazebo Classic (v11) is stable and widely used with ROS 2 Humble. Gazebo Garden is the next-generation version with improved rendering and native ROS 2 integration.
   </details>

2. **Why should you add noise to simulated sensors?**
   <details>
   <summary>Answer</summary>
   Real-world sensors always have noise. Adding noise in simulation makes algorithms more robust when deployed to real robots.
   </details>

3. **What does Real-Time Factor (RTF) mean?**
   <details>
   <summary>Answer</summary>
   RTF compares simulation speed to real time. RTF=1.0 means simulation runs at real-time speed. RTF=0.5 means simulation is half the speed of real time.
   </details>

### Exercises

1. **Spawn a Robot**: Create a launch file that spawns a simple mobile robot in an empty Gazebo world

2. **Add a LiDAR**: Add a LiDAR sensor to your robot and visualize the scan data in RViz2

3. **Teleop Control**: Use teleop_twist_keyboard to drive your robot around the world

4. **Custom World**: Create a custom Gazebo world with obstacles and test obstacle avoidance

5. **Monitor Performance**: Use `gz stats` to measure your simulation's real-time factor

---

## Next Steps

Now that you understand Gazebo basics, proceed to:

👉 [Chapter 2: Building Simulation Worlds](./02-worlds-environments.md) to create complex environments

---

## Additional Resources

- [Gazebo Tutorials](https://classic.gazebosim.org/tutorials)
- [Gazebo Plugins Reference](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)
- [SDF Format Specification](http://sdformat.org/)
- [ROS 2 + Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo.html)

---

**Key Takeaway**: Gazebo provides a safe, fast, and cost-effective way to develop and test robotic systems before deploying to real hardware. Mastering simulation is essential for modern robotics development.
