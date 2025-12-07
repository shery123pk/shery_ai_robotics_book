---
sidebar_position: 2
---

# Building Simulation Worlds

Learn to create realistic simulation environments, add objects, configure lighting, and design test scenarios for robot development.

## Learning Objectives

By the end of this chapter, you will be able to:

- Create custom Gazebo worlds using SDF format
- Add models from the Gazebo model database
- Design indoor and outdoor environments
- Configure lighting, shadows, and physics
- Create randomized test environments
- Build warehouse and manufacturing scenarios

---

## World File Structure (SDF)

Gazebo worlds use **SDF (Simulation Description Format)**, an XML format for defining environments.

### Basic World Template

```xml title="basic_world.world"
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="my_world">

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <!-- Scene settings -->
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Sun (primary light source) -->
    <include>
      <uri>model://sun</uri>
    </include>

  </world>
</sdf>
```

---

## Adding Models

### From Gazebo Model Database

Gazebo includes hundreds of pre-built models.

```xml
<!-- Add a table -->
<include>
  <uri>model://cafe_table</uri>
  <pose>2 0 0 0 0 0</pose>  <!-- x y z roll pitch yaw -->
</include>

<!-- Add a chair -->
<include>
  <uri>model://chair</uri>
  <pose>2 0.5 0 0 0 1.57</pose>
</include>

<!-- Add obstacles -->
<include>
  <uri>model://construction_cone</uri>
  <pose>3 3 0 0 0 0</pose>
</include>
```

### Custom Model (Box Obstacle)

```xml
<model name="box_obstacle">
  <static>true</static>  <!-- Doesn't move -->
  <pose>1 1 0.5 0 0 0</pose>

  <link name="link">
    <collision name="collision">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
    </collision>

    <visual name="visual">
      <geometry>
        <box>
          <size>1 1 1</size>
        </box>
      </geometry>
      <material>
        <ambient>0.8 0.2 0.2 1</ambient>
        <diffuse>0.8 0.2 0.2 1</diffuse>
        <specular>0.1 0.1 0.1 1</specular>
      </material>
    </visual>
  </link>
</model>
```

---

## Indoor Environment: Office

Let's create an office environment with walls, furniture, and obstacles.

```xml title="office.world"
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="office">

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>0.5 0.5 0.5 1</ambient>
      <background>0.9 0.9 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Ground -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Lighting -->
    <light name="ceiling_light_1" type="point">
      <pose>2 2 3 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>20</range>
        <linear>0.01</linear>
        <constant>0.5</constant>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <!-- Walls -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_south">
      <static>true</static>
      <pose>0 -5 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 0.2 2</size></box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_east">
      <static>true</static>
      <pose>5 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <model name="wall_west">
      <static>true</static>
      <pose>-5 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.2 10 2</size></box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Furniture -->
    <include>
      <uri>model://cafe_table</uri>
      <pose>0 0 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://chair</uri>
      <pose>0 1 0 0 0 3.14</pose>
    </include>

    <!-- Office supplies -->
    <include>
      <uri>model://bookshelf</uri>
      <pose>-4 -4 0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

---

## Outdoor Environment: Warehouse

A warehouse with loading docks, pallets, and navigation challenges.

```xml title="warehouse.world"
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="warehouse">

    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.5 0.5 0.5 1</background>
      <shadows>true</shadows>
    </scene>

    <!-- Large ground plane -->
    <model name="ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Sun -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Warehouse shelves (using boxes) -->
    <model name="shelf_1">
      <static>true</static>
      <pose>-5 0 1 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>0.5 3 2</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>0.5 3 2</size></box>
          </geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
          </material>
        </visual>
      </link>
    </model>

    <!-- Pallets with boxes -->
    <include>
      <uri>model://cardboard_box</uri>
      <pose>2 3 0 0 0 0</pose>
    </include>

    <include>
      <uri>model://cardboard_box</uri>
      <pose>2 3 0.3 0 0 0.5</pose>
    </include>

    <!-- Loading dock -->
    <model name="loading_dock">
      <static>true</static>
      <pose>10 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>5 8 1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>5 8 1</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

---

## Lighting Configuration

### Types of Lights

```xml
<!-- Directional light (like sun) -->
<light name="sun" type="directional">
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.1 -0.9</direction>
  <cast_shadows>true</cast_shadows>
</light>

<!-- Point light (like bulb) -->
<light name="ceiling_light" type="point">
  <pose>0 0 3 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <attenuation>
    <range>20</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <cast_shadows>false</cast_shadows>
</light>

<!-- Spotlight (like flashlight) -->
<light name="spot" type="spot">
  <pose>0 0 5 0 0 0</pose>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <direction>0 0 -1</direction>
  <attenuation>
    <range>50</range>
    <constant>0.5</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <spot>
    <inner_angle>0.6</inner_angle>
    <outer_angle>1.0</outer_angle>
    <falloff>1.0</falloff>
  </spot>
  <cast_shadows>true</cast_shadows>
</light>
```

---

## Dynamic Objects

### Spawn Objects at Runtime

```python title="spawn_objects.py"
#!/usr/bin/env python3
"""
Dynamically spawn objects in Gazebo
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import random


class ObjectSpawner(Node):
    """Spawns objects in Gazebo world."""

    def __init__(self):
        super().__init__('object_spawner')

        # Create service client
        self.spawn_client = self.create_client(
            SpawnEntity,
            '/spawn_entity'
        )

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')

        self.get_logger().info('Object Spawner ready')

    def spawn_box(self, name: str, x: float, y: float, z: float):
        """Spawn a box at specified position."""
        # SDF model string
        sdf = f'''
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="{name}">
            <pose>{x} {y} {z} 0 0 0</pose>
            <link name="link">
              <collision name="collision">
                <geometry>
                  <box><size>0.5 0.5 0.5</size></box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box><size>0.5 0.5 0.5</size></box>
                </geometry>
                <material>
                  <ambient>{random.random()} {random.random()} {random.random()} 1</ambient>
                </material>
              </visual>
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.083</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
                  <iyy>0.083</iyy><iyz>0.0</iyz>
                  <izz>0.083</izz>
                </inertia>
              </inertial>
            </link>
          </model>
        </sdf>
        '''

        # Create request
        request = SpawnEntity.Request()
        request.name = name
        request.xml = sdf

        # Call service
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result().success:
            self.get_logger().info(f'Spawned {name} at ({x}, {y}, {z})')
        else:
            self.get_logger().error(f'Failed to spawn {name}')


def main(args=None):
    rclpy.init(args=args)

    spawner = ObjectSpawner()

    # Spawn 5 random boxes
    for i in range(5):
        x = random.uniform(-5, 5)
        y = random.uniform(-5, 5)
        spawner.spawn_box(f'box_{i}', x, y, 0.25)

    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Randomized Test Environments

Create scenarios with randomized obstacles for robust testing.

```python title="randomize_world.py"
#!/usr/bin/env python3
"""
Generate randomized obstacle courses
"""

import random
import xml.etree.ElementTree as ET


def generate_random_world(num_obstacles: int, output_file: str):
    """Generate a world file with random obstacles."""

    # Create root
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='random_world')

    # Add ground and sun
    ET.SubElement(world, 'include').append(
        ET.Element('uri', text='model://ground_plane')
    )
    ET.SubElement(world, 'include').append(
        ET.Element('uri', text='model://sun')
    )

    # Generate random obstacles
    for i in range(num_obstacles):
        model = ET.SubElement(world, 'model', name=f'obstacle_{i}')
        ET.SubElement(model, 'static').text = 'true'

        # Random position
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        z = random.uniform(0.25, 1.0)
        ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 0 0'

        # Random size
        sx = random.uniform(0.3, 2.0)
        sy = random.uniform(0.3, 2.0)
        sz = random.uniform(0.5, 2.0)

        # Add link with geometry
        link = ET.SubElement(model, 'link', name='link')
        collision = ET.SubElement(link, 'collision', name='collision')
        geometry = ET.SubElement(collision, 'geometry')
        box = ET.SubElement(geometry, 'box')
        ET.SubElement(box, 'size').text = f'{sx} {sy} {sz}'

        # Visual (same as collision)
        visual = ET.SubElement(link, 'visual', name='visual')
        v_geometry = ET.SubElement(visual, 'geometry')
        v_box = ET.SubElement(v_geometry, 'box')
        ET.SubElement(v_box, 'size').text = f'{sx} {sy} {sz}'

    # Write to file
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='utf-8', xml_declaration=True)

    print(f'Generated world with {num_obstacles} obstacles: {output_file}')


if __name__ == '__main__':
    generate_random_world(20, 'random_obstacles.world')
```

---

## Launch World with Robot

```python title="simulation.launch.py"
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    # World file path
    world_file = os.path.join(
        os.getcwd(),
        'worlds',
        'warehouse.world'
    )

    return LaunchDescription([
        # Launch Gazebo with world
        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                world_file,
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        # Spawn robot (from previous chapter)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    'launch',
                    'spawn_robot.launch.py'
                )
            ])
        )
    ])
```

---

## Best Practices

1. **World Organization**
   ```
   simulation/
   ├── worlds/
   │   ├── office.world
   │   ├── warehouse.world
   │   └── outdoor.world
   ├── models/
   │   └── custom_models/
   └── launch/
       └── simulation.launch.py
   ```

2. **Model Reusability**
   - Create custom models in `~/.gazebo/models/`
   - Reference with `<uri>model://my_custom_model</uri>`

3. **Performance Optimization**
   - Use `<static>true</static>` for non-moving objects
   - Simplify collision geometry
   - Limit shadow casting lights

4. **Testing Scenarios**
   - Create multiple world files for different tests
   - Use randomization for robust algorithm testing

---

## Self-Assessment

### Questions

1. **What's the difference between SDF and URDF?**
   <details>
   <summary>Answer</summary>
   SDF is for world descriptions (multiple models, lighting, physics), while URDF is specifically for robot descriptions. SDF is more general-purpose.
   </details>

2. **Why use `<static>true</static>` for obstacles?**
   <details>
   <summary>Answer</summary>
   Static objects don't participate in physics calculations, improving simulation performance significantly.
   </details>

3. **What are the three main types of lights in Gazebo?**
   <details>
   <summary>Answer</summary>
   Directional (sun-like), Point (bulb-like), and Spot (flashlight-like).
   </details>

### Exercises

1. **Create an Office World**: Build a 10x10m office with walls, desks, and chairs

2. **Randomized Obstacles**: Use the randomizer script to create a world with 50 random obstacles

3. **Multi-Room Environment**: Design a world with multiple connected rooms and doorways

4. **Lighting Challenge**: Create a dark warehouse with only spotlight illumination

5. **Dynamic Spawning**: Write a script that spawns new obstacles every 5 seconds

---

## Next Steps

Now that you can build simulation environments, proceed to:

👉 [Chapter 3: Unity Integration](./03-unity-integration.md) to learn high-fidelity rendering

---

## Additional Resources

- [SDF Format Specification](http://sdformat.org/)
- [Gazebo Model Database](https://github.com/osrf/gazebo_models)
- [Building Editor Tutorial](https://classic.gazebosim.org/tutorials?tut=building_editor)
- [World Plugins](https://classic.gazebosim.org/tutorials?tut=plugins_world)

---

**Key Takeaway**: Well-designed simulation environments enable comprehensive robot testing across diverse scenarios. Mastering world creation is essential for developing robust robotic systems.
