---
sidebar_position: 1
---

# Introduction to NVIDIA Isaac Sim

Master NVIDIA Isaac Sim, the most advanced robotics simulator built on Omniverse, offering photorealistic rendering, accurate physics, and AI-ready synthetic data generation.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand Isaac Sim's architecture and capabilities
- Install Isaac Sim on Ubuntu 22.04 with RTX GPUs
- Navigate the Omniverse interface
- Import and simulate robots using USD format
- Configure physics engines (PhysX 5)
- Generate synthetic sensor data for AI training
- Integrate Isaac Sim with ROS 2

---

## What is NVIDIA Isaac Sim?

**Isaac Sim** is NVIDIA's robotics simulation platform built on **Omniverse**, providing:

- **Photorealistic Rendering**: Ray tracing, path tracing, real-time global illumination
- **Accurate Physics**: PhysX 5 with GPU acceleration
- **Sensor Simulation**: Cameras, LiDAR, depth, IMU, contact sensors
- **AI Integration**: Synthetic data generation for training perception models
- **Scalability**: Multi-GPU, distributed simulation
- **USD Format**: Universal Scene Description for interoperability

### Isaac Sim vs Gazebo vs Unity

| Feature | Gazebo | Unity | **Isaac Sim** |
|---------|--------|-------|---------------|
| **Physics** | ODE/Bullet | PhysX | **PhysX 5 (GPU)** |
| **Rendering** | OGRE | URP/HDRP | **RTX Ray Tracing** |
| **Sensor Accuracy** | Good | Good | **Excellent** |
| **AI Workflows** | Limited | Good | **Native (Omniverse)** |
| **ROS 2** | Native | Plugin | **Native Bridge** |
| **Multi-Robot** | Moderate | Good | **Excellent (GPU)** |
| **Hardware** | CPU | GPU | **RTX GPU Required** |

**When to use Isaac Sim**:
- Training vision-based AI models
- High-fidelity humanoid simulation
- Warehouse automation with 100+ robots
- Sim-to-real with domain randomization
- Digital twin applications

---

## Hardware Requirements

### Minimum Requirements

- **GPU**: NVIDIA RTX 2060 (6GB VRAM)
- **CPU**: Intel Core i7 / AMD Ryzen 7
- **RAM**: 32GB
- **Storage**: 50GB SSD
- **OS**: Ubuntu 20.04/22.04

### Recommended

- **GPU**: NVIDIA RTX 3090 / 4090 (24GB VRAM)
- **CPU**: Intel Core i9 / AMD Ryzen 9
- **RAM**: 64GB DDR5
- **Storage**: 100GB NVMe SSD

**Why RTX?**: Isaac Sim requires **ray tracing** cores for photorealistic rendering and **tensor cores** for AI inference.

---

## Installation (Ubuntu 22.04)

### Prerequisites

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install dependencies
sudo apt install build-essential cmake git wget curl -y

# Install NVIDIA drivers (if not already installed)
sudo ubuntu-drivers autoinstall
sudo reboot

# Verify GPU
nvidia-smi
```

### Download Isaac Sim

1. **Create NVIDIA Account**: [https://developer.nvidia.com](https://developer.nvidia.com)

2. **Download Omniverse Launcher**:
```bash
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

chmod +x omniverse-launcher-linux.AppImage
./omniverse-launcher-linux.AppImage
```

3. **Install Isaac Sim**:
   - Open Omniverse Launcher
   - Go to **Exchange** tab
   - Find **Isaac Sim**
   - Click **Install** (version 2023.1.1 or later)

**Installation Path**: `~/.local/share/ov/pkg/isaac_sim-2023.1.1`

### Verify Installation

```bash
# Navigate to Isaac Sim directory
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1

# Launch Isaac Sim
./isaac-sim.sh
```

**Expected**: Isaac Sim GUI launches with sample scenes.

---

## Isaac Sim Interface

### Main Components

1. **Viewport**: 3D scene visualization with ray tracing
2. **Stage**: Hierarchy of scene objects (USD prims)
3. **Property Panel**: Object properties and physics settings
4. **Content Browser**: Assets, robots, environments
5. **Toolbar**: Simulation controls (Play, Pause, Stop)

### Navigation

- **Orbit**: Alt + Left-click + drag
- **Pan**: Alt + Middle-click + drag
- **Zoom**: Alt + Right-click + drag (or scroll)
- **Fly Mode**: Right-click + WASD keys

---

## USD: Universal Scene Description

Isaac Sim uses **USD (Universal Scene Description)**, Pixar's scene format.

### Key Concepts

- **Prim (Primitive)**: Basic scene object (mesh, light, camera)
- **Stage**: Container for all prims
- **Layer**: Compositional unit for scene assembly
- **Xform**: Transformation (position, rotation, scale)

### Loading a Robot

```python title="load_robot.py"
#!/usr/bin/env python3
"""
Load a robot URDF into Isaac Sim
"""

from isaacsim import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.robots import Robot
import omni.usd


def load_robot():
    """Load robot from URDF."""
    # Create world
    world = World()

    # Import URDF extension
    from omni.importer.urdf import _urdf
    import_config = _urdf.ImportConfig()
    import_config.merge_fixed_joints = False
    import_config.convex_decomp = True
    import_config.import_inertia_tensor = True
    import_config.fix_base = False

    # Import robot
    robot_path = "/path/to/your/robot.urdf"
    _urdf.acquire_urdf_interface().parse_urdf(
        robot_path,
        "/World/Robot",
        import_config
    )

    # Get robot prim
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath("/World/Robot")

    print(f"Robot loaded: {robot_prim.GetName()}")

    # Reset world
    world.reset()

    # Run simulation
    while simulation_app.is_running():
        world.step(render=True)


if __name__ == "__main__":
    load_robot()
    simulation_app.close()
```

---

## Physics Configuration

Isaac Sim uses **NVIDIA PhysX 5** with GPU acceleration.

### Configure Physics Scene

```python title="physics_config.py"
from omni.isaac.core import PhysicsContext

def setup_physics():
    """Configure physics parameters."""
    # Get physics context
    physics_ctx = PhysicsContext()

    # Set parameters
    physics_ctx.set_gravity(-9.81)  # Earth gravity (m/s^2)
    physics_ctx.set_solver_type("TGS")  # Temporal Gauss-Seidel
    physics_ctx.set_broadphase_type("MBP")  # Multi Box Pruning

    # Physics scene settings
    physics_ctx.set_physics_dt(1/120.0)  # 120 Hz physics update

    print("Physics configured:")
    print(f"  Gravity: {physics_ctx.get_gravity()}")
    print(f"  Physics DT: {physics_ctx.get_physics_dt()}")
```

### Material Properties

```python
from omni.isaac.core.materials import PhysicsMaterial

# Create physics material
rubber = PhysicsMaterial(
    prim_path="/World/Materials/Rubber",
    static_friction=0.9,
    dynamic_friction=0.7,
    restitution=0.3  # Bounciness
)

# Apply to object
robot_link.apply_physics_material(rubber)
```

---

## Sensor Simulation

### Camera (RGB)

```python title="camera_sensor.py"
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.sensor import Camera
import numpy as np


def create_camera():
    """Create and configure camera sensor."""
    # Create camera prim
    camera = Camera(
        prim_path="/World/Camera",
        position=np.array([2.0, 0.0, 1.5]),
        orientation=np.array([1.0, 0.0, 0.0, 0.0]),
        frequency=30,  # 30 FPS
        resolution=(1280, 720)
    )

    camera.initialize()

    return camera


def get_camera_data(camera):
    """Retrieve camera image."""
    # Get RGBA image
    rgba = camera.get_rgba()

    # Get depth
    depth = camera.get_depth()

    # Get camera info
    intrinsics = camera.get_intrinsics_matrix()

    print(f"Image shape: {rgba.shape}")
    print(f"Depth shape: {depth.shape}")
    print(f"Intrinsics:\n{intrinsics}")

    return rgba, depth, intrinsics
```

### LiDAR (RTX)

```python title="lidar_sensor.py"
from omni.isaac.range_sensor import _range_sensor


def create_lidar():
    """Create RTX-accelerated LiDAR."""
    # Create LiDAR prim
    _, lidar_prim = omni.kit.commands.execute(
        "RangeSensorCreateLidar",
        path="/World/Lidar",
        parent=None,
        min_range=0.4,
        max_range=100.0,
        draw_points=True,
        draw_lines=False,
        horizontal_fov=360.0,
        vertical_fov=30.0,
        horizontal_resolution=0.4,  # 900 rays
        vertical_resolution=4.0,    # 8 beams
        rotation_rate=0.0,  # Static scan
        high_lod=True,      # High accuracy
        yaw_offset=0.0,
        enable_semantics=False
    )

    print(f"LiDAR created: {lidar_prim.GetPath()}")

    return lidar_prim


def get_lidar_data(lidar_interface, lidar_path):
    """Retrieve LiDAR point cloud."""
    # Get depth data
    depth_data = lidar_interface.get_linear_depth_data(lidar_path)

    # Convert to numpy
    points = np.array(depth_data)

    print(f"LiDAR points: {points.shape}")

    return points
```

---

## ROS 2 Integration

### Enable ROS 2 Bridge

```python title="ros2_bridge.py"
#!/usr/bin/env python3
"""
Isaac Sim ROS 2 Bridge
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS 2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

import rclpy


def setup_ros2_bridge():
    """Configure ROS 2 bridge."""
    # Create world
    world = World()

    # Import ROS 2 components
    from omni.isaac.ros2_bridge import Camera as ROS2Camera
    from omni.isaac.ros2_bridge import Clock

    # Create clock publisher
    clock = Clock()

    # Create camera publisher
    camera = ROS2Camera(
        prim_path="/World/Camera",
        topic_name="camera/image_raw",
        frame_id="camera_link"
    )

    print("ROS 2 Bridge initialized")

    return world, camera


if __name__ == "__main__":
    world, camera = setup_ros2_bridge()

    # Run simulation
    while simulation_app.is_running():
        world.step(render=True)

    simulation_app.close()
```

### Launch with ROS 2

**Terminal 1: Isaac Sim**
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.1
./isaac-sim.sh scripts/ros2_bridge.py
```

**Terminal 2: ROS 2**
```bash
# List topics
ros2 topic list

# Echo camera images
ros2 topic echo /camera/image_raw

# Visualize in RViz2
ros2 run rviz2 rviz2
```

---

## Sample Scenario: Mobile Robot

### Complete Example

```python title="mobile_robot_sim.py"
#!/usr/bin/env python3
"""
Complete mobile robot simulation in Isaac Sim
"""

from isaacsim import SimulationApp

# Launch
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.wheeled_robots.robots import WheeledRobot
from omni.isaac.wheeled_robots.controllers.differential_controller import DifferentialController
from omni.isaac.core.utils.nucleus import get_assets_root_path
import numpy as np


def main():
    """Run mobile robot simulation."""
    # Create world
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    world.scene.add_default_ground_plane()

    # Get Nucleus assets
    assets_root_path = get_assets_root_path()
    jetbot_asset = assets_root_path + "/Isaac/Robots/Jetbot/jetbot.usd"

    # Add robot
    jetbot = world.scene.add(
        WheeledRobot(
            prim_path="/World/Jetbot",
            name="jetbot",
            wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
            create_robot=True,
            usd_path=jetbot_asset,
            position=np.array([0, 0, 0.05])
        )
    )

    # Add controller
    controller = DifferentialController(
        name="diff_controller",
        wheel_radius=0.03,
        wheel_base=0.1125
    )

    # Reset
    world.reset()

    print("Simulation started - Robot will move in a circle")

    # Simulation loop
    while simulation_app.is_running():
        world.step(render=True)

        if world.is_playing():
            # Get robot state
            position, orientation = jetbot.get_world_pose()

            # Send velocity command (circle motion)
            controller_output = controller.forward(
                command=[0.2, 0.3]  # [linear, angular] velocities
            )

            # Apply to robot
            jetbot.apply_wheel_actions(controller_output)

    simulation_app.close()


if __name__ == "__main__":
    main()
```

---

## Performance Optimization

### GPU Utilization

```python
# Enable GPU dynamics
from omni.physx import get_physx_interface

physx = get_physx_interface()
physx.overwrite_gpu_setting(1)  # Force GPU
```

### Rendering Settings

- **RTX Quality**: Window → Rendering → Ray Tracing
  - Samples per pixel: 1 (fast) to 64 (quality)
  - Bounces: 2 (moderate) to 8 (photorealistic)

- **Viewport Resolution**: Lower for performance
  - 1280x720 for development
  - 1920x1080 for final renders

---

## Self-Assessment

### Questions

1. **What makes Isaac Sim different from Gazebo?**
   <details>
   <summary>Answer</summary>
   Isaac Sim offers photorealistic RTX rendering, GPU-accelerated physics (PhysX 5), native AI workflows, and superior sensor simulation for synthetic data generation.
   </details>

2. **What is USD and why is it important?**
   <details>
   <summary>Answer</summary>
   Universal Scene Description is Pixar's format for composable 3D scenes. It enables interoperability between tools and supports complex scene composition.
   </details>

3. **When should you use Isaac Sim vs Gazebo?**
   <details>
   <summary>Answer</summary>
   Use Isaac Sim for AI training (synthetic data), humanoid simulation, and large-scale multi-robot scenarios. Use Gazebo for traditional control/SLAM development with moderate hardware.
   </details>

### Exercises

1. **Install Isaac Sim**: Set up Isaac Sim on your RTX-enabled system

2. **Load Sample Robot**: Import a URDF robot into Isaac Sim

3. **Configure Physics**: Adjust gravity, friction, and time step parameters

4. **Camera Simulation**: Create a camera sensor and capture RGB-D images

5. **ROS 2 Bridge**: Set up the ROS 2 bridge and visualize topics in RViz2

---

## Next Steps

Now that you understand Isaac Sim basics, proceed to:

👉 [Chapter 2: Isaac ROS](./02-isaac-ros.md) to learn GPU-accelerated ROS 2 packages

---

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/)
- [Isaac Sim Tutorials](https://docs.omniverse.nvidia.com/isaacsim/latest/tutorial_intro.html)
- [USD Documentation](https://graphics.pixar.com/usd/docs/index.html)
- [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/)

---

**Key Takeaway**: Isaac Sim represents the cutting edge of robotics simulation, combining photorealistic rendering with accurate physics for AI-ready robot development. Mastering Isaac Sim enables training sophisticated perception systems with synthetic data.
