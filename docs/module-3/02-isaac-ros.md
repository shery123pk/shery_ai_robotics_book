---
sidebar_position: 2
---

# Isaac ROS: GPU-Accelerated ROS 2

Learn to use NVIDIA Isaac ROS packages for GPU-accelerated perception, providing 10-100x speedup over CPU-based algorithms.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand Isaac ROS architecture
- Install Isaac ROS packages on Jetson/x86
- Use DNN-based stereo depth estimation
- Implement AprilTag detection on GPU
- Process point clouds with CUDA acceleration
- Deploy Isaac ROS to Jetson edge devices

---

## What is Isaac ROS?

**Isaac ROS** is NVIDIA's collection of GPU-accelerated ROS 2 packages, offering:

- **10-100x Speedup**: CUDA acceleration vs CPU
- **Low Latency**: Real-time perception (under 50ms)
- **DNN Inference**: TensorRT-optimized models
- **Edge Deployment**: Runs on Jetson Orin/Xavier
- **Standard Interfaces**: Compatible with existing ROS 2 stacks

### Isaac ROS Packages

| Package | Purpose | Speedup |
|---------|---------|---------|
| **isaac_ros_visual_slam** | Visual SLAM (VO + mapping) | 20x |
| **isaac_ros_dnn_stereo_depth** | Stereo depth estimation | 50x |
| **isaac_ros_apriltag** | AprilTag detection | 100x |
| **isaac_ros_image_proc** | Image processing | 15x |
| **isaac_ros_object_detection** | Object detection (YOLO, DOPE) | 30x |
| **isaac_ros_pose_estimation** | 6-DOF pose estimation | 25x |

---

## Installation

### Prerequisites (Jetson Orin / x86 + RTX)

```bash
# Install ROS 2 Humble (if not already installed)
sudo apt install ros-humble-desktop -y

# Install Isaac ROS dependencies
sudo apt-get install -y \
    python3-pip \
    git-lfs \
    libssl-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    libceres-dev
```

### Install Isaac ROS Common

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone Isaac ROS Common
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Pull LFS files
cd isaac_ros_common
git lfs pull

# Build
cd ~/ros2_ws
colcon build --symlink-install

# Source
source install/setup.bash
```

### Install Specific Packages

```bash
cd ~/ros2_ws/src

# Visual SLAM
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# AprilTag
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

# DNN Stereo Depth
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_depth_segmentation.git

# Build all
cd ~/ros2_ws
colcon build --symlink-install
```

---

## Isaac ROS Visual SLAM

GPU-accelerated visual odometry and mapping.

### Launch Visual SLAM

```bash
# Source workspace
source ~/ros2_ws/install/setup.bash

# Launch with RealSense camera
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Custom Launch File

```python title="vslam_custom.launch.py"
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Isaac ROS Visual SLAM with custom camera."""

    return LaunchDescription([
        # Visual SLAM node
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam',
            parameters=[{
                'enable_rectified_pose': True,
                'denoise_input_images': True,
                'rectified_images': True,
                'enable_debug_mode': False,
                'debug_dump_path': '/tmp/cuvslam',
                'enable_slam_visualization': True,
                'enable_observations_view': True,
                'enable_landmarks_view': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'input_left_camera_frame': 'camera_infra1_optical_frame',
                'input_right_camera_frame': 'camera_infra2_optical_frame',
            }],
            remappings=[
                ('/stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                ('/stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                ('/stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                ('/stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
            ]
        ),

        # RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/path/to/vslam.rviz']
        )
    ])
```

### Monitor Performance

```bash
# Check FPS
ros2 topic hz /visual_slam/tracking/odometry

# Expected: 30 Hz (RealSense D435i)
# CPU version: ~3-5 Hz

# Check latency
ros2 topic echo /visual_slam/status
```

---

## AprilTag Detection (GPU)

Detect AprilTags 100x faster than CPU implementations.

### Launch AprilTag Detector

```bash
ros2 launch isaac_ros_apriltag isaac_ros_apriltag.launch.py
```

### Custom Detection Pipeline

```python title="apriltag_detector.py"
#!/usr/bin/env python3
"""
AprilTag detection with Isaac ROS
"""

import rclpy
from rclpy.node import Node
from isaac_ros_apriltag_interfaces.msg import AprilTagDetectionArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class AprilTagProcessor(Node):
    """Process AprilTag detections."""

    def __init__(self):
        super().__init__('apriltag_processor')

        # Subscribe to detections
        self.detection_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/tag_detections',
            self.detection_callback,
            10
        )

        # Subscribe to image for visualization
        self.image_sub = self.create_subscription(
            Image,
            '/image_rect',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_detections = None

        self.get_logger().info('AprilTag Processor started')

    def detection_callback(self, msg: AprilTagDetectionArray):
        """Process AprilTag detections."""
        self.latest_detections = msg

        for detection in msg.detections:
            tag_id = detection.id
            family = detection.family

            # Extract pose
            pose = detection.pose.pose.pose
            position = pose.position
            orientation = pose.orientation

            self.get_logger().info(
                f'Tag {tag_id} ({family}): '
                f'pos=({position.x:.2f}, {position.y:.2f}, {position.z:.2f})'
            )

    def image_callback(self, msg: Image):
        """Store latest image for visualization."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Draw detections
        if self.latest_detections:
            self.visualize_tags()

    def visualize_tags(self):
        """Draw detected tags on image."""
        if self.latest_image is None:
            return

        img = self.latest_image.copy()

        for detection in self.latest_detections.detections:
            # Get corners (simplified - actual corners in detection msg)
            center = (int(img.shape[1] / 2), int(img.shape[0] / 2))

            # Draw circle at tag center
            cv2.circle(img, center, 10, (0, 255, 0), -1)

            # Draw tag ID
            cv2.putText(
                img,
                f'ID: {detection.id}',
                (center[0] + 15, center[1]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        cv2.imshow('AprilTag Detection', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    processor = AprilTagProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## DNN Stereo Depth

Deep learning-based stereo depth estimation.

### Architecture

```
┌────────────┐     ┌───────────┐     ┌────────────┐
│ Left Image │ ──> │           │ ──> │   Depth    │
└────────────┘     │   DNN     │     │   Image    │
┌────────────┐     │ (TensorRT)│     │ (GPU Mem)  │
│Right Image │ ──> │           │ ──> │            │
└────────────┘     └───────────┘     └────────────┘
```

### Launch DNN Stereo

```bash
# Download pre-trained model
mkdir -p ~/ros2_ws/src/isaac_ros_depth_segmentation/models
cd ~/ros2_ws/src/isaac_ros_depth_segmentation/models

wget https://developer.nvidia.com/isaac/download/third_party/ess/ess-model.tar.gz
tar -xzf ess-model.tar.gz

# Launch
ros2 launch isaac_ros_ess isaac_ros_ess_stereo.launch.py
```

### Processing Pipeline

```python title="depth_processor.py"
#!/usr/bin/env python3
"""
Process DNN stereo depth output
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthProcessor(Node):
    """Process depth images from DNN stereo."""

    def __init__(self):
        super().__init__('depth_processor')

        self.bridge = CvBridge()

        # Subscribe to depth
        self.depth_sub = self.create_subscription(
            Image,
            '/depth/image',
            self.depth_callback,
            10
        )

        # Publish colored depth
        self.colored_pub = self.create_publisher(
            Image,
            '/depth/colored',
            10
        )

        self.get_logger().info('Depth Processor started')

    def depth_callback(self, msg: Image):
        """Process depth image."""
        # Convert to numpy
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Normalize for visualization
        depth_normalized = cv2.normalize(
            depth,
            None,
            0, 255,
            cv2.NORM_MINMAX,
            dtype=cv2.CV_8U
        )

        # Apply colormap
        depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        # Publish
        colored_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding='bgr8')
        colored_msg.header = msg.header
        self.colored_pub.publish(colored_msg)

        # Calculate statistics
        valid_depths = depth[depth > 0]
        if len(valid_depths) > 0:
            mean_depth = np.mean(valid_depths)
            min_depth = np.min(valid_depths)
            max_depth = np.max(valid_depths)

            self.get_logger().info(
                f'Depth - Mean: {mean_depth:.2f}m, '
                f'Min: {min_depth:.2f}m, Max: {max_depth:.2f}m'
            )


def main(args=None):
    rclpy.init(args=args)
    processor = DepthProcessor()
    rclpy.spin(processor)
    processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Image Processing (GPU)

Accelerated image rectification, resizing, and format conversion.

### Rectification Pipeline

```python title="image_rectification.launch.py"
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch GPU-accelerated image rectification."""

    rectify_container = ComposableNodeContainer(
        name='rectify_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # GPU Rectification
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::RectifyNode',
                name='rectify_node',
                parameters=[{
                    'output_width': 1280,
                    'output_height': 720,
                }],
                remappings=[
                    ('image_raw', '/camera/image_raw'),
                    ('camera_info', '/camera/camera_info'),
                    ('image_rect', '/camera/image_rect'),
                ]
            ),

            # GPU Resize
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ResizeNode',
                name='resize_node',
                parameters=[{
                    'output_width': 640,
                    'output_height': 480,
                    'encoding_desired': 'rgb8',
                }],
                remappings=[
                    ('image', '/camera/image_rect'),
                    ('camera_info', '/camera/camera_info'),
                    ('resize/image', '/camera/image_resized'),
                    ('resize/camera_info', '/camera/camera_info_resized'),
                ]
            ),
        ],
        output='screen'
    )

    return LaunchDescription([rectify_container])
```

---

## Performance Comparison

### Benchmark: AprilTag Detection

| Platform | Implementation | FPS | Latency |
|----------|----------------|-----|---------|
| **x86 CPU** | apriltag_ros | 3 | 330ms |
| **Jetson Orin** | apriltag_ros | 5 | 200ms |
| **Jetson Orin** | **Isaac ROS** | **60** | **16ms** |
| **RTX 4090** | **Isaac ROS** | **120** | **8ms** |

### Benchmark: Stereo Depth

| Platform | Method | FPS | Latency |
|----------|--------|-----|---------|
| **x86 CPU** | Block Matching | 15 | 66ms |
| **Jetson Xavier** | SGBM | 10 | 100ms |
| **Jetson Orin** | **DNN (Isaac ROS)** | **30** | **33ms** |
| **RTX 3090** | **DNN (Isaac ROS)** | **90** | **11ms** |

---

## Deployment to Jetson

### Jetson Orin Setup

```bash
# Flash Jetson with JetPack 5.1+
# (Use NVIDIA SDK Manager)

# Install Isaac ROS on Jetson
ssh jetson@192.168.1.100

# Follow installation steps from earlier
# Build with limited parallel jobs to avoid OOM
colcon build --symlink-install --parallel-workers 2
```

### Monitor Resource Usage

```bash
# Install jetson-stats
sudo pip3 install jetson-stats

# Monitor GPU/CPU/Memory
jtop
```

**Expected Usage (Visual SLAM)**:
- GPU: 60-80%
- CPU: 20-30%
- Memory: 4-6GB
- Power: 15-20W

---

## Best Practices

1. **Use Composition**: Combine nodes in single process for zero-copy
2. **Tune Buffer Sizes**: Balance latency vs throughput
3. **Monitor Thermal**: Jetson throttles at 85°C
4. **Profile Performance**: Use `ros2 topic hz` and `jtop`
5. **Update Firmware**: Keep JetPack current for optimizations

---

## Self-Assessment

### Questions

1. **What's the main advantage of Isaac ROS?**
   <details>
   <summary>Answer</summary>
   GPU acceleration provides 10-100x speedup over CPU implementations, enabling real-time perception on edge devices.
   </details>

2. **When should you use DNN stereo vs block matching?**
   <details>
   <summary>Answer</summary>
   Use DNN stereo when you have GPU (Jetson/RTX) and need better accuracy in textureless regions. Use block matching on CPU-only systems.
   </details>

3. **Why use composable nodes?**
   <details>
   <summary>Answer</summary>
   Composable nodes run in a single process with zero-copy message passing, reducing latency and improving performance.
   </details>

### Exercises

1. **Install Isaac ROS**: Set up Isaac ROS Common on your system (Jetson or x86+RTX)

2. **AprilTag Detection**: Run AprilTag detector and measure FPS vs CPU version

3. **Visual SLAM**: Launch Isaac ROS Visual SLAM with a stereo camera and visualize in RViz2

4. **Depth Estimation**: Run DNN stereo depth and compare quality vs SGBM

5. **Performance Profiling**: Use `jtop` (Jetson) or `nvidia-smi` (x86) to monitor GPU utilization

---

## Next Steps

Now that you understand Isaac ROS, proceed to:

👉 [Chapter 3: VSLAM and Perception](./03-vslam-perception.md) to build complete perception pipelines

---

## Additional Resources

- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Isaac ROS GitHub](https://github.com/NVIDIA-ISAAC-ROS)
- [Jetson Setup Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [TensorRT Optimization](https://docs.nvidia.com/deeplearning/tensorrt/)

---

**Key Takeaway**: Isaac ROS brings GPU acceleration to ROS 2 perception, enabling real-time performance on edge devices. Mastering Isaac ROS is essential for deploying sophisticated AI-based robots.
