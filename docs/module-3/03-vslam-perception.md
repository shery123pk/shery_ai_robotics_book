---
sidebar_position: 3
---

# VSLAM and Perception

Master Visual Simultaneous Localization and Mapping (VSLAM) for robot navigation and perception pipelines for understanding the environment.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand VSLAM fundamentals
- Deploy Isaac ROS cuVSLAM (GPU-accelerated VSLAM)
- Build perception pipelines combining multiple sensors
- Implement object detection and tracking
- Fuse sensor data for robust localization
- Deploy perception stacks to real robots

---

## VSLAM Fundamentals

**Visual SLAM** enables robots to build maps and localize simultaneously using only camera data.

### Key Components

```
┌─────────────┐
│   Camera    │
└──────┬──────┘
       │
       ▼
┌─────────────┐    ┌──────────────┐    ┌────────────┐
│   Feature   │ -> │   Tracking   │ -> │    Pose    │
│  Detection  │    │ (Frame-Frame)│    │  Estimate  │
└─────────────┘    └──────────────┘    └────────────┘
       │
       ▼
┌─────────────┐    ┌──────────────┐
│  Mapping    │ -> │     Loop     │
│ (Landmarks) │    │   Closure    │
└─────────────┘    └──────────────┘
```

### VSLAM Types

| Type | Sensors | Map | Use Case |
|------|---------|-----|----------|
| **Monocular** | 1 camera | Sparse | Scale ambiguity, indoor |
| **Stereo** | 2 cameras | Dense | Metric scale, outdoor |
| **RGB-D** | Depth camera | Dense | Indoor, close range |
| **Visual-Inertial** | Camera + IMU | Sparse | High-speed motion |

---

## Isaac ROS cuVSLAM

NVIDIA's GPU-accelerated Visual SLAM with stereo cameras.

### Installation

```bash
cd ~/ros2_ws/src

# Clone cuVSLAM
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# Build
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to isaac_ros_visual_slam

# Source
source install/setup.bash
```

### Launch with RealSense

```bash
# Install RealSense ROS 2 wrapper
sudo apt install ros-humble-realsense2-camera -y

# Launch VSLAM with RealSense D435i
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_realsense.launch.py
```

### Custom Launch File

```python title="vslam_custom.launch.py"
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch cuVSLAM with custom parameters."""

    # Parameters
    enable_rectified_pose = LaunchConfiguration('enable_rectified_pose', default=True)
    denoise_input_images = LaunchConfiguration('denoise_input_images', default=False)
    enable_slam_visualization = LaunchConfiguration('enable_slam_visualization', default=True)

    return LaunchDescription([
        # RealSense Camera
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_depth': False,  # Not needed for VSLAM
                'enable_color': False,
                'infra_width': 640,
                'infra_height': 480,
                'infra_fps': 30,
                'unite_imu_method': 'linear_interpolation',
            }]
        ),

        # cuVSLAM Container
        ComposableNodeContainer(
            name='visual_slam_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='isaac_ros_visual_slam',
                    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                    name='visual_slam',
                    parameters=[{
                        'enable_rectified_pose': enable_rectified_pose,
                        'denoise_input_images': denoise_input_images,
                        'rectified_images': True,
                        'enable_debug_mode': False,
                        'enable_slam_visualization': enable_slam_visualization,
                        'enable_observations_view': True,
                        'enable_landmarks_view': True,
                        'map_frame': 'map',
                        'odom_frame': 'odom',
                        'base_frame': 'base_link',
                        'input_left_camera_frame': 'camera_infra1_optical_frame',
                        'input_right_camera_frame': 'camera_infra2_optical_frame',
                        'path_max_size': 1024,
                    }],
                    remappings=[
                        ('/stereo_camera/left/image', '/camera/infra1/image_rect_raw'),
                        ('/stereo_camera/left/camera_info', '/camera/infra1/camera_info'),
                        ('/stereo_camera/right/image', '/camera/infra2/image_rect_raw'),
                        ('/stereo_camera/right/camera_info', '/camera/infra2/camera_info'),
                    ]
                ),
            ],
            output='screen'
        ),

        # Static transform (camera to base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link']
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/path/to/vslam.rviz']
        )
    ])
```

---

## Monitoring VSLAM Performance

### Check Tracking Status

```python title="vslam_monitor.py"
#!/usr/bin/env python3
"""
Monitor cuVSLAM performance
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
import time


class VSLAMMonitor(Node):
    """Monitor VSLAM tracking quality."""

    def __init__(self):
        super().__init__('vslam_monitor')

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        # Subscribe to status
        self.status_sub = self.create_subscription(
            VisualSlamStatus,
            '/visual_slam/status',
            self.status_callback,
            10
        )

        self.last_odom_time = None
        self.odom_count = 0
        self.start_time = time.time()

        # Timer for stats
        self.timer = self.create_timer(5.0, self.print_stats)

        self.get_logger().info('VSLAM Monitor started')

    def odom_callback(self, msg: Odometry):
        """Track odometry rate."""
        self.odom_count += 1
        self.last_odom_time = time.time()

    def status_callback(self, msg: VisualSlamStatus):
        """Monitor tracking status."""
        tracking_status = 'Tracking' if msg.vo_state == 2 else 'Lost'

        self.get_logger().info(
            f'Status: {tracking_status}, '
            f'Tracked Features: {msg.num_tracked_features}, '
            f'Loop Closures: {msg.num_loop_closures}'
        )

    def print_stats(self):
        """Print performance statistics."""
        elapsed = time.time() - self.start_time
        fps = self.odom_count / elapsed if elapsed > 0 else 0

        self.get_logger().info(f'VSLAM FPS: {fps:.1f} Hz')


def main(args=None):
    rclpy.init(args=args)
    monitor = VSLAMMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Object Detection Pipeline

Combine VSLAM with object detection for semantic mapping.

### YOLOv8 Integration

```python title="object_detector.py"
#!/usr/bin/env python3
"""
Object detection with YOLOv8 + TensorRT
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO


class ObjectDetector(Node):
    """YOLOv8 object detector."""

    def __init__(self):
        super().__init__('object_detector')

        # Load YOLO model
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        self.viz_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        self.get_logger().info(f'Object Detector started with {model_path}')

    def image_callback(self, msg: Image):
        """Detect objects in image."""
        # Convert to CV2
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run inference
        results = self.model(cv_image, conf=self.conf_threshold)[0]

        # Create detection message
        detection_array = Detection2DArray()
        detection_array.header = msg.header

        # Process each detection
        for box in results.boxes:
            # Extract bounding box
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = box.conf[0].item()
            cls = int(box.cls[0].item())
            class_name = self.model.names[cls]

            # Create Detection2D message
            detection = Detection2D()
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = conf

            detection.results.append(hypothesis)
            detection_array.detections.append(detection)

            # Draw on image
            cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(
                cv_image,
                f'{class_name} {conf:.2f}',
                (int(x1), int(y1) - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        # Publish detections
        self.detection_pub.publish(detection_array)

        # Publish visualization
        viz_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    detector = ObjectDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Sensor Fusion

Combine multiple sensors for robust perception.

### IMU + Visual Odometry Fusion

```python title="sensor_fusion.py"
#!/usr/bin/env python3
"""
Fuse Visual Odometry with IMU using EKF
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
import numpy as np


class SensorFusion(Node):
    """Extended Kalman Filter for VO + IMU fusion."""

    def __init__(self):
        super().__init__('sensor_fusion')

        # State [x, y, z, vx, vy, vz, roll, pitch, yaw]
        self.state = np.zeros(9)
        self.covariance = np.eye(9) * 0.1

        # Subscribers
        self.vo_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.vo_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publisher
        self.fused_pub = self.create_publisher(
            Odometry,
            '/fused/odometry',
            10
        )

        self.last_update_time = self.get_clock().now()

        self.get_logger().info('Sensor Fusion started')

    def vo_callback(self, msg: Odometry):
        """Update with visual odometry measurement."""
        # Extract position
        z_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        # Measurement update (simplified EKF)
        H = np.zeros((3, 9))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1

        R = np.eye(3) * 0.01  # VO covariance

        # Kalman gain
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        # Update
        innovation = z_pos - self.state[:3]
        self.state += K @ innovation
        self.covariance = (np.eye(9) - K @ H) @ self.covariance

        # Publish fused estimate
        self.publish_fused()

    def imu_callback(self, msg: Imu):
        """Prediction step with IMU."""
        dt = (self.get_clock().now() - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = self.get_clock().now()

        if dt > 1.0:  # Skip if too much time passed
            return

        # Extract IMU data
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Predict velocity (simplified)
        self.state[3] += acc_x * dt
        self.state[4] += acc_y * dt
        self.state[5] += (acc_z - 9.81) * dt

        # Predict position
        self.state[0] += self.state[3] * dt
        self.state[1] += self.state[4] * dt
        self.state[2] += self.state[5] * dt

        # Update covariance
        Q = np.eye(9) * 0.001 * dt  # Process noise
        self.covariance += Q

    def publish_fused(self):
        """Publish fused odometry."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.state[0]
        odom.pose.pose.position.y = self.state[1]
        odom.pose.pose.position.z = self.state[2]

        odom.twist.twist.linear.x = self.state[3]
        odom.twist.twist.linear.y = self.state[4]
        odom.twist.twist.linear.z = self.state[5]

        self.fused_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    fusion = SensorFusion()
    rclpy.spin(fusion)
    fusion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Complete Perception Pipeline

### Architecture

```
┌──────────┐
│ Cameras  │──┬──> cuVSLAM ──> Odometry ─┐
└──────────┘  │                           │
              └──> YOLOv8 ──> Detections ─┤
┌──────────┐                              ├──> Sensor Fusion ──> Nav2
│   IMU    │───────────────────────────────┤
└──────────┘                              │
┌──────────┐                              │
│  LiDAR   │──> Point Cloud ──> Obstacles ┘
└──────────┘
```

### Launch Complete Stack

```python title="perception_stack.launch.py"
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Launch complete perception pipeline."""

    return LaunchDescription([
        # cuVSLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                'vslam_custom.launch.py'
            ])
        ),

        # Object Detection
        Node(
            package='perception',
            executable='object_detector',
            parameters=[{
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.6
            }]
        ),

        # Sensor Fusion
        Node(
            package='perception',
            executable='sensor_fusion'
        ),

        # Point Cloud Processing
        Node(
            package='perception',
            executable='pointcloud_processor'
        ),

        # Semantic Mapping
        Node(
            package='perception',
            executable='semantic_mapper',
            parameters=[{
                'map_resolution': 0.05,
                'map_size': 100.0
            }]
        )
    ])
```

---

## Best Practices

1. **Lighting Conditions**: Ensure adequate lighting for Visual SLAM
2. **Camera Calibration**: Accurate intrinsics are critical
3. **Sensor Synchronization**: Time-sync cameras and IMU
4. **Loop Closure**: Enable for drift correction in long trajectories
5. **Failure Recovery**: Implement tracking loss detection and recovery

---

## Self-Assessment

### Questions

1. **What's the difference between monocular and stereo VSLAM?**
   <details>
   <summary>Answer</summary>
   Monocular VSLAM has scale ambiguity (unknown absolute scale), while stereo VSLAM provides metric scale from baseline distance.
   </details>

2. **Why fuse IMU with visual odometry?**
   <details>
   <summary>Answer</summary>
   IMU provides high-frequency orientation and acceleration data that complements visual odometry's position estimates, improving robustness during fast motion or poor visual features.
   </details>

3. **What causes VSLAM tracking loss?**
   <details>
   <summary>Answer</summary>
   Rapid motion, featureless scenes, lighting changes, motion blur, or occlusions can cause tracking loss.
   </details>

### Exercises

1. **Deploy cuVSLAM**: Run Isaac ROS Visual SLAM with a stereo camera and visualize trajectory in RViz2

2. **Object Detection**: Integrate YOLOv8 detector with camera stream and publish detection messages

3. **Sensor Fusion**: Implement basic IMU + VO fusion and compare with raw odometry

4. **Performance Tuning**: Optimize VSLAM parameters for your environment (indoor/outdoor)

5. **Failure Recovery**: Implement tracking loss detection and automatic recovery

---

## Next Steps

Now that you understand perception, proceed to:

👉 [Chapter 4: Navigation with Nav2](./04-nav2.md) to build autonomous navigation systems

---

## Additional Resources

- [cuVSLAM Paper](https://developer.nvidia.com/blog/accelerating-visual-slam-with-nvidia-jetson/)
- [Visual SLAM Tutorial](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Visual-SLAM.html)
- [Robot Localization Package](http://docs.ros.org/en/humble/p/robot_localization/)
- [YOLOv8 Documentation](https://docs.ultralytics.com/)

---

**Key Takeaway**: Robust perception combines multiple sensors (cameras, IMU, LiDAR) with GPU-accelerated algorithms to enable accurate localization and environment understanding. Mastering perception is essential for autonomous robots.
