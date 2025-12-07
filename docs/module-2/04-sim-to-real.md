---
sidebar_position: 4
---

# Sim-to-Real Transfer

Master the art of transferring knowledge from simulation to real robots, bridging the "reality gap" for successful deployment.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the sim-to-real reality gap
- Apply domain randomization techniques
- Calibrate simulation parameters to match hardware
- Validate algorithms in both sim and real environments
- Deploy sim-trained policies to physical robots
- Measure and close the performance gap

---

## The Reality Gap

The **reality gap** is the discrepancy between simulated and real-world behavior.

### Sources of Reality Gap

| Category | Simulation | Reality | Gap |
|----------|-----------|---------|-----|
| **Physics** | Perfect, deterministic | Noisy, stochastic | Friction, compliance |
| **Sensors** | Ideal measurements | Noisy, delayed | Calibration errors |
| **Actuators** | Instant response | Lag, backlash | Motor dynamics |
| **Environment** | Controlled | Uncontrolled | Lighting, weather |
| **Modeling** | Simplified geometry | Complex surfaces | Contact dynamics |

### Manifestations

**What works in simulation but fails in reality**:
- Precise grasping (simulation has perfect friction)
- Fast movements (simulation ignores motor dynamics)
- Exact localization (simulation has perfect sensors)
- Collision-free navigation (simulation has simple geometry)

---

## Bridging the Gap: Strategies

### 1. System Identification

Measure real-world parameters and update simulation.

```python title="system_identification.py"
#!/usr/bin/env python3
"""
System Identification: Measure real robot parameters
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import time


class SystemIdentifier(Node):
    """Identifies robot kinematic parameters."""

    def __init__(self):
        super().__init__('system_identifier')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        self.positions = []
        self.timestamps = []

        self.get_logger().info('System Identifier started')

    def odom_callback(self, msg: Odometry):
        """Record odometry."""
        self.positions.append([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        self.timestamps.append(time.time())

    def run_forward_test(self, velocity: float, duration: float):
        """Test forward motion to identify actual wheel radius."""
        self.get_logger().info(f'Forward test: {velocity} m/s for {duration}s')

        # Clear data
        self.positions = []
        self.timestamps = []

        # Send command
        cmd = Twist()
        cmd.linear.x = velocity

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        # Stop
        cmd.linear.x = 0.0
        self.cmd_pub.publish(cmd)

        # Analyze
        if len(self.positions) >= 2:
            start_pos = np.array(self.positions[0])
            end_pos = np.array(self.positions[-1])
            actual_distance = np.linalg.norm(end_pos - start_pos)

            expected_distance = velocity * duration
            error = (actual_distance - expected_distance) / expected_distance * 100

            self.get_logger().info(
                f'Expected: {expected_distance:.3f}m, '
                f'Actual: {actual_distance:.3f}m, '
                f'Error: {error:.1f}%'
            )

            # Wheel radius correction factor
            correction = actual_distance / expected_distance
            self.get_logger().info(f'Wheel radius correction: {correction:.4f}')

            return correction

        return 1.0

    def run_rotation_test(self, angular_vel: float, duration: float):
        """Test rotation to identify wheel separation."""
        self.get_logger().info(
            f'Rotation test: {angular_vel} rad/s for {duration}s'
        )

        cmd = Twist()
        cmd.angular.z = angular_vel

        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.1)

        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    identifier = SystemIdentifier()

    # Run tests
    time.sleep(2)  # Let system stabilize

    # Forward motion test
    correction = identifier.run_forward_test(velocity=0.2, duration=5.0)

    time.sleep(2)

    # Rotation test
    identifier.run_rotation_test(angular_vel=0.5, duration=4.0)

    identifier.get_logger().info('System identification complete')
    identifier.get_logger().info(
        f'Update URDF wheel radius by factor: {correction:.4f}'
    )

    identifier.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Use results to update**:
```xml
<!-- Before -->
<wheel_radius>0.100</wheel_radius>

<!-- After (if correction = 1.05) -->
<wheel_radius>0.105</wheel_radius>
```

---

### 2. Domain Randomization

Vary simulation parameters to cover real-world uncertainty.

```python title="domain_randomization.py"
#!/usr/bin/env python3
"""
Domain Randomization for Gazebo
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Twist
import random


class DomainRandomizer(Node):
    """Randomizes simulation parameters."""

    def __init__(self):
        super().__init__('domain_randomizer')

        # Service clients
        self.set_state_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        # Timer for periodic randomization
        self.timer = self.create_timer(10.0, self.randomize_environment)

        self.get_logger().info('Domain Randomizer started')

    def randomize_environment(self):
        """Randomize physics and visual parameters."""

        # Randomize ground friction
        self.randomize_friction('ground_plane', mu_min=0.5, mu_max=1.5)

        # Randomize lighting (would require Gazebo plugin)
        # In practice, use Gazebo's built-in randomization plugins

        # Randomize obstacle positions
        self.randomize_obstacle_position('obstacle_1')

        self.get_logger().info('Environment randomized')

    def randomize_friction(self, model_name: str, mu_min: float, mu_max: float):
        """Randomize surface friction."""
        # Note: Requires dynamic parameter update plugin
        mu = random.uniform(mu_min, mu_max)
        self.get_logger().info(f'Set {model_name} friction to {mu:.2f}')

    def randomize_obstacle_position(self, model_name: str):
        """Move obstacle to random position."""
        if not self.set_state_client.wait_for_service(timeout_sec=1.0):
            return

        # Random position
        x = random.uniform(-5.0, 5.0)
        y = random.uniform(-5.0, 5.0)

        # Create state
        state = EntityState()
        state.name = model_name
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = 0.5

        # Send request
        request = SetEntityState.Request()
        request.state = state

        future = self.set_state_client.call_async(request)
        self.get_logger().info(f'Moved {model_name} to ({x:.1f}, {y:.1f})')


def main(args=None):
    rclpy.init(args=args)
    randomizer = DomainRandomizer()
    rclpy.spin(randomizer)
    randomizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Randomization Parameters

| Parameter | Range | Impact |
|-----------|-------|--------|
| **Friction** | 0.5-1.5 | Slipping, traction |
| **Mass** | ±20% | Acceleration, inertia |
| **Sensor Noise** | ±50% | Perception errors |
| **Lighting** | 50-150 lux | Vision quality |
| **Delay** | 0-100ms | Control lag |

---

### 3. Sim-to-Real Transfer Pipeline

```
┌──────────────┐      ┌──────────────┐      ┌──────────────┐
│   Develop    │ ───> │   Validate   │ ───> │    Deploy    │
│ in Simulation│      │  in Sim+Real │      │  to Robot    │
└──────────────┘      └──────────────┘      └──────────────┘
      │                      │                      │
      │                      │                      │
   ┌──▼──┐              ┌───▼───┐             ┌────▼────┐
   │ Test│              │Compare│             │ Monitor │
   │Basic│              │ Perf. │             │ Real    │
   │Logic│              │       │             │ Metrics │
   └─────┘              └───┬───┘             └────┬────┘
                            │                      │
                        ┌───▼───┐                  │
                        │ Tune  │<─────────────────┘
                        │ Params│
                        └───────┘
```

---

## Sensor Calibration

### Camera Calibration

```python title="camera_calibration.py"
#!/usr/bin/env python3
"""
Compare simulated vs real camera intrinsics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np


class CameraCalibrationValidator(Node):
    """Validates camera calibration matches simulation."""

    def __init__(self):
        super().__init__('camera_calibration_validator')

        self.sim_info = None
        self.real_info = None

        # Subscribers
        self.sim_sub = self.create_subscription(
            CameraInfo, '/sim/camera_info', self.sim_callback, 10
        )
        self.real_sub = self.create_subscription(
            CameraInfo, '/real/camera_info', self.real_callback, 10
        )

        # Timer to compare
        self.timer = self.create_timer(5.0, self.compare_calibration)

        self.get_logger().info('Camera Calibration Validator started')

    def sim_callback(self, msg: CameraInfo):
        """Store sim camera info."""
        self.sim_info = msg

    def real_callback(self, msg: CameraInfo):
        """Store real camera info."""
        self.real_info = msg

    def compare_calibration(self):
        """Compare camera parameters."""
        if self.sim_info is None or self.real_info is None:
            self.get_logger().warn('Waiting for camera info from both sources')
            return

        # Extract K matrix (intrinsics)
        sim_K = np.array(self.sim_info.k).reshape(3, 3)
        real_K = np.array(self.real_info.k).reshape(3, 3)

        # Compare focal lengths
        sim_fx = sim_K[0, 0]
        sim_fy = sim_K[1, 1]
        real_fx = real_K[0, 0]
        real_fy = real_K[1, 1]

        fx_error = abs(sim_fx - real_fx) / real_fx * 100
        fy_error = abs(sim_fy - real_fy) / real_fy * 100

        self.get_logger().info('Camera Calibration Comparison:')
        self.get_logger().info(f'  Sim fx:  {sim_fx:.1f}, fy:  {sim_fy:.1f}')
        self.get_logger().info(f'  Real fx: {real_fx:.1f}, fy: {real_fy:.1f}')
        self.get_logger().info(f'  Error:   {fx_error:.1f}%, {fy_error:.1f}%')

        if fx_error > 5.0 or fy_error > 5.0:
            self.get_logger().error('Calibration mismatch > 5%! Update simulation')


def main(args=None):
    rclpy.init(args=args)
    validator = CameraCalibrationValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Validation Metrics

### Navigation Performance Comparison

```python title="performance_validator.py"
#!/usr/bin/env python3
"""
Compare navigation performance: Sim vs Real
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import matplotlib.pyplot as plt


class PerformanceValidator(Node):
    """Validates navigation performance."""

    def __init__(self):
        super().__init__('performance_validator')

        self.sim_paths = []
        self.real_paths = []

        # Metrics
        self.metrics = {
            'sim': {'success_rate': [], 'path_length': [], 'time': []},
            'real': {'success_rate': [], 'path_length': [], 'time': []}
        }

    def record_trial(self, environment: str, success: bool,
                     path_length: float, time_taken: float):
        """Record a navigation trial."""
        self.metrics[environment]['success_rate'].append(1 if success else 0)
        self.metrics[environment]['path_length'].append(path_length)
        self.metrics[environment]['time'].append(time_taken)

    def analyze_performance(self):
        """Compare sim vs real performance."""
        print("\n" + "="*50)
        print("SIM-TO-REAL PERFORMANCE COMPARISON")
        print("="*50)

        for env in ['sim', 'real']:
            sr = np.mean(self.metrics[env]['success_rate']) * 100
            avg_length = np.mean(self.metrics[env]['path_length'])
            avg_time = np.mean(self.metrics[env]['time'])

            print(f"\n{env.upper()}:")
            print(f"  Success Rate: {sr:.1f}%")
            print(f"  Avg Path Length: {avg_length:.2f}m")
            print(f"  Avg Time: {avg_time:.1f}s")

        # Calculate gap
        sr_gap = abs(
            np.mean(self.metrics['sim']['success_rate']) -
            np.mean(self.metrics['real']['success_rate'])
        ) * 100

        print(f"\nSuccess Rate Gap: {sr_gap:.1f}%")

        if sr_gap < 10:
            print("✓ Good sim-to-real transfer (< 10% gap)")
        elif sr_gap < 20:
            print("⚠ Moderate gap (10-20%). Consider tuning.")
        else:
            print("✗ Large gap (> 20%). Major calibration needed.")


def main(args=None):
    validator = PerformanceValidator()

    # Example trials (in practice, run real experiments)
    # Simulation trials
    validator.record_trial('sim', True, 5.2, 12.3)
    validator.record_trial('sim', True, 4.8, 11.5)
    validator.record_trial('sim', True, 5.5, 13.1)
    validator.record_trial('sim', False, 3.2, 20.0)

    # Real robot trials
    validator.record_trial('real', True, 5.8, 14.2)
    validator.record_trial('real', True, 5.3, 13.8)
    validator.record_trial('real', False, 4.1, 18.5)
    validator.record_trial('real', True, 6.2, 15.1)

    # Analyze
    validator.analyze_performance()


if __name__ == '__main__':
    main()
```

---

## Deployment Checklist

### Pre-Deployment Validation

- [ ] **Simulation validation**: Algorithm works in baseline scenario
- [ ] **Domain randomization**: Robust to parameter variations
- [ ] **System identification**: Simulation parameters match hardware
- [ ] **Sensor calibration**: Camera/LiDAR intrinsics validated
- [ ] **Safety limits**: Speed/torque limits match robot constraints
- [ ] **Emergency stop**: Verified in simulation and real hardware
- [ ] **Performance metrics**: Defined and measurable

### Deployment Steps

1. **Test in Controlled Real Environment**
   - Simple, obstacle-free space
   - Monitor all sensors
   - Manual override ready

2. **Compare Metrics**
   - Success rate
   - Path efficiency
   - Execution time

3. **Iterative Tuning**
   - Adjust parameters based on real-world data
   - Update simulation with findings
   - Re-test in sim

4. **Scale to Complex Environments**
   - Gradually increase difficulty
   - Monitor performance degradation

---

## Case Study: Navigation Algorithm

### Simulation Development

```python title="sim_navigation.py"
#!/usr/bin/env python3
"""
Navigation controller developed in simulation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class SimNavigator(Node):
    """Simple obstacle avoidance navigator."""

    def __init__(self):
        super().__init__('sim_navigator')

        # Parameters (tuned in simulation)
        self.declare_parameter('obstacle_threshold', 1.0)  # meters
        self.declare_parameter('forward_speed', 0.3)       # m/s
        self.declare_parameter('turn_speed', 0.5)          # rad/s

        self.obstacle_threshold = self.get_parameter(
            'obstacle_threshold'
        ).value
        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value

        # Publishers/Subscribers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        self.get_logger().info('Navigator started')

    def scan_callback(self, msg: LaserScan):
        """Process laser scan and navigate."""
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max

        # Check front sector
        front_ranges = ranges[len(ranges)//3 : 2*len(ranges)//3]
        min_front_dist = np.min(front_ranges)

        cmd = Twist()

        if min_front_dist > self.obstacle_threshold:
            # Clear ahead - move forward
            cmd.linear.x = self.forward_speed
        else:
            # Obstacle ahead - turn
            # Turn towards open space
            left_dist = np.min(ranges[:len(ranges)//3])
            right_dist = np.min(ranges[2*len(ranges)//3:])

            if left_dist > right_dist:
                cmd.angular.z = self.turn_speed  # Turn left
            else:
                cmd.angular.z = -self.turn_speed  # Turn right

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    navigator = SimNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Real-World Deployment

```yaml title="real_params.yaml"
# Adjusted parameters for real robot
sim_navigator:
  ros__parameters:
    obstacle_threshold: 1.2  # More conservative (was 1.0)
    forward_speed: 0.2       # Slower (was 0.3)
    turn_speed: 0.4          # Gentler (was 0.5)
```

**Launch with real parameters**:
```bash
ros2 run my_package sim_navigator --ros-args --params-file real_params.yaml
```

---

## Best Practices

1. **Start Simple**
   - Validate basic behaviors first
   - Gradually increase complexity

2. **Measure Everything**
   - Log all sensor data
   - Record success/failure cases
   - Compare sim and real quantitatively

3. **Safety First**
   - Always have emergency stop
   - Start with low speeds
   - Test in controlled environments

4. **Iterate**
   - Use real-world data to improve simulation
   - Continuous sim-to-real loop

5. **Document**
   - Record all parameter changes
   - Note failure modes
   - Track performance metrics

---

## Self-Assessment

### Questions

1. **What is the reality gap?**
   <details>
   <summary>Answer</summary>
   The discrepancy between simulated and real-world robot behavior due to imperfect modeling of physics, sensors, and environment.
   </details>

2. **Why use domain randomization?**
   <details>
   <summary>Answer</summary>
   It exposes the algorithm to a wide range of scenarios, making it more robust to real-world variations and uncertainties.
   </details>

3. **What should you measure first when validating sim-to-real transfer?**
   <details>
   <summary>Answer</summary>
   Basic kinematic parameters (wheel radius, robot dimensions) and sensor calibration (camera intrinsics, LiDAR accuracy).
   </details>

### Exercises

1. **System Identification**: Run the forward motion test on a real robot and calculate the wheel radius correction factor

2. **Performance Comparison**: Implement the performance validator and run 10 navigation trials in both sim and real

3. **Parameter Tuning**: Deploy a navigation algorithm from sim to real, measure the performance gap, and tune parameters to close it

4. **Domain Randomization**: Add friction randomization to your Gazebo world and test if your algorithm becomes more robust

---

## Next Steps

Congratulations on completing Module 2! You now understand simulation and sim-to-real transfer. Proceed to:

👉 [Module 3: NVIDIA Isaac](../module-3/01-isaac-sim-intro.md) to learn AI-accelerated robotics

---

## Additional Resources

- [OpenAI Domain Randomization](https://arxiv.org/abs/1703.06907)
- [Sim-to-Real Transfer in Robotics](https://arxiv.org/abs/1812.07252)
- [ROS 2 Parameter Tuning](https://docs.ros.org/en/humble/Tutorials/Parameters.html)
- [System Identification Techniques](https://arxiv.org/abs/1912.10612)

---

**Key Takeaway**: Successful sim-to-real transfer requires careful calibration, domain randomization, and iterative validation. Mastering this process enables rapid development in simulation with reliable real-world deployment.
