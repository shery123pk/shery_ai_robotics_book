---
sidebar_position: 1
---

# Introduction to Vision-Language-Action

Learn Vision-Language-Action (VLA) models - the future of robot control where natural language instructions directly produce robot actions.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand what VLA models are and why they matter
- Explain the evolution from scripted robots to VLA
- Identify VLA architectures (RT-1, RT-2, PaLM-E)
- Set up infrastructure for VLA experiments
- Understand the training data requirements
- Deploy simple VLA policies

---

## What is VLA?

**Vision-Language-Action (VLA)** models are AI systems that:

1. **See** the environment (vision encoders)
2. **Understand** natural language commands (language models)
3. **Act** by generating robot control actions (action decoders)

```
┌────────────┐     ┌──────────────┐     ┌───────────┐
│   Camera   │ ──> │              │     │   Robot   │
│   Images   │     │  VLA Model   │ ──> │  Actions  │
└────────────┘     │              │     │ (joints,  │
                   │              │     │  gripper) │
┌────────────┐     │              │     └───────────┘
│  Language  │ ──> │              │
│  "Pick cup"│     └──────────────┘
└────────────┘
```

### Why VLA?

**Traditional Robotics**:
```
Human programs robot → Robot follows exact steps
```

**VLA Robotics**:
```
Human gives high-level instruction → Robot figures out how to do it
```

**Benefits**:
- **Generalization**: One model, many tasks
- **Flexibility**: Natural language interface
- **Sample Efficiency**: Learns from demonstrations
- **Scalability**: Benefits from large datasets

---

## Evolution of Robot Control

### 1. Hardcoded (1960s-2000s)

```python
def pick_object():
    move_to([10, 20, 30])
    close_gripper()
    move_to([15, 25, 35])
```

**Pros**: Reliable for known tasks
**Cons**: Brittle, requires reprogramming for each new task

### 2. Classical Planning (2000s-2010s)

```python
goal = "object_in_bin"
planner.plan(start_state, goal_state)
```

**Pros**: More flexible
**Cons**: Requires perfect world model, struggles with uncertainty

### 3. Reinforcement Learning (2010s)

```python
# Learn policy from trials
for episode in range(1000000):
    action = policy(state)
    reward = environment.step(action)
    update_policy(reward)
```

**Pros**: Learns from experience
**Cons**: Sample inefficient, requires millions of trials

### 4. Vision-Language-Action (2020s)

```python
# One model, infinite tasks
command = "Pick up the red cup"
image = camera.capture()
action = vla_model(image, command)
robot.execute(action)
```

**Pros**: Generalizes across tasks, natural interface
**Cons**: Requires large datasets, compute-intensive

---

## VLA Architectures

### RT-1 (Robotics Transformer 1)

Google's first production VLA model (2022).

**Architecture**:
```
Images (300x300x3 @ 6fps)
    │
    ▼
Vision Encoder (EfficientNet-B3)
    │
    ▼
Tokenization
    │
    ├──> Language Encoder (BERT-like) <── "pick apple"
    │
    ▼
Transformer (8 layers)
    │
    ▼
Action Decoder
    │
    ▼
Actions (7-DOF arm + gripper)
```

**Training Data**: 130k robot demonstrations
**Tasks**: 700+ real-world manipulation tasks
**Performance**: 97% success on trained tasks

### RT-2 (Robotics Transformer 2)

Combines VLA with vision-language models (2023).

**Key Innovation**: Pre-training on web-scale data

```
PaLI-X (Vision-Language Model)
    │
    ├─ Pre-trained on web images + captions
    │
    ▼
Fine-tuned with robot data
    │
    ▼
RT-2 Model
```

**Performance**:
- Generalizes to novel objects (unseen during robot training)
- 3x better than RT-1 on unseen tasks

### PaLM-E (Embodied Language Model)

Google's 562B parameter embodied AI (2023).

**Architecture**:
```
PaLM-540B (Language Model)
    │
    ├──> Vision Encoder
    ├──> Sensor Fusion
    ├──> State Estimator
    │
    ▼
Embodied Reasoning
    │
    ▼
Robot Actions
```

**Capabilities**:
- Long-horizon planning
- Multi-modal reasoning
- Sim-to-real transfer

---

## VLA Pipeline

### 1. Data Collection

Collect robot demonstrations:

```python title="collect_demonstrations.py"
#!/usr/bin/env python3
"""
Collect teleoperation demonstrations
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
import h5py
import numpy as np
from datetime import datetime


class DemonstrationCollector(Node):
    """Collect robot demonstrations for VLA training."""

    def __init__(self):
        super().__init__('demonstration_collector')

        # Storage
        self.episodes = []
        self.current_episode = {
            'images': [],
            'actions': [],
            'language': ''
        }

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.recording = False

        self.get_logger().info('Demonstration Collector ready')

    def start_episode(self, language_command: str):
        """Start recording a new episode."""
        self.current_episode = {
            'images': [],
            'actions': [],
            'language': language_command,
            'rewards': []
        }
        self.recording = True

        self.get_logger().info(f'Started episode: "{language_command}"')

    def image_callback(self, msg: Image):
        """Store camera images."""
        if not self.recording:
            return

        # Convert to numpy (simplified)
        # In practice, use cv_bridge
        image_array = np.frombuffer(msg.data, dtype=np.uint8)
        image_array = image_array.reshape((msg.height, msg.width, 3))

        self.current_episode['images'].append(image_array)

    def joint_callback(self, msg: JointState):
        """Store joint actions."""
        if not self.recording:
            return

        action = {
            'positions': np.array(msg.position),
            'velocities': np.array(msg.velocity),
            'efforts': np.array(msg.effort)
        }

        self.current_episode['actions'].append(action)

    def stop_episode(self, success: bool):
        """Stop recording and save episode."""
        self.recording = False

        # Add success/failure label
        self.current_episode['success'] = success

        # Store episode
        self.episodes.append(self.current_episode.copy())

        self.get_logger().info(
            f'Stopped episode. Total episodes: {len(self.episodes)}'
        )

    def save_dataset(self, filename: str):
        """Save all episodes to HDF5."""
        with h5py.File(filename, 'w') as f:
            for idx, episode in enumerate(self.episodes):
                group = f.create_group(f'episode_{idx}')

                # Save images
                images = np.array(episode['images'])
                group.create_dataset('images', data=images)

                # Save actions
                actions = np.array([a['positions'] for a in episode['actions']])
                group.create_dataset('actions', data=actions)

                # Save language
                group.attrs['language'] = episode['language']
                group.attrs['success'] = episode['success']

        self.get_logger().info(f'Saved {len(self.episodes)} episodes to {filename}')


def main(args=None):
    rclpy.init(args=args)
    collector = DemonstrationCollector()

    # Example: Collect 3 episodes
    collector.start_episode("Pick up the red block")
    # ... teleoperate robot ...
    rclpy.spin_once(collector, timeout_sec=10.0)  # Record for 10s
    collector.stop_episode(success=True)

    collector.start_episode("Place block in bin")
    rclpy.spin_once(collector, timeout_sec=10.0)
    collector.stop_episode(success=True)

    # Save dataset
    collector.save_dataset('demonstrations.hdf5')

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2. Training

Train VLA model on collected data (covered in detail in Chapter 4).

### 3. Deployment

Deploy trained model to robot:

```python title="vla_inference.py"
#!/usr/bin/env python3
"""
Deploy VLA model for inference
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import torch
import numpy as np


class VLAController(Node):
    """VLA model inference for robot control."""

    def __init__(self):
        super().__init__('vla_controller')

        # Load model (placeholder - actual model loading varies)
        self.model = self.load_vla_model('/path/to/model.pth')

        # Current observation
        self.latest_image = None
        self.language_command = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        # Publisher
        self.action_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory', 10
        )

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('VLA Controller started')

    def load_vla_model(self, model_path: str):
        """Load pre-trained VLA model."""
        # Placeholder - actual loading depends on framework
        model = torch.load(model_path)
        model.eval()
        return model

    def image_callback(self, msg: Image):
        """Store latest image."""
        # Convert to tensor (simplified)
        self.latest_image = msg  # In practice, convert to tensor

    def command_callback(self, msg: String):
        """Receive language command."""
        self.language_command = msg.data
        self.get_logger().info(f'Command: "{self.language_command}"')

    def control_loop(self):
        """Run VLA inference and publish actions."""
        if self.latest_image is None or self.language_command is None:
            return

        # Prepare inputs (simplified)
        # In practice: preprocess image, tokenize language
        image_tensor = self.preprocess_image(self.latest_image)
        language_tensor = self.tokenize_command(self.language_command)

        # Run inference
        with torch.no_grad():
            action = self.model(image_tensor, language_tensor)

        # Convert to ROS message
        trajectory = self.action_to_trajectory(action)

        # Publish
        self.action_pub.publish(trajectory)

    def preprocess_image(self, image_msg):
        """Preprocess image for model input."""
        # Placeholder
        return torch.randn(1, 3, 224, 224)

    def tokenize_command(self, command: str):
        """Tokenize language command."""
        # Placeholder
        return torch.randint(0, 1000, (1, 20))

    def action_to_trajectory(self, action):
        """Convert model action to joint trajectory."""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']

        point = JointTrajectoryPoint()
        point.positions = action.cpu().numpy().tolist()
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000  # 0.1s

        trajectory.points.append(point)

        return trajectory


def main(args=None):
    rclpy.init(args=args)
    controller = VLAController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Data Requirements

### Dataset Sizes

| Model | Demonstrations | Diversity |
|-------|----------------|-----------|
| **RT-1** | 130,000 | 700 tasks, 13 robots |
| **RT-2** | 130,000 robot + Web images | 10,000+ objects |
| **PaLM-E** | Robot data + Language pre-training | Cross-modal |

### Data Quality

**Critical factors**:
- **Visual diversity**: Different lighting, backgrounds, camera angles
- **Task diversity**: Varied objects, goals, scenarios
- **Language diversity**: Different phrasings for same task
- **Success labeling**: Accurate task completion labels

---

## Challenges

1. **Data Scarcity**: Real robot data is expensive
   - **Solution**: Sim-to-real transfer, data augmentation

2. **Computational Cost**: Large models require GPUs
   - **Solution**: Model distillation, quantization

3. **Safety**: Model may hallucinate unsafe actions
   - **Solution**: Safety layers, human oversight

4. **Latency**: Inference can be slow
   - **Solution**: Edge deployment (Jetson), model optimization

---

## Self-Assessment

### Questions

1. **What are the three components of a VLA model?**
   <details>
   <summary>Answer</summary>
   Vision encoder (images), language encoder (commands), action decoder (robot controls).
   </details>

2. **How does RT-2 improve upon RT-1?**
   <details>
   <summary>Answer</summary>
   RT-2 pre-trains on web-scale vision-language data, enabling better generalization to novel objects and tasks.
   </details>

3. **What's the main advantage of VLA over traditional programming?**
   <details>
   <summary>Answer</summary>
   VLA models can generalize to new tasks and objects without reprogramming, using natural language instructions.
   </details>

### Exercises

1. **Collect Data**: Teleoperate a robot and collect 10 demonstrations for a single task

2. **Dataset Analysis**: Load an existing VLA dataset (e.g., Open X-Embodiment) and analyze image diversity

3. **Simple Inference**: Deploy a pre-trained VLA model (if available) and test with simple commands

4. **Data Augmentation**: Implement color jittering and random cropping for robot images

---

## Next Steps

Now that you understand VLA fundamentals, proceed to:

👉 [Chapter 2: LLMs for Robotics](./02-llm-robotics.md) to learn how to integrate language models with robots

---

## Additional Resources

- [RT-1 Paper](https://arxiv.org/abs/2212.06817)
- [RT-2 Paper](https://arxiv.org/abs/2307.15818)
- [PaLM-E Paper](https://arxiv.org/abs/2303.03378)
- [Open X-Embodiment Dataset](https://robotics-transformer-x.github.io/)

---

**Key Takeaway**: VLA models represent the future of robot control, enabling robots to understand natural language and perform complex tasks through learned policies. Mastering VLA is essential for building truly intelligent, adaptive robots.
