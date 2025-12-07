---
sidebar_position: 4
---

# End-to-End VLA Policies

Master end-to-end Vision-Language-Action policies that directly map sensory inputs and language commands to robot actions.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand end-to-end VLA architectures
- Collect and prepare training data
- Train VLA policies with behavioral cloning
- Deploy policies to real robots
- Implement safety mechanisms
- Build complete Physical AI systems

---

## End-to-End Learning

**Traditional Pipeline**:
```
Image → Detection → Segmentation → Planning → Control → Actions
  ↓         ↓            ↓            ↓          ↓         ↓
 Error    Error        Error        Error     Error    Compounds
```

**End-to-End VLA**:
```
[Image + Language Command] → VLA Model → Actions
                                            ↓
                                   Direct optimization
```

### Advantages

- **Simplicity**: One model vs. multiple components
- **Optimization**: End-to-end gradient flow
- **Generalization**: Learns latent representations
- **Robustness**: Doesn't rely on intermediate outputs

---

## Data Collection

### Teleoperation Setup

```python title="teleop_collector.py"
#!/usr/bin/env python3
"""
Collect demonstrations via teleoperation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image, JointState
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import h5py
import numpy as np
from datetime import datetime
import cv2


class TeleopCollector(Node):
    """Collect robot demonstrations for VLA training."""

    def __init__(self):
        super().__init__('teleop_collector')

        # Storage
        self.current_episode = {
            'observations': {
                'images': [],
                'joint_positions': [],
                'joint_velocities': [],
            },
            'actions': [],
            'language_command': '',
            'timestamps': []
        }

        self.bridge = CvBridge()
        self.recording = False
        self.episode_count = 0

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )

        # Latest observations
        self.latest_image = None
        self.latest_joints = None

        # Timer for data collection (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.collect_step)

        self.get_logger().info('Teleop Collector ready. Press START to begin episode.')

    def image_callback(self, msg: Image):
        """Store latest camera image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def joint_callback(self, msg: JointState):
        """Store latest joint state."""
        self.latest_joints = {
            'positions': np.array(msg.position),
            'velocities': np.array(msg.velocity),
            'names': msg.name
        }

    def joy_callback(self, msg: Joy):
        """Handle joystick commands."""
        # START button: Begin/end episode
        if msg.buttons[7] == 1:  # START button on PS4 controller
            if not self.recording:
                command = input("Enter language command for this episode: ")
                self.start_episode(command)
            else:
                success = input("Was episode successful? (y/n): ").lower() == 'y'
                self.stop_episode(success)

    def collect_step(self):
        """Collect one timestep of data."""
        if not self.recording:
            return

        if self.latest_image is None or self.latest_joints is None:
            return

        # Store observation
        self.current_episode['observations']['images'].append(
            cv2.resize(self.latest_image, (224, 224))  # Resize for model
        )
        self.current_episode['observations']['joint_positions'].append(
            self.latest_joints['positions']
        )
        self.current_episode['observations']['joint_velocities'].append(
            self.latest_joints['velocities']
        )

        # Store action (current joint velocities as action)
        self.current_episode['actions'].append(
            self.latest_joints['velocities']
        )

        # Store timestamp
        self.current_episode['timestamps'].append(
            self.get_clock().now().nanoseconds / 1e9
        )

    def start_episode(self, command: str):
        """Start recording episode."""
        self.current_episode = {
            'observations': {
                'images': [],
                'joint_positions': [],
                'joint_velocities': [],
            },
            'actions': [],
            'language_command': command,
            'timestamps': []
        }

        self.recording = True
        self.get_logger().info(f'Recording episode: "{command}"')

    def stop_episode(self, success: bool):
        """Stop recording and save episode."""
        self.recording = False

        # Add metadata
        self.current_episode['success'] = success
        self.current_episode['episode_id'] = self.episode_count

        # Save to disk
        self.save_episode()

        self.episode_count += 1
        self.get_logger().info(f'Episode {self.episode_count} saved.')

    def save_episode(self):
        """Save episode to HDF5."""
        filename = f'episode_{self.episode_count:04d}.hdf5'

        with h5py.File(filename, 'w') as f:
            # Observations
            obs_group = f.create_group('observations')
            obs_group.create_dataset(
                'images',
                data=np.array(self.current_episode['observations']['images'])
            )
            obs_group.create_dataset(
                'joint_positions',
                data=np.array(self.current_episode['observations']['joint_positions'])
            )

            # Actions
            f.create_dataset(
                'actions',
                data=np.array(self.current_episode['actions'])
            )

            # Metadata
            f.attrs['language_command'] = self.current_episode['language_command']
            f.attrs['success'] = self.current_episode['success']
            f.attrs['duration'] = len(self.current_episode['timestamps'])

        self.get_logger().info(f'Saved {filename}')


def main(args=None):
    rclpy.init(args=args)
    collector = TeleopCollector()
    rclpy.spin(collector)
    collector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Training VLA Policy

### Model Architecture

```python title="vla_model.py"
"""
VLA Model: Vision-Language-Action Policy
"""

import torch
import torch.nn as nn
from transformers import CLIPVisionModel, BertModel


class VLAPolicy(nn.Module):
    """End-to-end VLA policy network."""

    def __init__(
        self,
        action_dim: int = 7,  # Robot DOF
        hidden_dim: int = 512,
        num_layers: int = 4
    ):
        super().__init__()

        # Vision encoder (CLIP)
        self.vision_encoder = CLIPVisionModel.from_pretrained("openai/clip-vit-base-patch32")
        vision_dim = 768

        # Language encoder (BERT)
        self.language_encoder = BertModel.from_pretrained("bert-base-uncased")
        language_dim = 768

        # Fusion layer
        self.fusion = nn.Linear(vision_dim + language_dim, hidden_dim)

        # Transformer for temporal modeling
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=hidden_dim,
            nhead=8,
            dim_feedforward=2048,
            dropout=0.1,
            batch_first=True
        )
        self.transformer = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        # Action head
        self.action_head = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(hidden_dim, action_dim)
        )

    def forward(self, images, language_tokens, attention_mask):
        """
        Args:
            images: (batch, seq_len, 3, 224, 224)
            language_tokens: (batch, seq_len_lang)
            attention_mask: (batch, seq_len_lang)

        Returns:
            actions: (batch, seq_len, action_dim)
        """
        batch_size, seq_len = images.shape[:2]

        # Encode images
        images_flat = images.view(-1, *images.shape[2:])  # (batch*seq_len, 3, 224, 224)
        vision_features = self.vision_encoder(pixel_values=images_flat).pooler_output
        vision_features = vision_features.view(batch_size, seq_len, -1)

        # Encode language (once per batch)
        language_output = self.language_encoder(
            input_ids=language_tokens,
            attention_mask=attention_mask
        )
        language_features = language_output.pooler_output  # (batch, 768)

        # Expand language to match sequence length
        language_features = language_features.unsqueeze(1).expand(-1, seq_len, -1)

        # Fuse vision and language
        fused = torch.cat([vision_features, language_features], dim=-1)
        fused = self.fusion(fused)  # (batch, seq_len, hidden_dim)

        # Temporal modeling
        temporal_features = self.transformer(fused)

        # Predict actions
        actions = self.action_head(temporal_features)

        return actions
```

### Training Loop

```python title="train_vla.py"
"""
Train VLA policy with behavioral cloning
"""

import torch
from torch.utils.data import Dataset, DataLoader
import h5py
import glob
from transformers import BertTokenizer
import numpy as np


class VLADataset(Dataset):
    """Dataset for VLA training."""

    def __init__(self, data_dir, tokenizer, seq_len=10):
        self.data_files = glob.glob(f"{data_dir}/*.hdf5")
        self.tokenizer = tokenizer
        self.seq_len = seq_len

        # Load all episodes
        self.episodes = []
        for file in self.data_files:
            with h5py.File(file, 'r') as f:
                self.episodes.append({
                    'images': f['observations/images'][:],
                    'actions': f['actions'][:],
                    'language': f.attrs['language_command']
                })

    def __len__(self):
        return len(self.episodes)

    def __getitem__(self, idx):
        episode = self.episodes[idx]

        # Sample sequence
        ep_len = len(episode['images'])
        if ep_len < self.seq_len:
            # Pad short episodes
            images = np.pad(
                episode['images'],
                ((0, self.seq_len - ep_len), (0, 0), (0, 0), (0, 0)),
                mode='edge'
            )
            actions = np.pad(
                episode['actions'],
                ((0, self.seq_len - ep_len), (0, 0)),
                mode='edge'
            )
        else:
            # Random crop
            start_idx = np.random.randint(0, ep_len - self.seq_len + 1)
            images = episode['images'][start_idx:start_idx + self.seq_len]
            actions = episode['actions'][start_idx:start_idx + self.seq_len]

        # Tokenize language
        tokens = self.tokenizer(
            episode['language'],
            padding='max_length',
            max_length=32,
            truncation=True,
            return_tensors='pt'
        )

        return {
            'images': torch.from_numpy(images).float() / 255.0,  # Normalize
            'actions': torch.from_numpy(actions).float(),
            'language_tokens': tokens['input_ids'].squeeze(0),
            'attention_mask': tokens['attention_mask'].squeeze(0)
        }


def train_vla(data_dir, epochs=100, batch_size=8):
    """Train VLA policy."""
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    # Create dataset
    tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')
    dataset = VLADataset(data_dir, tokenizer)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    # Create model
    model = VLAPolicy(action_dim=7).to(device)

    # Optimizer
    optimizer = torch.optim.AdamW(model.parameters(), lr=1e-4, weight_decay=0.01)

    # Loss
    criterion = nn.MSELoss()

    # Training loop
    for epoch in range(epochs):
        total_loss = 0

        for batch in dataloader:
            images = batch['images'].to(device)
            actions = batch['actions'].to(device)
            language_tokens = batch['language_tokens'].to(device)
            attention_mask = batch['attention_mask'].to(device)

            # Forward pass
            predicted_actions = model(images, language_tokens, attention_mask)

            # Compute loss
            loss = criterion(predicted_actions, actions)

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        avg_loss = total_loss / len(dataloader)
        print(f"Epoch {epoch + 1}/{epochs}, Loss: {avg_loss:.4f}")

        # Save checkpoint
        if (epoch + 1) % 10 == 0:
            torch.save(model.state_dict(), f'vla_policy_epoch_{epoch + 1}.pth')

    print("Training complete!")


if __name__ == '__main__':
    train_vla('demonstrations/', epochs=100, batch_size=8)
```

---

## Deployment

### Real-Time Inference Node

```python title="vla_controller.py"
#!/usr/bin/env python3
"""
Deploy trained VLA policy for robot control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import torch
from collections import deque
import numpy as np
from vla_model import VLAPolicy
from transformers import BertTokenizer


class VLAController(Node):
    """Real-time VLA policy inference."""

    def __init__(self):
        super().__init__('vla_controller')

        # Load trained model
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.model = VLAPolicy(action_dim=7).to(self.device)
        self.model.load_state_dict(torch.load('vla_policy_final.pth'))
        self.model.eval()

        self.tokenizer = BertTokenizer.from_pretrained('bert-base-uncased')
        self.bridge = CvBridge()

        # History buffer
        self.image_history = deque(maxlen=10)
        self.current_command = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        # Publishers
        self.action_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        # Control loop (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.control_loop)

        self.get_logger().info('VLA Controller ready')

    def image_callback(self, msg: Image):
        """Store camera images."""
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        self.image_history.append(image)

    def command_callback(self, msg: String):
        """Receive new language command."""
        self.current_command = msg.data
        self.get_logger().info(f'New command: "{self.current_command}"')

    def control_loop(self):
        """Run VLA inference and publish actions."""
        if len(self.image_history) < 10 or self.current_command is None:
            return

        # Prepare inputs
        images = np.array(list(self.image_history))
        images = torch.from_numpy(images).float() / 255.0
        images = images.unsqueeze(0).to(self.device)  # (1, 10, H, W, 3)

        # Tokenize command
        tokens = self.tokenizer(
            self.current_command,
            padding='max_length',
            max_length=32,
            truncation=True,
            return_tensors='pt'
        ).to(self.device)

        # Run inference
        with torch.no_grad():
            actions = self.model(
                images,
                tokens['input_ids'],
                tokens['attention_mask']
            )

        # Get latest action
        action = actions[0, -1, :].cpu().numpy()

        # Publish action
        self.publish_action(action)

    def publish_action(self, action):
        """Publish joint trajectory."""
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']

        point = JointTrajectoryPoint()
        point.positions = action.tolist()
        point.time_from_start.nanosec = int(0.033 * 1e9)  # 30ms

        trajectory.points.append(point)

        self.action_pub.publish(trajectory)


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

## Safety and Monitoring

### Action Bounds

```python
def clip_actions(action, bounds):
    """Clip actions to safe ranges."""
    return np.clip(action, bounds['lower'], bounds['upper'])

# Define bounds
JOINT_BOUNDS = {
    'lower': np.array([-3.14, -1.57, -3.14, -1.57, -3.14, -1.57, 0.0]),
    'upper': np.array([3.14, 1.57, 3.14, 1.57, 3.14, 1.57, 0.08])
}

# Apply
safe_action = clip_actions(predicted_action, JOINT_BOUNDS)
```

### Emergency Stop

```python
class SafetyMonitor(Node):
    """Monitor robot state for safety violations."""

    def __init__(self):
        super().__init__('safety_monitor')

        self.emergency_stop = False

        # Check joint limits
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.check_limits, 10
        )

        # Emergency stop publisher
        self.estop_pub = self.create_publisher(String, '/emergency_stop', 10)

    def check_limits(self, msg: JointState):
        """Check for safety violations."""
        for position in msg.position:
            if abs(position) > 3.5:  # Near joint limit
                self.trigger_emergency_stop("Joint limit exceeded")

        for velocity in msg.velocity:
            if abs(velocity) > 2.0:  # Too fast
                self.trigger_emergency_stop("Velocity limit exceeded")

    def trigger_emergency_stop(self, reason: str):
        """Trigger emergency stop."""
        if not self.emergency_stop:
            self.emergency_stop = True
            self.get_logger().error(f'EMERGENCY STOP: {reason}')

            msg = String()
            msg.data = reason
            self.estop_pub.publish(msg)
```

---

## Evaluation Metrics

```python
def evaluate_policy(model, test_episodes):
    """Evaluate VLA policy."""
    success_count = 0
    total_reward = 0

    for episode in test_episodes:
        # Run episode
        final_state = simulate_episode(model, episode)

        # Check success
        if is_goal_reached(final_state, episode['goal']):
            success_count += 1

        total_reward += compute_reward(final_state)

    success_rate = success_count / len(test_episodes)
    avg_reward = total_reward / len(test_episodes)

    print(f"Success Rate: {success_rate * 100:.1f}%")
    print(f"Average Reward: {avg_reward:.2f}")

    return success_rate, avg_reward
```

---

## Best Practices

1. **Data Collection**
   - Collect diverse demonstrations (100+ per task)
   - Include failure cases for robust learning
   - Vary lighting, objects, backgrounds

2. **Training**
   - Start with imitation learning (BC)
   - Add data augmentation (color jitter, crops)
   - Use curriculum learning (easy → hard tasks)

3. **Deployment**
   - Test thoroughly in simulation first
   - Implement safety monitors
   - Start with low speeds
   - Human supervision initially

4. **Monitoring**
   - Log all predictions and actions
   - Track success rates
   - Retrain with failure cases

---

## Self-Assessment

### Questions

1. **What's the advantage of end-to-end learning?**
   <details>
   <summary>Answer</summary>
   End-to-end learning optimizes the entire pipeline together, avoiding error accumulation from separate modules and learning better latent representations.
   </details>

2. **Why use behavioral cloning instead of RL?**
   <details>
   <summary>Answer</summary>
   BC is sample-efficient (learns from expert demonstrations) and doesn't require reward engineering. However, it can't improve beyond demonstrations.
   </details>

3. **What are the key safety considerations for VLA deployment?**
   <details>
   <summary>Answer</summary>
   Action bounds, velocity limits, emergency stops, human supervision, and thorough testing in simulation before real-world deployment.
   </details>

### Exercises

1. **Collect Data**: Teleoperate robot for 3 tasks, 20 demos each

2. **Train Policy**: Train VLA model on collected data for 100 epochs

3. **Deploy and Test**: Deploy policy to simulated robot and measure success rate

4. **Safety**: Implement action clipping and emergency stop

5. **Evaluation**: Compare VLA policy vs. scripted baseline

---

## Next Steps

**Congratulations! You've completed the Physical AI & Humanoid Robotics course!**

You now have the knowledge to:
- Build ROS 2 robot systems
- Simulate robots in Gazebo and Unity
- Use NVIDIA Isaac for AI-powered robotics
- Deploy Vision-Language-Action models

**Continue Learning**:
- Join robotics communities (ROS Discourse, NVIDIA Forums)
- Contribute to open-source robotics projects
- Build your own Physical AI project for the capstone!

---

## Additional Resources

- [RT-1 Implementation](https://github.com/google-research/robotics_transformer)
- [OpenVLA](https://openvla.github.io/)
- [PyTorch Robotics](https://pytorch.org/blog/accelerating-robotics-research/)
- [Hugging Face Robotics](https://huggingface.co/blog/deep-rl-huggingface)

---

**Key Takeaway**: End-to-end VLA policies represent the frontier of robot learning, enabling robots to learn complex behaviors from demonstrations and generalize to new tasks. This is the path to truly autonomous, adaptive robots.

**Thank you for completing this journey into Physical AI!** 🤖✨
