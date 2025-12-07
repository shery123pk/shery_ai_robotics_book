---
sidebar_position: 2
---

# LLMs for Robotics

Learn to integrate Large Language Models (LLMs) with robot systems for high-level planning, reasoning, and natural human-robot interaction.

## Learning Objectives

By the end of this chapter, you will be able to:

- Integrate GPT-4/Claude with ROS 2
- Build voice-controlled robots with Whisper
- Implement LLM-based task planning
- Use LLMs for code generation
- Handle multimodal inputs (text + images)
- Deploy LLM-powered robots safely

---

## Why LLMs for Robotics?

**Traditional Robot Programming**:
```python
if command == "go forward":
    robot.move_forward()
elif command == "turn left":
    robot.turn_left()
# ... 1000 more if-else statements
```

**LLM-Powered Robot**:
```python
command = "Navigate to the kitchen and bring me a cup"
plan = llm.generate_plan(command, environment_context)
execute_plan(plan)
```

### Capabilities

| Task | Traditional | LLM-Powered |
|------|-------------|-------------|
| **Understanding** | Keywords only | Natural language |
| **Planning** | Pre-programmed | Dynamic reasoning |
| **Adaptation** | Brittle | Context-aware |
| **Explanation** | None | Can explain actions |
| **Learning** | Requires retraining | Few-shot learning |

---

## Architecture

```
┌──────────────┐
│ Voice Input  │ ──> Whisper (Speech-to-Text)
└──────────────┘
        │
        ▼
┌──────────────┐
│ Text Command │ ──> GPT-4 (Task Planning)
└──────────────┘
        │
        ▼
┌──────────────┐
│  Action Plan │ ──> ROS 2 Nodes (Execution)
└──────────────┘
        │
        ▼
┌──────────────┐
│    Robot     │
└──────────────┘
```

---

## OpenAI GPT-4 Integration

### Setup

```bash
# Install OpenAI Python SDK
pip install openai

# Set API key
export OPENAI_API_KEY='your-api-key-here'
```

### Basic LLM Node

```python title="llm_planner.py"
#!/usr/bin/env python3
"""
LLM-based task planner using GPT-4
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import json


class LLMPlanner(Node):
    """Use GPT-4 for high-level task planning."""

    def __init__(self):
        super().__init__('llm_planner')

        # Initialize OpenAI client
        self.client = OpenAI()

        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/voice_command',
            self.command_callback,
            10
        )

        # Publishers
        self.plan_pub = self.create_publisher(
            String,
            '/task_plan',
            10
        )

        # System prompt
        self.system_prompt = """
        You are a robot task planner. Given a natural language command,
        break it down into executable ROS 2 actions.

        Available actions:
        - navigate_to(location): Move to a location
        - pick_object(object_name): Pick up an object
        - place_object(location): Place held object
        - open_gripper(): Open gripper
        - close_gripper(): Close gripper
        - scan_environment(): Look around

        Respond with a JSON list of actions.
        Example: [{"action": "navigate_to", "params": {"location": "kitchen"}}]
        """

        self.get_logger().info('LLM Planner ready')

    def command_callback(self, msg: String):
        """Process natural language command."""
        command = msg.data
        self.get_logger().info(f'Command: "{command}"')

        # Query GPT-4
        plan = self.generate_plan(command)

        # Publish plan
        if plan:
            plan_msg = String()
            plan_msg.data = json.dumps(plan)
            self.plan_pub.publish(plan_msg)

            self.get_logger().info(f'Plan: {plan}')

    def generate_plan(self, command: str) -> list:
        """Generate action plan using GPT-4."""
        try:
            response = self.client.chat.completions.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.3,  # Lower for consistent planning
                max_tokens=500
            )

            # Extract plan
            content = response.choices[0].message.content
            plan = json.loads(content)

            return plan

        except Exception as e:
            self.get_logger().error(f'LLM error: {e}')
            return []


def main(args=None):
    rclpy.init(args=args)
    planner = LLMPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Test

```bash
# Terminal 1: Run planner
ros2 run llm_control llm_planner

# Terminal 2: Send command
ros2 topic pub /voice_command std_msgs/msg/String \
  "{data: 'Go to the kitchen and pick up the red cup'}"
```

**Expected Output**:
```json
[
  {"action": "navigate_to", "params": {"location": "kitchen"}},
  {"action": "scan_environment", "params": {}},
  {"action": "pick_object", "params": {"object_name": "red cup"}}
]
```

---

## Voice Control with Whisper

### Setup Whisper

```bash
# Install Whisper
pip install openai-whisper

# Install PyAudio for microphone
pip install pyaudio
```

### Voice Command Node

```python title="voice_commander.py"
#!/usr/bin/env python3
"""
Voice command interface using Whisper
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import whisper
import pyaudio
import wave
import numpy as np


class VoiceCommander(Node):
    """Capture voice and convert to text using Whisper."""

    def __init__(self):
        super().__init__('voice_commander')

        # Load Whisper model
        self.model = whisper.load_model("base")  # Options: tiny, base, small, medium, large

        # Audio settings
        self.sample_rate = 16000
        self.chunk_size = 1024
        self.channels = 1

        # Publisher
        self.command_pub = self.create_publisher(
            String,
            '/voice_command',
            10
        )

        # PyAudio
        self.audio = pyaudio.PyAudio()

        self.get_logger().info('Voice Commander ready. Press Enter to record...')

    def record_audio(self, duration: int = 5):
        """Record audio from microphone."""
        stream = self.audio.open(
            format=pyaudio.paInt16,
            channels=self.channels,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.chunk_size
        )

        self.get_logger().info(f'Recording for {duration} seconds...')

        frames = []
        for _ in range(0, int(self.sample_rate / self.chunk_size * duration)):
            data = stream.read(self.chunk_size)
            frames.append(data)

        stream.stop_stream()
        stream.close()

        # Convert to numpy array
        audio_data = np.frombuffer(b''.join(frames), dtype=np.int16)
        audio_data = audio_data.astype(np.float32) / 32768.0

        return audio_data

    def transcribe(self, audio_data):
        """Transcribe audio using Whisper."""
        result = self.model.transcribe(audio_data)
        return result['text']

    def run(self):
        """Main loop: record, transcribe, publish."""
        while rclpy.ok():
            # Wait for user input
            input("Press Enter to give voice command...")

            # Record audio
            audio = self.record_audio(duration=5)

            # Transcribe
            text = self.transcribe(audio)

            self.get_logger().info(f'Transcribed: "{text}"')

            # Publish
            msg = String()
            msg.data = text
            self.command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    commander = VoiceCommander()
    commander.run()
    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Multimodal LLM (GPT-4 Vision)

Combine camera images with language for context-aware planning.

```python title="multimodal_planner.py"
#!/usr/bin/env python3
"""
Multimodal planning with GPT-4 Vision
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
from openai import OpenAI


class MultimodalPlanner(Node):
    """Use GPT-4V for vision-language planning."""

    def __init__(self):
        super().__init__('multimodal_planner')

        self.client = OpenAI()
        self.bridge = CvBridge()

        # Latest image
        self.latest_image = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.command_sub = self.create_subscription(
            String, '/voice_command', self.command_callback, 10
        )

        # Publisher
        self.plan_pub = self.create_publisher(String, '/task_plan', 10)

        self.get_logger().info('Multimodal Planner ready')

    def image_callback(self, msg: Image):
        """Store latest camera image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def command_callback(self, msg: String):
        """Process command with visual context."""
        if self.latest_image is None:
            self.get_logger().warn('No camera image available')
            return

        command = msg.data
        plan = self.generate_multimodal_plan(command, self.latest_image)

        # Publish
        if plan:
            plan_msg = String()
            plan_msg.data = plan
            self.plan_pub.publish(plan_msg)

    def encode_image(self, image):
        """Encode image to base64."""
        _, buffer = cv2.imencode('.jpg', image)
        return base64.b64encode(buffer).decode('utf-8')

    def generate_multimodal_plan(self, command: str, image):
        """Generate plan using GPT-4V with image context."""
        # Encode image
        base64_image = self.encode_image(image)

        try:
            response = self.client.chat.completions.create(
                model="gpt-4-vision-preview",
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "text",
                                "text": f"You are a robot. Task: {command}. "
                                        f"Describe what you see and how to accomplish the task."
                            },
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}"
                                }
                            }
                        ]
                    }
                ],
                max_tokens=300
            )

            plan = response.choices[0].message.content
            self.get_logger().info(f'Plan: {plan}')

            return plan

        except Exception as e:
            self.get_logger().error(f'Multimodal LLM error: {e}')
            return None


def main(args=None):
    rclpy.init(args=args)
    planner = MultimodalPlanner()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## LLM for Code Generation

Use LLMs to generate robot code on-the-fly.

```python title="code_generator.py"
#!/usr/bin/env python3
"""
Generate and execute robot code using LLM
"""

from openai import OpenAI
import ast
import sys


class CodeGenerator:
    """Generate executable Python code for robot tasks."""

    def __init__(self):
        self.client = OpenAI()

        self.system_prompt = """
        You are a Python code generator for robot control.
        Generate safe, executable Python code using this API:

        - robot.move_forward(distance)
        - robot.turn_left(degrees)
        - robot.turn_right(degrees)
        - robot.pick_object()
        - robot.place_object()

        Only output Python code, no explanations.
        """

    def generate_code(self, task: str) -> str:
        """Generate Python code for task."""
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": f"Task: {task}"}
            ],
            temperature=0.2
        )

        code = response.choices[0].message.content

        # Extract code from markdown if present
        if "```python" in code:
            code = code.split("```python")[1].split("```")[0].strip()

        return code

    def validate_code(self, code: str) -> bool:
        """Validate generated code for safety."""
        try:
            # Parse AST
            tree = ast.parse(code)

            # Check for dangerous operations
            for node in ast.walk(tree):
                # Block file I/O
                if isinstance(node, ast.Import):
                    for alias in node.names:
                        if alias.name in ['os', 'sys', 'subprocess']:
                            return False

                # Block eval/exec
                if isinstance(node, ast.Call):
                    if hasattr(node.func, 'id'):
                        if node.func.id in ['eval', 'exec', '__import__']:
                            return False

            return True

        except SyntaxError:
            return False

    def execute_code(self, code: str, robot):
        """Execute generated code safely."""
        if not self.validate_code(code):
            print("Code validation failed - unsafe operations detected")
            return False

        try:
            # Create safe execution environment
            safe_globals = {
                'robot': robot,
                '__builtins__': {}
            }

            exec(code, safe_globals)
            return True

        except Exception as e:
            print(f"Execution error: {e}")
            return False


# Example usage
if __name__ == '__main__':
    generator = CodeGenerator()

    # Generate code
    task = "Move forward 2 meters, turn left 90 degrees, then pick up an object"
    code = generator.generate_code(task)

    print(f"Generated Code:\n{code}\n")

    # Validate
    if generator.validate_code(code):
        print("Code is safe to execute")
    else:
        print("Code contains unsafe operations")
```

---

## Safety Considerations

### Input Validation

```python
def validate_command(command: str) -> bool:
    """Validate user command for safety."""
    # Block dangerous keywords
    forbidden = ['delete', 'shutdown', 'reboot', 'crash', 'destroy']

    for word in forbidden:
        if word in command.lower():
            return False

    # Length check
    if len(command) > 500:
        return False

    return True
```

### Action Filtering

```python
def filter_actions(plan: list) -> list:
    """Filter out unsafe actions."""
    # Allowed actions
    whitelist = ['navigate_to', 'pick_object', 'place_object', 'scan_environment']

    filtered = []
    for action in plan:
        if action['action'] in whitelist:
            filtered.append(action)
        else:
            print(f"Blocked unsafe action: {action['action']}")

    return filtered
```

### Rate Limiting

```python
import time
from collections import deque

class RateLimiter:
    """Limit LLM API calls."""

    def __init__(self, max_calls: int = 10, window: int = 60):
        self.max_calls = max_calls
        self.window = window
        self.calls = deque()

    def allow_call(self) -> bool:
        """Check if call is allowed."""
        now = time.time()

        # Remove old calls
        while self.calls and self.calls[0] < now - self.window:
            self.calls.popleft()

        # Check limit
        if len(self.calls) >= self.max_calls:
            return False

        self.calls.append(now)
        return True
```

---

## Best Practices

1. **Prompt Engineering**
   - Be specific about robot capabilities
   - Provide examples (few-shot learning)
   - Constrain output format (JSON)

2. **Error Handling**
   - Always validate LLM outputs
   - Have fallback behaviors
   - Log all LLM interactions

3. **Cost Management**
   - Cache common queries
   - Use smaller models when possible
   - Implement rate limiting

4. **Safety**
   - Whitelist actions
   - Human-in-the-loop for critical tasks
   - Emergency stop mechanisms

---

## Self-Assessment

### Questions

1. **Why use GPT-4 instead of smaller models?**
   <details>
   <summary>Answer</summary>
   GPT-4 offers better reasoning, handles complex instructions, and generalizes better to novel scenarios. However, smaller models may suffice for simpler tasks.
   </details>

2. **What's the main risk of LLM-generated code execution?**
   <details>
   <summary>Answer</summary>
   LLMs may generate unsafe code (file deletion, network access, infinite loops). Always validate and sandbox execution.
   </details>

3. **How does multimodal LLM improve robot planning?**
   <details>
   <summary>Answer</summary>
   It combines visual context with language, enabling better understanding of the environment and more accurate task planning.
   </details>

### Exercises

1. **Integrate GPT-4**: Set up GPT-4 API and create a task planner for 3 basic actions

2. **Voice Control**: Implement Whisper-based voice commands for your robot

3. **Safety Filter**: Create an action whitelist and filter dangerous LLM outputs

4. **Multimodal Planning**: Use GPT-4V to describe a scene and generate a plan

---

## Next Steps

Now that you understand LLM integration, proceed to:

👉 [Chapter 3: Vision-Language Models](./03-vision-language.md) to learn multimodal perception

---

## Additional Resources

- [OpenAI API Documentation](https://platform.openai.com/docs/)
- [Whisper GitHub](https://github.com/openai/whisper)
- [LangChain for Robotics](https://python.langchain.com/)
- [Safety in LLM-Powered Robots](https://arxiv.org/abs/2308.14368)

---

**Key Takeaway**: LLMs enable natural language interaction and high-level reasoning for robots. Proper integration with safety measures unlocks powerful, adaptive robotic systems.
