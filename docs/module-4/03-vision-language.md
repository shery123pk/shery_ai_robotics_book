---
sidebar_position: 3
---

# Vision-Language Models

Master vision-language models (VLMs) that bridge visual perception and natural language understanding for embodied AI.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand VLM architectures (CLIP, BLIP, Flamingo)
- Use zero-shot object recognition with CLIP
- Implement visual question answering
- Fine-tune VLMs on robot data
- Build visual grounding pipelines
- Deploy VLMs for real-time robot perception

---

## What are Vision-Language Models?

**VLMs** learn joint embeddings of images and text, enabling:

- **Zero-shot recognition**: Identify novel objects without training
- **Visual question answering**: Answer questions about images
- **Image captioning**: Describe scenes in natural language
- **Visual grounding**: Locate objects from descriptions

```
Image: [Photo of red cup]
Text: "red cup"
         ↓
   VLM Encoder
         ↓
Shared Embedding Space
         ↓
Similarity: 0.95 ✓
```

---

## CLIP (Contrastive Language-Image Pre-training)

OpenAI's groundbreaking VLM trained on 400M image-text pairs.

### Architecture

```
┌────────────┐     ┌────────────┐
│   Image    │ ──> │   Image    │
│  Encoder   │     │ Embedding  │
│ (ResNet/   │     │            │
│  ViT)      │     └─────┬──────┘
└────────────┘           │
                         │ Cosine
                         │ Similarity
┌────────────┐           │
│    Text    │ ──> ┌─────▼──────┐
│  Encoder   │     │    Text    │
│  (Trans-   │     │ Embedding  │
│  former)   │     │            │
└────────────┘     └────────────┘
```

### Using CLIP for Object Recognition

```python title="clip_detector.py"
#!/usr/bin/env python3
"""
Zero-shot object detection with CLIP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
import torch
import clip
from PIL import Image as PILImage
import numpy as np
import cv2


class CLIPDetector(Node):
    """Zero-shot object detector using CLIP."""

    def __init__(self):
        super().__init__('clip_detector')

        # Load CLIP model
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model, self.preprocess = clip.load("ViT-B/32", device=self.device)

        # Object categories to detect
        self.declare_parameter('categories', ['cup', 'bottle', 'book', 'phone', 'laptop'])
        self.categories = self.get_parameter('categories').value

        # Precompute text embeddings
        self.text_tokens = clip.tokenize(self.categories).to(self.device)
        with torch.no_grad():
            self.text_features = self.model.encode_text(self.text_tokens)
            self.text_features /= self.text_features.norm(dim=-1, keepdim=True)

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/clip_detections', 10
        )

        self.viz_pub = self.create_publisher(
            Image, '/clip_viz', 10
        )

        self.get_logger().info(f'CLIP Detector ready. Categories: {self.categories}')

    def image_callback(self, msg: Image):
        """Detect objects in image using CLIP."""
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Detect objects
        detections = self.detect_objects(cv_image)

        # Publish detections
        detection_array = Detection2DArray()
        detection_array.header = msg.header
        detection_array.detections = detections

        self.detection_pub.publish(detection_array)

        # Visualize
        viz_image = self.visualize_detections(cv_image, detections)
        viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)

    def detect_objects(self, image):
        """Use CLIP to classify image regions."""
        # Simple approach: sliding window + CLIP
        # For production, use object proposals (Selective Search, RPN)

        detections = []
        height, width = image.shape[:2]

        # Multi-scale sliding windows
        for scale in [0.3, 0.5, 0.7]:
            window_size = int(min(height, width) * scale)

            for y in range(0, height - window_size, window_size // 2):
                for x in range(0, width - window_size, window_size // 2):
                    # Extract window
                    window = image[y:y+window_size, x:x+window_size]

                    # Classify with CLIP
                    category, confidence = self.classify_window(window)

                    if confidence > 0.25:  # Threshold
                        detection = Detection2D()
                        detection.bbox.center.x = x + window_size / 2
                        detection.bbox.center.y = y + window_size / 2
                        detection.bbox.size_x = window_size
                        detection.bbox.size_y = window_size

                        detections.append({
                            'bbox': (x, y, window_size, window_size),
                            'category': category,
                            'confidence': confidence
                        })

        # Non-maximum suppression (simplified)
        return self.nms(detections, iou_threshold=0.3)

    def classify_window(self, window):
        """Classify image window with CLIP."""
        # Preprocess
        pil_image = PILImage.fromarray(cv2.cvtColor(window, cv2.COLOR_BGR2RGB))
        image_input = self.preprocess(pil_image).unsqueeze(0).to(self.device)

        # Encode image
        with torch.no_grad():
            image_features = self.model.encode_image(image_input)
            image_features /= image_features.norm(dim=-1, keepdim=True)

            # Compute similarity
            similarity = (100.0 * image_features @ self.text_features.T).softmax(dim=-1)

        # Get top prediction
        values, indices = similarity[0].topk(1)
        category = self.categories[indices.item()]
        confidence = values.item()

        return category, confidence

    def nms(self, detections, iou_threshold=0.3):
        """Non-maximum suppression."""
        if not detections:
            return []

        # Sort by confidence
        detections = sorted(detections, key=lambda x: x['confidence'], reverse=True)

        keep = []
        while detections:
            best = detections.pop(0)
            keep.append(best)

            # Remove overlapping detections
            detections = [
                d for d in detections
                if self.compute_iou(best['bbox'], d['bbox']) < iou_threshold
            ]

        return keep

    def compute_iou(self, box1, box2):
        """Compute Intersection over Union."""
        x1, y1, w1, h1 = box1
        x2, y2, w2, h2 = box2

        # Intersection
        xi1 = max(x1, x2)
        yi1 = max(y1, y2)
        xi2 = min(x1 + w1, x2 + w2)
        yi2 = min(y1 + h1, y2 + h2)

        inter_area = max(0, xi2 - xi1) * max(0, yi2 - yi1)

        # Union
        box1_area = w1 * h1
        box2_area = w2 * h2
        union_area = box1_area + box2_area - inter_area

        return inter_area / union_area if union_area > 0 else 0

    def visualize_detections(self, image, detections):
        """Draw detections on image."""
        viz = image.copy()

        for det in detections:
            x, y, w, h = det['bbox']
            category = det['category']
            confidence = det['confidence']

            # Draw box
            cv2.rectangle(viz, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw label
            label = f"{category}: {confidence:.2f}"
            cv2.putText(
                viz, label, (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )

        return viz


def main(args=None):
    rclpy.init(args=args)
    detector = CLIPDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Visual Question Answering (VQA)

Answer questions about images using VLMs.

```python title="vqa_node.py"
#!/usr/bin/env python3
"""
Visual Question Answering with BLIP
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from transformers import BlipProcessor, BlipForQuestionAnswering
import torch
from PIL import Image as PILImage


class VQANode(Node):
    """Answer questions about robot's visual input."""

    def __init__(self):
        super().__init__('vqa_node')

        # Load BLIP model
        self.processor = BlipProcessor.from_pretrained("Salesforce/blip-vqa-base")
        self.model = BlipForQuestionAnswering.from_pretrained("Salesforce/blip-vqa-base")

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.question_sub = self.create_subscription(
            String, '/vqa/question', self.question_callback, 10
        )

        # Publisher
        self.answer_pub = self.create_publisher(String, '/vqa/answer', 10)

        self.get_logger().info('VQA Node ready')

    def image_callback(self, msg: Image):
        """Store latest image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def question_callback(self, msg: String):
        """Answer question about current view."""
        if self.latest_image is None:
            self.get_logger().warn('No image available')
            return

        question = msg.data
        answer = self.answer_question(self.latest_image, question)

        # Publish answer
        answer_msg = String()
        answer_msg.data = answer
        self.answer_pub.publish(answer_msg)

        self.get_logger().info(f'Q: "{question}" | A: "{answer}"')

    def answer_question(self, image, question: str) -> str:
        """Generate answer using BLIP."""
        # Convert to PIL
        pil_image = PILImage.fromarray(image)

        # Prepare inputs
        inputs = self.processor(pil_image, question, return_tensors="pt").to(self.device)

        # Generate answer
        with torch.no_grad():
            outputs = self.model.generate(**inputs)

        # Decode answer
        answer = self.processor.decode(outputs[0], skip_special_tokens=True)

        return answer


def main(args=None):
    rclpy.init(args=args)
    vqa = VQANode()
    rclpy.spin(vqa)
    vqa.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Usage**:
```bash
# Terminal 1: Run VQA node
ros2 run vision_language vqa_node

# Terminal 2: Ask questions
ros2 topic pub /vqa/question std_msgs/msg/String "{data: 'What color is the cup?'}"
ros2 topic pub /vqa/question std_msgs/msg/String "{data: 'How many objects are on the table?'}"

# Terminal 3: Listen to answers
ros2 topic echo /vqa/answer
```

---

## Image Captioning

Describe robot's view in natural language.

```python title="caption_generator.py"
#!/usr/bin/env python3
"""
Generate image captions for robot vision
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from transformers import BlipProcessor, BlipForConditionalGeneration
import torch
from PIL import Image as PILImage


class CaptionGenerator(Node):
    """Generate captions for robot's camera view."""

    def __init__(self):
        super().__init__('caption_generator')

        # Load BLIP captioning model
        self.processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
        self.model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Publishers
        self.caption_pub = self.create_publisher(String, '/scene_description', 10)

        # Timer for periodic captioning (1 Hz)
        self.timer = self.create_timer(1.0, self.caption_timer)

        self.latest_image = None

        self.get_logger().info('Caption Generator ready')

    def image_callback(self, msg: Image):
        """Store latest image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def caption_timer(self):
        """Generate caption periodically."""
        if self.latest_image is None:
            return

        caption = self.generate_caption(self.latest_image)

        # Publish
        caption_msg = String()
        caption_msg.data = caption
        self.caption_pub.publish(caption_msg)

        self.get_logger().info(f'Caption: "{caption}"')

    def generate_caption(self, image) -> str:
        """Generate caption using BLIP."""
        # Convert to PIL
        pil_image = PILImage.fromarray(image)

        # Process image
        inputs = self.processor(pil_image, return_tensors="pt").to(self.device)

        # Generate caption
        with torch.no_grad():
            outputs = self.model.generate(**inputs, max_new_tokens=50)

        # Decode
        caption = self.processor.decode(outputs[0], skip_special_tokens=True)

        return caption


def main(args=None):
    rclpy.init(args=args)
    generator = CaptionGenerator()
    rclpy.spin(generator)
    generator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Visual Grounding

Locate objects in images based on text descriptions.

```python title="visual_grounding.py"
#!/usr/bin/env python3
"""
Visual grounding: map text to image regions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import torch
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection
from PIL import Image as PILImage
import cv2
import numpy as np


class VisualGrounder(Node):
    """Locate objects from text descriptions."""

    def __init__(self):
        super().__init__('visual_grounder')

        # Load OWL-ViT (Open-Vocabulary Detection)
        self.processor = AutoProcessor.from_pretrained("google/owlvit-base-patch32")
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained("google/owlvit-base-patch32")

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model.to(self.device)

        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        self.query_sub = self.create_subscription(
            String, '/visual_query', self.query_callback, 10
        )

        # Publishers
        self.location_pub = self.create_publisher(Point, '/object_location', 10)
        self.viz_pub = self.create_publisher(Image, '/grounding_viz', 10)

        self.get_logger().info('Visual Grounder ready')

    def image_callback(self, msg: Image):
        """Store latest image."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def query_callback(self, msg: String):
        """Find object from text description."""
        if self.latest_image is None:
            return

        query = msg.data
        location, viz = self.ground_object(self.latest_image, query)

        if location:
            # Publish location
            loc_msg = Point()
            loc_msg.x = float(location[0])
            loc_msg.y = float(location[1])
            self.location_pub.publish(loc_msg)

            self.get_logger().info(f'Found "{query}" at ({location[0]}, {location[1]})')

        # Publish visualization
        if viz is not None:
            viz_msg = self.bridge.cv2_to_imgmsg(viz, encoding='rgb8')
            self.viz_pub.publish(viz_msg)

    def ground_object(self, image, query: str):
        """Locate object in image."""
        # Convert to PIL
        pil_image = PILImage.fromarray(image)

        # Prepare inputs
        inputs = self.processor(text=[query], images=pil_image, return_tensors="pt").to(self.device)

        # Detect
        with torch.no_grad():
            outputs = self.model(**inputs)

        # Post-process
        target_sizes = torch.tensor([pil_image.size[::-1]])
        results = self.processor.post_process_object_detection(
            outputs, threshold=0.1, target_sizes=target_sizes
        )[0]

        if len(results["boxes"]) == 0:
            self.get_logger().warn(f'Object "{query}" not found')
            return None, None

        # Get best detection
        best_idx = results["scores"].argmax()
        box = results["boxes"][best_idx].cpu().numpy()

        # Compute center
        center_x = (box[0] + box[2]) / 2
        center_y = (box[1] + box[3]) / 2

        # Visualize
        viz = image.copy()
        cv2.rectangle(viz, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
        cv2.circle(viz, (int(center_x), int(center_y)), 5, (255, 0, 0), -1)

        return (center_x, center_y), viz


def main(args=None):
    rclpy.init(args=args)
    grounder = VisualGrounder()
    rclpy.spin(grounder)
    grounder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Fine-Tuning VLMs

Adapt pre-trained models to your robot's domain.

```python title="fine_tune_clip.py"
"""
Fine-tune CLIP on custom robot dataset
"""

import torch
import clip
from torch.utils.data import Dataset, DataLoader
from PIL import Image
import os


class RobotDataset(Dataset):
    """Custom dataset for robot images and descriptions."""

    def __init__(self, image_dir, annotations, preprocess):
        self.image_dir = image_dir
        self.annotations = annotations  # List of (image_name, caption) tuples
        self.preprocess = preprocess

    def __len__(self):
        return len(self.annotations)

    def __getitem__(self, idx):
        image_name, caption = self.annotations[idx]
        image_path = os.path.join(self.image_dir, image_name)

        image = Image.open(image_path)
        image = self.preprocess(image)

        text = clip.tokenize([caption])[0]

        return image, text


def fine_tune_clip(dataset_dir, epochs=10):
    """Fine-tune CLIP on robot dataset."""
    # Load pre-trained CLIP
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, preprocess = clip.load("ViT-B/32", device=device)

    # Load dataset
    annotations = [
        ("robot_picks_cup.jpg", "robot picking up a red cup"),
        ("robot_places_book.jpg", "robot placing a book on shelf"),
        # ... more examples
    ]

    dataset = RobotDataset(dataset_dir, annotations, preprocess)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    # Optimizer
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-5)

    # Training loop
    for epoch in range(epochs):
        total_loss = 0

        for images, texts in dataloader:
            images = images.to(device)
            texts = texts.to(device)

            # Forward pass
            logits_per_image, logits_per_text = model(images, texts)

            # Contrastive loss
            labels = torch.arange(len(images)).to(device)
            loss_img = torch.nn.functional.cross_entropy(logits_per_image, labels)
            loss_txt = torch.nn.functional.cross_entropy(logits_per_text, labels)
            loss = (loss_img + loss_txt) / 2

            # Backward pass
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            total_loss += loss.item()

        print(f"Epoch {epoch + 1}/{epochs}, Loss: {total_loss / len(dataloader):.4f}")

    # Save model
    torch.save(model.state_dict(), "clip_fine_tuned.pth")
    print("Fine-tuning complete!")
```

---

## Best Practices

1. **Model Selection**
   - CLIP: Zero-shot recognition, image-text similarity
   - BLIP: VQA, captioning
   - OWL-ViT: Open-vocabulary detection

2. **Optimization**
   - Use GPU (Jetson Orin or desktop RTX)
   - Quantize models for edge deployment
   - Cache embeddings when possible

3. **Prompt Engineering**
   - Use specific queries ("red ceramic coffee mug" vs "cup")
   - Test with diverse phrasings

4. **Evaluation**
   - Test on robot-specific objects
   - Measure accuracy in target environment

---

## Self-Assessment

### Questions

1. **What's the key advantage of CLIP over traditional classifiers?**
   <details>
   <summary>Answer</summary>
   CLIP can recognize novel objects zero-shot by matching image and text embeddings, without requiring task-specific training.
   </details>

2. **When would you use VQA vs. image captioning?**
   <details>
   <summary>Answer</summary>
   VQA for specific queries ("Is the door open?"). Captioning for general scene understanding.
   </details>

3. **Why fine-tune instead of using pre-trained models?**
   <details>
   <summary>Answer</summary>
   Fine-tuning adapts models to robot-specific objects, environments, and phrasings, improving accuracy in your domain.
   </details>

### Exercises

1. **Zero-Shot Detection**: Use CLIP to detect 5 different objects in your environment

2. **Visual QA**: Implement VQA and ask 10 questions about a scene

3. **Captioning**: Generate captions for robot's camera view every second

4. **Visual Grounding**: Locate "blue bottle" in a cluttered scene

---

## Next Steps

Now that you understand vision-language models, proceed to:

👉 [Chapter 4: End-to-End VLA Policies](./04-end-to-end-policies.md) to learn complete VLA systems

---

## Additional Resources

- [CLIP Paper](https://arxiv.org/abs/2103.00020)
- [BLIP Paper](https://arxiv.org/abs/2201.12086)
- [OWL-ViT Paper](https://arxiv.org/abs/2205.06230)
- [Hugging Face Transformers](https://huggingface.co/docs/transformers/)

---

**Key Takeaway**: Vision-language models enable robots to understand scenes through natural language, bridging perception and reasoning. Mastering VLMs is essential for building truly intelligent embodied AI.
