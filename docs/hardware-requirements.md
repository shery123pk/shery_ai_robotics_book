---
sidebar_position: 2
---

# Hardware Requirements

This course is technically demanding, sitting at the intersection of three heavy computational loads: **Physics Simulation** (Isaac Sim/Gazebo), **Visual Perception** (SLAM/Computer Vision), and **Generative AI** (LLMs/VLA).

---

## Overview

Because the capstone involves a "Simulated Humanoid," the primary investment must be in **High-Performance Workstations**. However, to fulfill the "Physical AI" promise, you also need **Edge Computing Kits** (brains without bodies) or specific robot hardware.

---

## 1. The "Digital Twin" Workstation (Required per Student)

This is the **most critical component**. NVIDIA Isaac Sim is an Omniverse application that requires "RTX" (Ray Tracing) capabilities. Standard laptops (MacBooks or non-RTX Windows machines) will not work.

### GPU (The Bottleneck)

**NVIDIA RTX 4070 Ti** (12GB VRAM) or higher

**Why**: You need high VRAM to load the USD (Universal Scene Description) assets for the robot and environment, plus run the VLA (Vision-Language-Action) models simultaneously.

**Ideal**: RTX 3090 or 4090 (24GB VRAM) allows for smoother "Sim-to-Real" training.

### CPU

**Intel Core i7 (13th Gen+)** or **AMD Ryzen 9**

**Why**: Physics calculations (Rigid Body Dynamics) in Gazebo/Isaac are CPU-intensive.

### RAM

**64 GB DDR5** (32 GB is the absolute minimum, but will crash during complex scene rendering)

### Operating System

**Ubuntu 22.04 LTS**

**Note**: While Isaac Sim runs on Windows, ROS 2 (Humble/Iron) is native to Linux. Dual-booting or dedicated Linux machines are mandatory for a friction-free experience.

---

## 2. The "Physical AI" Edge Kit

Since a full humanoid robot is expensive, students learn "Physical AI" by setting up the nervous system on a desk before deploying it to a robot. This kit covers Module 3 (Isaac ROS) and Module 4 (VLA).

### The Brain

**NVIDIA Jetson Orin Nano** (8GB) or **Orin NX** (16GB)

**Role**: Industry standard for embodied AI. Students deploy their ROS 2 nodes here to understand resource constraints vs. their powerful workstations.

**Price**: ~$249 (Orin Nano Super Dev Kit)

### The Eyes (Vision)

**Intel RealSense D435i** or **D455**

**Role**: Provides RGB (Color) and Depth (Distance) data. Essential for VSLAM and Perception modules.

**Price**: ~$349

### The Inner Ear (Balance)

**USB IMU (BNO055)** (Often built into the RealSense D435i or Jetson boards)

A separate module helps teach IMU calibration.

**Price**: ~$30 (if separate)

### Voice Interface

**ReSpeaker USB Mic Array v2.0**

For the "Voice-to-Action" Whisper integration.

**Price**: ~$69

---

## 3. The Robot Lab

For the "Physical" part of the course, you have three tiers of options depending on budget.

### Option A: The "Proxy" Approach (Recommended for Budget)

Use a quadruped (dog) or a robotic arm as a proxy. The software principles (ROS 2, VSLAM, Isaac Sim) transfer 90% effectively to humanoids.

**Robot**: **Unitree Go2 Edu** (~$1,800 - $3,000)

**Pros**: Highly durable, excellent ROS 2 support, affordable enough to have multiple units

**Cons**: Not a biped (humanoid)

### Option B: The "Miniature Humanoid" Approach

Small, table-top humanoids.

**Robot**:
- **Unitree G1** (~$16,000) - Ideal for sim-to-real
- **Robotis OP3** (~$12,000) - Older but stable
- **Hiwonder TonyPi Pro** (~$600) - Budget alternative

**Warning**: The cheap kits (Hiwonder) usually run on Raspberry Pi, which cannot run NVIDIA Isaac ROS efficiently. You would use these only for kinematics (walking) and use the Jetson kits for AI.

### Option C: The "Premium" Lab (Sim-to-Real specific)

If the goal is to actually deploy the Capstone to a real humanoid:

**Robot**: **Unitree G1 Humanoid** (~$16,000)

**Why**: One of the few commercially available humanoids that can actually walk dynamically and has an SDK open enough for students to inject their own ROS 2 controllers.

---

## 4. Summary of Architecture

To teach this successfully, your lab infrastructure should look like this:

| Component | Hardware | Function |
|-----------|----------|----------|
| **Sim Rig** | PC with RTX 4080 + Ubuntu 22.04 | Runs Isaac Sim, Gazebo, Unity, trains LLM/VLA models |
| **Edge Brain** | Jetson Orin Nano | Runs "Inference" stack. Deploy code here |
| **Sensors** | RealSense Camera + IMU | Connected to Jetson to feed real-world data |
| **Actuator** | Unitree Go2 or G1 (Shared) | Receives motor commands from Jetson |

---

## 5. The Economy Jetson Student Kit

Best for: Learning ROS 2, Basic Computer Vision, and Sim-to-Real control.

| Component | Model | Price (Approx.) | Notes |
|-----------|-------|----------------|-------|
| **The Brain** | NVIDIA Jetson Orin Nano Super Dev Kit (8GB) | $249 | Capable of 40 TOPS |
| **The Eyes** | Intel RealSense D435i | $349 | Includes IMU |
| **The Ears** | ReSpeaker USB Mic Array v2.0 | $69 | Far-field microphone for voice commands |
| **Wi-Fi** | (Included in Dev Kit) | $0 | Pre-installed |
| **Power/Misc** | SD Card (128GB) + Jumper Wires | $30 | High-endurance microSD card |
| **TOTAL** | | **~$700 per kit** | |

---

## 6. Cloud-Based Alternative (High OpEx)

If you don't have access to RTX-enabled workstations, we can restructure the course to rely entirely on cloud-based instances (like AWS RoboMaker or NVIDIA's cloud delivery for Omniverse), though this introduces significant latency and cost complexity.

### Cloud Workstations (AWS/Azure)

**Instance Type**: AWS g5.2xlarge (A10G GPU, 24GB VRAM)

**Software**: NVIDIA Isaac Sim on Omniverse Cloud

**Cost Calculation**:
- Instance cost: ~$1.50/hour (spot/on-demand mix)
- Usage: 10 hours/week × 12 weeks = 120 hours
- Storage (EBS volumes): ~$25/quarter
- **Total Cloud Bill**: ~$205 per quarter per student

### The Latency Trap (Hidden Cost)

Simulating in the cloud works well, but controlling a real robot from a cloud instance is dangerous due to latency.

**Solution**: Students train in the Cloud, download the model (weights), and flash it to the local Jetson kit.

---

## 7. Minimum Requirements (Software-Only)

If physical hardware is not available, students can still complete the course using simulation only:

- **GPU**: NVIDIA RTX 3060 (12GB) or higher
- **CPU**: Intel Core i5 (11th Gen+) or AMD Ryzen 5
- **RAM**: 32GB DDR4
- **OS**: Ubuntu 22.04 LTS
- **Software**: ROS 2 Humble, Gazebo Classic, NVIDIA Isaac Sim

**Note**: This approach covers Modules 1-3 but limits hands-on Physical AI experience in Module 4.

---

## 8. Software Stack

All students will need to install:

- **ROS 2 Humble** or **Iron** (Ubuntu 22.04)
- **Gazebo Classic** or **Gazebo Garden**
- **NVIDIA Isaac Sim** (requires RTX GPU)
- **Unity** (optional, for high-fidelity rendering)
- **Python 3.10+** with libraries: `rclpy`, `numpy`, `opencv-python`
- **OpenAI Whisper** (for voice commands in Module 4)

Installation guides will be provided in each module.

---

## 9. Budget Planning

### Individual Student (Home Setup)

- **Workstation**: $2,500 - $4,000 (RTX 4070 Ti + components)
- **Jetson Kit**: $700
- **Robot** (optional): $600 - $3,000 (Hiwonder TonyPi to Unitree Go2)
- **Total**: $3,800 - $7,700

### Educational Institution (Per 20 Students)

- **Workstations**: $50,000 - $80,000 (20 PCs with RTX GPUs)
- **Jetson Kits**: $14,000 (20 kits)
- **Shared Robots**: $6,000 - $15,000 (3-5 Unitree Go2 units)
- **Total**: $70,000 - $109,000

### Cloud-Only Approach

- **Per Student**: ~$200/quarter (AWS cloud compute)
- **20 Students**: ~$4,000/quarter
- **Advantage**: No upfront CapEx, scales easily
- **Disadvantage**: Ongoing OpEx, latency issues for Physical AI

---

## 10. Conclusion

Building a "Physical AI" lab is a significant investment. You must choose between:

- **On-Premise Lab at Home**: High CapEx, full control, best for hands-on learning
- **Cloud-Native Lab**: High OpEx, accessible from anywhere, latency limitations

For the best learning experience, we recommend a **hybrid approach**: powerful workstations for simulation + Jetson edge kits for deployment.

---

**Ready to set up your environment?** Proceed to [Module 1: ROS 2 Introduction](./module-1/01-ros2-intro.md) to begin!
