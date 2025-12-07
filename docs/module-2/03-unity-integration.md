---
sidebar_position: 3
---

# Unity for High-Fidelity Rendering

Learn to use Unity for photorealistic robot simulation, combining Unity's rendering engine with ROS 2 for advanced visualization and training scenarios.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand when to use Unity vs Gazebo
- Install Unity and ROS-TCP-Connector
- Create photorealistic robot visualizations
- Generate synthetic data for computer vision training
- Build Unity-based digital twins
- Integrate Unity with ROS 2 workflows

---

## Unity vs Gazebo: When to Use Which?

| Feature | Gazebo | Unity |
|---------|---------|-------|
| **Physics Accuracy** | Excellent | Good |
| **Visual Fidelity** | Good | Excellent |
| **Sensor Simulation** | Native LiDAR, depth | Cameras, ray tracing |
| **Learning Curve** | Moderate | Moderate |
| **Best Use Case** | Control algorithms, SLAM | Computer vision, VR/AR |
| **Synthetic Data** | Limited | Excellent |
| **Performance** | Moderate | High (GPU accelerated) |

### When to Choose Unity

- **Photorealistic Rendering**: Training vision models with synthetic data
- **Large-Scale Datasets**: Generate thousands of annotated images
- **VR/AR**: Immersive robot teleoperation interfaces
- **Digital Twins**: High-fidelity visualization for stakeholders
- **Multi-Environment**: Quickly create varied scenarios

### When to Choose Gazebo

- **Physics-Critical**: Manipulation, contact dynamics
- **Traditional Robotics**: SLAM, navigation, control
- **Hardware-in-the-Loop**: Direct sensor simulation (LiDAR, IMU)
- **Standard Workflows**: Established ROS 2 integration

**Best Practice**: Use **both**! Gazebo for physics validation, Unity for visual training.

---

## Installation

### Install Unity Hub

```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

**Or download from**: [https://unity.com/download](https://unity.com/download)

### Install Unity Editor

1. Open Unity Hub
2. Click **Installs** → **Add**
3. Select **Unity 2021.3 LTS** (recommended for ROS 2)
4. Add modules:
   - **Linux Build Support** (if on Windows/Mac)
   - **WebGL Build Support** (optional)

### Install ROS-TCP-Connector

```bash
# Create Unity project directory
mkdir ~/UnityProjects/ROSRobotSim
cd ~/UnityProjects/ROSRobotSim

# Clone ROS-TCP-Connector
git clone https://github.com/Unity-Technologies/ROS-TCP-Connector.git
```

---

## Unity Robotics Hub

NVIDIA and Unity provide **Unity Robotics Hub** for ROS integration.

### Setup Unity Robotics

1. Open Unity Hub → **Create New Project**
2. Template: **3D Core**
3. Name: `ROSRobotSimulation`

**Install ROS-TCP-Connector Package**:

1. In Unity: **Window** → **Package Manager**
2. Click **+** → **Add package from git URL**
3. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`

**Install URDF Importer**:

1. Package Manager → **+** → **Add package from git URL**
2. Enter: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

---

## Creating a Unity Scene

### Basic Scene Setup

1. **Create Ground Plane**:
   - Right-click Hierarchy → **3D Object** → **Plane**
   - Scale: `X=10, Y=1, Z=10`
   - Material: Create new material, set color to gray

2. **Add Lighting**:
   - **GameObject** → **Light** → **Directional Light** (sun)
   - **GameObject** → **Light** → **Point Light** (ambient)

3. **Add Camera**:
   - Already exists in scene
   - Position: `X=5, Y=3, Z=-5`
   - Rotation: `X=20, Y=-45, Z=0`

### Import Robot URDF

1. **Assets** → **Import Robot from URDF**
2. Select your robot's URDF file
3. Configure import settings:
   - **Axis Type**: Y-Up (Unity convention)
   - **Mesh Decomposer**: VHACD (for physics)
4. Click **Import**

Your robot will appear in the scene with:
- Visual meshes
- Collision meshes
- Joint controllers

---

## ROS 2 Integration

### Setup ROS-TCP-Endpoint (ROS 2 side)

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_unity_ws/src
cd ~/ros2_unity_ws/src

# Clone ROS-TCP-Endpoint
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Build
cd ~/ros2_unity_ws
colcon build

# Source workspace
source install/setup.bash
```

### Launch ROS-TCP-Endpoint

```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=127.0.0.1
```

**Output**:
```
[INFO] [ros_tcp_endpoint]: Starting ROS-TCP endpoint...
[INFO] [ros_tcp_endpoint]: Listening on 127.0.0.1:10000
```

### Configure Unity Connection

1. In Unity: **Robotics** → **ROS Settings**
2. **ROS IP Address**: `127.0.0.1`
3. **ROS Port**: `10000`
4. **Protocol**: `ROS 2`
5. Click **Connect**

---

## Publishing Robot State from Unity

### Create Publisher Script

```csharp title="RobotPosePublisher.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class RobotPosePublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Publishing
    public string topicName = "robot_pose";
    public float publishRate = 10.0f; // Hz

    private float timeElapsed;

    void Start()
    {
        // Get ROS connection
        ros = ROSConnection.GetOrCreateInstance();

        // Register publisher
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1.0f / publishRate)
        {
            // Create message
            PoseStampedMsg poseMsg = new PoseStampedMsg
            {
                header = new RosMessageTypes.Std.HeaderMsg
                {
                    stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                    {
                        sec = (int)Time.time,
                        nanosec = (uint)((Time.time % 1) * 1e9)
                    },
                    frame_id = "world"
                },
                pose = new PoseMsg
                {
                    position = new PointMsg
                    {
                        x = transform.position.x,
                        y = transform.position.y,
                        z = transform.position.z
                    },
                    orientation = new QuaternionMsg
                    {
                        x = transform.rotation.x,
                        y = transform.rotation.y,
                        z = transform.rotation.z,
                        w = transform.rotation.w
                    }
                }
            };

            // Publish
            ros.Publish(topicName, poseMsg);

            timeElapsed = 0;
        }
    }
}
```

**Attach to Robot**:
1. Select robot GameObject
2. **Add Component** → **Robot Pose Publisher**

---

## Subscribing to ROS 2 Commands

### Create Velocity Subscriber

```csharp title="VelocitySubscriber.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocitySubscriber : MonoBehaviour
{
    private ROSConnection ros;

    public string topicName = "cmd_vel";
    public float speed = 2.0f;
    public float rotationSpeed = 100.0f;

    private TwistMsg latestCmd;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, UpdateVelocity);
    }

    void UpdateVelocity(TwistMsg msg)
    {
        latestCmd = msg;
    }

    void FixedUpdate()
    {
        if (latestCmd != null)
        {
            // Apply linear velocity
            float moveZ = (float)latestCmd.linear.x * speed * Time.fixedDeltaTime;
            transform.Translate(0, 0, moveZ);

            // Apply angular velocity
            float rotate = (float)latestCmd.angular.z * rotationSpeed * Time.fixedDeltaTime;
            transform.Rotate(0, rotate, 0);
        }
    }
}
```

**Test with ROS 2**:

```bash
# Terminal 1: ROS-TCP-Endpoint
ros2 run ros_tcp_endpoint default_server_endpoint

# Terminal 2: Send velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}, angular: {z: 0.5}}"
```

Your Unity robot should start moving!

---

## Camera Sensor Simulation

### Add Camera to Robot

1. Create empty GameObject: **GameObject** → **Create Empty**
2. Name it `CameraMount`
3. Position on robot (e.g., front at height)
4. Add Camera component: **Add Component** → **Camera**

### Publish Camera Images to ROS 2

```csharp title="CameraPublisher.cs"
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    public Camera imageCamera;
    public string topicName = "camera/image_raw";
    public int publishRate = 10;

    private ROSConnection ros;
    private float timeElapsed;
    private Texture2D texture2D;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        // Create texture for reading camera
        texture2D = new Texture2D(
            imageCamera.pixelWidth,
            imageCamera.pixelHeight,
            TextureFormat.RGB24,
            false
        );
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1.0f / publishRate)
        {
            PublishImage();
            timeElapsed = 0;
        }
    }

    void PublishImage()
    {
        // Capture camera image
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = imageCamera.targetTexture;

        imageCamera.Render();

        texture2D.ReadPixels(
            new Rect(0, 0, imageCamera.pixelWidth, imageCamera.pixelHeight),
            0, 0
        );
        texture2D.Apply();

        RenderTexture.active = currentRT;

        // Create ROS message
        ImageMsg msg = new ImageMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                frame_id = "camera_link"
            },
            height = (uint)texture2D.height,
            width = (uint)texture2D.width,
            encoding = "rgb8",
            step = (uint)texture2D.width * 3,
            data = texture2D.GetRawTextureData()
        };

        // Publish
        ros.Publish(topicName, msg);
    }
}
```

---

## Synthetic Data Generation

### Domain Randomization

Create varied environments for robust vision training.

```csharp title="DomainRandomizer.cs"
using UnityEngine;

public class DomainRandomizer : MonoBehaviour
{
    public Material[] materials;
    public Light directionalLight;

    public float randomizeInterval = 5.0f;
    private float timer;

    void Update()
    {
        timer += Time.deltaTime;

        if (timer >= randomizeInterval)
        {
            RandomizeEnvironment();
            timer = 0;
        }
    }

    void RandomizeEnvironment()
    {
        // Randomize lighting
        directionalLight.intensity = Random.Range(0.5f, 1.5f);
        directionalLight.color = new Color(
            Random.Range(0.8f, 1.0f),
            Random.Range(0.8f, 1.0f),
            Random.Range(0.8f, 1.0f)
        );

        // Randomize object materials
        foreach (GameObject obj in GameObject.FindGameObjectsWithTag("Randomizable"))
        {
            Renderer renderer = obj.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material = materials[Random.Range(0, materials.Length)];
            }
        }

        // Randomize camera position slightly
        Camera.main.transform.position += new Vector3(
            Random.Range(-0.5f, 0.5f),
            Random.Range(-0.2f, 0.2f),
            Random.Range(-0.5f, 0.5f)
        );

        Debug.Log("Environment randomized");
    }
}
```

---

## Perception Package Integration

### Generate Labeled Data

Unity's **Perception Package** creates annotated datasets.

**Install**:
1. Package Manager → **+** → **Add package from git URL**
2. `com.unity.perception`

**Add Labeling**:
1. Select objects (obstacles, robots)
2. **Add Component** → **Labeling**
3. Add labels: `robot`, `obstacle`, `floor`

**Setup Perception Camera**:
1. Select Camera
2. **Add Component** → **Perception Camera**
3. Enable:
   - **Bounding Box 2D**
   - **Semantic Segmentation**
   - **Instance Segmentation**

**Run Simulation**:
- Unity will save annotated images to `Assets/PerceptionOutput/`

---

## Best Practices

1. **Performance Optimization**
   - Use LOD (Level of Detail) for distant objects
   - Bake lighting instead of real-time
   - Limit shadow-casting lights

2. **Physics Accuracy**
   - Match Unity physics timestep to ROS publish rate
   - Use realistic mass and friction values

3. **Asset Organization**
   ```
   Assets/
   ├── Robots/
   ├── Environments/
   ├── Materials/
   ├── Scripts/
   └── Prefabs/
   ```

4. **Coordinate System**
   - Unity: Y-up, left-handed
   - ROS: Z-up, right-handed
   - Use `Unity.Robotics.ROSTCPConnector.ROSGeometry` for conversions

---

## Self-Assessment

### Questions

1. **What's Unity's main advantage over Gazebo?**
   <details>
   <summary>Answer</summary>
   Photorealistic rendering and synthetic data generation for training computer vision models.
   </details>

2. **How does ROS-TCP-Connector work?**
   <details>
   <summary>Answer</summary>
   It creates a TCP connection between Unity and a ROS 2 node (ROS-TCP-Endpoint) to exchange messages.
   </details>

3. **Why use domain randomization?**
   <details>
   <summary>Answer</summary>
   It creates diverse training scenarios, making vision models more robust to real-world variations.
   </details>

### Exercises

1. **Import Robot**: Import your URDF robot into Unity and visualize it

2. **ROS Integration**: Set up bi-directional communication between Unity and ROS 2

3. **Camera Stream**: Publish camera images from Unity to ROS 2 and view in RViz2

4. **Domain Randomization**: Implement randomized lighting and textures

5. **Synthetic Dataset**: Generate 1000 labeled images using Perception Package

---

## Next Steps

Now that you understand Unity for robotics, proceed to:

👉 [Chapter 4: Sim-to-Real Transfer](./04-sim-to-real.md) to bridge simulation and real robots

---

## Additional Resources

- [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- [Perception Package Documentation](https://github.com/Unity-Technologies/com.unity.perception)
- [ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector)
- [Unity ML-Agents](https://github.com/Unity-Technologies/ml-agents) (for RL training)

---

**Key Takeaway**: Unity excels at photorealistic rendering and synthetic data generation, making it invaluable for training vision-based robot systems. Combine with Gazebo for complete simulation coverage.
