# Diagram Generator Skill

## Description
Creates Mermaid diagrams for robotics architectures, workflows, and system designs that can be embedded directly in Markdown documentation.

## Usage
Invoke when you need visual representations of:
- ROS 2 node architectures
- Communication patterns (topics, services, actions)
- System workflows
- State machines
- Sequence diagrams
- Class hierarchies

## Input
Describe the system or process to visualize:
- Type of diagram needed (flowchart, sequence, class, state, architecture)
- Components and their relationships
- Data flow direction
- Key interactions

## Output Types

### 1. ROS 2 Architecture Diagram
```mermaid
graph LR
    A[Camera Node] -->|/image_raw| B[Vision Node]
    B -->|/detected_objects| C[Planning Node]
    C -->|/cmd_vel| D[Motion Controller]
    D --> E[Robot Hardware]

    F[Sensor Fusion] -->|/odom| C
    G[LiDAR Node] -->|/scan| F
    A -->|/image_raw| F

    style A fill:#e1f5ff
    style B fill:#e1f5ff
    style C fill:#ffe1e1
    style D fill:#e1ffe1
```

### 2. Sequence Diagram (Service Call)
```mermaid
sequenceDiagram
    participant Client
    participant Service
    participant Hardware

    Client->>+Service: Request(/set_mode, {mode: "autonomous"})
    Service->>Service: Validate mode
    Service->>+Hardware: Configure sensors
    Hardware-->>-Service: ACK
    Service->>+Hardware: Enable motors
    Hardware-->>-Service: ACK
    Service-->>-Client: Response({success: true})
```

### 3. State Machine (Lifecycle Node)
```mermaid
stateDiagram-v2
    [*] --> Unconfigured
    Unconfigured --> Inactive: configure()
    Inactive --> Active: activate()
    Active --> Inactive: deactivate()
    Inactive --> Unconfigured: cleanup()
    Active --> Finalized: shutdown()
    Inactive --> Finalized: shutdown()
    Unconfigured --> Finalized: shutdown()
    Finalized --> [*]
```

### 4. System Architecture (Multi-Level)
```mermaid
graph TB
    subgraph "Perception Layer"
        CAM[Camera]
        LID[LiDAR]
        IMU[IMU]
    end

    subgraph "Processing Layer"
        VIS[Vision Processing]
        SLAM[SLAM]
        FUSE[Sensor Fusion]
    end

    subgraph "Planning Layer"
        NAV[Navigation]
        PATH[Path Planning]
        OBS[Obstacle Avoidance]
    end

    subgraph "Control Layer"
        MOT[Motion Control]
        ACT[Actuators]
    end

    CAM --> VIS
    LID --> SLAM
    IMU --> SLAM
    VIS --> FUSE
    SLAM --> FUSE
    FUSE --> NAV
    NAV --> PATH
    PATH --> OBS
    OBS --> MOT
    MOT --> ACT
```

### 5. Flowchart (Algorithm)
```mermaid
flowchart TD
    Start([Start]) --> Input[Receive Sensor Data]
    Input --> Check{Data Valid?}
    Check -->|No| Error[Log Error]
    Error --> Input
    Check -->|Yes| Process[Process Data]
    Process --> Decide{Obstacle Detected?}
    Decide -->|Yes| Avoid[Compute Avoidance Path]
    Decide -->|No| Continue[Continue Current Path]
    Avoid --> Send
    Continue --> Send
    Send[Send Velocity Command]
    Send --> Input
```

### 6. Class Diagram (OOP Structure)
```mermaid
classDiagram
    class Node {
        +string name
        +Logger logger
        +create_publisher()
        +create_subscription()
        +create_service()
        +destroy_node()
    }

    class RobotController {
        -velocity_publisher
        -position_subscriber
        -current_pose
        +move_forward()
        +rotate()
        +stop()
    }

    class SensorHandler {
        -sensor_subscriptions
        -data_buffer
        +register_sensor()
        +get_latest_data()
    }

    Node <|-- RobotController
    Node <|-- SensorHandler
    RobotController --> SensorHandler: uses
```

## Best Practices
- ✅ Use descriptive node/edge labels
- ✅ Apply consistent styling (colors for component types)
- ✅ Include direction arrows for data flow
- ✅ Group related components in subgraphs
- ✅ Keep diagrams readable (max 10-15 nodes)
- ✅ Use standard ROS 2 naming conventions

## Styling Guide
```
- Perception nodes: Light blue (#e1f5ff)
- Processing nodes: Light yellow (#fff4e1)
- Planning nodes: Light red (#ffe1e1)
- Control nodes: Light green (#e1ffe1)
- Data topics: Solid arrows
- Service calls: Dashed arrows
- Actions: Thick arrows
```

## Example Invocations

**Input**: "Diagram showing a ROS 2 navigation stack with SLAM, path planning, and obstacle avoidance"

**Output**: Multi-layer architecture diagram with sensor inputs, processing nodes, and actuator outputs.

**Input**: "Sequence diagram for a robot arm pick-and-place task with vision and motion planning"

**Output**: Detailed sequence showing camera capture → object detection → motion planning → grasp execution.

## Integration with Docusaurus
All diagrams are Mermaid-compatible and render automatically in Docusaurus markdown files. Just paste the code block with triple backticks and `mermaid` language tag.
