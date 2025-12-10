---
id: unity-rendering
title: "Unity for High-Fidelity Robot Rendering"
sidebar_position: 4
description: "Use Unity for photorealistic robot visualization, HRI scenarios, and ROS 2 integration"
---

# Unity for High-Fidelity Robot Rendering

While Gazebo excels at physics simulation, **Unity**—the game engine powering millions of games and simulations—offers unparalleled rendering quality, asset ecosystems, and human-robot interaction (HRI) capabilities. From photorealistic indoor environments to character animations for human coworkers, Unity transforms your robot from a collection of cylinders and boxes into a lifelike agent operating in believable spaces.

In this chapter, you'll discover why Unity complements Gazebo, learn to integrate Unity with ROS 2 using the Unity Robotics Hub, explore asset workflows for realistic environments, and understand when to choose Unity over traditional robotics simulators.

## Why Unity for Robotics?

### Rendering Quality

**Unity's Universal Render Pipeline (URP)** and **High Definition Render Pipeline (HDRP)** deliver:
- **Real-time global illumination**: Light bounces off surfaces naturally
- **Post-processing effects**: Bloom, motion blur, depth of field for cinematic quality
- **Physically-based materials**: Realistic metal, wood, fabric shaders
- **High-quality shadows**: Soft shadows, contact shadows, cascaded shadow maps

**Comparison to Gazebo**:
| Feature | Gazebo (OGRE 2.x) | Unity HDRP |
|---------|-------------------|-----------|
| Global Illumination | Limited baked lightmaps | Real-time ray-traced GI |
| Material System | Basic PBR | Full PBR with layered materials |
| Character Animation | Limited COLLADA support | Full humanoid animation system |
| Post-Processing | Minimal | 30+ effects (DOF, bloom, color grading) |
| Asset Store | Limited Fuel models | 200,000+ assets (Unity Asset Store) |

**Example**: A humanoid serving coffee in a café looks like a tech demo in Gazebo, but appears photorealistic in Unity with proper lighting, reflections, and textures.

### Human-Robot Interaction (HRI)

Unity's game development heritage makes it ideal for HRI research:

**Character Animation**:
```csharp
// Control a virtual human with Unity's Animator
public class HumanBehavior : MonoBehaviour
{
    private Animator animator;

    void Start()
    {
        animator = GetComponent<Animator>();
    }

    public void ReactToRobotApproach()
    {
        // Trigger "wave" animation when robot approaches
        animator.SetTrigger("WaveGreeting");
        StartCoroutine(SpeakDialogue("Hello, robot!"));
    }
}
```

**Use cases**:
- **Service robotics**: Simulate robots delivering items to animated humans
- **Social robotics**: Test gaze behavior, personal space, gesture recognition
- **Teleoperation interfaces**: High-fidelity rendering for remote robot control
- **Training data**: Generate synthetic datasets with diverse human poses

### Asset Ecosystem

Unity Asset Store provides ready-made resources:
- **Environments**: Apartments, offices, warehouses, hospitals
- **Characters**: Rigged humanoid models with animations
- **Props**: Furniture, dishes, tools, electronics
- **Shaders**: Glass, liquids, holograms, UI elements

**Example workflow**:
1. Purchase "Modern Apartment" asset ($15-50)
2. Import humanoid robot URDF
3. Add animated human character from Mixamo (free)
4. Implement robot navigation and task execution
5. Record video for conference presentation

**Result**: Professional-quality demo in hours, not weeks.

## Unity Robotics Hub: Bridging ROS 2 and Unity

The **Unity Robotics Hub** (open-source by Unity Technologies) enables bidirectional communication between ROS 2 and Unity.

### Installation

**ROS 2 side** (install ROS-TCP-Endpoint):
```bash
cd ~/ros2_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
cd ~/ros2_ws
colcon build --packages-select ros_tcp_endpoint
source install/setup.bash

# Launch endpoint (default port 10000)
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Unity side** (install via Package Manager):
1. Open Unity Editor (2021.3+ recommended)
2. Window → Package Manager
3. Add package from git URL: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
4. Add URDF Importer: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

**Configure connection**:
```csharp
// In Unity: Robotics → ROS Settings
ROS IP Address: 192.168.1.100  // Your ROS 2 machine IP
ROS Port: 10000
Protocol: ROS 2
```

### Publishing ROS 2 Topics from Unity

**Example**: Send robot's camera feed from Unity to ROS 2 for perception processing:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraPublisher : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/unity/camera/image_raw";
    private Camera mainCamera;

    void Start()
    {
        // Connect to ROS
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<ImageMsg>(topicName);

        mainCamera = GetComponent<Camera>();

        InvokeRepeating("PublishImage", 1.0f, 0.033f); // 30 FPS
    }

    void PublishImage()
    {
        // Capture camera texture
        RenderTexture rt = new RenderTexture(640, 480, 24);
        mainCamera.targetTexture = rt;
        mainCamera.Render();

        // Convert to byte array
        Texture2D texture = new Texture2D(640, 480, TextureFormat.RGB24, false);
        RenderTexture.active = rt;
        texture.ReadPixels(new Rect(0, 0, 640, 480), 0, 0);
        texture.Apply();

        byte[] imageData = texture.EncodeToPNG();

        // Create ROS message
        ImageMsg msg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg
                {
                    sec = (int)Time.time,
                    nanosec = (uint)((Time.time % 1) * 1e9)
                },
                frame_id = "unity_camera"
            },
            height = 480,
            width = 640,
            encoding = "rgb8",
            is_bigendian = 0,
            step = 640 * 3,
            data = imageData
        };

        ros.Publish(topicName, msg);

        // Cleanup
        mainCamera.targetTexture = null;
        RenderTexture.active = null;
        Destroy(rt);
    }
}
```

**ROS 2 verification**:
```bash
ros2 topic list
# Should show /unity/camera/image_raw

ros2 topic hz /unity/camera/image_raw
# Should show ~30 Hz

ros2 run rqt_image_view rqt_image_view /unity/camera/image_raw
# Visual confirmation
```

### Subscribing to ROS 2 Topics in Unity

**Example**: Move Unity robot based on ROS 2 velocity commands:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocityController : MonoBehaviour
{
    private ROSConnection ros;
    private string topicName = "/cmd_vel";

    private Vector3 linearVelocity = Vector3.zero;
    private Vector3 angularVelocity = Vector3.zero;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<TwistMsg>(topicName, ReceiveVelocityCommand);
    }

    void ReceiveVelocityCommand(TwistMsg msg)
    {
        // Convert ROS coordinate frame to Unity (Y-up, Z-forward)
        linearVelocity = new Vector3(
            (float)msg.linear.x,
            (float)msg.linear.z,  // ROS Z → Unity Y
            (float)msg.linear.y   // ROS Y → Unity Z
        );

        angularVelocity = new Vector3(
            (float)msg.angular.x,
            (float)msg.angular.z,
            (float)msg.angular.y
        );
    }

    void Update()
    {
        // Apply velocities to robot
        transform.Translate(linearVelocity * Time.deltaTime, Space.Self);
        transform.Rotate(angularVelocity * Time.deltaTime, Space.Self);
    }
}
```

**ROS 2 command**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

## Importing URDF into Unity

Unity's URDF Importer converts robot descriptions directly:

**Method 1: Via UI**:
1. Assets → Import Robot from URDF
2. Select your `.urdf` file
3. Configure:
   - **Joint Type**: Articulation Body (PhysX) or Hinge Joint
   - **Axis Type**: Default or Custom
   - **Mesh Decomposer**: VHACD (for collision)
4. Click Import

**Method 2: Via Script**:
```csharp
using UnityEngine;
using Unity.Robotics.UrdfImporter;

public class RobotLoader : MonoBehaviour
{
    void Start()
    {
        string urdfPath = "Assets/Robots/humanoid_arm.urdf";

        GameObject robotRoot = UrdfRobotExtensions.CreateRuntime(
            urdfPath,
            new ImportSettings
            {
                convexMethod = ImportSettings.ConvexMethod.VHACD
            }
        );

        robotRoot.transform.position = new Vector3(0, 1, 0);
    }
}
```

**Coordinate frame conversion**:
- **URDF**: X-forward, Y-left, Z-up (ROS convention)
- **Unity**: X-right, Y-up, Z-forward
- **URDF Importer**: Automatically handles rotation (90° around X-axis)

**Common issues**:
1. **Missing materials**: Unity imports geometry but materials are gray → Assign materials manually
2. **Joint flipping**: Joints move in wrong direction → Check axis signs in URDF
3. **Scale mismatch**: Robot is tiny or huge → URDF uses meters, Unity uses meters (should match)

## Creating Realistic Environments

### Lighting Setup (HDRP)

**Step 1**: Enable HDRP
1. Window → Package Manager → High Definition RP → Install
2. Edit → Project Settings → Graphics → Set to HDRenderPipelineAsset
3. Window → Rendering → Lighting → Generate Lighting

**Step 2**: Add lights
```csharp
// Directional light (sun)
GameObject sun = new GameObject("Sun");
Light sunLight = sun.AddComponent<Light>();
sunLight.type = LightType.Directional;
sunLight.intensity = 1.5f;
sunLight.color = new Color(1.0f, 0.95f, 0.85f);
sun.transform.rotation = Quaternion.Euler(50, -30, 0);

// Point light (lamp)
GameObject lamp = new GameObject("Lamp");
Light lampLight = lamp.AddComponent<Light>();
lampLight.type = LightType.Point;
lampLight.range = 10.0f;
lampLight.intensity = 500;
lamp.transform.position = new Vector3(2, 2, 0);
```

**Step 3**: Bake global illumination
1. Window → Rendering → Lighting Settings
2. Check "Baked Global Illumination"
3. Generate Lighting

**Result**: Soft shadows, realistic light bounces, ambient occlusion

### Asset Integration

**Import from Asset Store**:
1. Window → Asset Store
2. Search "Modern Office Interior"
3. Purchase/Download → Import
4. Drag prefab into scene

**Spawn robot in environment**:
```csharp
public class SceneSetup : MonoBehaviour
{
    public GameObject robotPrefab;
    public GameObject officePrefab;

    void Start()
    {
        // Spawn office
        Instantiate(officePrefab, Vector3.zero, Quaternion.identity);

        // Spawn robot at desk
        Vector3 deskPosition = new Vector3(3.0f, 0.0f, 2.0f);
        Instantiate(robotPrefab, deskPosition, Quaternion.identity);
    }
}
```

### Human Character Animation

**Using Mixamo** (free):
1. Visit mixamo.com
2. Select character model (e.g., "Mannequin")
3. Choose animation (e.g., "Walking")
4. Download FBX for Unity
5. Import to Unity → Assets → Import New Asset

**Attach to scene**:
```csharp
public class HumanController : MonoBehaviour
{
    private Animator animator;
    private NavMeshAgent agent;

    void Start()
    {
        animator = GetComponent<Animator>();
        agent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        // Sync animation speed with movement
        float speed = agent.velocity.magnitude;
        animator.SetFloat("Speed", speed);
    }

    public void WalkToRobot(Vector3 robotPosition)
    {
        agent.SetDestination(robotPosition);
    }
}
```

## When to Use Unity vs Gazebo

| Use Case | Best Tool | Rationale |
|----------|-----------|-----------|
| **Physics-accurate control** | Gazebo | Validated physics engines (DART, ODE) |
| **Reinforcement learning** | Isaac Sim / Gazebo | GPU-accelerated parallel simulation |
| **HRI studies** | Unity | Character animation, facial expressions |
| **Marketing videos** | Unity | Photorealistic rendering, post-processing |
| **Sensor simulation (LIDAR, IMU)** | Gazebo | Accurate sensor models with noise |
| **VR teleoperation** | Unity | Native VR support (Oculus, SteamVR) |
| **Multi-robot swarms** | Gazebo | Optimized for large robot counts |
| **Indoor service robots** | Unity | Asset-rich environments (kitchens, hospitals) |

**Hybrid workflow**:
1. Develop control algorithms in Gazebo (accurate physics)
2. Port to Unity for visualization and HRI testing
3. Use ROS-TCP-Connector to run same control code

## Performance Optimization

### Rendering

**Reduce draw calls**:
```csharp
// Combine static meshes
StaticBatchingUtility.Combine(gameObject);

// Use GPU instancing for repeated objects
material.enableInstancing = true;
```

**Level of Detail (LOD)**:
```csharp
LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
Renderer[] renderers = GetComponentsInChildren<Renderer>();

LOD[] lods = new LOD[3];
lods[0] = new LOD(0.6f, renderers);  // High quality (60% screen)
lods[1] = new LOD(0.3f, GetLOD1Renderers());  // Medium (30%)
lods[2] = new LOD(0.1f, GetLOD2Renderers());  // Low (10%)

lodGroup.SetLODs(lods);
lodGroup.RecalculateBounds();
```

### Physics

**Simplify colliders**:
```csharp
// Bad: Mesh collider with 10,000 triangles
MeshCollider meshCollider = GetComponent<MeshCollider>();

// Good: Approximate with primitives
BoxCollider boxCollider = gameObject.AddComponent<BoxCollider>();
boxCollider.size = new Vector3(0.5f, 1.0f, 0.3f);
```

**Reduce physics timestep** (if simulation is stable):
```csharp
Time.fixedDeltaTime = 0.02f;  // 50 Hz instead of 100 Hz
```

## Practical Exercise

**Challenge**: Create a humanoid robot serving coffee in a Unity café:

1. Import humanoid URDF into Unity
2. Download "Café Interior" from Asset Store (or create simple room)
3. Add animated human character (Mixamo)
4. Implement:
   - ROS 2 subscriber for robot navigation commands
   - ROS 2 publisher for camera feed
   - Collision detection when robot reaches human
5. Record 30-second video

**Success criteria**:
- [ ] Robot navigates from counter to table (via ROS 2 /cmd_vel)
- [ ] Camera feed streams to ROS 2 at ≥ 15 FPS
- [ ] Human waves when robot approaches (animation trigger)
- [ ] Scene runs at ≥ 30 FPS (check Profiler)
- [ ] Video shows photorealistic rendering

## Key Takeaways

✅ **Unity strengths**: Photorealistic rendering, HRI, asset ecosystem
✅ **Unity Robotics Hub**: Bidirectional ROS 2 integration
✅ **URDF Importer**: Convert robot descriptions to Unity GameObjects
✅ **HDRP**: High-fidelity rendering with global illumination
✅ **Character animation**: Mixamo for animated humans
✅ **Use cases**: Marketing, HRI, VR teleoperation, indoor scenarios
✅ **Hybrid workflow**: Develop in Gazebo, visualize in Unity

**Next steps**: Transition to Module 3 to explore NVIDIA Isaac Sim, which combines Unity-level rendering with Gazebo-level physics accuracy using RTX ray-tracing and GPU PhysX.

---

**Related chapters**:
- [Gazebo Simulation](./gazebo-simulation.md) - Physics-focused simulation
- [Physics Simulation](./physics-simulation.md) - Understanding simulation fundamentals
- [Isaac Sim](../module3/isaac-sim.md) - Next module: GPU-accelerated alternative
