# Unity for High-Fidelity Humanoid Simulation

## Introduction to Unity in Robotics

Unity is a powerful 3D development platform that provides high-fidelity graphics, realistic physics, and flexible scripting capabilities. While Gazebo excels at physics simulation, Unity offers superior visual fidelity and interactive capabilities that are valuable for humanoid robotics development, particularly for:

- Photorealistic rendering for computer vision training
- Complex environment modeling with detailed textures
- Human-in-the-loop testing and teleoperation
- Virtual reality integration for immersive control
- Advanced sensor simulation (camera, LIDAR, etc.)

## Unity ROS 2 Integration

Unity can communicate with ROS 2 through the Unity Robotics Hub, which provides:

- **ROS-TCP-Connector**: Establishes communication between Unity and ROS 2
- **Message Pipelines**: Handles serialization and deserialization of ROS messages
- **Robotics Service**: Manages ROS communication within Unity

### Setting up Unity ROS 2 Bridge

The basic setup involves:

1. Installing the Unity Robotics Hub package
2. Configuring the ROS-TCP-Connector
3. Creating message publishers and subscribers
4. Synchronizing Unity transforms with ROS TF frames

### Basic Unity ROS 2 Connection Script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class HumanoidROSConnection : MonoBehaviour
{
    ROSConnection ros;
    public string rosIP = "127.0.0.1";
    public int rosPort = 10000;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Initialize(rosIP, rosPort);
    }

    void Update()
    {
        // Publish robot position to ROS
        PublishRobotPose();
    }

    void PublishRobotPose()
    {
        // Create a message
        Float32MultiArrayMsg pose_msg = new Float32MultiArrayMsg();

        // Fill with position data
        pose_msg.data = new float[] {
            transform.position.x,
            transform.position.y,
            transform.position.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w
        };

        // Publish to ROS topic
        ros.Send("humanoid_pose", pose_msg);
    }
}
```

## Creating Humanoid Models in Unity

### Importing Robot Models

Unity supports various 3D model formats for importing humanoid robots:

- **FBX**: Most common format for 3D models with animations
- **OBJ**: Simple geometry format
- **DAE**: Collada format that preserves materials and textures

### Rigging and Animation

For humanoid robots in Unity:

1. **Skeleton Setup**: Create a skeleton that matches your robot's joint structure
2. **Inverse Kinematics**: Implement IK for natural movement
3. **Animation Controllers**: Manage different movement states
4. **Blend Trees**: Smoothly transition between different animations

### Example: Humanoid Joint Control

```csharp
using UnityEngine;

public class HumanoidJointController : MonoBehaviour
{
    public Transform head;
    public Transform leftShoulder, leftElbow, leftWrist;
    public Transform rightShoulder, rightElbow, rightWrist;
    public Transform leftHip, leftKnee, leftAnkle;
    public Transform rightHip, rightKnee, rightAnkle;

    // Joint angles from ROS
    public float headYaw = 0f, headPitch = 0f;
    public float leftShoulderAngle = 0f, rightShoulderAngle = 0f;
    public float leftElbowAngle = 0f, rightElbowAngle = 0f;

    void Update()
    {
        // Apply joint angles to Unity transforms
        UpdateHead();
        UpdateArms();
        UpdateLegs();
    }

    void UpdateHead()
    {
        head.localRotation = Quaternion.Euler(headPitch, headYaw, 0);
    }

    void UpdateArms()
    {
        leftShoulder.localRotation = Quaternion.Euler(0, 0, leftShoulderAngle);
        leftElbow.localRotation = Quaternion.Euler(0, 0, leftElbowAngle);

        rightShoulder.localRotation = Quaternion.Euler(0, 0, rightShoulderAngle);
        rightElbow.localRotation = Quaternion.Euler(0, 0, rightElbowAngle);
    }

    void UpdateLegs()
    {
        // Similar for legs
        leftHip.localRotation = Quaternion.Euler(0, 0, 0); // Controlled by input
        leftKnee.localRotation = Quaternion.Euler(0, 0, 0);
        leftAnkle.localRotation = Quaternion.Euler(0, 0, 0);

        rightHip.localRotation = Quaternion.Euler(0, 0, 0);
        rightKnee.localRotation = Quaternion.Euler(0, 0, 0);
        rightAnkle.localRotation = Quaternion.Euler(0, 0, 0);
    }
}
```

## High-Fidelity Sensor Simulation

### Camera Simulation

Unity excels at camera simulation with realistic rendering:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class UnityCameraSimulator : MonoBehaviour
{
    Camera cam;
    ROSConnection ros;

    public string imageTopic = "camera/image_raw";
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float publishRate = 30.0f; // Hz

    RenderTexture renderTexture;
    Texture2D texture2D;

    float lastPublishTime;

    void Start()
    {
        cam = GetComponent<Camera>();
        ros = ROSConnection.instance;

        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        cam.targetTexture = renderTexture;

        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        lastPublishTime = Time.time;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= 1.0f / publishRate)
        {
            PublishCameraImage();
            lastPublishTime = Time.time;
        }
    }

    void PublishCameraImage()
    {
        // Read pixels from render texture
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();

        // Convert to ROS image format
        byte[] imageBytes = texture2D.EncodeToJPG();

        // Publish via ROS (simplified - actual implementation would require proper ROS image message)
        // ros.Send(imageTopic, imageBytes);
    }
}
```

### LiDAR Simulation

Unity can simulate LiDAR sensors using raycasting:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class UnityLidarSimulator : MonoBehaviour
{
    public int rayCount = 360;
    public float maxDistance = 10.0f;
    public float fov = 360.0f;

    List<float> ranges;

    void Start()
    {
        ranges = new List<float>(new float[rayCount]);
    }

    void Update()
    {
        SimulateLidar();
    }

    void SimulateLidar()
    {
        float angleStep = fov / rayCount;

        for (int i = 0; i < rayCount; i++)
        {
            float angle = transform.eulerAngles.y + (i * angleStep) - (fov / 2);
            Vector3 direction = Quaternion.Euler(0, angle, 0) * transform.forward;

            RaycastHit hit;
            if (Physics.Raycast(transform.position, direction, out hit, maxDistance))
            {
                ranges[i] = hit.distance;
            }
            else
            {
                ranges[i] = maxDistance;
            }
        }

        // Publish ranges to ROS
        PublishLidarData();
    }

    void PublishLidarData()
    {
        // Convert ranges to ROS LaserScan message and publish
        // Implementation would depend on ROS-TCP-Connector setup
    }
}
```

## Environment Design for Humanoid Robots

### Creating Realistic Environments

Unity's environment tools are ideal for creating realistic testing scenarios:

- **Terrain Tools**: Create outdoor environments with realistic ground
- **Lighting**: Dynamic lighting that affects computer vision algorithms
- **Weather Systems**: Rain, fog, and other conditions for robustness testing
- **Dynamic Objects**: Moving obstacles and interactive elements

### Example: Indoor Environment Setup

```csharp
using UnityEngine;

public class EnvironmentSetup : MonoBehaviour
{
    public GameObject[] furniturePrefabs;
    public int furnitureCount = 20;
    public Vector2 environmentSize = new Vector2(20, 20);

    void Start()
    {
        GenerateEnvironment();
    }

    void GenerateEnvironment()
    {
        for (int i = 0; i < furnitureCount; i++)
        {
            // Randomly place furniture
            Vector3 position = new Vector3(
                Random.Range(-environmentSize.x / 2, environmentSize.x / 2),
                0,
                Random.Range(-environmentSize.y / 2, environmentSize.y / 2)
            );

            // Choose random furniture type
            GameObject furniture = Instantiate(
                furniturePrefabs[Random.Range(0, furniturePrefabs.Length)],
                position,
                Quaternion.identity
            );

            // Ensure it doesn't collide with robot starting position
            if (Vector3.Distance(position, Vector3.zero) < 3.0f)
            {
                Destroy(furniture);
                i--; // Retry this placement
            }
        }
    }
}
```

## Virtual Reality Integration

Unity enables VR integration for immersive humanoid control:

```csharp
using UnityEngine;
using UnityEngine.XR;

public class VRHumanoidController : MonoBehaviour
{
    public Transform humanoidRobot;
    public Transform vrHeadset;
    public Transform[] vrControllers;

    void Update()
    {
        // Map VR headset position to robot head
        humanoidRobot.Find("Head").position = vrHeadset.position;
        humanoidRobot.Find("Head").rotation = vrHeadset.rotation;

        // Map controller inputs to robot hands
        if (vrControllers.Length >= 2)
        {
            humanoidRobot.Find("LeftHand").position = vrControllers[0].position;
            humanoidRobot.Find("RightHand").position = vrControllers[1].position;
        }
    }
}
```

## Unity vs Gazebo: When to Use Each

### Use Unity for:
- High-fidelity graphics and rendering
- Computer vision training with photorealistic images
- Human-in-the-loop testing
- Virtual reality applications
- Complex visual environments
- User interface development

### Use Gazebo for:
- Accurate physics simulation
- Realistic joint dynamics and contact forces
- Standard ROS sensor simulation
- Fast simulation for reinforcement learning
- Hardware-in-the-loop testing
- Standard robotics algorithms testing

## Best Practices for Unity Robotics

1. **Performance Optimization**: Use occlusion culling, LOD, and optimized meshes
2. **Realistic Lighting**: Include shadows, reflections, and dynamic lighting
3. **Physics Accuracy**: Balance visual fidelity with physics accuracy
4. **ROS Integration**: Maintain consistent TF frames and message formats
5. **Cross-Platform**: Ensure simulation works across different Unity platforms
6. **Asset Management**: Organize 3D models, materials, and scenes efficiently

## Advanced Unity Robotics Features

### Machine Learning Integration
Unity supports ML-Agents for training AI in simulation:

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class HumanoidAgent : Agent
{
    public override void OnEpisodeBegin()
    {
        // Reset robot position and environment
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Collect observations for ML training
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.rotation);
        // Add other sensor data
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Apply actions to humanoid robot
        float action = actions.ContinuousActions[0];
        // Move robot based on action
    }
}
```

### Multi-Simulation Environments
Create multiple parallel environments for faster training:

```csharp
// Unity can run multiple instances for parallel training
// Each instance runs in a separate Unity scene
// Results are aggregated for faster learning
```

## Exercises

1. Create a Unity scene with a humanoid robot and basic environment
2. Implement ROS communication for joint position control
3. Set up camera simulation and publish images to ROS
4. Create a VR interface for controlling the humanoid robot
5. Implement LiDAR simulation using Unity's physics raycasting