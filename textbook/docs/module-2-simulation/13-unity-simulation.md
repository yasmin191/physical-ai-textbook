---
sidebar_position: 5
title: "Chapter 13: Unity for High-Fidelity Simulation"
description: "Create photorealistic simulations and human-robot interaction scenarios using Unity"
---

# Chapter 13: Unity for High-Fidelity Simulation

While Gazebo excels at physics simulation, **Unity** provides superior visual fidelity and is ideal for human-robot interaction (HRI) scenarios, synthetic data generation, and user studies. This chapter explores using Unity alongside ROS 2 for comprehensive humanoid robot development.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand Unity's role in robotics simulation
- Set up Unity with ROS 2 integration
- Create photorealistic environments for robot testing
- Implement human-robot interaction scenarios
- Generate synthetic training data for machine learning

## 13.1 Unity vs Gazebo: Complementary Tools

### Comparison Matrix

| Aspect | Gazebo | Unity |
|--------|--------|-------|
| **Physics accuracy** | Excellent | Good |
| **Visual quality** | Good | Excellent |
| **Real-time rendering** | Limited | Excellent |
| **Asset availability** | Limited | Extensive |
| **Human models** | Basic | Photorealistic |
| **VR/AR support** | Limited | Native |
| **ROS integration** | Native | Via package |
| **Learning curve** | Moderate | Steeper |

### When to Use Each

**Use Gazebo for:**
- Physics-critical simulations (walking, manipulation)
- Control algorithm development
- Sensor algorithm testing
- Continuous integration testing

**Use Unity for:**
- Human-robot interaction studies
- Synthetic data generation
- Visualization and demos
- VR/AR robot interfaces
- User experience testing

## 13.2 Unity Robotics Hub

### Installation

1. **Install Unity Hub** from [unity.com](https://unity.com)
2. **Install Unity Editor** (2021.3 LTS or later recommended)
3. **Add Robotics packages** via Package Manager:

```
Window > Package Manager > + > Add package from git URL

https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

### Project Structure

```
UnityHumanoidProject/
├── Assets/
│   ├── Robots/
│   │   ├── Humanoid/
│   │   │   ├── humanoid.urdf
│   │   │   ├── Meshes/
│   │   │   └── Materials/
│   ├── Environments/
│   │   ├── IndoorLab/
│   │   └── OutdoorPark/
│   ├── Scripts/
│   │   ├── RobotController.cs
│   │   └── ROSConnection.cs
│   ├── Prefabs/
│   └── Scenes/
├── Packages/
└── ProjectSettings/
```

## 13.3 Importing Robot Models

### URDF Import Process

1. **Place URDF and meshes** in `Assets/Robots/`
2. **Import via menu**: `Assets > Import Robot from URDF`
3. **Configure import settings**:

```csharp
// Import settings (via script)
using Unity.Robotics.UrdfImporter;

public class RobotImporter : MonoBehaviour
{
    void ImportRobot()
    {
        ImportSettings settings = new ImportSettings
        {
            chosenAxis = ImportSettings.axisType.yAxis,
            convexMethod = ImportSettings.convexDecomposer.vHACD,
            // Physics settings
            useUrdfInertiaData = true,
            useGravity = true
        };
        
        UrdfRobotExtensions.Create("Assets/Robots/humanoid.urdf", settings);
    }
}
```

### Articulation Body System

Unity uses **Articulation Bodies** for robotics (not Rigidbodies):

```csharp
using UnityEngine;

public class JointController : MonoBehaviour
{
    private ArticulationBody[] joints;
    
    void Start()
    {
        // Get all articulation bodies in hierarchy
        joints = GetComponentsInChildren<ArticulationBody>();
    }
    
    public void SetJointTarget(int jointIndex, float targetPosition)
    {
        var joint = joints[jointIndex];
        var drive = joint.xDrive;
        drive.target = targetPosition * Mathf.Rad2Deg;
        joint.xDrive = drive;
    }
    
    public float GetJointPosition(int jointIndex)
    {
        return joints[jointIndex].jointPosition[0];
    }
}
```

## 13.4 ROS 2 Integration

### ROS-TCP-Connector Setup

**In Unity:**
```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Geometry;

public class ROSHumanoidBridge : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        // Initialize ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointStateMsg>("/joint_states");
        ros.Subscribe<TwistMsg>("/cmd_vel", OnCmdVelReceived);
    }
    
    void OnCmdVelReceived(TwistMsg msg)
    {
        // Apply velocity command to robot
        float linearX = (float)msg.linear.x;
        float angularZ = (float)msg.angular.z;
        // Move robot...
    }
    
    void PublishJointStates()
    {
        JointStateMsg msg = new JointStateMsg();
        msg.name = new string[] { "hip_pitch", "knee_pitch", "ankle_pitch" };
        msg.position = new double[] { 0.0, 0.5, -0.3 };
        ros.Publish("/joint_states", msg);
    }
}
```

**On ROS 2 side:**
```bash
# Start the TCP endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

### Message Definitions

Generate C# message classes from ROS messages:

```bash
# In Unity, use the Message Generation tool
# Window > Robotics > Generate ROS Messages

# Or via command line
ros2 run ros_tcp_endpoint generate_messages.py --output-path <unity_project>/Assets/RosMessages
```

## 13.5 Photorealistic Rendering

### High-Definition Render Pipeline (HDRP)

For photorealistic visuals:

1. **Create HDRP project** or convert existing
2. **Configure quality settings**:

```
Edit > Project Settings > Quality
- Shadows: Ultra
- Anti-aliasing: TAA
- Ambient Occlusion: Enabled
```

3. **Use physical lighting**:

```csharp
// Physically-based lighting setup
Light sun = GameObject.Find("Sun").GetComponent<Light>();
sun.type = LightType.Directional;
sun.intensity = 100000; // Lux (outdoor sunlight)
sun.colorTemperature = 6500; // Kelvin (daylight)
sun.useColorTemperature = true;
```

### Materials and Textures

```csharp
// Create realistic robot materials
Material robotMaterial = new Material(Shader.Find("HDRP/Lit"));
robotMaterial.SetFloat("_Metallic", 0.8f);
robotMaterial.SetFloat("_Smoothness", 0.6f);
robotMaterial.SetColor("_BaseColor", new Color(0.7f, 0.7f, 0.7f));
```

## 13.6 Human Models and Animation

### Importing Human Characters

Unity supports humanoid rigs with **Mecanim**:

```csharp
using UnityEngine;

public class HumanCharacterController : MonoBehaviour
{
    private Animator animator;
    
    void Start()
    {
        animator = GetComponent<Animator>();
    }
    
    public void Walk(float speed)
    {
        animator.SetFloat("Speed", speed);
    }
    
    public void PointAt(Vector3 worldPosition)
    {
        animator.SetIKPositionWeight(AvatarIKGoal.RightHand, 1);
        animator.SetIKPosition(AvatarIKGoal.RightHand, worldPosition);
    }
}
```

### Motion Capture Integration

```csharp
// Stream mocap data to human model
public class MocapReceiver : MonoBehaviour
{
    public Transform[] bones;
    
    void Update()
    {
        // Receive mocap data via network
        MocapFrame frame = ReceiveMocapFrame();
        
        for (int i = 0; i < bones.Length; i++)
        {
            bones[i].localRotation = frame.rotations[i];
        }
    }
}
```

## 13.7 Human-Robot Interaction Scenarios

### Scenario: Robot Assistant

```csharp
public class RobotAssistant : MonoBehaviour
{
    public Transform humanHead;
    public Transform robotHead;
    
    void Update()
    {
        // Robot looks at human
        Vector3 lookDirection = humanHead.position - robotHead.position;
        robotHead.rotation = Quaternion.LookRotation(lookDirection);
    }
    
    public void RespondToGesture(GestureType gesture)
    {
        switch (gesture)
        {
            case GestureType.Wave:
                PlayAnimation("wave_back");
                break;
            case GestureType.PointAt:
                NavigateToPointedLocation();
                break;
            case GestureType.Come:
                ApproachHuman();
                break;
        }
    }
}
```

### Scenario: Collaborative Task

```csharp
public class CollaborativeTask : MonoBehaviour
{
    public Transform robotHand;
    public Transform humanHand;
    public GameObject sharedObject;
    
    private TaskState state = TaskState.WaitingForHuman;
    
    void Update()
    {
        switch (state)
        {
            case TaskState.WaitingForHuman:
                if (HumanOffersObject())
                {
                    state = TaskState.Receiving;
                    MoveHandToReceive();
                }
                break;
                
            case TaskState.Receiving:
                if (ObjectInGripper())
                {
                    CloseGripper();
                    state = TaskState.Holding;
                }
                break;
                
            case TaskState.Holding:
                // Process object or hand back
                break;
        }
    }
}
```

## 13.8 Synthetic Data Generation

### Camera Sensor Simulation

```csharp
public class SyntheticDataGenerator : MonoBehaviour
{
    public Camera sensorCamera;
    public int imageWidth = 640;
    public int imageHeight = 480;
    
    private RenderTexture renderTexture;
    private Texture2D outputTexture;
    
    void Start()
    {
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        outputTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        sensorCamera.targetTexture = renderTexture;
    }
    
    public byte[] CaptureImage()
    {
        RenderTexture.active = renderTexture;
        outputTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        outputTexture.Apply();
        RenderTexture.active = null;
        
        return outputTexture.EncodeToPNG();
    }
    
    public void CaptureWithAnnotations(string savePath)
    {
        // Capture RGB image
        byte[] rgbImage = CaptureImage();
        File.WriteAllBytes($"{savePath}/rgb.png", rgbImage);
        
        // Capture segmentation mask
        SetSegmentationMode();
        byte[] segImage = CaptureImage();
        File.WriteAllBytes($"{savePath}/segmentation.png", segImage);
        
        // Capture depth
        SetDepthMode();
        byte[] depthImage = CaptureImage();
        File.WriteAllBytes($"{savePath}/depth.png", depthImage);
        
        // Save bounding boxes
        SaveBoundingBoxes($"{savePath}/annotations.json");
    }
}
```

### Domain Randomization

```csharp
public class DomainRandomizer : MonoBehaviour
{
    public Light[] lights;
    public Material[] materials;
    public GameObject[] distractorObjects;
    
    public void RandomizeEnvironment()
    {
        // Randomize lighting
        foreach (var light in lights)
        {
            light.intensity = Random.Range(0.5f, 2.0f);
            light.color = Random.ColorHSV(0, 1, 0.5f, 1, 0.5f, 1);
        }
        
        // Randomize materials
        foreach (var mat in materials)
        {
            mat.color = Random.ColorHSV();
        }
        
        // Randomize distractor positions
        foreach (var obj in distractorObjects)
        {
            obj.transform.position = new Vector3(
                Random.Range(-5f, 5f),
                Random.Range(0f, 2f),
                Random.Range(-5f, 5f)
            );
            obj.transform.rotation = Random.rotation;
        }
    }
}
```

## 13.9 Perception Pipeline

### Object Detection Integration

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Vision;

public class ObjectDetector : MonoBehaviour
{
    private ROSConnection ros;
    
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<Detection2DArrayMsg>("/detections");
    }
    
    void OnImageCaptured(Texture2D image)
    {
        // Run detection model (e.g., via Barracuda)
        List<Detection> detections = RunDetectionModel(image);
        
        // Convert to ROS message
        Detection2DArrayMsg msg = new Detection2DArrayMsg();
        msg.detections = detections.Select(d => new Detection2DMsg
        {
            bbox = new BoundingBox2DMsg
            {
                center = new Pose2DMsg { x = d.centerX, y = d.centerY },
                size_x = d.width,
                size_y = d.height
            },
            results = new ObjectHypothesisWithPoseMsg[]
            {
                new ObjectHypothesisWithPoseMsg { id = d.classId, score = d.confidence }
            }
        }).ToArray();
        
        ros.Publish("/detections", msg);
    }
}
```

## 13.10 VR/AR Robot Teleoperation

### VR Controller Mapping

```csharp
using UnityEngine.XR;

public class VRTeleoperation : MonoBehaviour
{
    public Transform robotRightHand;
    public Transform robotLeftHand;
    
    private InputDevice rightController;
    private InputDevice leftController;
    
    void Start()
    {
        var rightHandDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, rightHandDevices);
        if (rightHandDevices.Count > 0)
            rightController = rightHandDevices[0];
            
        var leftHandDevices = new List<InputDevice>();
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, leftHandDevices);
        if (leftHandDevices.Count > 0)
            leftController = leftHandDevices[0];
    }
    
    void Update()
    {
        // Map VR controller to robot hand
        if (rightController.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 rightPos))
        {
            robotRightHand.position = TransformToRobotSpace(rightPos);
        }
        
        if (rightController.TryGetFeatureValue(CommonUsages.deviceRotation, out Quaternion rightRot))
        {
            robotRightHand.rotation = rightRot;
        }
        
        // Gripper control via trigger
        if (rightController.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue))
        {
            SetGripperPosition(triggerValue);
        }
    }
}
```

## 13.11 Performance Optimization

### Level of Detail (LOD)

```csharp
// Configure LOD for robot meshes
LODGroup lodGroup = gameObject.AddComponent<LODGroup>();
LOD[] lods = new LOD[3];

lods[0] = new LOD(0.5f, new Renderer[] { highDetailRenderer });
lods[1] = new LOD(0.2f, new Renderer[] { mediumDetailRenderer });
lods[2] = new LOD(0.05f, new Renderer[] { lowDetailRenderer });

lodGroup.SetLODs(lods);
lodGroup.RecalculateBounds();
```

### Physics Optimization

```csharp
// Reduce physics overhead
Physics.defaultSolverIterations = 6; // Default is 6
Physics.defaultSolverVelocityIterations = 1;

// Use simplified colliders for non-critical objects
MeshCollider meshCol = GetComponent<MeshCollider>();
meshCol.convex = true; // Much faster than concave
```

## 13.12 Summary

In this chapter, you learned:

- **Unity's role**: High-fidelity visuals and HRI scenarios
- **Robot import**: URDF to Unity with Articulation Bodies
- **ROS integration**: Bidirectional communication via TCP
- **Photorealistic rendering**: HDRP for synthetic data
- **Human models**: Animation and interaction
- **Synthetic data**: Domain randomization for ML training
- **VR teleoperation**: Immersive robot control

Unity complements Gazebo by providing superior visual fidelity for perception training and human studies. Together, they form a comprehensive simulation toolkit for humanoid development.

## Review Questions

1. When would you choose Unity over Gazebo for simulation?
2. How does the Articulation Body system differ from standard Rigidbodies?
3. What is domain randomization and why is it important?
4. How do you establish communication between Unity and ROS 2?
5. What are the benefits of VR for robot teleoperation?

## Hands-On Exercise

1. Set up a new Unity project with robotics packages
2. Import your humanoid URDF into Unity
3. Configure the ROS-TCP connection
4. Create a simple indoor environment
5. Add a human character that can interact with the robot
6. Implement a basic image capture for synthetic data
