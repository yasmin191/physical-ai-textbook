---
sidebar_position: 3
title: "Chapter 3: Sensor Systems"
description: "LIDAR, cameras, IMUs, and force/torque sensors for robotics"
---

# Sensor Systems for Humanoid Robots

## Learning Objectives

By the end of this chapter, you will be able to:

1. Identify and explain the function of key sensor types used in humanoid robots
2. Understand sensor specifications and how to interpret datasheets
3. Compare different sensing modalities and their trade-offs
4. Select appropriate sensors for specific robotic applications

## Prerequisites

- [Chapter 1: Introduction to Physical AI](/module-1-ros2/intro-physical-ai)
- Basic understanding of physics (light, motion, forces)

## The Sensing Challenge

Robots must perceive their environment to interact safely and effectively. Unlike humans with billions of years of evolutionary refinement, robots rely on engineered sensors that each have specific strengths and limitations.

```
Human Perception          Robot Perception
───────────────────       ──────────────────
Eyes (vision)         →   Cameras, LIDAR, Depth sensors
Inner ear (balance)   →   IMU (Inertial Measurement Unit)
Skin (touch)          →   Force/torque sensors, tactile arrays
Proprioception        →   Joint encoders, motor current
```

## Vision Sensors

### RGB Cameras

Standard cameras capture color images, providing rich visual information.

**Specifications to Consider**:

| Specification | Typical Range | Impact |
|--------------|---------------|--------|
| Resolution | 720p - 4K | Detail, processing load |
| Frame rate | 30-120 fps | Motion capture quality |
| Field of View | 60° - 180° | Coverage area |
| Dynamic range | 60-120 dB | Low light performance |

**Common Cameras in Robotics**:
- Intel RealSense (D435, D455)
- Stereolabs ZED
- FLIR/Point Grey
- OAK-D (with on-device AI)

### Depth Sensors

Depth sensors provide 3D information about the environment.

**Technologies**:

```
Stereo Vision
├── Two cameras, compute disparity
├── Works outdoors
├── Accuracy decreases with distance
└── Examples: ZED, RealSense

Structured Light
├── Projects pattern, analyzes distortion
├── High accuracy at close range
├── Fails in sunlight
└── Examples: Kinect v1, RealSense SR300

Time-of-Flight (ToF)
├── Measures light travel time
├── Works in various lighting
├── Lower resolution
└── Examples: Kinect v2, RealSense L515
```

### Camera Data in ROS 2

```python
# sensor_msgs/msg/Image
# Standard message for camera data

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
    
    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Image properties
        height = msg.height  # e.g., 480
        width = msg.width    # e.g., 640
        encoding = msg.encoding  # e.g., 'bgr8'
```

## LIDAR (Light Detection and Ranging)

LIDAR uses laser pulses to create precise 3D maps of the environment.

### How LIDAR Works

```
1. Emit laser pulse
2. Pulse reflects off object
3. Measure return time
4. Calculate distance: d = (c × t) / 2

Where:
  c = speed of light (299,792,458 m/s)
  t = round-trip time
```

### LIDAR Types

| Type | Description | Use Case |
|------|-------------|----------|
| 2D LIDAR | Single plane scan | Navigation, obstacle avoidance |
| 3D LIDAR | Multiple planes or spinning | Mapping, autonomous vehicles |
| Solid-state | No moving parts | Automotive, compact robots |

### Common LIDAR Sensors

| Sensor | Type | Range | Points/sec | Price |
|--------|------|-------|------------|-------|
| RPLIDAR A1 | 2D | 12m | 8,000 | ~$100 |
| Velodyne VLP-16 | 3D | 100m | 300,000 | ~$4,000 |
| Ouster OS1 | 3D | 120m | 1,310,000 | ~$6,000 |
| Livox Mid-70 | 3D | 260m | 100,000 | ~$600 |

### LIDAR Data in ROS 2

```python
# sensor_msgs/msg/LaserScan (2D LIDAR)
# sensor_msgs/msg/PointCloud2 (3D LIDAR)

from sensor_msgs.msg import LaserScan, PointCloud2

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        
        # 2D LIDAR subscription
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
    
    def scan_callback(self, msg):
        # LaserScan message fields
        angle_min = msg.angle_min      # Start angle (rad)
        angle_max = msg.angle_max      # End angle (rad)
        angle_increment = msg.angle_increment  # Angular resolution
        ranges = msg.ranges            # Distance measurements
        
        # Find closest obstacle
        min_distance = min(r for r in ranges if r > msg.range_min)
        self.get_logger().info(f'Closest obstacle: {min_distance:.2f}m')
```

## IMU (Inertial Measurement Unit)

IMUs measure motion and orientation using accelerometers and gyroscopes.

### IMU Components

```
IMU
├── Accelerometer (3-axis)
│   └── Measures linear acceleration (m/s²)
├── Gyroscope (3-axis)
│   └── Measures angular velocity (rad/s)
└── Magnetometer (optional, 3-axis)
    └── Measures magnetic field (for heading)
```

### Key Specifications

| Specification | Description | Good Value |
|--------------|-------------|------------|
| Update rate | Samples per second | 200-1000 Hz |
| Noise density | Measurement noise | Lower is better |
| Bias stability | Drift over time | Lower is better |
| Range | Maximum measurable value | ±16g, ±2000°/s |

### IMU Data in ROS 2

```python
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
    
    def imu_callback(self, msg):
        # Orientation (quaternion)
        orientation = msg.orientation  # x, y, z, w
        
        # Angular velocity (rad/s)
        angular_velocity = msg.angular_velocity  # x, y, z
        
        # Linear acceleration (m/s²)
        linear_acceleration = msg.linear_acceleration  # x, y, z
        
        # Example: detect if robot is falling
        if abs(linear_acceleration.z - 9.81) > 5.0:
            self.get_logger().warn('Possible fall detected!')
```

## Force/Torque Sensors

Force/torque (F/T) sensors measure forces and torques, essential for manipulation and safe interaction.

### What They Measure

```
6-Axis Force/Torque Sensor
├── Forces: Fx, Fy, Fz (Newtons)
└── Torques: Tx, Ty, Tz (Newton-meters)

Application Examples:
- Fx, Fy: Contact forces during manipulation
- Fz: Weight of grasped object
- Torques: Rotational resistance
```

### Common F/T Sensors

| Sensor | Force Range | Torque Range | Interface |
|--------|-------------|--------------|-----------|
| ATI Mini45 | ±580N | ±10Nm | Analog, EtherCAT |
| OnRobot HEX | ±200N | ±5Nm | USB, Tool I/O |
| Robotiq FT300 | ±300N | ±30Nm | USB, Modbus |

### F/T Sensor Data in ROS 2

```python
from geometry_msgs.msg import WrenchStamped

class FTSensorSubscriber(Node):
    def __init__(self):
        super().__init__('ft_sensor_subscriber')
        
        self.subscription = self.create_subscription(
            WrenchStamped,
            '/ft_sensor/wrench',
            self.wrench_callback,
            10
        )
        
        self.force_threshold = 10.0  # Newtons
    
    def wrench_callback(self, msg):
        force = msg.wrench.force   # x, y, z
        torque = msg.wrench.torque # x, y, z
        
        # Calculate total force magnitude
        force_magnitude = (
            force.x**2 + force.y**2 + force.z**2
        ) ** 0.5
        
        if force_magnitude > self.force_threshold:
            self.get_logger().info(
                f'Contact detected: {force_magnitude:.2f}N'
            )
```

## Joint Sensors (Proprioception)

Humanoid robots need to know their own body configuration.

### Encoder Types

| Type | Resolution | Absolute/Incremental | Cost |
|------|------------|---------------------|------|
| Optical incremental | High | Incremental | Medium |
| Optical absolute | High | Absolute | High |
| Magnetic | Medium | Both available | Low |
| Resolver | Medium | Absolute | Medium |

### Joint State Message

```python
from sensor_msgs.msg import JointState

# JointState message contains:
# - name: list of joint names
# - position: joint positions (radians)
# - velocity: joint velocities (rad/s)
# - effort: joint torques (Nm)

def joint_state_callback(msg):
    for i, name in enumerate(msg.name):
        position = msg.position[i] if msg.position else None
        velocity = msg.velocity[i] if msg.velocity else None
        effort = msg.effort[i] if msg.effort else None
        
        print(f'{name}: pos={position:.3f}, vel={velocity:.3f}')
```

## Sensor Fusion

Real robots combine multiple sensors for robust perception.

### Common Fusion Approaches

```
Visual-Inertial Odometry (VIO)
├── Camera + IMU
├── Camera provides position updates
├── IMU provides motion between frames
└── Example: Intel RealSense T265

LIDAR-Visual-Inertial
├── LIDAR + Camera + IMU
├── Most robust for mapping
└── Used in autonomous vehicles

Force-Vision Fusion
├── Camera + F/T sensor
├── Visual servoing with force feedback
└── Used for delicate manipulation
```

## Summary

- **Cameras** provide rich visual information but require significant processing
- **LIDAR** offers precise 3D measurements, excellent for navigation
- **IMUs** enable motion tracking and balance control
- **Force/torque sensors** are essential for safe manipulation
- **Sensor fusion** combines modalities for robust perception

## Exercises

### Exercise 1: Sensor Selection
A warehouse robot needs to:
- Navigate aisles
- Pick items from shelves
- Avoid workers

Select and justify sensors for each capability.

<details>
<summary>Solution</summary>

- **Navigation**: 2D LIDAR (RPLIDAR A3) for obstacle detection + wheel encoders
- **Picking**: RGB-D camera (RealSense D435) for object recognition + F/T sensor for grasp verification
- **Worker avoidance**: 3D LIDAR or depth cameras for human detection

</details>

### Exercise 2: LIDAR Calculation
A LIDAR pulse returns after 66.7 nanoseconds. Calculate the distance to the object.

<details>
<summary>Solution</summary>

d = (c × t) / 2
d = (299,792,458 m/s × 66.7 × 10⁻⁹ s) / 2
d = 10 meters

</details>

### Exercise 3: IMU Noise Analysis
An IMU reports 9.85 m/s² when stationary (should be 9.81 m/s²). Calculate the error percentage and discuss implications for navigation.

<details>
<summary>Solution</summary>

Error = (9.85 - 9.81) / 9.81 × 100% = 0.41%

Implications:
- Small error per reading, but accumulates over time
- After 1 minute of integration: significant position drift
- Solution: fuse with other sensors (GPS, vision, LIDAR)

</details>

## References

1. Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011). "Introduction to Autonomous Mobile Robots." MIT Press.
2. Intel RealSense Documentation. https://dev.intelrealsense.com/docs
3. Velodyne LIDAR User Manual.
4. ATI Industrial Automation. "F/T Sensor System Manual."

---

**Next Chapter**: [ROS 2 Architecture & Core Concepts](/module-1-ros2/ros2-architecture)
