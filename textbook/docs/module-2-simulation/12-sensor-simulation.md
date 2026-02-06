---
sidebar_position: 4
title: "Chapter 12: Sensor Simulation"
description: "Simulate cameras, LiDAR, IMU, and other sensors with realistic noise models"
---

# Chapter 12: Sensor Simulation

Sensors are the eyes, ears, and proprioception of a humanoid robot. Simulating sensors accurately—including their noise characteristics and limitations—is crucial for developing algorithms that transfer to real hardware. This chapter covers implementing and configuring simulated sensors in Gazebo.

## Learning Objectives

By the end of this chapter, you will be able to:

- Implement camera sensors with realistic image generation
- Configure LiDAR sensors for environment perception
- Set up IMU sensors with proper noise models
- Add force/torque sensors for manipulation
- Bridge sensor data to ROS 2 topics

## 12.1 Sensor Simulation Fundamentals

### Why Accurate Sensor Simulation Matters

Real sensors have characteristics that affect perception algorithms:

| Characteristic | Impact on Algorithms |
|----------------|---------------------|
| Noise | Requires filtering and uncertainty handling |
| Latency | Affects control loop timing |
| Limited FOV | Requires sensor fusion and planning |
| Occlusion | Requires robust perception |
| Quantization | Affects precision of measurements |

### Sensor Plugin Architecture

Gazebo sensors use a plugin architecture:

```
┌─────────────────────────────────────────┐
│              Gazebo World               │
│  ┌─────────────────────────────────┐    │
│  │         Robot Model             │    │
│  │  ┌──────────┐  ┌──────────┐     │    │
│  │  │  Camera  │  │   IMU    │     │    │
│  │  │  Plugin  │  │  Plugin  │     │    │
│  │  └────┬─────┘  └────┬─────┘     │    │
│  └───────┼─────────────┼───────────┘    │
└──────────┼─────────────┼────────────────┘
           │             │
     ┌─────┴─────────────┴─────┐
     │     ros_gz_bridge       │
     └─────────────────────────┘
           │             │
     /camera/image    /imu/data
```

### Enabling Sensors System

```xml
<world name="sensor_world">
  <!-- Required for sensors -->
  <plugin filename="ignition-gazebo-sensors-system"
          name="ignition::gazebo::systems::Sensors">
    <render_engine>ogre2</render_engine>
  </plugin>
  
  <!-- For contact sensors -->
  <plugin filename="ignition-gazebo-contact-system"
          name="ignition::gazebo::systems::Contact"/>
  
  <!-- For force/torque sensors -->
  <plugin filename="ignition-gazebo-forcetorque-system"
          name="ignition::gazebo::systems::ForceTorque"/>
</world>
```

## 12.2 Camera Sensors

### RGB Camera

```xml
<link name="camera_link">
  <pose>0.1 0 0.5 0 0 0</pose>
  
  <sensor name="rgb_camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic>camera/image_raw</topic>
    
    <camera>
      <!-- Field of view (radians) -->
      <horizontal_fov>1.047</horizontal_fov>  <!-- ~60 degrees -->
      
      <!-- Image properties -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      
      <!-- Clipping planes -->
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      
      <!-- Gaussian noise -->
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
      
      <!-- Lens distortion (optional) -->
      <distortion>
        <k1>-0.25</k1>
        <k2>0.12</k2>
        <k3>0.0</k3>
        <p1>-0.00028</p1>
        <p2>-0.00005</p2>
        <center>0.5 0.5</center>
      </distortion>
    </camera>
  </sensor>
</link>
```

### Depth Camera (RGBD)

```xml
<sensor name="depth_camera" type="depth_camera">
  <always_on>true</always_on>
  <update_rate>15</update_rate>
  <topic>depth_camera</topic>
  
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10.0</far>
    </clip>
    <depth_camera>
      <clip>
        <near>0.3</near>
        <far>10.0</far>
      </clip>
    </depth_camera>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.005</stddev>  <!-- Depth noise in meters -->
    </noise>
  </camera>
</sensor>
```

### Stereo Camera

```xml
<!-- Left camera -->
<sensor name="stereo_left" type="camera">
  <pose>0 0.06 0 0 0 0</pose>  <!-- Baseline offset -->
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <topic>stereo/left/image_raw</topic>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip><near>0.1</near><far>100</far></clip>
  </camera>
</sensor>

<!-- Right camera -->
<sensor name="stereo_right" type="camera">
  <pose>0 -0.06 0 0 0 0</pose>  <!-- 12cm baseline -->
  <always_on>true</always_on>
  <update_rate>30</update_rate>
  <topic>stereo/right/image_raw</topic>
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
    </image>
    <clip><near>0.1</near><far>100</far></clip>
  </camera>
</sensor>
```

### Camera Calibration Parameters

To match real camera calibration:

```xml
<camera>
  <!-- Intrinsic matrix elements -->
  <lens>
    <intrinsics>
      <!-- fx, fy: focal length in pixels -->
      <fx>554.25</fx>
      <fy>554.25</fy>
      <!-- cx, cy: principal point -->
      <cx>320.5</cx>
      <cy>240.5</cy>
      <!-- s: skew (usually 0) -->
      <s>0</s>
    </intrinsics>
  </lens>
</camera>
```

## 12.3 LiDAR Sensors

### 2D LiDAR (Laser Scanner)

```xml
<sensor name="lidar_2d" type="gpu_lidar">
  <pose>0 0 0.3 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <topic>scan</topic>
  
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>1</samples>
        <resolution>1</resolution>
        <min_angle>0</min_angle>
        <max_angle>0</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
</sensor>
```

### 3D LiDAR (Point Cloud)

```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <pose>0 0 0.5 0 0 0</pose>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <topic>points</topic>
  
  <lidar>
    <scan>
      <horizontal>
        <samples>1800</samples>  <!-- 0.2 degree resolution -->
        <resolution>1</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>  <!-- 16-beam LiDAR -->
        <resolution>1</resolution>
        <min_angle>-0.261799</min_angle>  <!-- -15 degrees -->
        <max_angle>0.261799</max_angle>   <!-- +15 degrees -->
      </vertical>
    </scan>
    <range>
      <min>0.5</min>
      <max>100.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.02</stddev>
    </noise>
  </lidar>
</sensor>
```

### Common LiDAR Specifications

| Model | Beams | Range | Horizontal FOV | Update Rate |
|-------|-------|-------|----------------|-------------|
| Hokuyo UTM-30LX | 1 | 30m | 270° | 40 Hz |
| Velodyne VLP-16 | 16 | 100m | 360° | 20 Hz |
| Ouster OS1-64 | 64 | 120m | 360° | 20 Hz |

## 12.4 IMU Sensors

### Basic IMU Configuration

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate>
  <topic>imu/data</topic>
  
  <imu>
    <!-- Orientation reference frame -->
    <orientation_reference_frame>
      <localization>ENU</localization>  <!-- East-North-Up -->
    </orientation_reference_frame>
    
    <!-- Angular velocity (gyroscope) -->
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0002</stddev>  <!-- rad/s -->
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.0002</stddev>
          <bias_mean>0.0000075</bias_mean>
          <bias_stddev>0.0000008</bias_stddev>
        </noise>
      </z>
    </angular_velocity>
    
    <!-- Linear acceleration (accelerometer) -->
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>  <!-- m/s² -->
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0</mean>
          <stddev>0.017</stddev>
          <bias_mean>0.1</bias_mean>
          <bias_stddev>0.001</bias_stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
</sensor>
```

### IMU Noise Model Parameters

Based on real IMU specifications:

| Parameter | Low-cost IMU | MEMS Grade | Tactical Grade |
|-----------|--------------|------------|----------------|
| Gyro noise (°/s/√Hz) | 0.1 | 0.01 | 0.001 |
| Gyro bias stability (°/hr) | 10 | 1 | 0.1 |
| Accel noise (mg/√Hz) | 1.0 | 0.1 | 0.01 |
| Accel bias stability (mg) | 10 | 1 | 0.1 |

## 12.5 Force/Torque Sensors

### Wrist Force/Torque Sensor

```xml
<joint name="wrist_ft_joint" type="fixed">
  <parent>forearm</parent>
  <child>wrist_ft_link</child>
  
  <sensor name="wrist_ft" type="force_torque">
    <always_on>true</always_on>
    <update_rate>500</update_rate>
    <topic>wrist_ft</topic>
    
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
      
      <force>
        <x>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.1</stddev>  <!-- N -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.1</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.1</stddev>
          </noise>
        </z>
      </force>
      
      <torque>
        <x>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.01</stddev>  <!-- N·m -->
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </torque>
    </force_torque>
  </sensor>
</joint>
```

### Foot Contact/Pressure Sensor

```xml
<link name="foot_left">
  <collision name="foot_collision">
    <geometry><box><size>0.2 0.1 0.02</size></box></geometry>
  </collision>
  
  <sensor name="foot_contact" type="contact">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>foot_left/contact</topic>
    
    <contact>
      <collision>foot_collision</collision>
    </contact>
  </sensor>
</link>
```

## 12.6 Joint State Sensors

### Joint Position/Velocity/Effort

Joint states are automatically published through ros2_control or via plugins:

```xml
<gazebo>
  <plugin filename="libgazebo_ros_joint_state_publisher.so"
          name="joint_state_publisher">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>joint_states:=joint_states</remapping>
    </ros>
    <joint_name>hip_pitch_left</joint_name>
    <joint_name>knee_pitch_left</joint_name>
    <joint_name>ankle_pitch_left</joint_name>
    <joint_name>hip_pitch_right</joint_name>
    <joint_name>knee_pitch_right</joint_name>
    <joint_name>ankle_pitch_right</joint_name>
    <update_rate>100</update_rate>
  </plugin>
</gazebo>
```

## 12.7 Bridging Sensors to ROS 2

### Complete Bridge Configuration

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            
            # Camera
            '/camera/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            
            # Depth camera
            '/depth_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/depth_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            
            # LiDAR
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            
            # IMU
            '/imu/data@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            
            # Force/Torque
            '/wrist_ft@geometry_msgs/msg/WrenchStamped[ignition.msgs.Wrench',
            
            # Contacts
            '/foot_left/contact@gazebo_msgs/msg/ContactsState[ignition.msgs.Contacts',
            
            # Joint states
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen'
    )
    
    return LaunchDescription([bridge])
```

### Image Bridge with Compression

```python
# For image topics that need processing
image_bridge = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=['/camera/image_raw'],
    output='screen'
)
```

## 12.8 Sensor Fusion Considerations

### Time Synchronization

Sensors have different update rates and latencies:

```python
import message_filters
from sensor_msgs.msg import Image, Imu, LaserScan

class SensorFusion(Node):
    def __init__(self):
        super().__init__('sensor_fusion')
        
        # Subscribers
        image_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        imu_sub = message_filters.Subscriber(self, Imu, '/imu/data')
        scan_sub = message_filters.Subscriber(self, LaserScan, '/scan')
        
        # Approximate time synchronizer
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, imu_sub, scan_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.synced_callback)
    
    def synced_callback(self, image, imu, scan):
        # Process synchronized sensor data
        pass
```

### Sensor Transform Setup

```python
from launch_ros.actions import Node

# Static transforms for sensors
static_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=[
        '0.1', '0', '0.5',  # xyz
        '0', '0', '0', '1',  # quaternion (xyzw)
        'base_link', 'camera_link'
    ]
)
```

## 12.9 Realistic Sensor Limitations

### Camera Limitations

```xml
<camera>
  <!-- Motion blur (based on exposure time) -->
  <exposure>
    <auto>false</auto>
    <time>0.02</time>  <!-- 20ms exposure -->
  </exposure>
  
  <!-- Limited dynamic range -->
  <dynamic_range>60</dynamic_range>  <!-- dB -->
</camera>
```

### LiDAR Limitations

```xml
<lidar>
  <!-- Reflectivity-based returns -->
  <visibility_mask>0xFFFF</visibility_mask>
  
  <!-- Beam divergence -->
  <beam>
    <width>0.003</width>  <!-- 3mm at 1m -->
  </beam>
</lidar>
```

## 12.10 Summary

In this chapter, you learned:

- **Camera sensors**: RGB, depth, stereo with noise and distortion
- **LiDAR sensors**: 2D and 3D configuration with realistic noise
- **IMU sensors**: Gyroscope and accelerometer with bias models
- **Force/torque sensors**: Wrist and foot contact sensing
- **ROS 2 bridge**: Connecting Gazebo sensors to ROS topics
- **Sensor fusion**: Time synchronization and transforms

Accurate sensor simulation enables development of perception algorithms that transfer to real hardware. In the next chapter, we'll explore Unity for high-fidelity rendering and human-robot interaction.

## Review Questions

1. Why is sensor noise modeling important for algorithm development?
2. What are the key parameters for configuring a camera sensor?
3. How does LiDAR resolution affect perception algorithms?
4. What is the difference between IMU noise and bias?
5. How do you synchronize data from multiple sensors?

## Hands-On Exercise

1. Add a camera sensor to your humanoid's head
2. Configure a 2D LiDAR at the robot's torso level
3. Add an IMU sensor to the robot's base link
4. Set up the ros_gz_bridge for all sensors
5. Visualize sensor data in RViz2
6. Implement a simple obstacle detector using LiDAR data
