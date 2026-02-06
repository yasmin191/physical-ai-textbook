---
sidebar_position: 1
title: "Chapter 9: Gazebo Simulation Environment"
description: "Set up and configure Gazebo for simulating humanoid robots with physics and sensors"
---

# Chapter 9: Gazebo Simulation Environment

**Gazebo** is the premier open-source robotics simulator, providing accurate physics simulation, high-quality graphics, and seamless ROS 2 integration. For humanoid robotics development, Gazebo allows you to test locomotion algorithms, manipulation tasks, and sensor processing without risking expensive hardware.

## Learning Objectives

By the end of this chapter, you will be able to:

- Install and configure Gazebo for ROS 2 Humble
- Understand Gazebo's architecture and plugin system
- Create and customize simulation worlds
- Spawn robots into the simulation
- Control simulated robots from ROS 2 nodes

## 9.1 Introduction to Robot Simulation

### Why Simulate?

Physical robot development without simulation is like software development without debugging tools:

| Benefit | Description |
|---------|-------------|
| **Safety** | Test dangerous maneuvers without risking hardware or humans |
| **Speed** | Run experiments faster than real-time; parallelize testing |
| **Cost** | No wear on motors, no broken parts, no power consumption |
| **Reproducibility** | Reset to exact same state; deterministic testing |
| **Accessibility** | Develop without physical robot access |

### The Reality Gap

Simulations are approximations. The **reality gap** refers to differences between simulated and real-world behavior:

- Physics models are simplified
- Sensor noise models are imperfect
- Environmental factors (temperature, humidity) are absent
- Wear and manufacturing variations aren't captured

Techniques to bridge this gap include:
- **Domain randomization**: Vary simulation parameters
- **Sim-to-real transfer**: Techniques covered in Module 3
- **System identification**: Calibrate simulation to match hardware

## 9.2 Gazebo Architecture

### Gazebo Versions

The Gazebo ecosystem has two major versions:

| Version | Also Known As | ROS 2 Support |
|---------|---------------|---------------|
| Gazebo Classic | Gazebo 11 | ROS 2 Humble (gazebo_ros_pkgs) |
| Gazebo Sim | Ignition, Gz | ROS 2 Humble+ (ros_gz) |

This textbook focuses on **Gazebo Sim** (the modern version), but concepts apply to both.

### Core Components

```
┌─────────────────────────────────────────────────┐
│                  Gazebo Sim                      │
├─────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────┐ │
│  │   Physics   │  │  Rendering  │  │ Sensors │ │
│  │   Engine    │  │   Engine    │  │  Plugin │ │
│  │   (DART)    │  │   (OGRE2)   │  │         │ │
│  └─────────────┘  └─────────────┘  └─────────┘ │
│         │                │              │       │
│         └────────────────┼──────────────┘       │
│                          │                      │
│              ┌───────────┴───────────┐          │
│              │    Transport Layer    │          │
│              │   (Ignition Transport)│          │
│              └───────────────────────┘          │
│                          │                      │
│              ┌───────────┴───────────┐          │
│              │      ROS 2 Bridge     │          │
│              │       (ros_gz)        │          │
│              └───────────────────────┘          │
└─────────────────────────────────────────────────┘
```

**Key Components**:
- **Physics Engine**: DART (default), Bullet, or ODE for dynamics simulation
- **Rendering Engine**: OGRE2 for visualization
- **Sensor Plugins**: Simulate cameras, LiDAR, IMU, etc.
- **Transport**: Internal messaging system
- **ros_gz**: Bridge between Gazebo and ROS 2

## 9.3 Installation

### Installing Gazebo Sim (Fortress)

For ROS 2 Humble, Gazebo Fortress is the recommended version:

```bash
# Add Gazebo repository
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Install Gazebo Fortress
sudo apt-get update
sudo apt-get install ignition-fortress

# Install ROS 2 - Gazebo integration
sudo apt-get install ros-humble-ros-gz
```

### Verify Installation

```bash
# Launch empty Gazebo world
ign gazebo empty.sdf

# Or using the gz command
gz sim empty.sdf
```

You should see the Gazebo GUI with an empty world.

## 9.4 Gazebo GUI Overview

The Gazebo interface consists of several panels:

```
┌─────────────────────────────────────────────────────┐
│  Menu Bar                                           │
├──────────┬──────────────────────────────┬───────────┤
│          │                              │           │
│  Entity  │                              │  Component│
│   Tree   │       3D Viewport            │  Inspector│
│          │                              │           │
│          │                              │           │
├──────────┴──────────────────────────────┴───────────┤
│  Playback Controls   │  Time  │  Resource Spawner   │
└─────────────────────────────────────────────────────┘
```

### Key Features

- **Entity Tree**: Hierarchical view of all objects in the world
- **3D Viewport**: Visual representation with camera controls
- **Component Inspector**: View and modify entity properties
- **Playback Controls**: Play, pause, step simulation
- **Resource Spawner**: Add models from Fuel (online repository)

### Camera Controls

| Action | Control |
|--------|---------|
| Rotate view | Right-click + drag |
| Pan view | Middle-click + drag |
| Zoom | Scroll wheel |
| Focus on entity | Double-click in Entity Tree |
| Reset view | Ctrl + 0 |

## 9.5 SDF: Simulation Description Format

While URDF describes robots, **SDF (Simulation Description Format)** describes entire simulation worlds, including:

- World properties (gravity, physics settings)
- Multiple robots and objects
- Lighting and atmosphere
- Plugins and sensors

### SDF World Structure

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="humanoid_world">
    <!-- Physics configuration -->
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
    
    <!-- Include robot model -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
    
    <!-- Plugins -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
  </world>
</sdf>
```

### Essential World Plugins

```xml
<!-- Physics engine -->
<plugin filename="ignition-gazebo-physics-system" 
        name="ignition::gazebo::systems::Physics"/>

<!-- Required for visualization -->
<plugin filename="ignition-gazebo-scene-broadcaster-system" 
        name="ignition::gazebo::systems::SceneBroadcaster"/>

<!-- Enables GUI interactions (spawn, move, delete) -->
<plugin filename="ignition-gazebo-user-commands-system" 
        name="ignition::gazebo::systems::UserCommands"/>

<!-- Contact detection -->
<plugin filename="ignition-gazebo-contact-system" 
        name="ignition::gazebo::systems::Contact"/>

<!-- Sensor processing -->
<plugin filename="ignition-gazebo-sensors-system" 
        name="ignition::gazebo::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```

## 9.6 Converting URDF to SDF

Gazebo can load URDF directly, but converting to SDF provides more control:

```bash
# Convert URDF to SDF
gz sdf -p robot.urdf > robot.sdf
```

### Adding Gazebo-Specific Elements to URDF

Use `<gazebo>` tags in URDF for Gazebo-specific properties:

```xml
<robot name="humanoid">
  <link name="base_link">
    <!-- Standard URDF content -->
  </link>
  
  <!-- Gazebo-specific properties for base_link -->
  <gazebo reference="base_link">
    <material>Gazebo/Blue</material>
    <mu1>0.8</mu1>  <!-- Friction coefficient -->
    <mu2>0.8</mu2>
    <kp>1000000.0</kp>  <!-- Contact stiffness -->
    <kd>100.0</kd>  <!-- Contact damping -->
  </gazebo>
  
  <!-- Add a sensor to a link -->
  <gazebo reference="head">
    <sensor name="camera" type="camera">
      <pose>0.05 0 0 0 0 0</pose>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
        </image>
        <clip>
          <near>0.1</near>
          <far>100</far>
        </clip>
      </camera>
      <always_on>1</always_on>
      <update_rate>30</update_rate>
    </sensor>
  </gazebo>
</robot>
```

## 9.7 Launching Gazebo with ROS 2

### Basic Launch File

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Paths
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_my_robot = FindPackageShare('my_humanoid')
    
    # World file
    world_file = PathJoinSubstitution([
        pkg_my_robot, 'worlds', 'humanoid_world.sdf'
    ])
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_file]
        }.items()
    )
    
    return LaunchDescription([
        gazebo,
    ])
```

### Spawning Robots Dynamically

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_humanoid')
    urdf_file = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')
    
    robot_description = Command(['xacro ', urdf_file])
    
    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid',
            '-topic', 'robot_description',
            '-x', '0',
            '-y', '0', 
            '-z', '1.0',
        ],
        output='screen',
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    
    return LaunchDescription([
        robot_state_publisher,
        spawn_robot,
    ])
```

## 9.8 The ROS-Gazebo Bridge

The `ros_gz_bridge` enables communication between ROS 2 and Gazebo:

```python
from launch_ros.actions import Node

# Bridge configuration
bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        # Topic bridges: ROS_topic@ROS_type@GZ_type
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
        '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        '/camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
        '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
    ],
    output='screen'
)
```

### Bridge Direction Symbols

| Symbol | Direction |
|--------|-----------|
| `[` | Gazebo → ROS 2 (one-way) |
| `]` | ROS 2 → Gazebo (one-way) |
| `@` | Bidirectional |

## 9.9 Physics Configuration

### Tuning Physics Parameters

```xml
<physics type="dart">
  <!-- Simulation step size (seconds) -->
  <max_step_size>0.001</max_step_size>
  
  <!-- Real-time factor: 1.0 = real-time, >1 = faster, <1 = slower -->
  <real_time_factor>1.0</real_time_factor>
  
  <!-- Maximum real-time update rate -->
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- DART-specific settings -->
  <dart>
    <collision_detector>bullet</collision_detector>
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
  </dart>
</physics>
```

### Step Size Considerations

| Step Size | Use Case |
|-----------|----------|
| 0.001s | High-fidelity, fast dynamics (walking, manipulation) |
| 0.004s | Standard robotics simulation |
| 0.01s | Faster-than-realtime testing |

Smaller step sizes increase accuracy but reduce simulation speed.

## 9.10 Creating Custom Worlds

### Indoor Environment Example

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <world name="indoor_lab">
    <physics type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    
    <!-- Plugins -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    
    <!-- Ambient light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    
    <!-- Floor -->
    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box><size>10 10 0.1</size></box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box><size>10 10 0.1</size></box>
          </geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>0 0 -0.05 0 0 0</pose>
    </model>
    
    <!-- Walls -->
    <model name="wall_north">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>10 0.2 2.5</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>10 0.2 2.5</size></box></geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
          </material>
        </visual>
      </link>
      <pose>0 5 1.25 0 0 0</pose>
    </model>
    
    <!-- Table -->
    <model name="table">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>1.2 0.8 0.75</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>1.2 0.8 0.75</size></box></geometry>
          <material>
            <ambient>0.6 0.4 0.2 1</ambient>
            <diffuse>0.7 0.5 0.3 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>2 0 0.375 0 0 0</pose>
    </model>
    
    <!-- Objects to manipulate -->
    <model name="red_cube">
      <link name="link">
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.000017</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.000017</iyy><iyz>0</iyz>
            <izz>0.000017</izz>
          </inertia>
        </inertial>
        <collision name="collision">
          <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.9 0.2 0.2 1</diffuse>
          </material>
        </visual>
      </link>
      <pose>2 0 0.8 0 0 0</pose>
    </model>
    
  </world>
</sdf>
```

## 9.11 Summary

In this chapter, you learned:

- **Gazebo architecture**: Physics, rendering, sensors, and ROS bridge
- **Installation**: Setting up Gazebo Sim with ROS 2 Humble
- **SDF format**: Describing simulation worlds
- **ROS integration**: Launching Gazebo and bridging topics
- **World creation**: Building custom simulation environments

Gazebo provides the foundation for safe, efficient robot development. In the next chapter, we'll explore robot description formats in more depth for simulation.

## Review Questions

1. What is the "reality gap" and how can it be addressed?
2. What are the differences between URDF and SDF?
3. How does the ros_gz_bridge enable communication between ROS 2 and Gazebo?
4. Why would you use a smaller physics step size?
5. What plugins are essential for a Gazebo world to function?

## Hands-On Exercise

1. Install Gazebo Fortress and verify the installation
2. Create a custom world with a ground plane, lighting, and walls
3. Write a launch file to start Gazebo with your world
4. Add a table and some objects to manipulate
5. Configure the ros_gz_bridge for clock and basic topics
