---
sidebar_position: 4
title: "Chapter 4: ROS 2 Architecture"
description: "Understanding the Robot Operating System 2 architecture and core concepts"
---

# ROS 2 Architecture & Core Concepts

## Learning Objectives

By the end of this chapter, you will be able to:

1. Explain the architecture and design principles of ROS 2
2. Understand the differences between ROS 1 and ROS 2
3. Describe the DDS middleware and its role in ROS 2
4. Set up a ROS 2 development environment

## Prerequisites

- [Chapter 3: Sensor Systems](/module-1-ros2/sensor-systems)
- Ubuntu 22.04 LTS installed
- Basic Linux command line knowledge

## What is ROS 2?

**ROS 2** (Robot Operating System 2) is not actually an operating system—it's a flexible framework for writing robot software. It provides:

- **Communication infrastructure** between processes
- **Hardware abstraction** for sensors and actuators
- **Tools** for development, debugging, and visualization
- **Libraries** for common robotics tasks
- **Ecosystem** of reusable packages

```
ROS 2 = Middleware + Tools + Libraries + Community
```

## Why ROS 2?

ROS 2 was developed to address limitations of ROS 1:

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Real-time support | Limited | Yes (with RTOS) |
| Multi-robot | Difficult | Native support |
| Security | None | DDS Security |
| Platform | Linux only | Linux, Windows, macOS |
| Network | Single master | Decentralized |
| Production ready | Research focus | Industry focus |

## ROS 2 Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                     Application Layer                        │
│              (Your robot code, packages)                     │
├─────────────────────────────────────────────────────────────┤
│                      ROS 2 Client Libraries                  │
│              rclpy (Python)    rclcpp (C++)                 │
├─────────────────────────────────────────────────────────────┤
│                      ROS 2 Middleware (RMW)                  │
│                    Abstract DDS Interface                    │
├─────────────────────────────────────────────────────────────┤
│                      DDS Implementation                      │
│         Fast DDS | Cyclone DDS | Connext DDS                │
├─────────────────────────────────────────────────────────────┤
│                      Operating System                        │
│              Linux | Windows | macOS | RTOS                  │
└─────────────────────────────────────────────────────────────┘
```

## DDS: The Communication Backbone

**DDS** (Data Distribution Service) is the middleware that powers ROS 2 communication.

### Key DDS Concepts

```
Publisher/Subscriber Pattern
├── Decoupled communication
├── No central broker required
├── Automatic discovery
└── Quality of Service (QoS) policies

Domain Participant
├── Represents a node in the DDS network
├── Discovers other participants automatically
└── Manages data readers and writers
```

### Quality of Service (QoS)

QoS policies let you tune communication behavior:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Reliable delivery (like TCP)
reliable_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Best effort (like UDP) - for high-frequency sensor data
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)
```

| QoS Policy | Options | Use Case |
|------------|---------|----------|
| Reliability | RELIABLE, BEST_EFFORT | Commands vs sensor data |
| Durability | VOLATILE, TRANSIENT_LOCAL | Late joiners |
| History | KEEP_LAST, KEEP_ALL | Buffer management |
| Deadline | Time duration | Real-time requirements |

## Core ROS 2 Concepts

### Nodes

A **node** is the fundamental unit of computation in ROS 2. Each node should do one thing well.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')  # Node name
        self.get_logger().info('Node started!')

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics

**Topics** are named buses for publishing/subscribing messages.

```
Publisher Node          Topic           Subscriber Node(s)
    [A] ──────────→  /sensor_data  ──────────→ [B]
                                   ──────────→ [C]
                                   ──────────→ [D]
```

### Services

**Services** provide request/response communication (like RPC).

```
Client Node              Service             Server Node
    [A] ───request───→  /calculate  ←──────── [B]
        ←──response───                
```

### Actions

**Actions** handle long-running tasks with feedback.

```
Client Node              Action              Server Node
    [A] ───goal────→   /navigate    ←─────── [B]
        ←──feedback──              
        ←──result────              
```

### Parameters

**Parameters** are node configuration values that can be changed at runtime.

```python
class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Declare parameter with default value
        self.declare_parameter('speed', 1.0)
        
        # Get parameter value
        speed = self.get_parameter('speed').value
```

## ROS 2 Distributions

ROS 2 releases are named alphabetically, like Ubuntu:

| Distribution | Release Date | EOL | Ubuntu |
|--------------|--------------|-----|--------|
| Humble Hawksbill | May 2022 | May 2027 | 22.04 |
| Iron Irwini | May 2023 | Nov 2024 | 22.04 |
| Jazzy Jalisco | May 2024 | May 2029 | 24.04 |

**This course uses ROS 2 Humble** (LTS release).

## Installing ROS 2 Humble

### Setup Sources

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2

```bash
sudo apt update
sudo apt upgrade

# Desktop install (recommended)
sudo apt install ros-humble-desktop

# Development tools
sudo apt install ros-dev-tools
```

### Environment Setup

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Should output: ros2 (humble)
```

## ROS 2 Command Line Tools

### Essential Commands

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Echo topic data
ros2 topic echo /topic_name

# Get topic info
ros2 topic info /topic_name

# Publish to topic
ros2 topic pub /topic_name std_msgs/String "data: 'Hello'"

# List services
ros2 service list

# Call a service
ros2 service call /service_name srv_type "{request_data}"

# List parameters
ros2 param list

# Get parameter value
ros2 param get /node_name parameter_name
```

### Introspection Example

```bash
# Terminal 1: Run turtlesim
ros2 run turtlesim turtlesim_node

# Terminal 2: Explore
ros2 node list
# Output: /turtlesim

ros2 topic list
# Output:
# /turtle1/cmd_vel
# /turtle1/pose
# ...

ros2 topic echo /turtle1/pose
# Shows turtle position in real-time
```

## ROS 2 Workspace Structure

```
ros2_ws/                    # Workspace root
├── src/                    # Source packages
│   ├── my_package/
│   │   ├── package.xml     # Package manifest
│   │   ├── setup.py        # Python build config
│   │   ├── my_package/     # Python module
│   │   │   ├── __init__.py
│   │   │   └── my_node.py
│   │   └── resource/
│   │       └── my_package
│   └── another_package/
├── build/                  # Build artifacts (generated)
├── install/                # Installed packages (generated)
└── log/                    # Build logs (generated)
```

### Creating a Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Summary

- ROS 2 is a framework for robot software development, not an OS
- DDS middleware provides decentralized, secure communication
- Key concepts: Nodes, Topics, Services, Actions, Parameters
- QoS policies tune communication behavior
- ROS 2 Humble is the recommended LTS distribution
- Workspaces organize your ROS 2 packages

## Exercises

### Exercise 1: Installation Verification
Install ROS 2 Humble and verify by running:
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```

### Exercise 2: Topic Exploration
Run turtlesim and answer:
1. What topics does it publish?
2. What is the message type of `/turtle1/pose`?
3. What QoS profile is used?

<details>
<summary>Solution</summary>

```bash
ros2 run turtlesim turtlesim_node
ros2 topic list -t  # Shows types
ros2 topic info /turtle1/pose --verbose  # Shows QoS
```

Topics: `/turtle1/cmd_vel`, `/turtle1/pose`, etc.
Message type: `turtlesim/msg/Pose`
QoS: Reliable, Volatile, Keep Last (depth 10)

</details>

### Exercise 3: QoS Selection
For each scenario, select appropriate QoS settings:
1. Robot emergency stop command
2. 100Hz IMU data stream
3. Map data for late-joining nodes

<details>
<summary>Solution</summary>

1. **Emergency stop**: RELIABLE (must not lose), KEEP_LAST(1)
2. **IMU stream**: BEST_EFFORT (speed over reliability), KEEP_LAST(1)
3. **Map data**: RELIABLE, TRANSIENT_LOCAL (for late joiners)

</details>

## References

1. ROS 2 Documentation. https://docs.ros.org/en/humble/
2. OMG DDS Specification. https://www.omg.org/spec/DDS/
3. Open Robotics. "ROS 2 Design." https://design.ros2.org/
4. Maruyama, Y., et al. (2016). "Exploring the performance of ROS2."

---

**Next Chapter**: [Nodes, Topics, Services, and Actions](/module-1-ros2/nodes-topics-services)
