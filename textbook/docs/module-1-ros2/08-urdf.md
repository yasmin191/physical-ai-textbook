---
sidebar_position: 8
title: "Chapter 8: URDF - Unified Robot Description Format"
description: "Define robot structure, kinematics, and visual properties using URDF for simulation and visualization"
---

# Chapter 8: URDF - Unified Robot Description Format

The **Unified Robot Description Format (URDF)** is an XML-based format for describing robot models in ROS 2. URDF defines the robot's physical structure, including links (rigid bodies), joints (connections between links), and their visual and collision properties. Understanding URDF is essential for simulation, motion planning, and visualization of humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the structure and syntax of URDF files
- Define links with visual, collision, and inertial properties
- Configure different joint types for robot articulation
- Use Xacro macros to create modular robot descriptions
- Visualize robots in RViz2 and prepare them for Gazebo simulation

## 8.1 Introduction to URDF

URDF serves as the standard way to describe robots in the ROS ecosystem. It captures:

- **Kinematic structure**: How links are connected through joints
- **Visual geometry**: What the robot looks like for visualization
- **Collision geometry**: Simplified shapes for collision detection
- **Inertial properties**: Mass and inertia for physics simulation
- **Sensor attachments**: Where sensors are mounted on the robot

### Why URDF Matters for Humanoid Robots

Humanoid robots have complex kinematic chains with dozens of joints:

```
                    Head
                     |
                   Neck
                     |
    Left Arm --- Torso --- Right Arm
       |           |           |
    L.Elbow    Waist      R.Elbow
       |           |           |
    L.Wrist   Pelvis      R.Wrist
       |      /      \        |
    L.Hand  L.Hip   R.Hip  R.Hand
             |        |
           L.Knee  R.Knee
             |        |
          L.Ankle  R.Ankle
             |        |
           L.Foot  R.Foot
```

URDF allows you to precisely define this entire structure in a machine-readable format.

## 8.2 URDF Basic Structure

A URDF file consists of a `<robot>` root element containing links and joints:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <!-- Link properties go here -->
  </link>
  
  <link name="torso">
    <!-- Torso properties -->
  </link>
  
  <!-- Joints connect links -->
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/>
  </joint>
</robot>
```

### The Robot Element

The `<robot>` element is the root container:

```xml
<robot name="my_humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- All links and joints defined here -->
</robot>
```

- `name`: Unique identifier for the robot
- `xmlns:xacro`: Optional namespace for Xacro macros

## 8.3 Defining Links

Links represent rigid bodies in the robot. Each link can have visual, collision, and inertial properties.

### Visual Properties

The `<visual>` element defines how the link appears in visualization tools:

```xml
<link name="torso">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.3 0.2 0.5"/>
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 0.8 1.0"/>
    </material>
  </visual>
</link>
```

#### Supported Geometry Types

```xml
<!-- Box: length x width x height -->
<geometry>
  <box size="0.3 0.2 0.5"/>
</geometry>

<!-- Cylinder: radius and length -->
<geometry>
  <cylinder radius="0.05" length="0.3"/>
</geometry>

<!-- Sphere: radius only -->
<geometry>
  <sphere radius="0.1"/>
</geometry>

<!-- Mesh: external 3D model file -->
<geometry>
  <mesh filename="package://my_robot/meshes/torso.dae" scale="1.0 1.0 1.0"/>
</geometry>
```

### Collision Properties

The `<collision>` element defines simplified geometry for physics simulation:

```xml
<link name="torso">
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.32 0.22 0.52"/>
    </geometry>
  </collision>
</link>
```

**Best Practice**: Use simple primitives for collision (boxes, cylinders, spheres) even when visual geometry uses complex meshes. This improves simulation performance.

### Inertial Properties

The `<inertial>` element specifies mass and inertia for dynamics simulation:

```xml
<link name="torso">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="10.0"/>
    <inertia 
      ixx="0.1" ixy="0.0" ixz="0.0"
      iyy="0.15" iyz="0.0"
      izz="0.08"/>
  </inertial>
</link>
```

#### Calculating Inertia

For a solid box with mass `m` and dimensions `(d, w, h)`:

```
Ixx = (1/12) * m * (w² + h²)
Iyy = (1/12) * m * (d² + h²)
Izz = (1/12) * m * (d² + w²)
```

For a solid cylinder with mass `m`, radius `r`, and height `h`:

```
Ixx = Iyy = (1/12) * m * (3r² + h²)
Izz = (1/2) * m * r²
```

### Complete Link Example

```xml
<link name="upper_arm_right">
  <visual>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.04" length="0.3"/>
    </geometry>
    <material name="skin">
      <color rgba="0.96 0.8 0.69 1.0"/>
    </material>
  </visual>
  
  <collision>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <geometry>
      <cylinder radius="0.045" length="0.32"/>
    </geometry>
  </collision>
  
  <inertial>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <mass value="2.0"/>
    <inertia ixx="0.016" ixy="0" ixz="0" iyy="0.016" iyz="0" izz="0.002"/>
  </inertial>
</link>
```

## 8.4 Defining Joints

Joints connect parent and child links and define how they can move relative to each other.

### Joint Types

URDF supports six joint types:

| Type | DOF | Description |
|------|-----|-------------|
| `fixed` | 0 | No motion allowed |
| `revolute` | 1 | Rotation around axis with limits |
| `continuous` | 1 | Rotation around axis, no limits |
| `prismatic` | 1 | Linear motion along axis with limits |
| `floating` | 6 | Unconstrained motion (rarely used) |
| `planar` | 3 | Motion in a plane |

### Revolute Joint (Most Common)

Used for elbows, knees, shoulders, and most humanoid joints:

```xml
<joint name="right_elbow" type="revolute">
  <parent link="upper_arm_right"/>
  <child link="forearm_right"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="0.0" upper="2.5" effort="50.0" velocity="2.0"/>
  <dynamics damping="0.5" friction="0.1"/>
</joint>
```

**Joint Elements**:
- `parent/child`: Links being connected
- `origin`: Transform from parent to joint frame
- `axis`: Axis of rotation (x, y, or z)
- `limit`: Motion constraints
  - `lower/upper`: Position limits (radians)
  - `effort`: Maximum torque (N·m)
  - `velocity`: Maximum velocity (rad/s)
- `dynamics`: Damping and friction coefficients

### Continuous Joint

For wheels or joints that rotate indefinitely:

```xml
<joint name="head_pan" type="continuous">
  <parent link="neck"/>
  <child link="head"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <dynamics damping="0.1"/>
</joint>
```

### Fixed Joint

For rigidly attached components:

```xml
<joint name="camera_mount" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
</joint>
```

### Prismatic Joint

For linear actuators:

```xml
<joint name="gripper_finger" type="prismatic">
  <parent link="hand"/>
  <child link="finger"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="0.0" upper="0.05" effort="10.0" velocity="0.1"/>
</joint>
```

## 8.5 Coordinate Frames and Transforms

Understanding coordinate frames is crucial for URDF:

### The Origin Element

The `<origin>` element specifies a transform using position (xyz) and orientation (rpy):

```xml
<origin xyz="x y z" rpy="roll pitch yaw"/>
```

- `xyz`: Translation in meters
- `rpy`: Rotation in radians (roll around X, pitch around Y, yaw around Z)

### Joint Frame Convention

The joint frame is positioned at the joint's origin in the parent link's frame:

```
Parent Link Frame
       |
       | (joint origin transform)
       v
Joint Frame / Child Link Frame
```

### Example: Arm Kinematic Chain

```xml
<!-- Shoulder to upper arm -->
<joint name="shoulder_pitch" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.15 0 0.4" rpy="0 0 0"/>  <!-- Shoulder position -->
  <axis xyz="0 1 0"/>  <!-- Pitch axis -->
</joint>

<!-- Upper arm to elbow -->
<joint name="elbow_pitch" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>  <!-- Elbow at end of upper arm -->
  <axis xyz="0 1 0"/>
</joint>

<!-- Forearm to wrist -->
<joint name="wrist_roll" type="revolute">
  <parent link="forearm"/>
  <child link="hand"/>
  <origin xyz="0 0 -0.25" rpy="0 0 0"/>  <!-- Wrist at end of forearm -->
  <axis xyz="0 0 1"/>  <!-- Roll axis -->
</joint>
```

## 8.6 Xacro: XML Macros for URDF

**Xacro** extends URDF with macros, properties, and conditional logic. This reduces redundancy and makes robot descriptions more maintainable.

### Converting URDF to Xacro

Change file extension from `.urdf` to `.urdf.xacro` and add the namespace:

```xml
<?xml version="1.0"?>
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <!-- Xacro content -->
</robot>
```

### Properties (Variables)

Define reusable values:

```xml
<xacro:property name="torso_height" value="0.5"/>
<xacro:property name="torso_width" value="0.3"/>
<xacro:property name="torso_mass" value="10.0"/>

<link name="torso">
  <visual>
    <geometry>
      <box size="${torso_width} 0.2 ${torso_height}"/>
    </geometry>
  </visual>
  <inertial>
    <mass value="${torso_mass}"/>
  </inertial>
</link>
```

### Math Expressions

Use Python expressions inside `${}`:

```xml
<xacro:property name="arm_length" value="0.3"/>

<joint name="elbow">
  <origin xyz="0 0 ${-arm_length}" rpy="0 0 0"/>
</joint>

<!-- More complex expressions -->
<xacro:property name="pi" value="3.14159"/>
<origin rpy="0 ${pi/2} 0"/>

<inertia ixx="${(1/12) * mass * (width**2 + height**2)}"/>
```

### Macros (Reusable Blocks)

Define macros for repeated structures like limbs:

```xml
<xacro:macro name="limb_segment" params="name length radius mass">
  <link name="${name}">
    <visual>
      <origin xyz="0 0 ${-length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${radius}" length="${length}"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 ${-length/2}" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="${radius * 1.1}" length="${length}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${-length/2}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia 
        ixx="${(1/12) * mass * (3*radius**2 + length**2)}"
        ixy="0" ixz="0"
        iyy="${(1/12) * mass * (3*radius**2 + length**2)}"
        iyz="0"
        izz="${0.5 * mass * radius**2}"/>
    </inertial>
  </link>
</xacro:macro>

<!-- Use the macro -->
<xacro:limb_segment name="upper_arm_right" length="0.3" radius="0.04" mass="2.0"/>
<xacro:limb_segment name="forearm_right" length="0.25" radius="0.035" mass="1.5"/>
```

### Including Files

Split large robot descriptions into multiple files:

```xml
<!-- main_robot.urdf.xacro -->
<robot name="humanoid" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:include filename="$(find my_robot)/urdf/materials.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/torso.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/arm.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/leg.xacro"/>
  <xacro:include filename="$(find my_robot)/urdf/head.xacro"/>
  
  <!-- Instantiate components -->
  <xacro:torso/>
  <xacro:arm side="left"/>
  <xacro:arm side="right"/>
  <xacro:leg side="left"/>
  <xacro:leg side="right"/>
  <xacro:head/>
</robot>
```

### Conditional Logic

```xml
<xacro:macro name="arm" params="side">
  <xacro:if value="${side == 'left'}">
    <xacro:property name="reflect" value="1"/>
  </xacro:if>
  <xacro:if value="${side == 'right'}">
    <xacro:property name="reflect" value="-1"/>
  </xacro:if>
  
  <joint name="${side}_shoulder">
    <origin xyz="${reflect * 0.2} 0 0.4" rpy="0 0 0"/>
  </joint>
</xacro:macro>
```

## 8.7 Processing Xacro Files

Convert Xacro to URDF using the `xacro` command:

```bash
# Generate URDF from Xacro
ros2 run xacro xacro robot.urdf.xacro > robot.urdf

# With arguments
ros2 run xacro xacro robot.urdf.xacro arm_length:=0.35 > robot.urdf
```

In launch files, use the `xacro` Python module:

```python
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Process Xacro file
    xacro_file = '/path/to/robot.urdf.xacro'
    robot_description = xacro.process_file(xacro_file).toxml()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
    ])
```

## 8.8 Robot State Publisher

The **robot_state_publisher** node reads the URDF and publishes transforms based on joint states:

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('my_humanoid')
    xacro_file = os.path.join(pkg_path, 'urdf', 'humanoid.urdf.xacro')
    
    robot_description = Command(['xacro ', xacro_file])
    
    return LaunchDescription([
        # Publish robot description and transforms
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),
        
        # GUI for manually setting joint positions
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        
        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'view_robot.rviz')]
        ),
    ])
```

## 8.9 Visualizing in RViz2

To visualize your robot in RViz2:

1. **Add RobotModel Display**: Click "Add" → "RobotModel"
2. **Set Description Topic**: Set to `/robot_description`
3. **Add TF Display**: To see coordinate frames
4. **Set Fixed Frame**: Usually `base_link`

### RViz2 Configuration File

Save your configuration for reuse:

```yaml
# view_robot.rviz
Panels:
  - Class: rviz_common/Displays
Visualization Manager:
  Global Options:
    Fixed Frame: base_link
  Displays:
    - Class: rviz_default_plugins/RobotModel
      Description Topic: /robot_description
      Name: RobotModel
    - Class: rviz_default_plugins/TF
      Name: TF
      Show Names: true
```

## 8.10 URDF Validation

Always validate your URDF before use:

```bash
# Check URDF syntax
check_urdf robot.urdf

# Visualize in terminal (shows tree structure)
urdf_to_graphviz robot.urdf

# View as PDF
evince robot.pdf
```

### Common URDF Errors

1. **Missing inertial properties**: Required for Gazebo simulation
2. **Zero mass or inertia**: Causes simulation instability
3. **Disconnected links**: Every link must be reachable from root
4. **Multiple root links**: Only one link should have no parent
5. **Self-collision**: Links intersecting at rest position

## 8.11 Practical Exercise: Simple Humanoid Upper Body

Let's create a simple humanoid upper body:

```xml
<?xml version="1.0"?>
<robot name="humanoid_upper_body" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Properties -->
  <xacro:property name="torso_h" value="0.4"/>
  <xacro:property name="torso_w" value="0.3"/>
  <xacro:property name="torso_d" value="0.15"/>
  <xacro:property name="arm_radius" value="0.03"/>
  <xacro:property name="upper_arm_len" value="0.25"/>
  <xacro:property name="forearm_len" value="0.22"/>
  
  <!-- Materials -->
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1.0"/>
  </material>
  <material name="skin">
    <color rgba="0.96 0.8 0.69 1.0"/>
  </material>
  
  <!-- Inertia macro for cylinder -->
  <xacro:macro name="cylinder_inertia" params="m r h">
    <inertia 
      ixx="${(1/12)*m*(3*r*r + h*h)}" ixy="0" ixz="0"
      iyy="${(1/12)*m*(3*r*r + h*h)}" iyz="0"
      izz="${0.5*m*r*r}"/>
  </xacro:macro>
  
  <!-- Base link (pelvis) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.25 0.1"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>
  
  <!-- Torso -->
  <link name="torso">
    <visual>
      <origin xyz="0 0 ${torso_h/2}"/>
      <geometry>
        <box size="${torso_d} ${torso_w} ${torso_h}"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <origin xyz="0 0 ${torso_h/2}"/>
      <geometry>
        <box size="${torso_d} ${torso_w} ${torso_h}"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 ${torso_h/2}"/>
      <mass value="8.0"/>
      <inertia ixx="0.15" ixy="0" ixz="0" iyy="0.12" iyz="0" izz="0.08"/>
    </inertial>
  </link>
  
  <joint name="base_to_torso" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.05"/>
  </joint>
  
  <!-- Arm macro -->
  <xacro:macro name="arm" params="side reflect">
    <!-- Upper arm -->
    <link name="${side}_upper_arm">
      <visual>
        <origin xyz="0 0 ${-upper_arm_len/2}"/>
        <geometry>
          <cylinder radius="${arm_radius}" length="${upper_arm_len}"/>
        </geometry>
        <material name="skin"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-upper_arm_len/2}"/>
        <geometry>
          <cylinder radius="${arm_radius}" length="${upper_arm_len}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-upper_arm_len/2}"/>
        <mass value="1.5"/>
        <xacro:cylinder_inertia m="1.5" r="${arm_radius}" h="${upper_arm_len}"/>
      </inertial>
    </link>
    
    <!-- Shoulder joint -->
    <joint name="${side}_shoulder_pitch" type="revolute">
      <parent link="torso"/>
      <child link="${side}_upper_arm"/>
      <origin xyz="0 ${reflect * (torso_w/2 + 0.02)} ${torso_h - 0.05}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="3.14" effort="50" velocity="2.0"/>
    </joint>
    
    <!-- Forearm -->
    <link name="${side}_forearm">
      <visual>
        <origin xyz="0 0 ${-forearm_len/2}"/>
        <geometry>
          <cylinder radius="${arm_radius * 0.9}" length="${forearm_len}"/>
        </geometry>
        <material name="skin"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-forearm_len/2}"/>
        <geometry>
          <cylinder radius="${arm_radius * 0.9}" length="${forearm_len}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-forearm_len/2}"/>
        <mass value="1.0"/>
        <xacro:cylinder_inertia m="1.0" r="${arm_radius * 0.9}" h="${forearm_len}"/>
      </inertial>
    </link>
    
    <!-- Elbow joint -->
    <joint name="${side}_elbow" type="revolute">
      <parent link="${side}_upper_arm"/>
      <child link="${side}_forearm"/>
      <origin xyz="0 0 ${-upper_arm_len}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.5" effort="30" velocity="2.0"/>
    </joint>
  </xacro:macro>
  
  <!-- Instantiate arms -->
  <xacro:arm side="left" reflect="1"/>
  <xacro:arm side="right" reflect="-1"/>
  
  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="4.0"/>
      <inertia ixx="0.016" ixy="0" ixz="0" iyy="0.016" iyz="0" izz="0.016"/>
    </inertial>
  </link>
  
  <joint name="neck" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 ${torso_h + 0.12}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1.5"/>
  </joint>
  
</robot>
```

## 8.12 Summary

In this chapter, you learned:

- **URDF structure**: Links, joints, and their properties
- **Link properties**: Visual, collision, and inertial elements
- **Joint types**: revolute, continuous, prismatic, fixed
- **Xacro**: Properties, macros, includes, and conditionals
- **Visualization**: Using robot_state_publisher and RViz2
- **Best practices**: Validation, collision geometry simplification

URDF is the foundation for all robot simulation and visualization in ROS 2. In the next module, we'll use these robot descriptions in Gazebo for physics simulation.

## Review Questions

1. What are the three main properties that can be defined for a link?
2. When would you use a `revolute` joint vs a `continuous` joint?
3. Why is it important to use simplified geometry for collision detection?
4. How do Xacro macros reduce redundancy in robot descriptions?
5. What is the purpose of the `robot_state_publisher` node?

## Hands-On Exercise

1. Create a URDF for a simple 2-DOF robot arm with a base, upper arm, and forearm
2. Add visual and collision properties to all links
3. Calculate and add appropriate inertial properties
4. Convert to Xacro using properties for link dimensions
5. Visualize the robot in RViz2 using joint_state_publisher_gui

## Further Reading

- [URDF Specification](http://wiki.ros.org/urdf/XML)
- [Xacro Documentation](http://wiki.ros.org/xacro)
- [ROS 2 Robot State Publisher](https://github.com/ros/robot_state_publisher)
