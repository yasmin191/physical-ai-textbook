---
sidebar_position: 2
title: "Chapter 10: URDF and SDF for Simulation"
description: "Advanced robot description techniques for physics simulation in Gazebo"
---

# Chapter 10: URDF and SDF for Simulation

While Chapter 8 introduced URDF basics, simulation requires additional elements for physics accuracy. This chapter covers advanced URDF techniques and the Simulation Description Format (SDF) used by Gazebo for complete simulation environments.

## Learning Objectives

By the end of this chapter, you will be able to:

- Add Gazebo-specific properties to URDF models
- Configure joints for simulation with transmission and control
- Create complete SDF models for Gazebo
- Set up proper collision and friction parameters
- Integrate sensors into robot descriptions

## 10.1 URDF Extensions for Gazebo

Standard URDF lacks simulation-specific properties. The `<gazebo>` tag extends URDF with Gazebo-specific elements.

### Material and Appearance

```xml
<robot name="humanoid">
  <link name="torso">
    <visual>
      <geometry><box size="0.3 0.2 0.5"/></geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Gazebo material override -->
  <gazebo reference="torso">
    <material>Gazebo/Blue</material>
    <!-- Or custom material -->
    <visual>
      <material>
        <ambient>0.1 0.1 0.8 1</ambient>
        <diffuse>0.2 0.2 0.9 1</diffuse>
        <specular>0.5 0.5 0.5 1</specular>
      </material>
    </visual>
  </gazebo>
</robot>
```

### Contact and Friction Properties

Proper contact parameters are crucial for walking and manipulation:

```xml
<gazebo reference="foot_left">
  <!-- Surface friction (Coulomb model) -->
  <mu1>1.0</mu1>  <!-- Primary friction direction -->
  <mu2>1.0</mu2>  <!-- Secondary friction direction -->
  
  <!-- Contact stiffness and damping -->
  <kp>1000000.0</kp>  <!-- Contact stiffness (N/m) -->
  <kd>100.0</kd>       <!-- Contact damping (N·s/m) -->
  
  <!-- Minimum contact depth before force applied -->
  <minDepth>0.001</minDepth>
  
  <!-- Maximum interpenetration correction velocity -->
  <maxVel>1.0</maxVel>
</gazebo>
```

### Friction Guidelines for Humanoids

| Surface | μ (mu1/mu2) | Notes |
|---------|-------------|-------|
| Rubber foot on concrete | 0.8 - 1.0 | Good traction |
| Rubber foot on tile | 0.5 - 0.7 | Indoor walking |
| Metal on metal | 0.3 - 0.5 | Robot joints |
| Gripper on object | 0.6 - 1.2 | Depends on material |

## 10.2 Joint Transmission and Control

To control joints in Gazebo, you need transmissions and the `ros2_control` framework.

### Transmission Definition

```xml
<robot name="humanoid">
  <joint name="knee_right" type="revolute">
    <parent link="thigh_right"/>
    <child link="shin_right"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.5" effort="100" velocity="5"/>
  </joint>
  
  <!-- Transmission connects joint to actuator -->
  <transmission name="knee_right_transmission">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="knee_right">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="knee_right_motor">
      <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
      <mechanicalReduction>1</mechanicalReduction>
    </actuator>
  </transmission>
</robot>
```

### ros2_control Integration

Modern approach using ros2_control:

```xml
<ros2_control name="HumanoidSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>
  
  <!-- Position-controlled joint -->
  <joint name="shoulder_pitch_right">
    <command_interface name="position">
      <param name="min">-3.14</param>
      <param name="max">3.14</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
  
  <!-- Velocity-controlled joint -->
  <joint name="wheel_left">
    <command_interface name="velocity">
      <param name="min">-10</param>
      <param name="max">10</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  
  <!-- Effort-controlled joint (for force control) -->
  <joint name="gripper_finger">
    <command_interface name="effort">
      <param name="min">-50</param>
      <param name="max">50</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### Gazebo ros2_control Plugin

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find my_humanoid)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

### Controller Configuration (YAML)

```yaml
# controllers.yaml
controller_manager:
  ros__parameters:
    update_rate: 1000  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    leg_controller:
      type: effort_controllers/JointGroupEffortController

arm_controller:
  ros__parameters:
    joints:
      - shoulder_pitch_right
      - shoulder_roll_right
      - elbow_pitch_right
      - wrist_roll_right
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

leg_controller:
  ros__parameters:
    joints:
      - hip_pitch_right
      - knee_pitch_right
      - ankle_pitch_right
```

## 10.3 SDF Model Format

SDF (Simulation Description Format) is Gazebo's native format, offering more features than URDF.

### SDF Model Structure

```xml
<?xml version="1.0"?>
<sdf version="1.8">
  <model name="humanoid_robot">
    <static>false</static>
    
    <!-- Self-collision settings -->
    <self_collide>false</self_collide>
    
    <!-- Enable wind effects -->
    <enable_wind>false</enable_wind>
    
    <!-- Links -->
    <link name="base_link">
      <pose>0 0 0 0 0 0</pose>
      
      <inertial>
        <mass>5.0</mass>
        <inertia>
          <ixx>0.1</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.1</iyy><iyz>0</iyz>
          <izz>0.05</izz>
        </inertia>
      </inertial>
      
      <collision name="base_collision">
        <geometry>
          <box><size>0.2 0.3 0.1</size></box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000</kp>
              <kd>100</kd>
            </ode>
          </contact>
        </surface>
      </collision>
      
      <visual name="base_visual">
        <geometry>
          <box><size>0.2 0.3 0.1</size></box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.3 0.3 0.9 1</diffuse>
        </material>
      </visual>
    </link>
    
    <!-- Joints -->
    <joint name="torso_joint" type="fixed">
      <parent>base_link</parent>
      <child>torso</child>
      <pose>0 0 0.05 0 0 0</pose>
    </joint>
    
  </model>
</sdf>
```

### SDF vs URDF Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| Multiple robots | No | Yes |
| World description | No | Yes |
| Nested models | No | Yes |
| Sensor noise models | Limited | Full |
| Physics parameters | Via `<gazebo>` | Native |
| Link pose | Relative to parent | World or relative |
| Materials | Basic | Advanced (PBR) |

## 10.4 Advanced Joint Types in SDF

SDF supports additional joint features:

### Joint with Dynamics

```xml
<joint name="knee" type="revolute">
  <parent>thigh</parent>
  <child>shin</child>
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>0</lower>
      <upper>2.5</upper>
      <effort>100</effort>
      <velocity>5</velocity>
    </limit>
    <dynamics>
      <damping>0.5</damping>
      <friction>0.1</friction>
      <spring_reference>0</spring_reference>
      <spring_stiffness>0</spring_stiffness>
    </dynamics>
  </axis>
  
  <!-- Joint physics properties -->
  <physics>
    <ode>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <limit>
        <cfm>0.0</cfm>
        <erp>0.9</erp>
      </limit>
    </ode>
  </physics>
</joint>
```

### Ball Joint (3-DOF)

```xml
<joint name="hip_ball" type="ball">
  <parent>pelvis</parent>
  <child>thigh</child>
  <pose>0 0.1 0 0 0 0</pose>
</joint>
```

### Universal Joint (2-DOF)

```xml
<joint name="wrist" type="universal">
  <parent>forearm</parent>
  <child>hand</child>
  <axis>
    <xyz>1 0 0</xyz>
    <limit>
      <lower>-1.57</lower>
      <upper>1.57</upper>
    </limit>
  </axis>
  <axis2>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>-0.5</lower>
      <upper>0.5</upper>
    </limit>
  </axis2>
</joint>
```

## 10.5 Adding Sensors to Robot Models

### IMU Sensor

```xml
<link name="imu_link">
  <pose>0 0 0.5 0 0 0</pose>
  <inertial>
    <mass>0.01</mass>
    <inertia>
      <ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz>
    </inertia>
  </inertial>
  
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>imu</topic>
    <imu>
      <angular_velocity>
        <x><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise></x>
        <y><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise></y>
        <z><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise></z>
      </angular_velocity>
      <linear_acceleration>
        <x><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise></x>
        <y><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise></y>
        <z><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise></z>
      </linear_acceleration>
    </imu>
  </sensor>
</link>
```

### Camera Sensor

```xml
<link name="camera_link">
  <pose>0.1 0 0.6 0 0 0</pose>
  
  <sensor name="camera" type="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <topic>camera/image</topic>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0</mean>
        <stddev>0.007</stddev>
      </noise>
      <distortion>
        <k1>0.0</k1>
        <k2>0.0</k2>
        <k3>0.0</k3>
        <p1>0.0</p1>
        <p2>0.0</p2>
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
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.3</near>
      <far>10</far>
    </clip>
    <depth_camera>
      <clip>
        <near>0.3</near>
        <far>10</far>
      </clip>
    </depth_camera>
  </camera>
</sensor>
```

### LiDAR Sensor

```xml
<sensor name="lidar" type="gpu_lidar">
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
        <samples>16</samples>
        <resolution>1</resolution>
        <min_angle>-0.26</min_angle>
        <max_angle>0.26</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0</mean>
      <stddev>0.01</stddev>
    </noise>
  </lidar>
</sensor>
```

### Force/Torque Sensor

```xml
<joint name="wrist_ft" type="fixed">
  <parent>forearm</parent>
  <child>hand</child>
  <sensor name="wrist_force_torque" type="force_torque">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <topic>wrist_ft</topic>
    <force_torque>
      <frame>child</frame>
      <measure_direction>child_to_parent</measure_direction>
      <force>
        <x><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise></x>
        <y><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise></y>
        <z><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.1</stddev>
        </noise></z>
      </force>
      <torque>
        <x><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise></x>
        <y><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise></y>
        <z><noise type="gaussian">
          <mean>0</mean>
          <stddev>0.01</stddev>
        </noise></z>
      </torque>
    </force_torque>
  </sensor>
</joint>
```

### Contact Sensor

```xml
<link name="foot_left">
  <collision name="foot_collision">
    <geometry><box><size>0.15 0.08 0.03</size></box></geometry>
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

## 10.6 URDF to SDF Conversion Best Practices

### Automatic Conversion

```bash
# Basic conversion
gz sdf -p robot.urdf > robot.sdf

# From Xacro
ros2 run xacro xacro robot.urdf.xacro | gz sdf -p /dev/stdin > robot.sdf
```

### Manual Optimization

After conversion, optimize for simulation:

1. **Add sensor noise models** - Unrealistic without noise
2. **Tune contact parameters** - Default values often unstable
3. **Simplify collision geometry** - Use primitives over meshes
4. **Add self-collision filtering** - Prevent impossible collisions

### Collision Filtering

```xml
<model name="humanoid">
  <!-- Disable collision between adjacent links -->
  <collision_filter>
    <disabled_by_default>false</disabled_by_default>
    <enable_contact>
      <!-- Enable foot-ground contact -->
      <collision>foot_left::collision</collision>
      <collision>ground::collision</collision>
    </enable_contact>
    <disable_contact>
      <!-- Disable collision between connected links -->
      <collision>thigh_left::collision</collision>
      <collision>shin_left::collision</collision>
    </disable_contact>
  </collision_filter>
</model>
```

## 10.7 Complete Humanoid Leg Example

```xml
<?xml version="1.0"?>
<robot name="humanoid_leg" xmlns:xacro="http://www.ros.org/wiki/xacro">
  
  <!-- Properties -->
  <xacro:property name="thigh_length" value="0.4"/>
  <xacro:property name="shin_length" value="0.38"/>
  <xacro:property name="foot_length" value="0.2"/>
  <xacro:property name="leg_radius" value="0.04"/>
  <xacro:property name="thigh_mass" value="5.0"/>
  <xacro:property name="shin_mass" value="3.0"/>
  <xacro:property name="foot_mass" value="1.0"/>
  
  <!-- Inertia macros -->
  <xacro:macro name="cylinder_inertia" params="m r l">
    <inertia 
      ixx="${(1/12)*m*(3*r*r + l*l)}" ixy="0" ixz="0"
      iyy="${(1/12)*m*(3*r*r + l*l)}" iyz="0"
      izz="${0.5*m*r*r}"/>
  </xacro:macro>
  
  <xacro:macro name="box_inertia" params="m x y z">
    <inertia 
      ixx="${(1/12)*m*(y*y + z*z)}" ixy="0" ixz="0"
      iyy="${(1/12)*m*(x*x + z*z)}" iyz="0"
      izz="${(1/12)*m*(x*x + y*y)}"/>
  </xacro:macro>
  
  <!-- Leg macro -->
  <xacro:macro name="leg" params="side reflect">
    
    <!-- Hip link (minimal, for joint attachment) -->
    <link name="${side}_hip"/>
    
    <!-- Thigh -->
    <link name="${side}_thigh">
      <visual>
        <origin xyz="0 0 ${-thigh_length/2}"/>
        <geometry>
          <cylinder radius="${leg_radius}" length="${thigh_length}"/>
        </geometry>
        <material name="skin"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-thigh_length/2}"/>
        <geometry>
          <cylinder radius="${leg_radius*1.1}" length="${thigh_length}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-thigh_length/2}"/>
        <mass value="${thigh_mass}"/>
        <xacro:cylinder_inertia m="${thigh_mass}" r="${leg_radius}" l="${thigh_length}"/>
      </inertial>
    </link>
    
    <!-- Shin -->
    <link name="${side}_shin">
      <visual>
        <origin xyz="0 0 ${-shin_length/2}"/>
        <geometry>
          <cylinder radius="${leg_radius*0.9}" length="${shin_length}"/>
        </geometry>
        <material name="skin"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-shin_length/2}"/>
        <geometry>
          <cylinder radius="${leg_radius}" length="${shin_length}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-shin_length/2}"/>
        <mass value="${shin_mass}"/>
        <xacro:cylinder_inertia m="${shin_mass}" r="${leg_radius*0.9}" l="${shin_length}"/>
      </inertial>
    </link>
    
    <!-- Foot -->
    <link name="${side}_foot">
      <visual>
        <origin xyz="0.03 0 -0.015"/>
        <geometry>
          <box size="${foot_length} 0.08 0.03"/>
        </geometry>
        <material name="dark_gray"/>
      </visual>
      <collision>
        <origin xyz="0.03 0 -0.015"/>
        <geometry>
          <box size="${foot_length} 0.08 0.03"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0.03 0 -0.015"/>
        <mass value="${foot_mass}"/>
        <xacro:box_inertia m="${foot_mass}" x="${foot_length}" y="0.08" z="0.03"/>
      </inertial>
    </link>
    
    <!-- Hip Pitch Joint -->
    <joint name="${side}_hip_pitch" type="revolute">
      <parent link="${side}_hip"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="150" velocity="5"/>
      <dynamics damping="1.0" friction="0.1"/>
    </joint>
    
    <!-- Knee Joint -->
    <joint name="${side}_knee" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 ${-thigh_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.5" effort="120" velocity="5"/>
      <dynamics damping="0.8" friction="0.1"/>
    </joint>
    
    <!-- Ankle Joint -->
    <joint name="${side}_ankle" type="revolute">
      <parent link="${side}_shin"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 ${-shin_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.8" upper="0.8" effort="80" velocity="5"/>
      <dynamics damping="0.5" friction="0.1"/>
    </joint>
    
    <!-- Gazebo friction for foot -->
    <gazebo reference="${side}_foot">
      <mu1>1.0</mu1>
      <mu2>1.0</mu2>
      <kp>1000000</kp>
      <kd>100</kd>
      <minDepth>0.001</minDepth>
    </gazebo>
    
    <!-- ros2_control -->
    <ros2_control name="${side}_leg_control" type="system">
      <hardware>
        <plugin>gazebo_ros2_control/GazeboSystem</plugin>
      </hardware>
      <joint name="${side}_hip_pitch">
        <command_interface name="effort"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      <joint name="${side}_knee">
        <command_interface name="effort"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
      <joint name="${side}_ankle">
        <command_interface name="effort"/>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
        <state_interface name="effort"/>
      </joint>
    </ros2_control>
    
  </xacro:macro>
  
  <!-- Materials -->
  <material name="skin">
    <color rgba="0.96 0.8 0.69 1"/>
  </material>
  <material name="dark_gray">
    <color rgba="0.3 0.3 0.3 1"/>
  </material>
  
</robot>
```

## 10.8 Summary

In this chapter, you learned:

- **Gazebo extensions**: Adding friction, contact, and material properties
- **Joint control**: Transmissions and ros2_control integration
- **SDF format**: Native Gazebo format with advanced features
- **Sensor integration**: IMU, cameras, LiDAR, force/torque sensors
- **Best practices**: Collision filtering, parameter tuning

Properly configured robot descriptions are essential for accurate simulation. In the next chapter, we'll explore physics simulation in depth.

## Review Questions

1. Why are friction parameters important for humanoid walking simulation?
2. What is the difference between URDF transmissions and ros2_control?
3. When would you use SDF instead of URDF?
4. How do you add realistic noise to simulated sensors?
5. What is collision filtering and why is it necessary?

## Hands-On Exercise

1. Take the URDF from Chapter 8 and add Gazebo friction properties
2. Configure ros2_control for position control of arm joints
3. Add an IMU sensor to the robot's torso
4. Add a camera to the robot's head
5. Convert to SDF and verify the simulation works in Gazebo
