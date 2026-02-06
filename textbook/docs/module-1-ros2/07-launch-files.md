---
sidebar_position: 7
title: "Chapter 7: Launch Files"
description: "Launch files and parameter management in ROS 2"
---

# Launch Files & Parameter Management

## Learning Objectives

By the end of this chapter, you will be able to:

1. Create Python-based launch files for ROS 2
2. Configure and pass parameters to nodes
3. Use YAML files for parameter configuration
4. Organize complex multi-node launches

## Prerequisites

- [Chapter 6: Building ROS 2 Packages](/module-1-ros2/ros2-python-packages)

## Why Launch Files?

Launch files allow you to:
- Start multiple nodes with a single command
- Configure node parameters
- Set up namespaces and remappings
- Define node dependencies and lifecycle

## Basic Launch File

```python
# launch/robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='controller',
            output='screen',
        ),
        Node(
            package='my_robot_package',
            executable='sensor_reader',
            name='sensors',
            output='screen',
        ),
    ])
```

## Parameters in Launch Files

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.5',
        description='Maximum robot speed'
    )
    
    return LaunchDescription([
        max_speed_arg,
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='controller',
            parameters=[{
                'max_speed': LaunchConfiguration('max_speed'),
                'safety_distance': 0.3,
            }],
            output='screen',
        ),
    ])
```

## Using YAML Parameter Files

```yaml
# config/robot_params.yaml
robot_controller:
  ros__parameters:
    max_speed: 0.5
    safety_distance: 0.3
    control_frequency: 10.0
    
sensor_reader:
  ros__parameters:
    publish_rate: 30.0
    sensor_frame: "base_link"
```

```python
# launch/robot_with_params.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_package'),
        'config',
        'robot_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='my_robot_package',
            executable='robot_controller',
            name='robot_controller',
            parameters=[config],
            output='screen',
        ),
    ])
```

## Remapping and Namespaces

```python
Node(
    package='my_robot_package',
    executable='robot_controller',
    namespace='robot1',
    remappings=[
        ('cmd_vel', 'robot1/cmd_vel'),
        ('scan', 'robot1/scan'),
    ],
)
```

## Including Other Launch Files

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('other_package'),
        '/launch/other.launch.py'
    ]),
    launch_arguments={'arg_name': 'value'}.items(),
)
```

## Running Launch Files

```bash
# Run launch file
ros2 launch my_robot_package robot.launch.py

# With arguments
ros2 launch my_robot_package robot.launch.py max_speed:=1.0
```

## Summary

- Launch files start multiple nodes with configuration
- Use YAML files for clean parameter management
- Namespaces and remappings enable multi-robot setups
- Include other launch files for modular organization

## Exercises

### Exercise 1
Create a launch file that starts a publisher and subscriber with configurable topic names.

### Exercise 2
Create a multi-robot launch file using namespaces.

## References

1. ROS 2 Launch Documentation. https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html

---

**Next Chapter**: [URDF: Unified Robot Description Format](/module-1-ros2/urdf)
