---
sidebar_position: 6
title: "Chapter 6: Building ROS 2 Packages"
description: "Creating and organizing ROS 2 packages with Python"
---

# Building ROS 2 Packages with Python

## Learning Objectives

By the end of this chapter, you will be able to:

1. Create a ROS 2 Python package from scratch
2. Configure package.xml and setup.py correctly
3. Organize code following ROS 2 best practices
4. Build and install packages using colcon

## Prerequisites

- [Chapter 5: Nodes, Topics, Services, and Actions](/module-1-ros2/nodes-topics-services)
- ROS 2 workspace set up

## Package Structure

A ROS 2 Python package follows this structure:

```
my_robot_package/
├── package.xml              # Package manifest
├── setup.py                 # Python package setup
├── setup.cfg                # Setup configuration
├── resource/
│   └── my_robot_package     # Marker file for ament
├── my_robot_package/        # Python module
│   ├── __init__.py
│   ├── robot_controller.py
│   └── sensor_reader.py
├── launch/                  # Launch files
│   └── robot.launch.py
├── config/                  # Configuration files
│   └── params.yaml
└── test/                    # Test files
    └── test_controller.py
```

## Creating a Package

```bash
cd ~/ros2_ws/src

# Create a Python package
ros2 pkg create --build-type ament_python my_robot_package \
    --dependencies rclpy std_msgs geometry_msgs
```

## package.xml Configuration

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.1.0</version>
  <description>My robot control package</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>Apache-2.0</license>

  <!-- Build dependencies -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Runtime dependencies -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>

  <!-- Test dependencies -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## setup.py Configuration

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Developer',
    maintainer_email='dev@example.com',
    description='My robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = my_robot_package.robot_controller:main',
            'sensor_reader = my_robot_package.sensor_reader:main',
        ],
    },
)
```

## Example Node Implementation

```python
#!/usr/bin/env python3
"""Robot controller node implementation."""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class RobotController(Node):
    """Controls robot movement based on sensor input."""

    def __init__(self):
        super().__init__('robot_controller')
        
        # Declare parameters
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('safety_distance', 0.5)
        
        # Get parameters
        self.max_speed = self.get_parameter('max_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10
        )
        
        # Create subscriber for laser scan
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.obstacle_detected = False
        self.get_logger().info('Robot controller initialized')

    def scan_callback(self, msg):
        """Process laser scan data."""
        # Check front readings for obstacles
        front_ranges = msg.ranges[len(msg.ranges)//3:2*len(msg.ranges)//3]
        min_distance = min(r for r in front_ranges if r > msg.range_min)
        
        self.obstacle_detected = min_distance < self.safety_distance

    def control_loop(self):
        """Main control loop."""
        cmd = Twist()
        
        if self.obstacle_detected:
            # Stop and turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().warn('Obstacle detected! Turning...')
        else:
            # Move forward
            cmd.linear.x = self.max_speed
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Building the Package

```bash
cd ~/ros2_ws

# Build specific package
colcon build --packages-select my_robot_package

# Build with symlink install (for development)
colcon build --packages-select my_robot_package --symlink-install

# Source the workspace
source install/setup.bash

# Run the node
ros2 run my_robot_package robot_controller
```

## Summary

- ROS 2 Python packages have a specific structure with package.xml and setup.py
- Use `ros2 pkg create` to scaffold new packages
- Configure entry_points in setup.py to create executable nodes
- Use `colcon build` to build packages

## Exercises

### Exercise 1
Create a package called `my_first_robot` with a node that publishes odometry data.

### Exercise 2
Add parameters for robot dimensions and make them configurable via a YAML file.

### Exercise 3
Create a launch file that starts multiple nodes with different configurations.

## References

1. ROS 2 Creating Packages. https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

---

**Next Chapter**: [Launch Files & Parameter Management](/module-1-ros2/launch-files)
