---
sidebar_position: 2
title: "Chapter 15: Isaac Sim for Robotics"
description: "Create photorealistic simulations and generate synthetic data for humanoid robots"
---

# Chapter 15: Isaac Sim for Robotics

**Isaac Sim** provides the most advanced robotics simulation capabilities available, combining photorealistic rendering with accurate physics. This chapter covers practical use of Isaac Sim for humanoid robot development, from importing models to generating synthetic training data.

## Learning Objectives

By the end of this chapter, you will be able to:

- Navigate the Isaac Sim interface and workspace
- Import and configure humanoid robot models
- Set up simulation environments and scenarios
- Configure sensors with realistic noise models
- Generate synthetic data for machine learning

## 15.1 Isaac Sim Interface

### Main Components

```
┌─────────────────────────────────────────────────────────────┐
│  Menu Bar                                      ▢ ▢ ✕       │
├──────────┬──────────────────────────────────────┬───────────┤
│          │                                      │           │
│  Stage   │           Viewport                   │  Property │
│  Panel   │        (3D Scene View)               │   Panel   │
│          │                                      │           │
│          │                                      │           │
├──────────┴──────────────────────────────────────┴───────────┤
│  Console / Content Browser / Script Editor                  │
├─────────────────────────────────────────────────────────────┤
│  Play ▶ │ Pause ⏸ │ Stop ⏹ │ Step ⏭    Timeline            │
└─────────────────────────────────────────────────────────────┘
```

### Key Panels

| Panel | Purpose |
|-------|---------|
| **Stage** | Hierarchical view of scene prims |
| **Viewport** | 3D visualization and interaction |
| **Property** | Edit selected prim attributes |
| **Content** | Browse assets and files |
| **Console** | Python scripting and logs |

### Viewport Navigation

| Action | Control |
|--------|---------|
| Orbit | Alt + Left Mouse |
| Pan | Alt + Middle Mouse |
| Zoom | Alt + Right Mouse / Scroll |
| Focus | F (with object selected) |
| Frame All | Shift + F |

## 15.2 Importing Robot Models

### URDF Import

```python
import omni.isaac.core.utils.extensions as ext_utils
ext_utils.enable_extension("omni.importer.urdf")

from omni.importer.urdf import _urdf
from omni.isaac.core.utils.stage import add_reference_to_stage

# Configure import settings
import_config = _urdf.ImportConfig()
import_config.merge_fixed_joints = False
import_config.fix_base = False
import_config.make_default_prim = True
import_config.create_physics_scene = True

# Import URDF
urdf_interface = _urdf.acquire_urdf_interface()
result = urdf_interface.parse_urdf(
    "/path/to/humanoid.urdf",
    import_config
)

# Import to stage
urdf_interface.import_robot(
    "/path/to/humanoid.urdf",
    result,
    import_config,
    "/World/Humanoid"
)
```

### USD Import

```python
from omni.isaac.core.utils.stage import add_reference_to_stage

# Add robot USD to stage
add_reference_to_stage(
    usd_path="/path/to/humanoid.usd",
    prim_path="/World/Humanoid"
)
```

### Configuring the Robot

```python
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.prims import get_prim_at_path

# Get robot articulation
robot = Articulation(prim_path="/World/Humanoid")

# Initialize after world reset
world.reset()
robot.initialize()

# Get joint information
joint_names = robot.dof_names
num_dofs = robot.num_dof

print(f"Robot has {num_dofs} DOFs:")
for name in joint_names:
    print(f"  - {name}")
```

## 15.3 Setting Up Articulations

### Articulation Root

```python
from pxr import UsdPhysics, PhysxSchema

# Get robot prim
robot_prim = get_prim_at_path("/World/Humanoid")

# Apply articulation root
articulation_api = UsdPhysics.ArticulationRootAPI.Apply(robot_prim)

# Configure articulation
physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)
physx_articulation.CreateEnabledSelfCollisionsAttr(False)
physx_articulation.CreateSolverPositionIterationCountAttr(32)
physx_articulation.CreateSolverVelocityIterationCountAttr(16)
```

### Joint Configuration

```python
from pxr import UsdPhysics, Gf

# Configure a revolute joint
joint_prim = get_prim_at_path("/World/Humanoid/knee_right")
revolute_joint = UsdPhysics.RevoluteJoint(joint_prim)

# Set joint limits
revolute_joint.CreateLowerLimitAttr(0)  # radians
revolute_joint.CreateUpperLimitAttr(2.5)

# Set drive (position control)
drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
drive_api.CreateTypeAttr("force")
drive_api.CreateTargetPositionAttr(0)
drive_api.CreateDampingAttr(1000)
drive_api.CreateStiffnessAttr(10000)
drive_api.CreateMaxForceAttr(1000)
```

### Joint Control

```python
# Position control
robot.set_joint_positions(
    positions=[0.0, 0.5, -0.3, 0.0, 0.5, -0.3],  # rad
    joint_indices=[0, 1, 2, 3, 4, 5]
)

# Velocity control
robot.set_joint_velocities(
    velocities=[0.1, 0.1, 0.1],
    joint_indices=[0, 1, 2]
)

# Effort (torque) control
robot.set_joint_efforts(
    efforts=[10.0, 20.0, 15.0],  # N·m
    joint_indices=[0, 1, 2]
)
```

## 15.4 Physics Configuration

### Physics Scene Settings

```python
from omni.isaac.core import World
from omni.isaac.core.physics_context import PhysicsContext

# Create world with physics
world = World(physics_dt=1/500, rendering_dt=1/60, stage_units_in_meters=1.0)

# Configure physics context
physics_context = world.get_physics_context()
physics_context.set_gravity(value=[0, 0, -9.81])
physics_context.set_solver_type("TGS")  # Temporal Gauss-Seidel
physics_context.enable_gpu_dynamics(True)
physics_context.enable_ccd(True)  # Continuous collision detection
```

### Contact and Friction

```python
from pxr import PhysxSchema, UsdPhysics

# Get collision prim
foot_collision = get_prim_at_path("/World/Humanoid/foot_left/collision")

# Apply material
material = UsdPhysics.MaterialAPI.Apply(foot_collision)
material.CreateStaticFrictionAttr(0.8)
material.CreateDynamicFrictionAttr(0.7)
material.CreateRestitutionAttr(0.1)

# PhysX specific settings
physx_material = PhysxSchema.PhysxMaterialAPI.Apply(foot_collision)
physx_material.CreateFrictionCombineModeAttr("multiply")
physx_material.CreateRestitutionCombineModeAttr("min")
```

## 15.5 Environment Setup

### Ground Plane

```python
from omni.isaac.core.objects import GroundPlane

# Add ground with physics
ground = world.scene.add(
    GroundPlane(
        prim_path="/World/Ground",
        size=100,
        color=np.array([0.5, 0.5, 0.5])
    )
)
```

### Indoor Environment

```python
import omni.isaac.core.utils.prims as prim_utils

# Load environment USD
prim_utils.create_prim(
    prim_path="/World/Environment",
    usd_path="/Isaac/Environments/Simple_Room/simple_room.usd"
)

# Or create basic room
from omni.isaac.core.objects import FixedCuboid

# Floor
floor = FixedCuboid(
    prim_path="/World/Room/Floor",
    size=np.array([10, 10, 0.1]),
    position=np.array([0, 0, -0.05])
)

# Walls
wall_north = FixedCuboid(
    prim_path="/World/Room/WallNorth",
    size=np.array([10, 0.2, 3]),
    position=np.array([0, 5, 1.5])
)
```

### Dynamic Objects

```python
from omni.isaac.core.objects import DynamicCuboid, DynamicSphere

# Add manipulable objects
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Objects/RedCube",
        size=np.array([0.05, 0.05, 0.05]),
        position=np.array([0.5, 0, 1.0]),
        color=np.array([1, 0, 0]),
        mass=0.1
    )
)

sphere = world.scene.add(
    DynamicSphere(
        prim_path="/World/Objects/BlueSphere",
        radius=0.03,
        position=np.array([0.6, 0, 1.0]),
        color=np.array([0, 0, 1]),
        mass=0.05
    )
)
```

## 15.6 Sensor Configuration

### RGB Camera

```python
from omni.isaac.sensor import Camera
import omni.replicator.core as rep

# Add camera sensor
camera = Camera(
    prim_path="/World/Humanoid/head/camera",
    position=np.array([0.05, 0, 0]),
    frequency=30,
    resolution=(640, 480)
)

# Initialize camera
camera.initialize()

# Get RGB data
rgb_data = camera.get_rgba()[:, :, :3]  # Remove alpha

# Get camera intrinsics
intrinsics = camera.get_intrinsics_matrix()
```

### Depth Camera

```python
# Configure depth output
camera.add_distance_to_image_plane_to_frame()
camera.add_distance_to_camera_to_frame()

# Get depth data
depth_data = camera.get_distance_to_image_plane()

# Point cloud
points = camera.get_pointcloud()
```

### LiDAR

```python
from omni.isaac.sensor import RotatingLidarPhysX

lidar = RotatingLidarPhysX(
    prim_path="/World/Humanoid/torso/lidar",
    rotation_frequency=10,  # Hz
    fov=(360, 30),  # (horizontal, vertical) degrees
    resolution=(0.2, 0.4),  # angular resolution
    valid_range=(0.1, 100)  # meters
)

lidar.initialize()

# Get point cloud
point_cloud = lidar.get_current_frame()["point_cloud"]
```

### IMU

```python
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(
    prim_path="/World/Humanoid/base/imu",
    name="base_imu",
    frequency=200,
    translation=np.array([0, 0, 0])
)

imu.initialize()

# Get IMU reading
imu_data = imu.get_current_frame()
linear_acceleration = imu_data["lin_acc"]
angular_velocity = imu_data["ang_vel"]
orientation = imu_data["orientation"]
```

### Contact Sensor

```python
from omni.isaac.sensor import ContactSensor

foot_contact = ContactSensor(
    prim_path="/World/Humanoid/foot_left/contact_sensor",
    min_threshold=0.1,
    max_threshold=10000000,
    radius=0.05
)

foot_contact.initialize()

# Check contact
contact_data = foot_contact.get_current_frame()
is_in_contact = contact_data["in_contact"]
contact_force = contact_data["force"]
```

## 15.7 Synthetic Data Generation

### Replicator Setup

```python
import omni.replicator.core as rep

# Create randomizers
with rep.trigger.on_frame():
    # Randomize lighting
    lights = rep.get.light()
    with lights:
        rep.modify.attribute("intensity", rep.distribution.uniform(500, 2000))
        rep.modify.attribute("color", rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1)))
    
    # Randomize object positions
    cubes = rep.get.prims(path_pattern="/World/Objects/.*")
    with cubes:
        rep.modify.pose(
            position=rep.distribution.uniform((-1, -1, 0.8), (1, 1, 1.2))
        )
```

### Data Writers

```python
# RGB writer
rgb_writer = rep.WriterRegistry.get("BasicWriter")
rgb_writer.initialize(
    output_dir="/output/rgb",
    rgb=True
)
rgb_writer.attach([camera._render_product_path])

# Annotation writers
with rep.new_layer():
    # Bounding boxes
    rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
    
    # Semantic segmentation
    rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
    
    # Instance segmentation
    rep.AnnotatorRegistry.get_annotator("instance_segmentation")
    
    # Depth
    rep.AnnotatorRegistry.get_annotator("distance_to_camera")
```

### Domain Randomization

```python
import omni.replicator.core as rep

def setup_domain_randomization():
    # Material randomization
    with rep.trigger.on_frame(num_frames=100):
        mats = rep.get.material(path_pattern="/World/Environment/.*")
        with mats:
            rep.modify.attribute(
                "diffuse_color",
                rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9))
            )
    
    # Texture randomization
    textures = [
        "/Isaac/Materials/Textures/Patterns/nv_asphalt_2.jpg",
        "/Isaac/Materials/Textures/Patterns/nv_concrete_clean.jpg",
        "/Isaac/Materials/Textures/Patterns/nv_tile_hexagon.jpg",
    ]
    
    floor_mat = rep.get.material(path_pattern="/World/Room/Floor.*")
    with floor_mat:
        rep.modify.attribute(
            "diffuse_texture",
            rep.distribution.choice(textures)
        )
```

### Generating Dataset

```python
# Run data generation
async def generate_data(num_frames=1000):
    for i in range(num_frames):
        # Step simulation
        await world.step_async()
        
        # Capture frame
        await rep.orchestrator.step_async()
        
        if i % 100 == 0:
            print(f"Generated {i}/{num_frames} frames")

# Run
import asyncio
asyncio.ensure_future(generate_data(1000))
```

## 15.8 ROS 2 Bridge

### Enabling ROS 2

```python
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS 2 bridge
enable_extension("omni.isaac.ros2_bridge")

# Import ROS utilities
from omni.isaac.ros2_bridge import read_camera_info
```

### Publishing Sensors to ROS

```python
import omni.graph.core as og

# Create OmniGraph for ROS 2 publishing
keys = og.Controller.Keys
og.Controller.edit(
    {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("CameraHelper", "omni.isaac.ros2_bridge.ROS2CameraHelper"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "CameraHelper.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ],
        keys.SET_VALUES: [
            ("CameraHelper.inputs:cameraPrim", "/World/Humanoid/head/camera"),
            ("CameraHelper.inputs:topicName", "camera"),
            ("CameraHelper.inputs:frameId", "camera_link"),
            ("CameraHelper.inputs:type", "rgb"),
        ],
    }
)
```

### Subscribing to ROS Commands

```python
# Create subscriber graph
og.Controller.edit(
    {"graph_path": "/World/ROS2SubGraph"},
    {
        keys.CREATE_NODES: [
            ("SubscribeTwist", "omni.isaac.ros2_bridge.ROS2SubscribeTwist"),
            ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        ],
        keys.CONNECT: [
            ("SubscribeTwist.outputs:linearVelocity", "ArticulationController.inputs:velocityCommand"),
        ],
        keys.SET_VALUES: [
            ("SubscribeTwist.inputs:topicName", "cmd_vel"),
            ("ArticulationController.inputs:robotPath", "/World/Humanoid"),
        ],
    }
)
```

## 15.9 Standalone Scripts

### Complete Simulation Script

```python
#!/usr/bin/env python3
"""Standalone Isaac Sim script for humanoid simulation."""

from omni.isaac.kit import SimulationApp

# Launch simulation app
config = {
    "headless": False,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(config)

# Now import Isaac modules
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.sensor import Camera, IMUSensor

# Enable ROS 2
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

# Create world
world = World(physics_dt=1/500, rendering_dt=1/60)
world.scene.add_default_ground_plane()

# Add robot
add_reference_to_stage(
    usd_path="/path/to/humanoid.usd",
    prim_path="/World/Humanoid"
)

# Create articulation wrapper
robot = Articulation(prim_path="/World/Humanoid")
world.scene.add(robot)

# Add sensors
camera = Camera(
    prim_path="/World/Humanoid/head/camera",
    resolution=(640, 480),
    frequency=30
)

imu = IMUSensor(
    prim_path="/World/Humanoid/base/imu",
    frequency=200
)

# Initialize
world.reset()
robot.initialize()
camera.initialize()
imu.initialize()

# Control loop
step = 0
while simulation_app.is_running():
    # Step physics
    world.step(render=True)
    
    # Get sensor data
    if step % 10 == 0:  # 50 Hz
        rgb = camera.get_rgba()
        imu_data = imu.get_current_frame()
        
    # Simple standing pose
    target_positions = np.zeros(robot.num_dof)
    robot.set_joint_position_targets(target_positions)
    
    step += 1

# Cleanup
simulation_app.close()
```

## 15.10 Summary

In this chapter, you learned:

- **Isaac Sim interface**: Navigation and key panels
- **Robot import**: URDF and USD model import
- **Articulations**: Joint configuration and control
- **Physics**: Simulation parameters and materials
- **Sensors**: Camera, LiDAR, IMU, contact sensors
- **Synthetic data**: Domain randomization and data generation
- **ROS 2 integration**: Publishing and subscribing via bridge

Isaac Sim provides the foundation for developing perception and control algorithms with photorealistic simulation. In the next chapter, we'll explore Isaac ROS for hardware-accelerated perception.

## Review Questions

1. How do you import a URDF robot into Isaac Sim?
2. What is the difference between position and effort control?
3. How does domain randomization improve sim-to-real transfer?
4. What sensors are available in Isaac Sim?
5. How do you publish camera data to ROS 2?

## Hands-On Exercise

1. Launch Isaac Sim and import a humanoid robot
2. Configure joints for position control
3. Add camera and IMU sensors
4. Set up ROS 2 bridge for sensor publishing
5. Create a script that moves the robot's arm
6. Generate a synthetic dataset with domain randomization
