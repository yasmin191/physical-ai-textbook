---
sidebar_position: 4
title: "Chapter 17: Visual SLAM and Navigation"
description: "Simultaneous localization and mapping for humanoid robots"
---

# Chapter 17: Visual SLAM and Navigation

**Visual SLAM (Simultaneous Localization and Mapping)** enables robots to build maps of unknown environments while tracking their position within those maps. For humanoid robots, Visual SLAM is essential for autonomous navigation in dynamic, human-populated spaces.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the fundamentals of Visual SLAM
- Configure and tune Visual SLAM for humanoid robots
- Integrate Visual SLAM with IMU for robust tracking
- Handle challenging scenarios (dynamic objects, lighting changes)
- Build and save maps for repeated navigation

## 17.1 SLAM Fundamentals

### The SLAM Problem

SLAM solves two coupled problems simultaneously:

```
┌─────────────────────────────────────────────────┐
│                    SLAM                          │
│  ┌─────────────────┐    ┌──────────────────┐    │
│  │   Localization  │◄──►│     Mapping      │    │
│  │  "Where am I?"  │    │  "What's around" │    │
│  └─────────────────┘    └──────────────────┘    │
│           │                      │              │
│           └──────────┬───────────┘              │
│                      │                          │
│              ┌───────▼───────┐                  │
│              │   Landmarks   │                  │
│              │  (features)   │                  │
│              └───────────────┘                  │
└─────────────────────────────────────────────────┘
```

### Visual SLAM Pipeline

```
Camera Images
      │
      ▼
┌─────────────┐
│   Feature   │  ◄── ORB, SIFT, or learned features
│  Detection  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Feature   │  ◄── Match across frames
│  Matching   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│    Motion   │  ◄── Estimate camera movement
│  Estimation │
└──────┬──────┘
       │
       ├────────────────────┐
       ▼                    ▼
┌─────────────┐      ┌─────────────┐
│    Local    │      │    Loop     │
│    Map      │      │   Closure   │
└─────────────┘      └─────────────┘
       │                    │
       └────────┬───────────┘
                │
                ▼
┌───────────────────────────┐
│      Global Map &         │
│    Optimized Trajectory   │
└───────────────────────────┘
```

## 17.2 Visual SLAM Methods

### Feature-Based Methods

**ORB-SLAM3** is a popular feature-based method:

```python
# ORB-SLAM3 configuration for stereo camera
%YAML:1.0

Camera.type: "PinHole"
Camera.fx: 458.654
Camera.fy: 457.296
Camera.cx: 367.215
Camera.cy: 248.375

Camera.k1: -0.28340811
Camera.k2: 0.07395907
Camera.p1: 0.00019359
Camera.p2: 1.76187114e-05

# Stereo baseline
Camera.bf: 47.9

# ORB parameters
ORBextractor.nFeatures: 1200
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7
```

### Direct Methods

Direct methods use pixel intensities directly:

| Method | Approach | Advantages |
|--------|----------|------------|
| LSD-SLAM | Semi-dense | Works in low-texture |
| DSO | Direct sparse | Accurate, efficient |
| DVSO | Deep + direct | Learned depth priors |

### Learning-Based Methods

Modern approaches use neural networks:

- **SuperPoint**: Learned feature detection
- **SuperGlue**: Learned feature matching
- **DROID-SLAM**: End-to-end learned SLAM

## 17.3 Isaac ROS Visual SLAM

### Configuration

```python
from launch_ros.descriptions import ComposableNode

visual_slam_node = ComposableNode(
    name='visual_slam_node',
    package='isaac_ros_visual_slam',
    plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
    parameters=[{
        # Camera model
        'rectified_images': True,
        'enable_rectified_pose': True,
        
        # Tracking settings
        'enable_localization_n_mapping': True,
        'enable_observations_view': True,
        'enable_landmarks_view': True,
        
        # Map settings
        'map_frame': 'map',
        'odom_frame': 'odom',
        'base_frame': 'base_link',
        'input_left_camera_frame': 'camera_left',
        'input_right_camera_frame': 'camera_right',
        
        # Image processing
        'denoise_input_images': True,
        'horizontal_stereo_camera': True,
        
        # Feature detection
        'num_cameras': 2,
        'min_num_images': 2,
        
        # IMU fusion
        'enable_imu_fusion': True,
        'imu_frame': 'imu_link',
        'gyro_noise_density': 0.000244,
        'gyro_random_walk': 0.000019393,
        'accel_noise_density': 0.001862,
        'accel_random_walk': 0.003,
        
        # Verbosity
        'enable_debug_mode': False,
        'debug_dump_path': '/tmp/vslam_debug',
    }],
    remappings=[
        ('stereo_camera/left/image', '/stereo/left/image_rect'),
        ('stereo_camera/left/camera_info', '/stereo/left/camera_info'),
        ('stereo_camera/right/image', '/stereo/right/image_rect'),
        ('stereo_camera/right/camera_info', '/stereo/right/camera_info'),
        ('visual_slam/imu', '/imu/data'),
    ]
)
```

### IMU-Camera Calibration

The transform between IMU and camera must be precise:

```yaml
# Camera-IMU extrinsics (T_cam_imu)
# Transform from IMU frame to camera frame
T_cam_imu:
  - [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975]
  - [0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768]
  - [-0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949]
  - [0.0, 0.0, 0.0, 1.0]
```

### Kalibr Calibration

Use Kalibr for camera-IMU calibration:

```bash
# Camera calibration
rosrun kalibr kalibr_calibrate_cameras \
  --bag camera.bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-radtan pinhole-radtan \
  --target april_6x6.yaml

# IMU calibration
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag imu_camera.bag \
  --cam camera.yaml \
  --imu imu.yaml \
  --target april_6x6.yaml
```

## 17.4 Visual-Inertial Odometry

### VIO Architecture

```
                    ┌───────────────────────────┐
                    │    Visual-Inertial        │
                    │       Odometry            │
                    └───────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌───────────────┐     ┌───────────────┐     ┌───────────────┐
│    Vision     │     │     IMU       │     │    Fusion     │
│   Frontend    │     │  Integration  │     │   Backend     │
└───────────────┘     └───────────────┘     └───────────────┘
        │                     │                     │
        │  Features           │  Pre-integration   │  Optimization
        │  Optical Flow       │  Bias estimation   │  Graph SLAM
```

### IMU Pre-integration

IMU measurements are pre-integrated between visual frames:

```python
import numpy as np

class IMUPreintegration:
    def __init__(self, gravity=9.81):
        self.gravity = np.array([0, 0, -gravity])
        self.reset()
    
    def reset(self):
        self.delta_R = np.eye(3)
        self.delta_v = np.zeros(3)
        self.delta_p = np.zeros(3)
        self.dt_sum = 0
    
    def integrate(self, gyro, accel, dt):
        """Integrate IMU measurement."""
        # Remove bias (simplified)
        omega = gyro  # - gyro_bias
        a = accel  # - accel_bias
        
        # Rotation update
        dR = self.exp_so3(omega * dt)
        self.delta_R = self.delta_R @ dR
        
        # Velocity update
        self.delta_v += self.delta_R @ a * dt
        
        # Position update
        self.delta_p += self.delta_v * dt + 0.5 * self.delta_R @ a * dt**2
        
        self.dt_sum += dt
    
    def exp_so3(self, omega):
        """Exponential map for SO(3)."""
        theta = np.linalg.norm(omega)
        if theta < 1e-10:
            return np.eye(3)
        k = omega / theta
        K = np.array([
            [0, -k[2], k[1]],
            [k[2], 0, -k[0]],
            [-k[1], k[0], 0]
        ])
        return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * K @ K
```

## 17.5 Loop Closure

### Place Recognition

Loop closure detects when the robot revisits a location:

```python
import numpy as np
from sklearn.neighbors import NearestNeighbors

class LoopClosureDetector:
    def __init__(self, threshold=0.3):
        self.threshold = threshold
        self.descriptors = []  # Global descriptors for each keyframe
        self.poses = []
    
    def add_keyframe(self, descriptor, pose):
        """Add a new keyframe."""
        self.descriptors.append(descriptor)
        self.poses.append(pose)
    
    def detect_loop(self, current_descriptor, current_pose, min_distance=10):
        """Check for loop closure candidates."""
        if len(self.descriptors) < min_distance:
            return None
        
        # Only search keyframes far enough in the past
        candidates = self.descriptors[:-min_distance]
        
        if len(candidates) == 0:
            return None
        
        # Find nearest neighbor in descriptor space
        nn = NearestNeighbors(n_neighbors=1, metric='cosine')
        nn.fit(candidates)
        
        distances, indices = nn.kneighbors([current_descriptor])
        
        if distances[0][0] < self.threshold:
            loop_idx = indices[0][0]
            return {
                'keyframe_idx': loop_idx,
                'pose': self.poses[loop_idx],
                'confidence': 1.0 - distances[0][0]
            }
        
        return None
```

### Pose Graph Optimization

After loop closure detection, optimize the trajectory:

```python
import g2o
import numpy as np

def optimize_pose_graph(poses, loop_closures):
    """Optimize pose graph with loop closure constraints."""
    
    optimizer = g2o.SparseOptimizer()
    solver = g2o.BlockSolverSE3(g2o.LinearSolverDenseSE3())
    algorithm = g2o.OptimizationAlgorithmLevenberg(solver)
    optimizer.set_algorithm(algorithm)
    
    # Add vertices (poses)
    for i, pose in enumerate(poses):
        v = g2o.VertexSE3()
        v.set_id(i)
        v.set_estimate(g2o.Isometry3d(pose))
        v.set_fixed(i == 0)  # Fix first pose
        optimizer.add_vertex(v)
    
    # Add odometry edges (sequential constraints)
    info = np.eye(6) * 100  # Information matrix
    for i in range(len(poses) - 1):
        edge = g2o.EdgeSE3()
        edge.set_vertex(0, optimizer.vertex(i))
        edge.set_vertex(1, optimizer.vertex(i + 1))
        # Relative transform between poses
        relative = np.linalg.inv(poses[i]) @ poses[i + 1]
        edge.set_measurement(g2o.Isometry3d(relative))
        edge.set_information(info)
        optimizer.add_edge(edge)
    
    # Add loop closure edges
    loop_info = np.eye(6) * 500  # Higher confidence
    for lc in loop_closures:
        edge = g2o.EdgeSE3()
        edge.set_vertex(0, optimizer.vertex(lc['from_idx']))
        edge.set_vertex(1, optimizer.vertex(lc['to_idx']))
        edge.set_measurement(g2o.Isometry3d(lc['relative_pose']))
        edge.set_information(loop_info)
        optimizer.add_edge(edge)
    
    # Optimize
    optimizer.initialize_optimization()
    optimizer.optimize(20)
    
    # Extract optimized poses
    optimized_poses = []
    for i in range(len(poses)):
        optimized_poses.append(
            optimizer.vertex(i).estimate().matrix()
        )
    
    return optimized_poses
```

## 17.6 Mapping for Navigation

### Occupancy Grid Generation

Convert 3D map to 2D occupancy grid for navigation:

```python
import numpy as np
from nav_msgs.msg import OccupancyGrid

class OccupancyGridGenerator:
    def __init__(self, resolution=0.05, height_range=(0.1, 1.5)):
        self.resolution = resolution
        self.height_min, self.height_max = height_range
    
    def generate(self, point_cloud, robot_pose):
        """Generate occupancy grid from point cloud."""
        # Filter points by height (obstacles at robot height)
        points = point_cloud[
            (point_cloud[:, 2] > self.height_min) &
            (point_cloud[:, 2] < self.height_max)
        ]
        
        if len(points) == 0:
            return None
        
        # Calculate grid bounds
        x_min, x_max = points[:, 0].min(), points[:, 0].max()
        y_min, y_max = points[:, 1].min(), points[:, 1].max()
        
        width = int((x_max - x_min) / self.resolution) + 1
        height = int((y_max - y_min) / self.resolution) + 1
        
        # Create grid
        grid = np.full((height, width), -1, dtype=np.int8)  # Unknown
        
        # Mark occupied cells
        for point in points:
            x_idx = int((point[0] - x_min) / self.resolution)
            y_idx = int((point[1] - y_min) / self.resolution)
            grid[y_idx, x_idx] = 100  # Occupied
        
        # Create ROS message
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = x_min
        msg.info.origin.position.y = y_min
        msg.data = grid.flatten().tolist()
        
        return msg
```

### Costmap Integration

```python
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped

class NavigationInterface:
    def __init__(self, node):
        self.node = node
        self.goal_pub = node.create_publisher(
            PoseStamped, 
            '/goal_pose', 
            10
        )
    
    def navigate_to(self, x, y, theta):
        """Send navigation goal."""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = np.sin(theta / 2)
        goal.pose.orientation.w = np.cos(theta / 2)
        
        self.goal_pub.publish(goal)
```

## 17.7 Handling Dynamic Environments

### Dynamic Object Detection

```python
class DynamicObjectFilter:
    def __init__(self, motion_threshold=0.1):
        self.motion_threshold = motion_threshold
        self.prev_points = None
    
    def filter_dynamic(self, current_points, current_pose, prev_pose):
        """Remove dynamic objects from point cloud."""
        if self.prev_points is None:
            self.prev_points = current_points
            return current_points
        
        # Transform previous points to current frame
        transform = np.linalg.inv(current_pose) @ prev_pose
        prev_in_current = self.transform_points(self.prev_points, transform)
        
        # Find static points (points that haven't moved much)
        static_mask = np.ones(len(current_points), dtype=bool)
        
        for i, point in enumerate(current_points):
            # Find nearest point in previous frame
            distances = np.linalg.norm(prev_in_current - point, axis=1)
            min_dist = distances.min()
            
            # If no corresponding point found, might be dynamic
            if min_dist > self.motion_threshold:
                static_mask[i] = False
        
        self.prev_points = current_points
        
        return current_points[static_mask]
```

### Semantic SLAM

Combine semantic segmentation with SLAM:

```python
class SemanticSLAM:
    def __init__(self):
        self.semantic_classes = {
            'person': {'dynamic': True, 'navigate_around': True},
            'chair': {'dynamic': True, 'navigate_around': True},
            'wall': {'dynamic': False, 'navigate_around': True},
            'floor': {'dynamic': False, 'navigate_around': False},
        }
    
    def process_frame(self, rgb_image, depth_image, segmentation):
        """Process frame with semantic information."""
        points = []
        
        for label, info in self.semantic_classes.items():
            mask = segmentation == label
            
            if info['dynamic']:
                # Don't add to permanent map
                continue
            
            # Extract points for this class
            class_points = self.depth_to_points(depth_image, mask)
            
            if info['navigate_around']:
                points.extend(class_points)
        
        return np.array(points)
```

## 17.8 Relocalization

### Lost State Recovery

```python
class Relocalization:
    def __init__(self, map_features, map_descriptors):
        self.map_features = map_features  # 3D points
        self.map_descriptors = map_descriptors
        self.is_localized = True
    
    def check_tracking(self, num_tracked_features, threshold=30):
        """Check if tracking is healthy."""
        if num_tracked_features < threshold:
            self.is_localized = False
            return False
        return True
    
    def relocalize(self, current_descriptors, camera_matrix):
        """Attempt to relocalize against the map."""
        # Match current features to map
        matches = self.match_features(current_descriptors, self.map_descriptors)
        
        if len(matches) < 10:
            return None  # Not enough matches
        
        # Get 2D-3D correspondences
        points_2d = np.array([m['point_2d'] for m in matches])
        points_3d = np.array([self.map_features[m['map_idx']] for m in matches])
        
        # Solve PnP
        success, rvec, tvec, inliers = cv2.solvePnPRansac(
            points_3d, points_2d, camera_matrix, None
        )
        
        if success and len(inliers) > 20:
            # Convert to pose matrix
            R, _ = cv2.Rodrigues(rvec)
            pose = np.eye(4)
            pose[:3, :3] = R
            pose[:3, 3] = tvec.flatten()
            
            self.is_localized = True
            return pose
        
        return None
```

## 17.9 Summary

In this chapter, you learned:

- **SLAM fundamentals**: Localization and mapping coupling
- **Visual SLAM methods**: Feature-based, direct, and learned
- **Visual-inertial odometry**: IMU fusion for robustness
- **Loop closure**: Place recognition and pose graph optimization
- **Navigation maps**: Occupancy grid generation
- **Dynamic environments**: Filtering moving objects
- **Relocalization**: Recovering from tracking loss

Visual SLAM enables humanoid robots to navigate autonomously in unknown environments. In the next chapter, we'll explore Nav2 for path planning.

## Review Questions

1. What are the two problems SLAM solves simultaneously?
2. Why is IMU fusion important for Visual SLAM?
3. How does loop closure improve map accuracy?
4. How do you handle dynamic objects in SLAM?
5. What is relocalization and when is it needed?

## Hands-On Exercise

1. Configure Isaac ROS Visual SLAM with stereo camera
2. Calibrate camera-IMU extrinsics using Kalibr
3. Build a map of a room while walking around
4. Test relocalization by restarting in the mapped area
5. Generate an occupancy grid from the 3D map
6. Use the map for autonomous navigation
