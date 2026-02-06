---
sidebar_position: 3
title: "Chapter 23: Manipulation & Grasping"
description: "Object manipulation with humanoid hands and arms"
---

# Chapter 23: Manipulation & Grasping

Humanoid robots need to interact with the world through manipulation—picking up objects, opening doors, using tools. This chapter covers grasp planning, manipulation control, and integration with perception for dexterous humanoid manipulation.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand grasp mechanics and grasp quality metrics
- Implement grasp planning algorithms
- Control humanoid arms for manipulation tasks
- Integrate vision for object detection and pose estimation
- Handle contact-rich manipulation scenarios

## 23.1 Humanoid Manipulation Systems

### Arm and Hand Configuration

```
Humanoid Arm (7 DOF):
┌─────────────────────────────────────────────────┐
│  Shoulder (3 DOF) ─► Elbow (1 DOF) ─► Wrist (3 DOF)
│                                           │
│  Pitch/Roll/Yaw       Pitch          Roll/Pitch/Yaw
└─────────────────────────────────────────────────┘

Humanoid Hand Options:
┌────────────────────────────────────────────────────┐
│  Type              DOF     Actuation   Use Case    │
├────────────────────────────────────────────────────┤
│  Simple Gripper    1-2     Direct      Basic grasp │
│  Underactuated     6-10    Tendon      Adaptive    │
│  Fully Actuated    16-20   Direct      Dexterous   │
│  Soft Gripper      N/A     Pneumatic   Delicate    │
└────────────────────────────────────────────────────┘
```

### Workspace Analysis

```python
import numpy as np

class WorkspaceAnalyzer:
    def __init__(self, kinematics):
        self.kin = kinematics
    
    def compute_reachable_workspace(self, resolution=0.05):
        """Compute points reachable by end-effector."""
        reachable_points = []
        
        # Sample joint space
        joint_samples = self.sample_joint_space(resolution)
        
        for q in joint_samples:
            T = self.kin.forward_kinematics(q)
            pos = T[:3, 3]
            reachable_points.append(pos)
        
        return np.array(reachable_points)
    
    def check_reachability(self, target_pos, target_orient=None):
        """Check if target is reachable."""
        # Try IK from multiple initial guesses
        for _ in range(10):
            q0 = np.random.uniform(self.kin.joint_lower, self.kin.joint_upper)
            q, success = self.kin.inverse_kinematics(target_pos, target_orient, q0)
            
            if success:
                return True, q
        
        return False, None
```

## 23.2 Grasp Theory

### Grasp Contacts

```python
class GraspContact:
    """Represents a contact point in a grasp."""
    
    POINT_CONTACT = 'point'  # 1 constraint (normal force)
    POINT_WITH_FRICTION = 'point_friction'  # 3 constraints (force cone)
    SOFT_FINGER = 'soft_finger'  # 4 constraints (+ torsional friction)
    
    def __init__(self, position, normal, contact_type='point_friction', friction_coef=0.5):
        self.position = np.array(position)
        self.normal = np.array(normal) / np.linalg.norm(normal)
        self.contact_type = contact_type
        self.mu = friction_coef
    
    def get_wrench_basis(self):
        """Get basis of wrenches this contact can apply."""
        # Contact frame
        n = self.normal
        t1 = np.array([n[1], -n[0], 0]) if abs(n[2]) < 0.9 else np.array([0, n[2], -n[1]])
        t1 = t1 / np.linalg.norm(t1)
        t2 = np.cross(n, t1)
        
        if self.contact_type == self.POINT_CONTACT:
            # Only normal force
            wrench = np.zeros(6)
            wrench[:3] = n
            wrench[3:] = np.cross(self.position, n)
            return [wrench]
        
        elif self.contact_type == self.POINT_WITH_FRICTION:
            # Friction cone approximation (4 edges)
            wrenches = []
            for angle in [0, np.pi/2, np.pi, 3*np.pi/2]:
                f_dir = n + self.mu * (np.cos(angle) * t1 + np.sin(angle) * t2)
                f_dir = f_dir / np.linalg.norm(f_dir)
                
                wrench = np.zeros(6)
                wrench[:3] = f_dir
                wrench[3:] = np.cross(self.position, f_dir)
                wrenches.append(wrench)
            
            return wrenches
        
        return []
```

### Grasp Quality Metrics

```python
class GraspQuality:
    @staticmethod
    def force_closure(contacts):
        """Check if grasp has force closure."""
        # Build grasp matrix
        G = []
        for contact in contacts:
            G.extend(contact.get_wrench_basis())
        G = np.array(G).T  # 6 x m
        
        # Check if origin is inside convex hull
        # Equivalently, check if we can resist any wrench
        from scipy.spatial import ConvexHull
        try:
            hull = ConvexHull(G.T)
            return True
        except:
            return False
    
    @staticmethod
    def ferrari_canny_metric(contacts, num_samples=1000):
        """
        Ferrari-Canny grasp quality: largest ball inscribed in wrench space.
        Higher is better.
        """
        # Build grasp wrench space
        G = []
        for contact in contacts:
            G.extend(contact.get_wrench_basis())
        G = np.array(G).T
        
        # Compute minimum singular value
        # (approximation of inscribed ball radius)
        _, s, _ = np.linalg.svd(G)
        
        return s[-1] if len(s) == 6 else 0
    
    @staticmethod
    def grasp_isotropy(contacts):
        """Measure how uniformly forces can be applied in all directions."""
        G = []
        for contact in contacts:
            G.extend(contact.get_wrench_basis())
        G = np.array(G).T
        
        _, s, _ = np.linalg.svd(G)
        
        if len(s) < 6 or s[-1] < 1e-6:
            return 0
        
        return s[-1] / s[0]  # Ratio of min to max singular value
```

## 23.3 Grasp Planning

### Antipodal Grasps

```python
class AntipodalGraspSampler:
    """Sample antipodal grasps on object surface."""
    
    def __init__(self, friction_coef=0.5):
        self.mu = friction_coef
        self.friction_cone_angle = np.arctan(friction_coef)
    
    def sample_grasps(self, point_cloud, normals, num_samples=100):
        """Sample antipodal grasp candidates."""
        grasps = []
        
        for _ in range(num_samples * 10):  # Oversample
            # Random pair of points
            idx1, idx2 = np.random.choice(len(point_cloud), 2, replace=False)
            p1, p2 = point_cloud[idx1], point_cloud[idx2]
            n1, n2 = normals[idx1], normals[idx2]
            
            # Check antipodal condition
            line = p2 - p1
            line_norm = line / np.linalg.norm(line)
            
            # Normals should point towards each other (within friction cone)
            angle1 = np.arccos(np.clip(np.dot(n1, line_norm), -1, 1))
            angle2 = np.arccos(np.clip(np.dot(n2, -line_norm), -1, 1))
            
            if angle1 < self.friction_cone_angle and angle2 < self.friction_cone_angle:
                grasp = self.create_grasp(p1, p2, n1, n2)
                grasps.append(grasp)
                
                if len(grasps) >= num_samples:
                    break
        
        return grasps
    
    def create_grasp(self, p1, p2, n1, n2):
        """Create grasp from two contact points."""
        center = (p1 + p2) / 2
        approach = (n1 + n2) / 2
        approach = approach / np.linalg.norm(approach)
        
        # Grasp width
        width = np.linalg.norm(p2 - p1)
        
        return {
            'center': center,
            'approach': approach,
            'width': width,
            'contacts': [
                GraspContact(p1, n1),
                GraspContact(p2, n2)
            ]
        }
```

### Grasp Network (Deep Learning)

```python
import torch
import torch.nn as nn

class GraspNet(nn.Module):
    """Neural network for 6-DOF grasp prediction."""
    
    def __init__(self, num_points=1024):
        super().__init__()
        
        # PointNet-like encoder
        self.conv1 = nn.Conv1d(3, 64, 1)
        self.conv2 = nn.Conv1d(64, 128, 1)
        self.conv3 = nn.Conv1d(128, 256, 1)
        
        # Global feature
        self.fc1 = nn.Linear(256, 256)
        self.fc2 = nn.Linear(256, 128)
        
        # Grasp prediction heads
        self.position_head = nn.Linear(128, 3)
        self.orientation_head = nn.Linear(128, 4)  # Quaternion
        self.width_head = nn.Linear(128, 1)
        self.quality_head = nn.Linear(128, 1)
    
    def forward(self, point_cloud):
        # point_cloud: (B, N, 3)
        x = point_cloud.transpose(1, 2)  # (B, 3, N)
        
        x = torch.relu(self.conv1(x))
        x = torch.relu(self.conv2(x))
        x = torch.relu(self.conv3(x))
        
        # Global max pooling
        x = torch.max(x, dim=2)[0]  # (B, 256)
        
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        
        # Predictions
        position = self.position_head(x)
        orientation = self.orientation_head(x)
        orientation = orientation / torch.norm(orientation, dim=-1, keepdim=True)
        width = torch.sigmoid(self.width_head(x)) * 0.1  # Max 10cm
        quality = torch.sigmoid(self.quality_head(x))
        
        return {
            'position': position,
            'orientation': orientation,
            'width': width,
            'quality': quality
        }
```

## 23.4 Motion Planning for Manipulation

### Arm Motion Planning

```python
class ArmMotionPlanner:
    def __init__(self, robot_kinematics, collision_checker):
        self.kin = robot_kinematics
        self.collision = collision_checker
    
    def plan_to_grasp(self, current_q, grasp_pose, pre_grasp_distance=0.1):
        """Plan arm motion to grasp pose."""
        # Compute pre-grasp pose (offset along approach)
        approach = grasp_pose[:3, :3] @ np.array([0, 0, 1])
        pre_grasp_pose = grasp_pose.copy()
        pre_grasp_pose[:3, 3] -= approach * pre_grasp_distance
        
        # IK for pre-grasp
        q_pre_grasp, success = self.kin.inverse_kinematics(
            pre_grasp_pose[:3, 3],
            pre_grasp_pose[:3, :3],
            current_q
        )
        
        if not success:
            return None
        
        # IK for grasp
        q_grasp, success = self.kin.inverse_kinematics(
            grasp_pose[:3, 3],
            grasp_pose[:3, :3],
            q_pre_grasp
        )
        
        if not success:
            return None
        
        # Plan paths
        path_to_pregrasp = self.rrt_connect(current_q, q_pre_grasp)
        path_to_grasp = self.linear_interpolation(q_pre_grasp, q_grasp)
        
        if path_to_pregrasp is None:
            return None
        
        return {
            'approach': path_to_pregrasp,
            'grasp': path_to_grasp,
            'q_pre_grasp': q_pre_grasp,
            'q_grasp': q_grasp
        }
    
    def rrt_connect(self, q_start, q_goal, max_iterations=1000):
        """RRT-Connect motion planner."""
        tree_start = [q_start]
        tree_goal = [q_goal]
        parent_start = {0: -1}
        parent_goal = {0: -1}
        
        step_size = 0.1
        
        for iteration in range(max_iterations):
            # Sample random configuration
            if np.random.random() < 0.1:
                q_rand = q_goal if iteration % 2 == 0 else q_start
            else:
                q_rand = self.random_config()
            
            # Extend trees alternately
            if iteration % 2 == 0:
                tree, parent = tree_start, parent_start
                other_tree = tree_goal
            else:
                tree, parent = tree_goal, parent_goal
                other_tree = tree_start
            
            # Find nearest in tree
            nearest_idx = self.nearest_neighbor(tree, q_rand)
            q_near = tree[nearest_idx]
            
            # Extend towards random
            q_new = self.steer(q_near, q_rand, step_size)
            
            if not self.collision.check_collision(q_new):
                new_idx = len(tree)
                tree.append(q_new)
                parent[new_idx] = nearest_idx
                
                # Check connection to other tree
                other_nearest_idx = self.nearest_neighbor(other_tree, q_new)
                other_nearest = other_tree[other_nearest_idx]
                
                if np.linalg.norm(q_new - other_nearest) < step_size:
                    # Found connection
                    return self.extract_path(tree_start, parent_start,
                                           tree_goal, parent_goal,
                                           new_idx if iteration % 2 == 0 else other_nearest_idx)
        
        return None
    
    def random_config(self):
        return np.random.uniform(self.kin.joint_lower, self.kin.joint_upper)
    
    def nearest_neighbor(self, tree, q):
        distances = [np.linalg.norm(np.array(node) - q) for node in tree]
        return np.argmin(distances)
    
    def steer(self, q_from, q_to, step_size):
        direction = q_to - q_from
        distance = np.linalg.norm(direction)
        if distance < step_size:
            return q_to
        return q_from + direction / distance * step_size
```

## 23.5 Grasp Execution

### Grasp Controller

```python
class GraspController:
    def __init__(self, robot, hand):
        self.robot = robot
        self.hand = hand
    
    def execute_grasp(self, grasp_plan):
        """Execute a grasp plan."""
        # Move to pre-grasp
        success = self.robot.execute_trajectory(grasp_plan['approach'])
        if not success:
            return False
        
        # Open hand
        self.hand.open()
        
        # Move to grasp pose
        success = self.robot.execute_trajectory(grasp_plan['grasp'])
        if not success:
            return False
        
        # Close hand
        grasp_success = self.hand.close_until_contact()
        
        if not grasp_success:
            return False
        
        # Verify grasp (check finger positions, force)
        return self.verify_grasp()
    
    def verify_grasp(self):
        """Verify object is grasped."""
        finger_positions = self.hand.get_finger_positions()
        expected_closed = self.hand.get_fully_closed_positions()
        
        # If fingers aren't fully closed, something is grasped
        grasp_width = np.mean(finger_positions - expected_closed)
        
        return grasp_width > 0.01  # At least 1cm object

class AdaptiveGraspController(GraspController):
    """Grasp controller with force feedback."""
    
    def __init__(self, robot, hand, target_force=5.0):
        super().__init__(robot, hand)
        self.target_force = target_force
    
    def close_with_force_control(self):
        """Close hand with force control."""
        kp = 0.01
        
        while True:
            forces = self.hand.get_finger_forces()
            positions = self.hand.get_finger_positions()
            
            # Check if target force reached on all fingers
            if all(f >= self.target_force for f in forces):
                break
            
            # Move fingers that haven't reached force target
            velocity_cmd = []
            for i, (f, p) in enumerate(zip(forces, positions)):
                if f < self.target_force:
                    velocity_cmd.append(kp * (self.target_force - f))
                else:
                    velocity_cmd.append(0)
            
            self.hand.set_finger_velocities(velocity_cmd)
            
            # Safety check
            if all(p > 0.95 for p in positions):  # Fingers almost closed
                break
        
        return True
```

## 23.6 Object Pose Estimation

### Point Cloud-Based Pose Estimation

```python
class PoseEstimator:
    def __init__(self, object_model):
        self.model = object_model  # Point cloud of object
    
    def estimate_pose(self, scene_points, initial_guess=None):
        """Estimate object pose using ICP."""
        from scipy.spatial import KDTree
        
        if initial_guess is None:
            initial_guess = np.eye(4)
        
        transform = initial_guess.copy()
        model_points = self.model.copy()
        
        for iteration in range(50):
            # Transform model
            transformed_model = (transform[:3, :3] @ model_points.T).T + transform[:3, 3]
            
            # Find correspondences
            tree = KDTree(scene_points)
            distances, indices = tree.query(transformed_model)
            
            # Remove outliers
            inlier_mask = distances < np.percentile(distances, 90)
            src = transformed_model[inlier_mask]
            dst = scene_points[indices[inlier_mask]]
            
            # Compute optimal transformation (SVD)
            src_centered = src - src.mean(axis=0)
            dst_centered = dst - dst.mean(axis=0)
            
            H = src_centered.T @ dst_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            t = dst.mean(axis=0) - R @ src.mean(axis=0)
            
            # Update transform
            delta_T = np.eye(4)
            delta_T[:3, :3] = R
            delta_T[:3, 3] = t
            
            transform = delta_T @ transform
            
            # Convergence check
            if np.linalg.norm(t) < 1e-5 and np.allclose(R, np.eye(3), atol=1e-5):
                break
        
        return transform

class DeepPoseEstimator:
    """Deep learning-based 6-DOF pose estimation."""
    
    def __init__(self, model_path):
        self.model = self.load_model(model_path)
    
    def estimate_pose(self, rgb_image, depth_image, object_mask):
        """Estimate pose from RGB-D image."""
        # Extract object region
        masked_rgb = rgb_image * object_mask[:, :, np.newaxis]
        masked_depth = depth_image * object_mask
        
        # Convert to point cloud
        points = self.depth_to_points(masked_depth)
        
        # Run network
        with torch.no_grad():
            input_tensor = self.preprocess(masked_rgb, points)
            output = self.model(input_tensor)
        
        # Decode pose
        translation = output['translation'].numpy()
        rotation = output['rotation'].numpy()  # Quaternion
        
        pose = np.eye(4)
        pose[:3, :3] = self.quat_to_matrix(rotation)
        pose[:3, 3] = translation
        
        return pose
```

## 23.7 Manipulation Primitives

### Pick and Place

```python
class PickAndPlace:
    def __init__(self, arm_planner, grasp_controller, pose_estimator):
        self.planner = arm_planner
        self.grasp = grasp_controller
        self.pose = pose_estimator
    
    def pick(self, object_point_cloud):
        """Pick up an object."""
        # Estimate object pose
        object_pose = self.pose.estimate_pose(object_point_cloud)
        
        # Generate grasp candidates
        sampler = AntipodalGraspSampler()
        grasps = sampler.sample_grasps(object_point_cloud, self.compute_normals(object_point_cloud))
        
        # Rank by quality
        qualities = [GraspQuality.ferrari_canny_metric(g['contacts']) for g in grasps]
        best_grasp = grasps[np.argmax(qualities)]
        
        # Plan motion
        grasp_pose = self.grasp_to_pose(best_grasp, object_pose)
        current_q = self.arm_planner.robot.get_joint_positions()
        plan = self.planner.plan_to_grasp(current_q, grasp_pose)
        
        if plan is None:
            return False
        
        # Execute
        return self.grasp.execute_grasp(plan)
    
    def place(self, place_pose):
        """Place held object at pose."""
        # Plan motion to place pose
        current_q = self.planner.robot.get_joint_positions()
        plan = self.planner.plan_to_grasp(current_q, place_pose)
        
        if plan is None:
            return False
        
        # Execute approach
        self.planner.robot.execute_trajectory(plan['approach'])
        self.planner.robot.execute_trajectory(plan['grasp'])
        
        # Release
        self.grasp.hand.open()
        
        # Retreat
        retreat_plan = self.planner.plan_retreat(place_pose)
        self.planner.robot.execute_trajectory(retreat_plan)
        
        return True
```

## 23.8 Summary

In this chapter, you learned:

- **Grasp theory**: Contacts, force closure, quality metrics
- **Grasp planning**: Antipodal sampling, neural networks
- **Motion planning**: RRT-Connect for arm trajectories
- **Grasp execution**: Force-controlled grasping
- **Pose estimation**: ICP and deep learning methods
- **Manipulation primitives**: Pick and place

Manipulation enables humanoids to interact with their environment. In the next chapter, we'll add voice commands for natural interaction.

## Review Questions

1. What is force closure and why is it important?
2. How does the Ferrari-Canny metric measure grasp quality?
3. What is an antipodal grasp?
4. How does RRT-Connect find collision-free paths?
5. Why is force control important during grasping?

## Hands-On Exercise

1. Implement antipodal grasp sampling
2. Compute grasp quality metrics
3. Plan arm motion to a grasp pose
4. Implement a force-controlled grasp
5. Create a complete pick-and-place pipeline
