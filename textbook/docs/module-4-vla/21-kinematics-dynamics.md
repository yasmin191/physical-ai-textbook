---
sidebar_position: 1
title: "Chapter 21: Humanoid Kinematics & Dynamics"
description: "Mathematical foundations for humanoid robot motion and control"
---

# Chapter 21: Humanoid Kinematics & Dynamics

Understanding the mathematics of humanoid motion is essential for developing control algorithms. This chapter covers the kinematic and dynamic foundations that underpin walking, manipulation, and whole-body control of humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:

- Apply forward and inverse kinematics to humanoid robots
- Understand the Jacobian and its role in robot control
- Model humanoid dynamics using the equations of motion
- Compute center of mass and zero moment point
- Implement basic kinematic algorithms

## 21.1 Kinematic Chains

### Humanoid Structure

A humanoid robot consists of multiple kinematic chains:

```
                        Head
                         │
                    ┌────┴────┐
                    │  Neck   │
                    └────┬────┘
            ┌────────────┴────────────┐
            │                         │
     ┌──────┴──────┐           ┌──────┴──────┐
     │  Left Arm   │           │  Right Arm  │
     │   Chain     │           │    Chain    │
     │  (7 DOF)    │           │   (7 DOF)   │
     └─────────────┘           └─────────────┘
            │                         │
     ┌──────┴──────┐           ┌──────┴──────┐
     │   Torso     │───────────│   Torso     │
     │   (3 DOF)   │           │   (3 DOF)   │
     └──────┬──────┘           └──────┬──────┘
            │                         │
     ┌──────┴──────┐           ┌──────┴──────┐
     │  Left Leg   │           │  Right Leg  │
     │   Chain     │           │    Chain    │
     │  (6 DOF)    │           │   (6 DOF)   │
     └─────────────┘           └─────────────┘
```

### Degrees of Freedom

| Body Part | Typical DOF | Joints |
|-----------|-------------|--------|
| Leg | 6 | Hip (3), Knee (1), Ankle (2) |
| Arm | 7 | Shoulder (3), Elbow (1), Wrist (3) |
| Torso | 2-3 | Waist yaw, pitch, (roll) |
| Head | 2 | Neck pan, tilt |
| **Total** | **28-34** | |

## 21.2 Rigid Body Transformations

### Homogeneous Transformation Matrix

Position and orientation are represented using 4×4 matrices:


Where:
-  is a 3×3 rotation matrix
-  is a 3×1 position vector

### Rotation Representations

```python
import numpy as np
from scipy.spatial.transform import Rotation

# Rotation matrix from Euler angles (ZYX convention)
def euler_to_matrix(roll, pitch, yaw):
    r = Rotation.from_euler('zyx', [yaw, pitch, roll])
    return r.as_matrix()

# Rotation matrix from axis-angle
def axis_angle_to_matrix(axis, angle):
    axis = np.array(axis) / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * K @ K
    return R

# Quaternion to rotation matrix
def quat_to_matrix(quat):  # [x, y, z, w]
    r = Rotation.from_quat(quat)
    return r.as_matrix()
```

### Transformation Composition

```python
def compose_transforms(T1, T2):
    """Compose two homogeneous transformations."""
    return T1 @ T2

def inverse_transform(T):
    """Compute inverse of homogeneous transformation."""
    R = T[:3, :3]
    p = T[:3, 3]
    
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p
    
    return T_inv
```

## 21.3 Forward Kinematics

### DH Parameters

The Denavit-Hartenberg convention describes joint transformations:

| Parameter | Description |
|-----------|-------------|
|  | Joint angle (rotation about z) |
|  | Link offset (translation along z) |
|  | Link length (translation along x) |
|  | Link twist (rotation about x) |

### DH Transformation


### Implementation

```python
def dh_transform(theta, d, a, alpha):
    """Compute transformation matrix from DH parameters."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

class HumanoidKinematics:
    def __init__(self):
        # DH parameters for right leg: [theta_offset, d, a, alpha]
        self.right_leg_dh = [
            [0, 0, 0, -np.pi/2],      # Hip yaw
            [0, 0, 0, np.pi/2],       # Hip roll
            [0, 0, 0.085, 0],         # Hip pitch
            [0, 0, 0.35, 0],          # Knee
            [0, 0, 0.35, 0],          # Ankle pitch
            [0, 0, 0, np.pi/2],       # Ankle roll
        ]
    
    def forward_kinematics_leg(self, joint_angles, side='right'):
        """Compute foot position from joint angles."""
        T = np.eye(4)
        
        dh_params = self.right_leg_dh if side == 'right' else self.left_leg_dh
        
        for i, (theta_offset, d, a, alpha) in enumerate(dh_params):
            theta = joint_angles[i] + theta_offset
            T = T @ dh_transform(theta, d, a, alpha)
        
        return T
    
    def forward_kinematics_all(self, joint_positions):
        """Compute all end-effector positions."""
        positions = {}
        
        # Extract joint groups
        right_leg_joints = joint_positions[0:6]
        left_leg_joints = joint_positions[6:12]
        right_arm_joints = joint_positions[12:19]
        left_arm_joints = joint_positions[19:26]
        
        # Compute transforms
        positions['right_foot'] = self.forward_kinematics_leg(right_leg_joints, 'right')
        positions['left_foot'] = self.forward_kinematics_leg(left_leg_joints, 'left')
        positions['right_hand'] = self.forward_kinematics_arm(right_arm_joints, 'right')
        positions['left_hand'] = self.forward_kinematics_arm(left_arm_joints, 'left')
        
        return positions
```

## 21.4 Inverse Kinematics

### Problem Statement

Given desired end-effector pose T_desired, find joint angles q such that:

