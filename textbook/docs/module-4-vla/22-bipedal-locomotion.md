---
sidebar_position: 2
title: "Chapter 22: Bipedal Locomotion"
description: "Walking, running, and balance control for humanoid robots"
---

# Chapter 22: Bipedal Locomotion

Bipedal walking is one of the defining capabilities of humanoid robots. This chapter covers the principles, algorithms, and control strategies for achieving stable and efficient bipedal locomotion.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the gait cycle and walking phases
- Implement basic walking pattern generators
- Apply balance control using CoM and ZMP
- Handle push recovery and disturbance rejection
- Integrate locomotion with perception

## 22.1 The Gait Cycle

### Walking Phases

```
┌─────────────────────────────────────────────────────────────┐
│                     Gait Cycle (100%)                        │
├──────────────────────────────┬──────────────────────────────┤
│       Stance Phase (60%)     │       Swing Phase (40%)      │
├──────────────────────────────┴──────────────────────────────┤
│                                                             │
│  Right Leg:                                                 │
│  ═══════════════════════════╗                               │
│  Heel    Mid-    Toe      ║  Swing through                 │
│  Strike  Stance  Off      ║  (leg in air)                  │
│                            ╚════════════════════════════    │
│                                                             │
│  Left Leg:                                                  │
│                 ╔════════════════════════════               │
│  Swing         ║  Heel    Mid-    Toe                      │
│  (leg in air)  ║  Strike  Stance  Off                      │
│  ══════════════╝                                            │
│                                                             │
│  Double     Single      Double    Single                    │
│  Support    Support     Support   Support                   │
│   (10%)      (40%)       (10%)     (40%)                    │
└─────────────────────────────────────────────────────────────┘
```

### Gait Parameters

| Parameter | Description | Typical Value |
|-----------|-------------|---------------|
| Step length | Distance per step | 0.3-0.5 m |
| Step width | Lateral foot spacing | 0.1-0.2 m |
| Step height | Maximum foot lift | 0.05-0.1 m |
| Cycle time | Duration of full gait cycle | 0.8-1.2 s |
| Double support ratio | Time both feet on ground | 10-20% |

## 22.2 Walking Pattern Generation

### Footstep Planning

```python
import numpy as np

class FootstepPlanner:
    def __init__(self, step_length=0.3, step_width=0.15, step_height=0.05):
        self.step_length = step_length
        self.step_width = step_width
        self.step_height = step_height
    
    def plan_footsteps(self, start_pose, goal_pose, num_steps=None):
        """Plan a sequence of footsteps from start to goal."""
        # Direction to goal
        direction = goal_pose[:2] - start_pose[:2]
        distance = np.linalg.norm(direction)
        
        if num_steps is None:
            num_steps = int(np.ceil(distance / self.step_length))
        
        direction_normalized = direction / distance if distance > 0 else np.array([1, 0])
        heading = np.arctan2(direction_normalized[1], direction_normalized[0])
        
        footsteps = []
        current_pos = start_pose[:2].copy()
        
        for i in range(num_steps):
            # Alternate left and right
            is_right = (i % 2 == 0)
            lateral_offset = self.step_width / 2 * (-1 if is_right else 1)
            
            # Compute footstep position
            step_pos = current_pos + direction_normalized * self.step_length
            
            # Apply lateral offset (perpendicular to direction)
            perp = np.array([-direction_normalized[1], direction_normalized[0]])
            foot_pos = step_pos + perp * lateral_offset
            
            footsteps.append({
                'position': np.array([foot_pos[0], foot_pos[1], 0]),
                'orientation': heading,
                'is_right': is_right,
                'timing': i * 0.5  # Half second per step
            })
            
            current_pos = step_pos
        
        return footsteps
```

### Swing Foot Trajectory

```python
class SwingFootTrajectory:
    def __init__(self, step_height=0.05, swing_duration=0.4):
        self.step_height = step_height
        self.swing_duration = swing_duration
    
    def generate(self, start_pos, end_pos, t):
        """
        Generate swing foot position at time t.
        t: normalized time [0, 1]
        """
        # Horizontal: smooth interpolation
        pos_xy = start_pos[:2] + (end_pos[:2] - start_pos[:2]) * self.smooth_step(t)
        
        # Vertical: parabolic trajectory
        pos_z = start_pos[2] + self.step_height * 4 * t * (1 - t)
        
        # Add end height
        if t > 0.5:
            pos_z += (end_pos[2] - start_pos[2]) * (t - 0.5) * 2
        
        return np.array([pos_xy[0], pos_xy[1], pos_z])
    
    def smooth_step(self, t):
        """Smooth interpolation (ease in/out)."""
        return t * t * (3 - 2 * t)
    
    def generate_trajectory(self, start_pos, end_pos, dt=0.01):
        """Generate full swing trajectory."""
        num_points = int(self.swing_duration / dt)
        trajectory = []
        
        for i in range(num_points + 1):
            t = i / num_points
            pos = self.generate(start_pos, end_pos, t)
            trajectory.append(pos)
        
        return np.array(trajectory)
```

## 22.3 ZMP-Based Walking

### Linear Inverted Pendulum Model (LIPM)

The LIPM simplifies the robot to a point mass at constant height:


```python
class LinearInvertedPendulum:
    def __init__(self, com_height=0.8, gravity=9.81):
        self.z_c = com_height
        self.g = gravity
        self.omega = np.sqrt(gravity / com_height)
    
    def dynamics(self, x, zmp):
        """
        LIPM dynamics.
        x: [com_pos, com_vel]
        Returns: [com_vel, com_acc]
        """
        com_pos, com_vel = x
        com_acc = self.omega**2 * (com_pos - zmp)
        return np.array([com_vel, com_acc])
    
    def analytical_solution(self, x0, zmp, t):
        """Analytical solution for constant ZMP."""
        com0, vel0 = x0
        
        com = (com0 - zmp) * np.cosh(self.omega * t) + ; 
              vel0 / self.omega * np.sinh(self.omega * t) + zmp
        
        vel = (com0 - zmp) * self.omega * np.sinh(self.omega * t) + ; 
              vel0 * np.cosh(self.omega * t)
        
        return np.array([com, vel])
```

### Preview Control

```python
class PreviewController:
    def __init__(self, com_height=0.8, preview_time=1.6, dt=0.01):
        self.lipm = LinearInvertedPendulum(com_height)
        self.preview_time = preview_time
        self.dt = dt
        self.preview_steps = int(preview_time / dt)
        
        # Compute preview gains using LQR
        self._compute_gains()
    
    def _compute_gains(self):
        """Compute preview control gains."""
        omega = self.lipm.omega
        dt = self.dt
        
        # Discrete-time LIPM with ZMP as state
        A = np.array([
            [np.cosh(omega * dt), np.sinh(omega * dt) / omega, 1 - np.cosh(omega * dt)],
            [omega * np.sinh(omega * dt), np.cosh(omega * dt), -omega * np.sinh(omega * dt)],
            [0, 0, 1]
        ])
        
        B = np.array([
            [1 - np.cosh(omega * dt)],
            [-omega * np.sinh(omega * dt)],
            [1]
        ])
        
        C = np.array([1, 0, -1])
        
        # LQR weights
        Q = np.eye(3)
        R = np.array([[1e-6]])
        
        # Solve discrete Riccati equation
        from scipy.linalg import solve_discrete_are
        P = solve_discrete_are(A, B, C.reshape(-1, 1) @ C.reshape(1, -1) + Q, R)
        
        # Feedback gain
        self.K = np.linalg.inv(R + B.T @ P @ B) @ B.T @ P @ A
        
        # Preview gains
        self.Gp = []
        temp = np.eye(3)
        for i in range(self.preview_steps):
            gp = np.linalg.inv(R + B.T @ P @ B) @ B.T @ temp.T @ P @ np.array([[1 - np.cosh(omega * dt)], [-omega * np.sinh(omega * dt)], [1]])
            self.Gp.append(gp[0, 0])
            temp = (A - B @ self.K) @ temp
    
    def compute_zmp_reference(self, footsteps, t):
        """Compute ZMP reference trajectory from footsteps."""
        zmp_ref = []
        
        for i in range(len(footsteps) - 1):
            current_foot = footsteps[i]
            next_foot = footsteps[i + 1]
            
            # ZMP moves from current foot to next foot
            step_duration = next_foot['timing'] - current_foot['timing']
            num_samples = int(step_duration / self.dt)
            
            for j in range(num_samples):
                # ZMP at support foot center
                if j < num_samples * 0.1:  # Double support start
                    alpha = j / (num_samples * 0.1) * 0.5
                elif j > num_samples * 0.9:  # Double support end
                    alpha = 0.5 + (j - num_samples * 0.9) / (num_samples * 0.1) * 0.5
                else:  # Single support
                    alpha = 0.5
                
                zmp = (1 - alpha) * current_foot['position'][:2] + alpha * next_foot['position'][:2]
                zmp_ref.append(zmp)
        
        return np.array(zmp_ref)
    
    def generate_com_trajectory(self, zmp_ref, initial_state):
        """Generate CoM trajectory using preview control."""
        com_traj = []
        state = np.array([initial_state[0], initial_state[1], zmp_ref[0, 0]])  # [com, vel, zmp]
        
        for i in range(len(zmp_ref)):
            # Feedback
            u = -self.K @ state
            
            # Preview feedforward
            for j in range(min(self.preview_steps, len(zmp_ref) - i)):
                u += self.Gp[j] * zmp_ref[i + j, 0]
            
            # Update state
            state[2] = u  # ZMP command
            state = self.A @ state + self.B @ np.array([u])
            
            com_traj.append(state[0])
        
        return np.array(com_traj)
```

## 22.4 Capture Point Control

### Definition

The Capture Point (CP) is where the robot should step to stop:


```python
class CapturePointController:
    def __init__(self, com_height=0.8):
        self.omega = np.sqrt(9.81 / com_height)
    
    def compute_capture_point(self, com_pos, com_vel):
        """Compute the capture point."""
        return com_pos + com_vel / self.omega
    
    def compute_desired_cop(self, com_pos, com_vel, cp_desired, kp=3.0):
        """Compute desired center of pressure (CoP) for CP tracking."""
        cp_current = self.compute_capture_point(com_pos, com_vel)
        cp_error = cp_current - cp_desired
        
        # CoP = CP + k * (CP - CP_desired)
        cop_desired = cp_current + kp * cp_error / self.omega
        
        return cop_desired
    
    def step_adjustment(self, com_pos, com_vel, nominal_step_pos):
        """Adjust step position to capture the robot."""
        cp = self.compute_capture_point(com_pos, com_vel)
        
        # Adjust step to be at capture point with margin
        margin = 0.02  # Safety margin
        adjusted_step = cp + margin * (cp - com_pos) / np.linalg.norm(cp - com_pos)
        
        return adjusted_step
```

## 22.5 Whole-Body Control

### Task-Space Control

```python
class WholeBodyController:
    def __init__(self, robot_dynamics):
        self.dynamics = robot_dynamics
    
    def compute_torques(self, q, dq, tasks, contact_jacobians):
        """
        Compute joint torques for multiple tasks.
        
        tasks: list of (J, x_ddot_desired, priority)
        contact_jacobians: Jacobians of contact points
        """
        n = len(q)
        
        # Build QP problem
        # minimize ||tau||^2
        # subject to: M*ddq + h = tau + Jc^T * fc
        #            task constraints (prioritized)
        #            torque limits
        #            friction cone
        
        M = self.dynamics.mass_matrix(q)
        h = self.dynamics.coriolis(q, dq) + self.dynamics.gravity(q)
        
        # Stack contact Jacobians
        Jc = np.vstack(contact_jacobians) if contact_jacobians else np.zeros((0, n))
        nc = Jc.shape[0]
        
        # Decision variables: [ddq, tau, fc]
        num_vars = n + n + nc
        
        # Build cost matrix (minimize torques)
        H = np.zeros((num_vars, num_vars))
        H[n:2*n, n:2*n] = np.eye(n)  # ||tau||^2
        
        # Equality constraint: M*ddq + h = tau + Jc^T * fc
        Aeq = np.zeros((n, num_vars))
        Aeq[:, :n] = M
        Aeq[:, n:2*n] = -np.eye(n)
        Aeq[:, 2*n:] = -Jc.T
        beq = -h
        
        # Task constraints (as soft constraints in cost)
        for J, x_ddot_des, weight in tasks:
            # J * ddq = x_ddot_des
            task_cost = weight * (J.T @ J)
            H[:n, :n] += task_cost
        
        # Solve QP
        from scipy.optimize import minimize
        
        def objective(x):
            return 0.5 * x @ H @ x
        
        def eq_constraint(x):
            return Aeq @ x - beq
        
        x0 = np.zeros(num_vars)
        result = minimize(objective, x0, constraints={'type': 'eq', 'fun': eq_constraint})
        
        tau = result.x[n:2*n]
        return tau
```

### Balance Controller

```python
class BalanceController:
    def __init__(self, robot_dynamics, com_height=0.8):
        self.dynamics = robot_dynamics
        self.cp_controller = CapturePointController(com_height)
        
        # Gains
        self.kp_com = np.array([100, 100, 200])
        self.kd_com = np.array([20, 20, 40])
        self.kp_orient = 100
        self.kd_orient = 20
    
    def compute_torques(self, q, dq, com_desired, com_vel_desired, orientation_desired):
        """Compute torques for balance."""
        # Current state
        com = self.dynamics.compute_com(q)
        J_com = self.dynamics.compute_com_jacobian(q)
        com_vel = J_com @ dq
        
        # CoM task
        com_acc_des = self.kp_com * (com_desired - com) + self.kd_com * (com_vel_desired - com_vel)
        
        # Orientation task (simplified)
        # ... orientation control
        
        # Build task list
        tasks = [
            (J_com, com_acc_des, 1.0),  # CoM tracking
        ]
        
        # Contact Jacobians (assume double support)
        J_left_foot = self.dynamics.compute_frame_jacobian(q, "left_foot")
        J_right_foot = self.dynamics.compute_frame_jacobian(q, "right_foot")
        
        # Compute torques
        wbc = WholeBodyController(self.dynamics)
        tau = wbc.compute_torques(q, dq, tasks, [J_left_foot[:3], J_right_foot[:3]])
        
        return tau
```

## 22.6 Push Recovery

### Strategies

```
┌─────────────────────────────────────────────────────────────┐
│              Push Recovery Strategies                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Ankle Strategy (small pushes)                           │
│     ┌─────┐                                                 │
│     │  ●  │  Shift CoP within foot support                  │
│     └──┬──┘                                                 │
│        ▼                                                    │
│     ◄─────►                                                 │
│                                                             │
│  2. Hip Strategy (medium pushes)                            │
│     ┌─────┐                                                 │
│     │  ●  │  Rotate torso to shift CoM                      │
│     └──┬──┘                                                 │
│      ◄─┼─►                                                  │
│        │                                                    │
│                                                             │
│  3. Stepping Strategy (large pushes)                        │
│     ┌─────┐         ┌─────┐                                 │
│     │  ●  │  ─►     │  ●  │  Take a step to capture        │
│     └──┬──┘         └──┬──┘                                 │
│                          │                                  │
└─────────────────────────────────────────────────────────────┘
```

### Implementation

```python
class PushRecoveryController:
    def __init__(self, robot_dynamics):
        self.dynamics = robot_dynamics
        self.cp_controller = CapturePointController()
        
        # Thresholds
        self.ankle_threshold = 0.05  # CP error for ankle strategy
        self.hip_threshold = 0.15    # CP error for hip strategy
    
    def select_strategy(self, com_pos, com_vel, support_polygon):
        """Select recovery strategy based on CP location."""
        cp = self.cp_controller.compute_capture_point(com_pos, com_vel)
        
        # Distance from CP to support polygon edge
        cp_margin = self.distance_to_polygon_edge(cp, support_polygon)
        
        if cp_margin > self.ankle_threshold:
            return 'ankle'
        elif cp_margin > -self.hip_threshold:
            return 'hip'
        else:
            return 'stepping'
    
    def ankle_strategy(self, q, dq, com_pos, com_vel):
        """Ankle strategy: adjust CoP within support."""
        cp = self.cp_controller.compute_capture_point(com_pos, com_vel)
        
        # Desired CoP to push CP back
        cop_desired = self.cp_controller.compute_desired_cop(
            com_pos, com_vel, 
            cp_desired=com_pos  # Want CP at CoM
        )
        
        # Convert CoP to ankle torques
        # tau_ankle = (cop_desired - ankle_pos) × F_z
        return self.cop_to_ankle_torque(cop_desired, q)
    
    def hip_strategy(self, q, dq, com_pos, com_vel):
        """Hip strategy: rotate torso to shift angular momentum."""
        cp = self.cp_controller.compute_capture_point(com_pos, com_vel)
        
        # Compute desired hip rotation
        cp_error = cp - com_pos
        desired_hip_angle = np.arctan2(cp_error[1], cp_error[0]) * 0.5
        
        return desired_hip_angle
    
    def stepping_strategy(self, com_pos, com_vel, current_stance):
        """Stepping strategy: compute where to step."""
        cp = self.cp_controller.compute_capture_point(com_pos, com_vel)
        
        # Step to capture point with margin
        step_pos = cp + 0.02 * (cp - com_pos) / (np.linalg.norm(cp - com_pos) + 1e-6)
        
        # Determine which foot to move
        if current_stance == 'left':
            swing_foot = 'right'
        else:
            swing_foot = 'left'
        
        return step_pos, swing_foot
```

## 22.7 Integration with Perception

### Terrain Adaptation

```python
class TerrainAdaptiveWalking:
    def __init__(self, footstep_planner, depth_processor):
        self.planner = footstep_planner
        self.depth = depth_processor
    
    def plan_footsteps_with_terrain(self, start, goal, depth_image, camera_info):
        """Plan footsteps considering terrain."""
        # Get terrain height map
        height_map = self.depth.depth_to_height_map(depth_image, camera_info)
        
        # Plan nominal footsteps
        footsteps = self.planner.plan_footsteps(start, goal)
        
        # Adjust for terrain
        for i, step in enumerate(footsteps):
            pos = step['position']
            
            # Get terrain height at footstep location
            terrain_height = self.sample_height(height_map, pos[:2])
            
            # Get terrain normal
            normal = self.estimate_normal(height_map, pos[:2])
            
            # Adjust footstep
            footsteps[i]['position'][2] = terrain_height
            footsteps[i]['orientation_adjust'] = self.normal_to_foot_orientation(normal)
        
        return footsteps
    
    def sample_height(self, height_map, position):
        """Sample height from height map."""
        # Convert position to pixel coordinates
        px, py = self.world_to_pixel(position)
        return height_map[int(py), int(px)]
    
    def estimate_normal(self, height_map, position, window=5):
        """Estimate surface normal from height map."""
        px, py = self.world_to_pixel(position)
        
        # Compute gradients
        dx = (height_map[int(py), int(px)+1] - height_map[int(py), int(px)-1]) / 2
        dy = (height_map[int(py)+1, int(px)] - height_map[int(py)-1, int(px)]) / 2
        
        normal = np.array([-dx, -dy, 1])
        return normal / np.linalg.norm(normal)
```

## 22.8 Summary

In this chapter, you learned:

- **Gait cycle**: Phases and parameters of walking
- **Pattern generation**: Footstep planning and swing trajectories
- **ZMP control**: Linear inverted pendulum and preview control
- **Capture point**: Balance metric and control
- **Whole-body control**: Task-space control for locomotion
- **Push recovery**: Ankle, hip, and stepping strategies
- **Terrain adaptation**: Integrating perception with walking

Bipedal locomotion is a complex but solvable problem. In the next chapter, we'll explore manipulation with humanoid hands.

## Review Questions

1. What are the phases of the gait cycle?
2. What is the Linear Inverted Pendulum Model (LIPM)?
3. How does preview control improve ZMP tracking?
4. What is the capture point and how is it used?
5. What are the three push recovery strategies?

## Hands-On Exercise

1. Implement a basic footstep planner
2. Generate swing foot trajectories
3. Create a ZMP preview controller
4. Implement capture point computation
5. Test push recovery on a simulated humanoid
