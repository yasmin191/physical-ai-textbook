# Appendix E: Mathematical Foundations

This appendix provides the essential mathematical background for understanding humanoid robotics, including linear algebra, kinematics, dynamics, and control theory.

## Learning Objectives

By the end of this appendix, you will be able to:
- Apply linear algebra concepts to robotics transformations
- Understand rotation representations and conversions
- Derive forward and inverse kinematics equations
- Apply dynamics equations to robot motion planning
- Understand control theory fundamentals for robotics

## E.1 Linear Algebra Fundamentals

### E.1.1 Vectors and Vector Operations

A vector in 3D space represents position, velocity, force, or direction. In robotics, we commonly work with:

- **Position vectors**: Location of points in space
- **Velocity vectors**: Rate of change of position
- **Force vectors**: Forces acting on bodies

```python
import numpy as np

# Vector creation
v1 = np.array([1.0, 2.0, 3.0])
v2 = np.array([4.0, 5.0, 6.0])

# Addition
v_sum = v1 + v2  # [5, 7, 9]

# Scalar multiplication
v_scaled = 2.0 * v1  # [2, 4, 6]

# Dot product (scalar result)
dot = np.dot(v1, v2)  # 1*4 + 2*5 + 3*6 = 32

# Cross product (vector result)
cross = np.cross(v1, v2)  # [-3, 6, -3]

# Magnitude (norm)
magnitude = np.linalg.norm(v1)  # sqrt(1 + 4 + 9) = 3.74

# Unit vector
unit = v1 / np.linalg.norm(v1)
```

### E.1.2 Matrices and Matrix Operations

Matrices are fundamental for representing transformations, rotations, and system dynamics.

```python
# Matrix creation
A = np.array([
    [1, 2, 3],
    [4, 5, 6],
    [7, 8, 9]
])

# Matrix multiplication
B = np.eye(3)  # Identity matrix
C = A @ B  # or np.matmul(A, B)

# Transpose
A_T = A.T

# Inverse (if exists)
A_inv = np.linalg.inv(A)

# Determinant
det = np.linalg.det(A)

# Eigenvalues and eigenvectors
eigenvalues, eigenvectors = np.linalg.eig(A)
```

### E.1.3 Special Matrices in Robotics

**Identity Matrix**: A 3x3 matrix with ones on diagonal and zeros elsewhere.

**Rotation Matrices**: Orthogonal matrices where R^T = R^(-1) and det(R) = 1.

**Skew-Symmetric Matrix**: Used for cross product computation.

```python
def skew_symmetric(v):
    """Create skew-symmetric matrix from vector."""
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

# Cross product via skew-symmetric
v1 = np.array([1, 2, 3])
v2 = np.array([4, 5, 6])
cross = skew_symmetric(v1) @ v2  # Same as np.cross(v1, v2)
```

## E.2 Rotation Representations

### E.2.1 Rotation Matrices

Elementary rotations about the principal axes:

**Rotation about X-axis (Roll)**:
```python
def rotation_x(angle):
    """Rotation matrix about X-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s, c]
    ])
```

**Rotation about Y-axis (Pitch)**:
```python
def rotation_y(angle):
    """Rotation matrix about Y-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, 0, s],
        [0, 1, 0],
        [-s, 0, c]
    ])
```

**Rotation about Z-axis (Yaw)**:
```python
def rotation_z(angle):
    """Rotation matrix about Z-axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([
        [c, -s, 0],
        [s, c, 0],
        [0, 0, 1]
    ])
```

### E.2.2 Euler Angles

Euler angles represent orientation as three successive rotations. The ZYX convention (Yaw-Pitch-Roll) is most common in robotics:

```python
def euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert ZYX Euler angles to rotation matrix."""
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)

def rotation_matrix_to_euler(R):
    """Extract ZYX Euler angles from rotation matrix."""
    # Handle gimbal lock
    if abs(R[2, 0]) >= 1.0:
        yaw = 0.0
        if R[2, 0] < 0:
            pitch = np.pi / 2
            roll = np.arctan2(R[0, 1], R[0, 2])
        else:
            pitch = -np.pi / 2
            roll = np.arctan2(-R[0, 1], -R[0, 2])
    else:
        pitch = np.arcsin(-R[2, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
        yaw = np.arctan2(R[1, 0], R[0, 0])
    
    return roll, pitch, yaw
```

**Gimbal Lock**: When pitch = ±90°, the first and third rotation axes align, losing one degree of freedom.

### E.2.3 Quaternions

Quaternions avoid gimbal lock and provide smooth interpolation. A unit quaternion for rotation has the form:

q = (w, x, y, z) where w² + x² + y² + z² = 1

```python
class Quaternion:
    def __init__(self, w, x, y, z):
        self.w = w
        self.x = x
        self.y = y
        self.z = z
    
    @classmethod
    def from_axis_angle(cls, axis, angle):
        """Create quaternion from axis-angle."""
        axis = np.array(axis) / np.linalg.norm(axis)
        half_angle = angle / 2
        w = np.cos(half_angle)
        xyz = np.sin(half_angle) * axis
        return cls(w, xyz[0], xyz[1], xyz[2])
    
    def normalize(self):
        """Normalize to unit quaternion."""
        norm = np.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        return Quaternion(self.w/norm, self.x/norm, self.y/norm, self.z/norm)
    
    def conjugate(self):
        """Quaternion conjugate (inverse for unit quaternions)."""
        return Quaternion(self.w, -self.x, -self.y, -self.z)
    
    def to_rotation_matrix(self):
        """Convert to 3x3 rotation matrix."""
        w, x, y, z = self.w, self.x, self.y, self.z
        return np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
```

### E.2.4 Axis-Angle Representation

A rotation can be described by a unit axis and angle. Rodrigues' formula converts to rotation matrix:

```python
def axis_angle_to_rotation_matrix(axis, angle):
    """Convert axis-angle to rotation matrix using Rodrigues' formula."""
    axis = np.array(axis) / np.linalg.norm(axis)
    K = skew_symmetric(axis)
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R
```

## E.3 Homogeneous Transformations

### E.3.1 Transformation Matrices

A 4×4 homogeneous transformation matrix combines rotation and translation:

```python
def transformation_matrix(R, p):
    """Create 4x4 homogeneous transformation matrix."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T

def transform_point(T, point):
    """Transform a 3D point using homogeneous transformation."""
    point_h = np.append(point, 1)  # Convert to homogeneous
    transformed = T @ point_h
    return transformed[:3]  # Convert back to 3D

def inverse_transform(T):
    """Compute inverse of transformation matrix."""
    R = T[:3, :3]
    p = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ p
    return T_inv
```

### E.3.2 Composition of Transformations

Transformations compose by matrix multiplication:

```python
# Example: Transform from base to end-effector
T_base_link1 = transformation_matrix(R1, p1)
T_link1_link2 = transformation_matrix(R2, p2)
T_link2_ee = transformation_matrix(R3, p3)

# Complete transformation
T_base_ee = T_base_link1 @ T_link1_link2 @ T_link2_ee
```

## E.4 Forward Kinematics

### E.4.1 Denavit-Hartenberg Parameters

The DH convention uses four parameters per joint:

| Parameter | Symbol | Description |
|-----------|--------|-------------|
| Link length | a | Distance along x from z_(i-1) to z_i |
| Link twist | α | Angle about x from z_(i-1) to z_i |
| Link offset | d | Distance along z_(i-1) from x_(i-1) to x_i |
| Joint angle | θ | Angle about z_(i-1) from x_(i-1) to x_i |

```python
def dh_transform(theta, d, a, alpha):
    """Compute DH transformation matrix."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st*ca, st*sa, a*ct],
        [st, ct*ca, -ct*sa, a*st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])

class RobotArm:
    def __init__(self, dh_params):
        """
        dh_params: List of (theta, d, a, alpha) tuples
        """
        self.dh_params = dh_params
        self.n_joints = len(dh_params)
    
    def forward_kinematics(self, joint_angles):
        """Compute end-effector pose from joint angles."""
        T = np.eye(4)
        for i, (_, d, a, alpha) in enumerate(self.dh_params):
            theta = joint_angles[i]
            T = T @ dh_transform(theta, d, a, alpha)
        return T
```

### E.4.2 Example: 2-DOF Planar Arm

```python
# 2-DOF planar arm
L1, L2 = 1.0, 0.8

def forward_kinematics_2dof(q1, q2):
    """Forward kinematics for 2-DOF planar arm."""
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    theta = q1 + q2  # End-effector orientation
    return x, y, theta
```

## E.5 Inverse Kinematics

### E.5.1 Analytical Methods

For simple mechanisms, closed-form solutions exist:

```python
def inverse_kinematics_2dof(x, y, L1, L2):
    """
    Inverse kinematics for 2-DOF planar arm.
    Returns two solutions (elbow up/down).
    """
    # Distance to target
    r = np.sqrt(x**2 + y**2)
    
    # Check reachability
    if r > L1 + L2 or r < abs(L1 - L2):
        return None, None
    
    # Law of cosines for elbow angle
    cos_q2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    
    # Elbow down solution
    q2_down = np.arccos(cos_q2)
    q1_down = np.arctan2(y, x) - np.arctan2(
        L2 * np.sin(q2_down),
        L1 + L2 * np.cos(q2_down)
    )
    
    # Elbow up solution
    q2_up = -np.arccos(cos_q2)
    q1_up = np.arctan2(y, x) - np.arctan2(
        L2 * np.sin(q2_up),
        L1 + L2 * np.cos(q2_up)
    )
    
    return (q1_down, q2_down), (q1_up, q2_up)
```

### E.5.2 Numerical Methods

For complex robots, use iterative methods like the Jacobian pseudo-inverse:

```python
def numerical_ik(robot, target_pose, initial_guess, max_iter=100, tol=1e-6):
    """Numerical inverse kinematics using Jacobian pseudo-inverse."""
    q = np.array(initial_guess)
    
    for i in range(max_iter):
        # Current end-effector pose
        T_current = robot.forward_kinematics(q)
        
        # Compute pose error
        error = compute_pose_error(target_pose, T_current)
        
        if np.linalg.norm(error) < tol:
            return q, True
        
        # Compute Jacobian
        J = robot.compute_jacobian(q)
        
        # Compute joint velocity (pseudo-inverse)
        J_pinv = np.linalg.pinv(J)
        dq = J_pinv @ error
        
        # Update joint angles
        q = q + dq
    
    return q, False  # Did not converge
```

## E.6 Jacobian Matrix

### E.6.1 Definition

The Jacobian relates joint velocities to end-effector velocities. For an n-DOF robot:

- Linear velocity (3x1) + Angular velocity (3x1) = Jacobian (6×n) × Joint velocity (n×1)

### E.6.2 Computing the Jacobian

```python
def compute_jacobian_numerical(robot, q, delta=1e-6):
    """Compute Jacobian numerically using finite differences."""
    n = len(q)
    J = np.zeros((6, n))
    
    T0 = robot.forward_kinematics(q)
    p0 = T0[:3, 3]
    R0 = T0[:3, :3]
    
    for i in range(n):
        q_plus = q.copy()
        q_plus[i] += delta
        
        T_plus = robot.forward_kinematics(q_plus)
        p_plus = T_plus[:3, 3]
        
        # Linear velocity Jacobian
        J[:3, i] = (p_plus - p0) / delta
        
        # Angular velocity (simplified)
        R_diff = T_plus[:3, :3] @ R0.T
        J[3:, i] = rotation_to_axis_angle(R_diff) / delta
    
    return J

# Manipulability measure
def manipulability(robot, q):
    """Compute manipulability index (Yoshikawa)."""
    J = compute_jacobian_numerical(robot, q)
    return np.sqrt(np.linalg.det(J @ J.T))

# Singularity detection
def is_singular(robot, q, threshold=0.01):
    """Check if configuration is near singularity."""
    return manipulability(robot, q) < threshold
```

## E.7 Robot Dynamics

### E.7.1 Equation of Motion

The manipulator dynamics are described by:

M(q) × q̈ + C(q, q̇) × q̇ + G(q) = τ

Where:
- M(q) is the mass/inertia matrix
- C(q, q̇) captures Coriolis and centrifugal effects
- G(q) is the gravity vector
- τ is the vector of joint torques

### E.7.2 Inverse Dynamics

```python
class RobotDynamics:
    def __init__(self, masses, inertias, com_positions, link_lengths):
        self.masses = masses
        self.inertias = inertias
        self.com_positions = com_positions
        self.link_lengths = link_lengths
        self.n = len(masses)
        self.g = 9.81
    
    def forward_dynamics(self, q, q_dot, tau):
        """Compute joint accelerations from torques."""
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, q_dot)
        G = self.gravity_vector(q)
        
        # Solve M * q_ddot = tau - C * q_dot - G
        q_ddot = np.linalg.solve(M, tau - C @ q_dot - G)
        return q_ddot
    
    def inverse_dynamics(self, q, q_dot, q_ddot):
        """Compute torques from desired accelerations."""
        M = self.mass_matrix(q)
        C = self.coriolis_matrix(q, q_dot)
        G = self.gravity_vector(q)
        
        tau = M @ q_ddot + C @ q_dot + G
        return tau
```

## E.8 Control Theory Fundamentals

### E.8.1 PID Control

```python
class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.output_limits = output_limits
        
        self.integral = 0
        self.prev_error = 0
    
    def compute(self, setpoint, measurement):
        """Compute PID control output."""
        error = setpoint - measurement
        
        # Proportional term
        P = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * self.dt
        I = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / self.dt
        D = self.kd * derivative
        
        self.prev_error = error
        
        output = P + I + D
        
        # Apply limits
        if self.output_limits:
            output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        return output
    
    def reset(self):
        """Reset controller state."""
        self.integral = 0
        self.prev_error = 0
```

### E.8.2 Computed Torque Control

```python
class ComputedTorqueController:
    def __init__(self, robot_dynamics, kp, kd):
        self.dynamics = robot_dynamics
        self.kp = np.diag(kp)
        self.kd = np.diag(kd)
    
    def compute_torque(self, q, q_dot, q_desired, q_dot_desired, q_ddot_desired):
        """Computed torque control law."""
        # Position and velocity errors
        e = q_desired - q
        e_dot = q_dot_desired - q_dot
        
        # Reference acceleration
        q_ddot_ref = q_ddot_desired + self.kd @ e_dot + self.kp @ e
        
        # Compute required torque using inverse dynamics
        tau = self.dynamics.inverse_dynamics(q, q_dot, q_ddot_ref)
        
        return tau
```

## E.9 Trajectory Generation

### E.9.1 Polynomial Trajectories

```python
def cubic_trajectory(q0, qf, v0, vf, tf):
    """Generate cubic polynomial trajectory."""
    # Solve for coefficients
    a0 = q0
    a1 = v0
    a2 = (3*(qf - q0) - (2*v0 + vf)*tf) / tf**2
    a3 = (2*(q0 - qf) + (v0 + vf)*tf) / tf**3
    
    def trajectory(t):
        q = a0 + a1*t + a2*t**2 + a3*t**3
        q_dot = a1 + 2*a2*t + 3*a3*t**2
        q_ddot = 2*a2 + 6*a3*t
        return q, q_dot, q_ddot
    
    return trajectory
```

### E.9.2 Trapezoidal Velocity Profile

```python
def trapezoidal_profile(q0, qf, v_max, a_max, dt):
    """Generate trajectory with trapezoidal velocity profile."""
    distance = abs(qf - q0)
    sign = 1 if qf > q0 else -1
    
    # Check if we can reach max velocity
    t_accel = v_max / a_max
    d_accel = 0.5 * a_max * t_accel**2
    
    if 2 * d_accel > distance:
        # Triangular profile
        t_accel = np.sqrt(distance / a_max)
        v_peak = a_max * t_accel
        t_cruise = 0
    else:
        v_peak = v_max
        d_cruise = distance - 2 * d_accel
        t_cruise = d_cruise / v_max
    
    t_total = 2 * t_accel + t_cruise
    
    times = np.arange(0, t_total + dt, dt)
    positions = []
    velocities = []
    
    for t in times:
        if t < t_accel:
            # Acceleration phase
            v = a_max * t
            q = q0 + sign * 0.5 * a_max * t**2
        elif t < t_accel + t_cruise:
            # Cruise phase
            t_rel = t - t_accel
            q = q0 + sign * (d_accel + v_peak * t_rel)
            v = v_peak
        else:
            # Deceleration phase
            t_rel = t - t_accel - t_cruise
            v = v_peak - a_max * t_rel
            q = qf - sign * 0.5 * a_max * (t_total - t)**2
        
        positions.append(q)
        velocities.append(sign * v)
    
    return times, np.array(positions), np.array(velocities)
```

## Summary

This appendix covered the essential mathematical foundations for humanoid robotics:

1. **Linear Algebra**: Vectors, matrices, and operations crucial for spatial computations
2. **Rotation Representations**: Rotation matrices, Euler angles, quaternions, axis-angle
3. **Homogeneous Transformations**: 4×4 matrices combining rotation and translation
4. **Forward Kinematics**: DH parameters and computing end-effector pose
5. **Inverse Kinematics**: Analytical and numerical methods for joint angles
6. **Jacobian Matrix**: Relating joint and end-effector velocities
7. **Robot Dynamics**: Mass matrix, Coriolis forces, gravity compensation
8. **Control Theory**: PID, computed torque, state-space methods
9. **Trajectory Generation**: Polynomial and trapezoidal velocity profiles

These mathematical tools form the foundation for all advanced topics in humanoid robotics.

## Quick Reference

| Concept | Key Equation |
|---------|-------------|
| Rotation Matrix Property | R^T × R = I, det(R) = 1 |
| Unit Quaternion | w² + x² + y² + z² = 1 |
| Jacobian Definition | ẋ = J(q) × q̇ |
| Dynamics Equation | M(q)q̈ + C(q,q̇)q̇ + G(q) = τ |
| PID Control | u = Kp×e + Ki×∫e dt + Kd×ė |

## Additional Resources

- Craig, J.J. "Introduction to Robotics: Mechanics and Control"
- Siciliano, B. "Robotics: Modelling, Planning and Control"
- Murray, R.M. "A Mathematical Introduction to Robotic Manipulation"
- Spong, M.W. "Robot Modeling and Control"
