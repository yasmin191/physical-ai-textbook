---
sidebar_position: 3
title: "Chapter 11: Physics Simulation"
description: "Understanding rigid body dynamics, collision detection, and physics engines for humanoid simulation"
---

# Chapter 11: Physics Simulation

Accurate physics simulation is the foundation of realistic humanoid robot development. This chapter explores how physics engines model rigid body dynamics, contact forces, and constraints that govern robot behavior in simulation.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand rigid body dynamics fundamentals
- Configure physics engine parameters for stable simulation
- Implement proper collision detection and response
- Tune contact forces for realistic walking and manipulation
- Debug common physics simulation issues

## 11.1 Fundamentals of Rigid Body Dynamics

### Newton-Euler Equations

The motion of a rigid body is governed by Newton's second law for translation and Euler's equation for rotation:

**Linear Motion:**
F = ma

**Angular Motion:**
\tau = I\alpha + \omega \times (I\omega)

Where:
-  = net force
-  = mass
-  = linear acceleration
-  = net torque
-  = inertia tensor
-  = angular acceleration
-  = angular velocity

### Inertia Tensor

The 3×3 inertia tensor describes how mass is distributed:

```
    ⎡ Ixx  -Ixy  -Ixz ⎤
I = ⎢-Ixy   Iyy  -Iyz ⎥
    ⎣-Ixz  -Iyz   Izz ⎦
```

For simulation stability:
- All diagonal elements must be positive
- The tensor must be symmetric
- Triangle inequality:  (and permutations)

### Computing Inertia for Common Shapes

```python
def box_inertia(mass, width, height, depth):
    """Solid box inertia about center of mass."""
    ixx = (1/12) * mass * (height**2 + depth**2)
    iyy = (1/12) * mass * (width**2 + depth**2)
    izz = (1/12) * mass * (width**2 + height**2)
    return ixx, iyy, izz

def cylinder_inertia(mass, radius, height):
    """Solid cylinder inertia (axis along z)."""
    ixx = iyy = (1/12) * mass * (3*radius**2 + height**2)
    izz = 0.5 * mass * radius**2
    return ixx, iyy, izz

def sphere_inertia(mass, radius):
    """Solid sphere inertia."""
    i = (2/5) * mass * radius**2
    return i, i, i
```

## 11.2 Physics Engines in Gazebo

### Available Engines

| Engine | Description | Best For |
|--------|-------------|----------|
| **DART** | Dynamic Animation and Robotics Toolkit | Articulated robots, humanoids |
| **ODE** | Open Dynamics Engine | General robotics |
| **Bullet** | Game physics engine | Fast simulation |
| **Simbody** | Biomechanics-focused | Human modeling |

### DART for Humanoids

DART is recommended for humanoid simulation due to:
- Superior joint constraint handling
- Better contact stability
- Accurate articulated body dynamics
- Support for soft contacts

### Configuring Physics in SDF

```xml
<physics type="dart">
  <!-- Time stepping -->
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>
  
  <!-- DART-specific settings -->
  <dart>
    <!-- Collision detector -->
    <collision_detector>fcl</collision_detector>
    
    <!-- Constraint solver -->
    <solver>
      <solver_type>dantzig</solver_type>
    </solver>
  </dart>
</physics>
```

## 11.3 Time Stepping and Integration

### Numerical Integration Methods

Physics engines use numerical methods to solve differential equations:

| Method | Order | Stability | Speed |
|--------|-------|-----------|-------|
| Euler | 1st | Poor | Fast |
| Semi-implicit Euler | 1st | Good | Fast |
| Runge-Kutta 4 | 4th | Excellent | Slow |
| Verlet | 2nd | Good | Medium |

Most physics engines use **semi-implicit Euler** (symplectic Euler) for the best balance.

### Step Size Selection

The simulation step size affects accuracy and stability:

```
Smaller step size:
  ✓ More accurate
  ✓ More stable contacts
  ✗ Slower simulation
  
Larger step size:
  ✓ Faster simulation
  ✗ Less accurate
  ✗ Potential instability
```

**Guidelines for humanoids:**

| Scenario | Recommended Step Size |
|----------|----------------------|
| Walking simulation | 0.001s (1ms) |
| Manipulation tasks | 0.001s - 0.002s |
| General testing | 0.004s |
| Fast prototyping | 0.01s |

### Real-Time Factor

```xml
<!-- Run at real-time -->
<real_time_factor>1.0</real_time_factor>

<!-- Run 2x faster than real-time -->
<real_time_factor>2.0</real_time_factor>

<!-- Run as fast as possible (no limit) -->
<real_time_factor>0</real_time_factor>
```

## 11.4 Collision Detection

### Broad Phase vs Narrow Phase

Collision detection occurs in two phases:

```
Broad Phase (fast, approximate):
  - Axis-Aligned Bounding Boxes (AABB)
  - Spatial hashing
  - Identifies potential collision pairs

Narrow Phase (slow, exact):
  - GJK algorithm (convex shapes)
  - Mesh-mesh intersection
  - Computes contact points and normals
```

### Collision Geometry Types

| Type | Speed | Accuracy | Use Case |
|------|-------|----------|----------|
| Sphere | Fastest | Low | Quick prototyping |
| Box | Fast | Medium | Torso, links |
| Cylinder | Fast | Medium | Limbs |
| Capsule | Fast | Good | Limbs (preferred) |
| Convex mesh | Medium | High | Complex parts |
| Triangle mesh | Slow | Exact | Static environment |

### Collision Geometry Best Practices

```xml
<!-- GOOD: Simple primitive for collision -->
<collision name="torso_collision">
  <geometry>
    <box><size>0.3 0.2 0.5</size></box>
  </geometry>
</collision>

<!-- GOOD: Capsule for limbs -->
<collision name="arm_collision">
  <geometry>
    <capsule>
      <radius>0.04</radius>
      <length>0.3</length>
    </capsule>
  </geometry>
</collision>

<!-- AVOID: Complex mesh for moving parts -->
<collision name="arm_collision">
  <geometry>
    <mesh>
      <uri>model://arm_detailed.stl</uri>  <!-- Too complex! -->
    </mesh>
  </geometry>
</collision>
```

## 11.5 Contact Dynamics

### Contact Force Model

When two bodies collide, contact forces are computed:

```
Contact Force = Normal Force + Friction Force

Normal Force: Fn = kp * penetration + kd * velocity
Friction Force: Ff ≤ μ * Fn (Coulomb friction)
```

### Contact Parameters

```xml
<surface>
  <contact>
    <ode>
      <!-- Contact stiffness (spring constant) -->
      <kp>1000000</kp>
      
      <!-- Contact damping -->
      <kd>100</kd>
      
      <!-- Maximum correcting velocity -->
      <max_vel>100</max_vel>
      
      <!-- Minimum penetration before force applied -->
      <min_depth>0.001</min_depth>
    </ode>
  </contact>
  
  <friction>
    <ode>
      <!-- Primary friction coefficient -->
      <mu>0.8</mu>
      
      <!-- Secondary friction coefficient -->
      <mu2>0.8</mu2>
      
      <!-- Friction direction (optional) -->
      <fdir1>1 0 0</fdir1>
    </ode>
  </friction>
</surface>
```

### Tuning Contact Parameters

**Problem: Robot bounces on ground**
```xml
<!-- Increase damping -->
<kd>500</kd>
```

**Problem: Robot sinks through ground**
```xml
<!-- Increase stiffness -->
<kp>10000000</kp>
<min_depth>0.0001</min_depth>
```

**Problem: Robot slides when walking**
```xml
<!-- Increase friction -->
<mu>1.2</mu>
<mu2>1.2</mu2>
```

**Problem: Jittering at contacts**
```xml
<!-- Reduce stiffness, increase damping -->
<kp>100000</kp>
<kd>1000</kd>
<!-- Use smaller time step -->
<max_step_size>0.0005</max_step_size>
```

## 11.6 Joint Constraints

### Constraint-Based Dynamics

Joints are implemented as constraints that restrict relative motion:

```
Fixed: 6 constraints (no motion)
Revolute: 5 constraints (1 DOF rotation)
Prismatic: 5 constraints (1 DOF translation)
Ball: 3 constraints (3 DOF rotation)
```

### Joint Constraint Parameters

```xml
<joint name="knee" type="revolute">
  <axis>
    <xyz>0 1 0</xyz>
    <limit>
      <lower>0</lower>
      <upper>2.5</upper>
      <effort>100</effort>
      <velocity>5</velocity>
    </limit>
    <dynamics>
      <!-- Viscous damping (N·m·s/rad) -->
      <damping>1.0</damping>
      
      <!-- Static friction (N·m) -->
      <friction>0.1</friction>
    </dynamics>
  </axis>
  
  <!-- Constraint solver parameters -->
  <physics>
    <ode>
      <!-- Constraint force mixing -->
      <cfm>0.0</cfm>
      
      <!-- Error reduction parameter -->
      <erp>0.2</erp>
    </ode>
  </physics>
</joint>
```

### CFM and ERP Explained

**CFM (Constraint Force Mixing):**
- Adds softness to constraints
- Higher values = softer, more forgiving constraints
- Default: 0.0 (rigid constraints)
- For soft contacts: 0.001 - 0.01

**ERP (Error Reduction Parameter):**
- Controls how fast constraint errors are corrected
- Range: 0.0 to 1.0
- Default: 0.2
- Higher = faster correction but potential instability

```xml
<!-- Soft joint (mimics compliance) -->
<physics>
  <ode>
    <cfm>0.01</cfm>
    <erp>0.8</erp>
  </ode>
</physics>

<!-- Rigid joint -->
<physics>
  <ode>
    <cfm>0.0</cfm>
    <erp>0.2</erp>
  </ode>
</physics>
```

## 11.7 Gravity and External Forces

### Configuring Gravity

```xml
<world name="earth">
  <gravity>0 0 -9.81</gravity>
</world>

<!-- Moon gravity -->
<world name="moon">
  <gravity>0 0 -1.62</gravity>
</world>

<!-- Zero gravity (space) -->
<world name="space">
  <gravity>0 0 0</gravity>
</world>
```

### Applying External Forces

Use Gazebo services to apply forces programmatically:

```python
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import ApplyBodyWrench
from geometry_msgs.msg import Wrench
from builtin_interfaces.msg import Duration

class ForceApplier(Node):
    def __init__(self):
        super().__init__('force_applier')
        self.client = self.create_client(
            ApplyBodyWrench, 
            '/apply_body_wrench'
        )
        
    def apply_force(self, body_name, force, duration_sec):
        request = ApplyBodyWrench.Request()
        request.body_name = body_name
        request.wrench = Wrench()
        request.wrench.force.x = force[0]
        request.wrench.force.y = force[1]
        request.wrench.force.z = force[2]
        request.duration = Duration(sec=int(duration_sec))
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
```

## 11.8 Debugging Physics Issues

### Common Problems and Solutions

#### 1. Robot Explodes on Spawn

**Symptoms:** Parts fly apart immediately

**Causes:**
- Overlapping collision geometries
- Invalid inertia values
- Joints at impossible angles

**Solutions:**
```xml
<!-- Check for collision overlap -->
<!-- Ensure collision geometries don't intersect at rest -->

<!-- Verify inertia is physically valid -->
<inertial>
  <mass value="1.0"/>  <!-- Must be positive -->
  <inertia ixx="0.01" iyy="0.01" izz="0.01"/>  <!-- All positive -->
</inertial>

<!-- Spawn above ground with safe distance -->
<pose>0 0 1.5 0 0 0</pose>
```

#### 2. Robot Drifts or Slides

**Symptoms:** Robot moves without input

**Causes:**
- Asymmetric inertia
- Numerical precision issues
- Low friction

**Solutions:**
```xml
<!-- Ensure symmetric mass distribution -->
<!-- Increase friction -->
<mu>1.0</mu>
<mu2>1.0</mu2>

<!-- Use smaller time step -->
<max_step_size>0.0005</max_step_size>
```

#### 3. Jittering and Vibration

**Symptoms:** Links oscillate rapidly

**Causes:**
- Stiff contacts with large time step
- Underdamped joints
- Conflicting constraints

**Solutions:**
```xml
<!-- Add joint damping -->
<dynamics>
  <damping>5.0</damping>
</dynamics>

<!-- Soften contacts -->
<kp>100000</kp>
<kd>1000</kd>

<!-- Reduce time step -->
<max_step_size>0.0005</max_step_size>
```

#### 4. Robot Falls Through Ground

**Symptoms:** Robot passes through surfaces

**Causes:**
- Missing collision geometry
- Contact parameters too soft
- Step size too large

**Solutions:**
```xml
<!-- Ensure collision elements exist -->
<collision name="foot_collision">
  <geometry>...</geometry>
</collision>

<!-- Stiffen contact -->
<kp>10000000</kp>
<min_depth>0.0001</min_depth>

<!-- Reduce step size -->
<max_step_size>0.001</max_step_size>
```

### Visualization Tools

```bash
# View collision geometry in Gazebo
# Enable "View > Collisions" in GUI

# View center of mass
# Enable "View > Center of Mass"

# View contact points
# Enable "View > Contacts"

# View joint axes
# Enable "View > Joints"
```

## 11.9 Performance Optimization

### Reducing Computational Load

1. **Simplify collision geometry**
```xml
<!-- Use primitives instead of meshes -->
<collision>
  <geometry>
    <box><size>0.1 0.1 0.1</size></box>
  </geometry>
</collision>
```

2. **Disable unnecessary collisions**
```xml
<model name="robot">
  <self_collide>false</self_collide>
</model>
```

3. **Use appropriate step size**
```xml
<!-- Balance accuracy and speed -->
<max_step_size>0.002</max_step_size>
```

4. **Limit sensor update rates**
```xml
<sensor type="camera">
  <update_rate>15</update_rate>  <!-- Not 60fps if not needed -->
</sensor>
```

### Parallel Simulation

```xml
<!-- Enable multi-threaded physics (ODE) -->
<physics type="ode">
  <ode>
    <solver>
      <island_threads>4</island_threads>
    </solver>
  </ode>
</physics>
```

## 11.10 Summary

In this chapter, you learned:

- **Rigid body dynamics**: Newton-Euler equations and inertia
- **Physics engines**: DART, ODE, Bullet selection and configuration
- **Time stepping**: Step size selection and integration methods
- **Collision detection**: Broad/narrow phase, geometry types
- **Contact dynamics**: Force models and parameter tuning
- **Debugging**: Common problems and solutions

Proper physics configuration is essential for realistic humanoid simulation. In the next chapter, we'll explore sensor simulation.

## Review Questions

1. Why is the inertia tensor important for simulation stability?
2. What is the difference between CFM and ERP?
3. How does step size affect simulation accuracy and speed?
4. Why should you use simple collision geometry for moving parts?
5. What causes jittering in simulation and how can it be fixed?

## Hands-On Exercise

1. Create a simple pendulum and observe its motion
2. Experiment with different step sizes and note the effect
3. Add a humanoid leg to the simulation and tune contact parameters
4. Apply external forces and observe the response
5. Debug a provided "broken" robot model with physics issues
