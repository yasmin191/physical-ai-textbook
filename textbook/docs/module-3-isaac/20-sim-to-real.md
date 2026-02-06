---
sidebar_position: 7
title: "Chapter 20: Sim-to-Real Transfer"
description: "Deploy simulation-trained policies to real humanoid robots"
---

# Chapter 20: Sim-to-Real Transfer

The ultimate goal of simulation is to develop algorithms that work on real robots. **Sim-to-real transfer** bridges the gap between simulated and physical environments. This chapter covers techniques to successfully deploy policies trained in Isaac Sim/Lab to physical humanoid robots.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the reality gap and its sources
- Apply domain randomization effectively
- Implement system identification for simulation calibration
- Deploy policies to real hardware using ROS 2
- Debug and refine sim-to-real transfers

## 20.1 The Reality Gap

### Sources of the Gap

```
┌─────────────────────────────────────────────────────────────┐
│                    Reality Gap Sources                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Simulation                          Real World             │
│  ┌──────────────────┐               ┌──────────────────┐    │
│  │ Perfect sensors  │ ──────────── │ Noisy sensors    │    │
│  │ Instant response │               │ Latency/delay    │    │
│  │ Simplified physics│              │ Complex dynamics │    │
│  │ Known parameters │               │ Unknown params   │    │
│  │ Deterministic    │               │ Stochastic       │    │
│  └──────────────────┘               └──────────────────┘    │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Gap Categories

| Category | Simulation | Reality |
|----------|------------|---------|
| **Physics** | Idealized models | Complex contacts, deformation |
| **Sensors** | Perfect data | Noise, bias, latency |
| **Actuators** | Instant torque | Delays, saturation, friction |
| **Environment** | Known geometry | Unknown obstacles, surfaces |
| **Calibration** | Exact parameters | Manufacturing variance |

### Quantifying the Gap

```python
def measure_reality_gap(sim_env, real_robot, test_trajectory):
    """Measure discrepancy between sim and real."""
    
    # Execute same trajectory in both
    sim_states = sim_env.execute_trajectory(test_trajectory)
    real_states = real_robot.execute_trajectory(test_trajectory)
    
    # Compute state differences
    position_error = np.mean(np.abs(sim_states['pos'] - real_states['pos']))
    velocity_error = np.mean(np.abs(sim_states['vel'] - real_states['vel']))
    
    # Compute trajectory divergence
    divergence_rate = compute_trajectory_divergence(sim_states, real_states)
    
    return {
        'position_error': position_error,
        'velocity_error': velocity_error,
        'divergence_rate': divergence_rate
    }
```

## 20.2 Domain Randomization

### Principle

Train with varied simulation parameters so the policy becomes robust to real-world variations:

```
┌─────────────────────────────────────────────────────────────┐
│                  Domain Randomization                        │
│                                                             │
│   Training: Sample from distribution of parameters          │
│                                                             │
│   ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐  ┌─────┐              │
│   │ μ-2σ │  │ μ-σ │  │  μ  │  │ μ+σ │  │ μ+2σ │              │
│   └─────┘  └─────┘  └─────┘  └─────┘  └─────┘              │
│      ▲        ▲        ▲        ▲        ▲                  │
│      └────────┴────────┴────────┴────────┘                  │
│                        │                                    │
│              Real world falls somewhere                     │
│              within trained distribution                    │
└─────────────────────────────────────────────────────────────┘
```

### Physics Randomization

```python
@configclass
class DomainRandomizationCfg:
    # Mass randomization
    mass_randomization: bool = True
    mass_range: tuple = (0.8, 1.2)  # ±20%
    
    # Friction randomization
    friction_randomization: bool = True
    friction_range: tuple = (0.5, 1.5)
    
    # Center of mass shift
    com_randomization: bool = True
    com_displacement_range: tuple = (-0.05, 0.05)  # meters
    
    # Joint properties
    joint_friction_range: tuple = (0.0, 0.5)
    joint_damping_range: tuple = (0.5, 1.5)
    
    # Motor properties
    motor_strength_range: tuple = (0.8, 1.2)
    motor_offset_range: tuple = (-0.05, 0.05)  # radians

class PhysicsRandomizer:
    def __init__(self, cfg: DomainRandomizationCfg):
        self.cfg = cfg
    
    def randomize(self, env):
        """Apply randomization to environment."""
        num_envs = env.num_envs
        
        if self.cfg.mass_randomization:
            mass_scale = torch.empty(num_envs).uniform_(*self.cfg.mass_range)
            env.robot.scale_link_masses(mass_scale)
        
        if self.cfg.friction_randomization:
            friction = torch.empty(num_envs).uniform_(*self.cfg.friction_range)
            env.scene.set_ground_friction(friction)
        
        if self.cfg.com_randomization:
            com_shift = torch.empty(num_envs, 3).uniform_(*self.cfg.com_displacement_range)
            env.robot.shift_center_of_mass(com_shift)
```

### Sensor Randomization

```python
class SensorRandomizer:
    def __init__(self):
        # IMU noise parameters
        self.gyro_noise_density = 0.0002  # rad/s/√Hz
        self.accel_noise_density = 0.002  # m/s²/√Hz
        self.gyro_bias_range = (-0.01, 0.01)  # rad/s
        self.accel_bias_range = (-0.1, 0.1)  # m/s²
        
        # Joint encoder noise
        self.position_noise_std = 0.001  # rad
        self.velocity_noise_std = 0.01  # rad/s
        
        # Latency
        self.latency_range = (0, 20)  # ms
    
    def add_noise(self, obs, dt):
        """Add realistic sensor noise."""
        # IMU noise
        gyro_noise = torch.randn_like(obs['gyro']) * self.gyro_noise_density * np.sqrt(1/dt)
        accel_noise = torch.randn_like(obs['accel']) * self.accel_noise_density * np.sqrt(1/dt)
        
        # Add bias (constant per episode)
        obs['gyro'] += gyro_noise + self.gyro_bias
        obs['accel'] += accel_noise + self.accel_bias
        
        # Joint sensor noise
        obs['joint_pos'] += torch.randn_like(obs['joint_pos']) * self.position_noise_std
        obs['joint_vel'] += torch.randn_like(obs['joint_vel']) * self.velocity_noise_std
        
        return obs
    
    def reset_biases(self, num_envs):
        """Randomize biases at episode start."""
        self.gyro_bias = torch.empty(num_envs, 3).uniform_(*self.gyro_bias_range)
        self.accel_bias = torch.empty(num_envs, 3).uniform_(*self.accel_bias_range)
```

### Action Randomization

```python
class ActuatorRandomizer:
    def __init__(self):
        self.delay_steps_range = (0, 3)  # Action delay in sim steps
        self.strength_range = (0.85, 1.15)
        self.offset_range = (-0.02, 0.02)  # rad
    
    def apply(self, actions, env_indices):
        """Apply actuator randomization."""
        # Strength variation
        strength = torch.empty(len(env_indices)).uniform_(*self.strength_range)
        actions = actions * strength.unsqueeze(-1)
        
        # Position offset
        offset = torch.empty_like(actions).uniform_(*self.offset_range)
        actions = actions + offset
        
        return actions
```

### External Disturbances

```python
class DisturbanceGenerator:
    def __init__(self):
        self.push_force_range = (0, 200)  # N
        self.push_duration = 0.1  # seconds
        self.push_interval_range = (2, 10)  # seconds between pushes
    
    def generate_push(self, robot, num_envs):
        """Apply random push to robot."""
        # Random direction
        theta = torch.rand(num_envs) * 2 * np.pi
        direction = torch.stack([torch.cos(theta), torch.sin(theta), torch.zeros(num_envs)], dim=-1)
        
        # Random magnitude
        magnitude = torch.empty(num_envs).uniform_(*self.push_force_range)
        
        force = direction * magnitude.unsqueeze(-1)
        
        # Apply to torso
        robot.apply_external_force(
            body_name="torso",
            force=force,
            position=robot.get_body_position("torso")
        )
```

## 20.3 System Identification

### Calibrating Simulation to Reality

```python
from scipy.optimize import minimize

class SystemIdentification:
    def __init__(self, sim_env, real_data):
        self.sim_env = sim_env
        self.real_data = real_data  # Recorded trajectories from real robot
    
    def objective(self, params):
        """Minimize difference between sim and real trajectories."""
        # Unpack parameters
        mass_scale, friction, joint_damping = params[:3]
        motor_params = params[3:]
        
        # Apply parameters to simulation
        self.sim_env.set_mass_scale(mass_scale)
        self.sim_env.set_friction(friction)
        self.sim_env.set_joint_damping(joint_damping)
        self.sim_env.set_motor_params(motor_params)
        
        # Replay trajectory in simulation
        total_error = 0
        for real_traj in self.real_data:
            sim_traj = self.sim_env.replay_actions(real_traj['actions'])
            
            # Compare states
            pos_error = np.mean((sim_traj['pos'] - real_traj['pos'])**2)
            vel_error = np.mean((sim_traj['vel'] - real_traj['vel'])**2)
            
            total_error += pos_error + 0.1 * vel_error
        
        return total_error
    
    def identify(self):
        """Run optimization to find best parameters."""
        # Initial guess
        x0 = [1.0, 0.8, 1.0] + [1.0] * self.sim_env.num_motors
        
        # Bounds
        bounds = [(0.5, 1.5)] * 3 + [(0.5, 1.5)] * self.sim_env.num_motors
        
        result = minimize(
            self.objective,
            x0,
            method='L-BFGS-B',
            bounds=bounds
        )
        
        return result.x
```

### Parameter Estimation from Data

```python
def estimate_motor_model(joint_data):
    """Estimate motor dynamics from recorded data."""
    from sklearn.linear_model import Ridge
    
    # Features: [position, velocity, commanded_torque]
    X = np.column_stack([
        joint_data['position'],
        joint_data['velocity'],
        joint_data['commanded_torque']
    ])
    
    # Target: actual torque (or acceleration)
    y = joint_data['measured_torque']
    
    # Fit linear model: τ_actual = α*τ_cmd + β*q̇ + γ
    model = Ridge(alpha=0.1)
    model.fit(X, y)
    
    return {
        'torque_scale': model.coef_[2],
        'viscous_friction': model.coef_[1],
        'offset': model.intercept_
    }
```

## 20.4 Policy Architecture for Transfer

### Robust Architectures

```python
class TransferablePolicy(nn.Module):
    """Policy architecture designed for sim-to-real transfer."""
    
    def __init__(self, obs_dim, action_dim, hidden_dims=[256, 256]):
        super().__init__()
        
        # Observation normalization (learned from sim data)
        self.obs_mean = nn.Parameter(torch.zeros(obs_dim), requires_grad=False)
        self.obs_std = nn.Parameter(torch.ones(obs_dim), requires_grad=False)
        
        # Policy network with moderate capacity
        layers = []
        prev_dim = obs_dim
        for hidden_dim in hidden_dims:
            layers.append(nn.Linear(prev_dim, hidden_dim))
            layers.append(nn.LayerNorm(hidden_dim))
            layers.append(nn.ELU())
            prev_dim = hidden_dim
        
        self.backbone = nn.Sequential(*layers)
        self.action_head = nn.Linear(prev_dim, action_dim)
        
        # Action smoothing (low-pass filter)
        self.action_filter_alpha = 0.2
        self.prev_action = None
    
    def forward(self, obs):
        # Normalize observations
        obs_norm = (obs - self.obs_mean) / (self.obs_std + 1e-8)
        
        # Compute action
        features = self.backbone(obs_norm)
        action = self.action_head(features)
        
        # Apply tanh for bounded actions
        action = torch.tanh(action)
        
        # Low-pass filter for smoothness
        if self.prev_action is not None:
            action = self.action_filter_alpha * action + (1 - self.action_filter_alpha) * self.prev_action
        self.prev_action = action.detach()
        
        return action
    
    def set_normalization(self, mean, std):
        self.obs_mean.data = mean
        self.obs_std.data = std
```

### Adapters for Real-World

```python
class RealWorldAdapter(nn.Module):
    """Small adapter network fine-tuned on real data."""
    
    def __init__(self, base_policy, adapter_dim=32):
        super().__init__()
        self.base_policy = base_policy
        
        # Freeze base policy
        for param in self.base_policy.parameters():
            param.requires_grad = False
        
        # Small adapter layers
        self.adapter = nn.Sequential(
            nn.Linear(base_policy.obs_dim, adapter_dim),
            nn.ReLU(),
            nn.Linear(adapter_dim, base_policy.action_dim),
        )
    
    def forward(self, obs):
        base_action = self.base_policy(obs)
        correction = self.adapter(obs) * 0.1  # Small correction
        return base_action + correction
```

## 20.5 Deployment to Hardware

### ROS 2 Policy Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64MultiArray
import torch
import numpy as np

class PolicyDeploymentNode(Node):
    def __init__(self):
        super().__init__('policy_node')
        
        # Load trained policy
        self.policy = torch.jit.load('/path/to/policy.pt')
        self.policy.eval()
        
        # State buffers
        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)
        self.imu_data = np.zeros(6)  # gyro + accel
        self.prev_action = np.zeros(12)
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        
        # Publisher
        self.action_pub = self.create_publisher(
            Float64MultiArray, '/joint_commands', 10
        )
        
        # Control loop timer
        self.control_freq = 50.0  # Hz
        self.timer = self.create_timer(1.0 / self.control_freq, self.control_callback)
        
        self.get_logger().info('Policy deployment node started')
    
    def joint_callback(self, msg):
        self.joint_pos = np.array(msg.position)
        self.joint_vel = np.array(msg.velocity)
    
    def imu_callback(self, msg):
        self.imu_data = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
    
    def control_callback(self):
        # Build observation
        obs = self.build_observation()
        
        # Run policy inference
        with torch.no_grad():
            obs_tensor = torch.from_numpy(obs).float().unsqueeze(0)
            action = self.policy(obs_tensor).squeeze().numpy()
        
        # Apply action limits
        action = np.clip(action, -1.0, 1.0)
        
        # Scale to joint position targets
        action_scaled = action * 0.5  # Adjust scale as needed
        
        # Publish
        msg = Float64MultiArray()
        msg.data = action_scaled.tolist()
        self.action_pub.publish(msg)
        
        self.prev_action = action
    
    def build_observation(self):
        # Projected gravity (from IMU orientation)
        projected_gravity = self.compute_projected_gravity()
        
        # Assemble observation vector (must match training)
        obs = np.concatenate([
            self.joint_pos - self.default_joint_pos,  # Joint position offset
            self.joint_vel,                            # Joint velocities
            self.imu_data[:3],                        # Gyro
            projected_gravity,                         # Gravity projection
            self.prev_action,                          # Previous action
        ])
        
        return obs

def main():
    rclpy.init()
    node = PolicyDeploymentNode()
    rclpy.spin(node)
    rclpy.shutdown()
```

### Safety Monitors

```python
class SafetyMonitor:
    """Monitor robot state and stop if unsafe."""
    
    def __init__(self, node):
        self.node = node
        
        # Safety thresholds
        self.max_joint_velocity = 10.0  # rad/s
        self.max_joint_torque = 100.0  # N·m
        self.max_tilt_angle = 0.7  # rad (~40 degrees)
        self.min_height = 0.3  # m
        
        self.is_safe = True
    
    def check(self, joint_vel, joint_torque, orientation, height):
        """Check if robot is in safe state."""
        
        # Joint velocity limit
        if np.any(np.abs(joint_vel) > self.max_joint_velocity):
            self.trigger_stop("Joint velocity limit exceeded")
            return False
        
        # Joint torque limit
        if np.any(np.abs(joint_torque) > self.max_joint_torque):
            self.trigger_stop("Joint torque limit exceeded")
            return False
        
        # Tilt angle
        tilt = self.compute_tilt(orientation)
        if tilt > self.max_tilt_angle:
            self.trigger_stop(f"Tilt angle exceeded: {np.degrees(tilt):.1f}°")
            return False
        
        # Height
        if height < self.min_height:
            self.trigger_stop(f"Height too low: {height:.2f}m")
            return False
        
        return True
    
    def trigger_stop(self, reason):
        self.is_safe = False
        self.node.get_logger().error(f"Safety stop: {reason}")
        # Command zero torques / safe position
        self.node.emergency_stop()
```

## 20.6 Iterative Refinement

### Real-World Fine-Tuning

```python
class RealWorldFineTuner:
    def __init__(self, policy, buffer_size=10000):
        self.policy = policy
        self.buffer = ReplayBuffer(buffer_size)
        self.optimizer = torch.optim.Adam(policy.parameters(), lr=1e-4)
    
    def collect_experience(self, robot, num_steps=1000):
        """Collect real-world experience."""
        obs = robot.get_observation()
        
        for _ in range(num_steps):
            action = self.policy(obs)
            next_obs, reward = robot.step(action)
            
            self.buffer.add(obs, action, reward, next_obs)
            obs = next_obs
    
    def fine_tune(self, num_updates=100):
        """Fine-tune on collected data."""
        for _ in range(num_updates):
            batch = self.buffer.sample(256)
            
            # Behavior cloning on successful trajectories
            obs, actions, rewards, _ = batch
            
            predicted_actions = self.policy(obs)
            loss = F.mse_loss(predicted_actions, actions, reduction='none')
            
            # Weight by reward (focus on good actions)
            weights = F.softmax(rewards, dim=0)
            loss = (loss * weights).mean()
            
            self.optimizer.zero_grad()
            loss.backward()
            self.optimizer.step()
```

### Diagnostic Tools

```python
class TransferDiagnostics:
    """Tools for diagnosing sim-to-real issues."""
    
    def __init__(self):
        self.sim_data = []
        self.real_data = []
    
    def compare_distributions(self):
        """Compare state distributions between sim and real."""
        import matplotlib.pyplot as plt
        
        fig, axes = plt.subplots(3, 4, figsize=(16, 12))
        
        state_names = ['joint_pos', 'joint_vel', 'base_orient', 'base_vel']
        
        for i, name in enumerate(state_names):
            sim_states = np.array([d[name] for d in self.sim_data])
            real_states = np.array([d[name] for d in self.real_data])
            
            for j in range(min(3, sim_states.shape[1])):
                ax = axes[j, i]
                ax.hist(sim_states[:, j], bins=50, alpha=0.5, label='Sim', density=True)
                ax.hist(real_states[:, j], bins=50, alpha=0.5, label='Real', density=True)
                ax.set_title(f'{name}[{j}]')
                ax.legend()
        
        plt.tight_layout()
        plt.savefig('distribution_comparison.png')
    
    def analyze_failure_modes(self, failure_episodes):
        """Analyze what causes failures."""
        for episode in failure_episodes:
            # Find divergence point
            sim_traj = episode['sim_prediction']
            real_traj = episode['real_trajectory']
            
            divergence_idx = self.find_divergence_point(sim_traj, real_traj)
            
            # Analyze state at divergence
            state_at_divergence = real_traj[divergence_idx]
            
            print(f"Divergence at step {divergence_idx}")
            print(f"  Joint positions: {state_at_divergence['joint_pos']}")
            print(f"  Contact state: {state_at_divergence['contacts']}")
```

## 20.7 Summary

In this chapter, you learned:

- **Reality gap**: Sources and quantification
- **Domain randomization**: Physics, sensors, actuators
- **System identification**: Calibrating sim to match real
- **Policy architecture**: Designs that transfer well
- **Hardware deployment**: ROS 2 integration and safety
- **Iterative refinement**: Real-world fine-tuning

Successful sim-to-real transfer requires careful attention to the gap between simulation and reality. With proper randomization, calibration, and safety measures, policies trained in simulation can achieve robust performance on physical humanoid robots.

## Review Questions

1. What are the main sources of the reality gap?
2. How does domain randomization help with transfer?
3. What is system identification and when should it be used?
4. What safety measures are essential for real robot deployment?
5. How can you diagnose sim-to-real transfer failures?

## Hands-On Exercise

1. Implement domain randomization for mass and friction
2. Add sensor noise to your training environment
3. Record data from a real robot (or high-fidelity sim)
4. Perform system identification to calibrate simulation
5. Deploy a trained policy with safety monitoring
6. Compare sim vs real performance and iterate
