---
sidebar_position: 6
title: "Chapter 19: Reinforcement Learning for Robotics"
description: "Train humanoid locomotion and manipulation policies using reinforcement learning"
---

# Chapter 19: Reinforcement Learning for Robotics

**Reinforcement Learning (RL)** has revolutionized humanoid robot control, enabling robots to learn complex behaviors like walking, running, and manipulation through trial and error. This chapter covers the fundamentals of RL and its application to humanoid robotics using NVIDIA Isaac Lab.

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the RL framework for robotics
- Design reward functions for humanoid tasks
- Configure parallel training environments
- Train locomotion and manipulation policies
- Evaluate and debug trained policies

## 19.1 Reinforcement Learning Fundamentals

### The RL Framework

```
┌─────────────────────────────────────────────────────────┐
│               Reinforcement Learning                     │
│                                                         │
│    ┌─────────┐        action aₜ        ┌─────────────┐  │
│    │  Agent  │ ──────────────────────► │ Environment │  │
│    │ (Policy)│                         │  (Robot +   │  │
│    │   πθ    │ ◄────────────────────── │   World)    │  │
│    └─────────┘   state sₜ, reward rₜ   └─────────────┘  │
│                                                         │
│    Goal: Maximize expected cumulative reward            │
│          E[Σ γᵗ rₜ]                                     │
└─────────────────────────────────────────────────────────┘
```

### Key Components

| Component | Description | Example |
|-----------|-------------|---------|
| **State (s)** | Observation of environment | Joint positions, IMU |
| **Action (a)** | Control output | Joint torques |
| **Reward (r)** | Feedback signal | Forward velocity |
| **Policy (π)** | State → Action mapping | Neural network |
| **Value (V)** | Expected future reward | Critic network |

### Common RL Algorithms

| Algorithm | Type | Characteristics |
|-----------|------|-----------------|
| PPO | On-policy | Stable, sample efficient |
| SAC | Off-policy | Maximum entropy, robust |
| TD3 | Off-policy | Deterministic, twin critics |
| DDPG | Off-policy | Continuous actions |

## 19.2 Isaac Lab for RL Training

### Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    Isaac Lab                             │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────┐    │
│  │         Parallel Environments (GPU)              │    │
│  │  ┌────┐ ┌────┐ ┌────┐ ┌────┐      ┌────┐        │    │
│  │  │ E1 │ │ E2 │ │ E3 │ │ E4 │ ...  │ EN │        │    │
│  │  └────┘ └────┘ └────┘ └────┘      └────┘        │    │
│  │              (N = 4096+)                        │    │
│  └─────────────────────────────────────────────────┘    │
│                         │                               │
│              ┌──────────▼──────────┐                    │
│              │    PhysX on GPU     │                    │
│              └──────────┬──────────┘                    │
│                         │                               │
│              ┌──────────▼──────────┐                    │
│              │   RL Algorithm      │                    │
│              │   (RSL_RL / SB3)    │                    │
│              └─────────────────────┘                    │
└─────────────────────────────────────────────────────────┘
```

### Installation

```bash
# Clone Isaac Lab
git clone https://github.com/NVIDIA-Omniverse/IsaacLab.git
cd IsaacLab

# Create conda environment
./isaaclab.sh --conda

# Install RL dependencies
./isaaclab.sh --install rsl_rl
./isaaclab.sh --install sb3
```

### Environment Configuration

```python
from omni.isaac.lab.envs import DirectRLEnv, DirectRLEnvCfg
from omni.isaac.lab.scene import InteractiveSceneCfg
from omni.isaac.lab.assets import ArticulationCfg
import omni.isaac.lab.sim as sim_utils

@configclass
class HumanoidEnvCfg(DirectRLEnvCfg):
    # Simulation
    sim: sim_utils.SimulationCfg = sim_utils.SimulationCfg(
        dt=0.005,
        render_interval=4,
    )
    
    # Scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(
        num_envs=4096,
        env_spacing=2.5,
    )
    
    # Robot
    robot: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
        spawn=sim_utils.UsdFileCfg(usd_path="humanoid.usd"),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.0, 1.0),
            joint_pos={".*": 0.0},
        ),
        actuators={
            "legs": ImplicitActuatorCfg(
                joint_names_expr=[".*_hip_.*", ".*_knee_.*", ".*_ankle_.*"],
                stiffness=100.0,
                damping=10.0,
            ),
        },
    )
    
    # Episode
    episode_length_s = 20.0
    
    # Observations and actions
    num_observations = 48
    num_actions = 12
```

## 19.3 Designing Reward Functions

### Reward Components for Walking

```python
class HumanoidEnv(DirectRLEnv):
    def _compute_rewards(self):
        # Get robot state
        root_lin_vel = self.robot.data.root_lin_vel_w
        root_ang_vel = self.robot.data.root_ang_vel_w
        joint_torques = self.robot.data.applied_torque
        joint_vel = self.robot.data.joint_vel
        
        # Forward velocity reward
        forward_vel = root_lin_vel[:, 0]  # x-direction
        reward_forward = forward_vel * self.cfg.reward_forward_scale
        
        # Upright reward (penalize tilting)
        root_quat = self.robot.data.root_quat_w
        up_vec = quat_rotate(root_quat, torch.tensor([0, 0, 1]))
        upright = up_vec[:, 2]  # z-component of up vector
        reward_upright = upright * self.cfg.reward_upright_scale
        
        # Alive bonus
        base_height = self.robot.data.root_pos_w[:, 2]
        is_alive = (base_height > 0.5).float()
        reward_alive = is_alive * self.cfg.reward_alive_scale
        
        # Energy penalty (smooth motion)
        energy = torch.sum(torch.abs(joint_torques * joint_vel), dim=-1)
        reward_energy = -energy * self.cfg.reward_energy_scale
        
        # Action rate penalty (smooth actions)
        action_diff = self.actions - self.prev_actions
        action_rate = torch.sum(action_diff ** 2, dim=-1)
        reward_action_rate = -action_rate * self.cfg.reward_action_rate_scale
        
        # Joint limit penalty
        joint_pos = self.robot.data.joint_pos
        lower_limit = self.robot.data.soft_joint_pos_limits[:, :, 0]
        upper_limit = self.robot.data.soft_joint_pos_limits[:, :, 1]
        
        at_lower = (joint_pos < lower_limit + 0.1).float()
        at_upper = (joint_pos > upper_limit - 0.1).float()
        joint_limit_penalty = torch.sum(at_lower + at_upper, dim=-1)
        reward_limits = -joint_limit_penalty * self.cfg.reward_limits_scale
        
        # Total reward
        reward = (
            reward_forward +
            reward_upright +
            reward_alive +
            reward_energy +
            reward_action_rate +
            reward_limits
        )
        
        return reward
```

### Curriculum Learning

```python
class CurriculumManager:
    def __init__(self, cfg):
        self.current_level = 0
        self.levels = [
            {"terrain_roughness": 0.0, "push_force": 0.0},
            {"terrain_roughness": 0.02, "push_force": 50.0},
            {"terrain_roughness": 0.05, "push_force": 100.0},
            {"terrain_roughness": 0.1, "push_force": 200.0},
        ]
    
    def update(self, success_rate):
        if success_rate > 0.8 and self.current_level < len(self.levels) - 1:
            self.current_level += 1
            return True
        return False
    
    def get_params(self):
        return self.levels[self.current_level]
```

### Reward Shaping Tips

| Goal | Reward Component | Weight |
|------|------------------|--------|
| Move forward | Forward velocity | 1.0 |
| Stay upright | Orientation penalty | 0.5 |
| Stay alive | Height threshold | 2.0 |
| Smooth motion | Energy penalty | 0.01 |
| Smooth actions | Action rate penalty | 0.1 |
| Respect limits | Joint limit penalty | 0.2 |

## 19.4 Observation and Action Spaces

### Observations for Locomotion

```python
def _get_observations(self):
    # Base state (in body frame)
    base_lin_vel = quat_rotate_inverse(
        self.robot.data.root_quat_w,
        self.robot.data.root_lin_vel_w
    )
    base_ang_vel = quat_rotate_inverse(
        self.robot.data.root_quat_w,
        self.robot.data.root_ang_vel_w
    )
    
    # Projected gravity (tells robot which way is down)
    gravity = torch.tensor([0, 0, -1], device=self.device)
    projected_gravity = quat_rotate_inverse(
        self.robot.data.root_quat_w,
        gravity.expand(self.num_envs, -1)
    )
    
    # Joint states
    joint_pos = self.robot.data.joint_pos - self.default_joint_pos
    joint_vel = self.robot.data.joint_vel
    
    # Previous actions (for smooth control)
    prev_actions = self.prev_actions
    
    # Command (desired velocity)
    commands = self.commands  # [vx, vy, omega_z]
    
    obs = torch.cat([
        base_lin_vel,           # 3
        base_ang_vel,           # 3
        projected_gravity,      # 3
        commands,               # 3
        joint_pos,              # num_joints
        joint_vel,              # num_joints
        prev_actions,           # num_actions
    ], dim=-1)
    
    return {"policy": obs}
```

### Actions (Joint Targets)

```python
def _apply_actions(self):
    # Actions are position targets relative to default
    # Scale actions to joint range
    scaled_actions = self.actions * self.cfg.action_scale
    
    # Add to default pose
    target_pos = self.default_joint_pos + scaled_actions
    
    # Apply via PD control
    self.robot.set_joint_position_target(target_pos)
```

### Noise and Domain Randomization

```python
def _add_observation_noise(self, obs):
    noise = torch.randn_like(obs) * self.cfg.obs_noise_scale
    return obs + noise

def _randomize_domain(self):
    # Randomize physics
    if self.cfg.randomize_friction:
        friction = torch.rand(self.num_envs) * 0.5 + 0.5  # [0.5, 1.0]
        self.scene.set_friction(friction)
    
    # Randomize robot properties
    if self.cfg.randomize_mass:
        mass_scale = torch.rand(self.num_envs) * 0.2 + 0.9  # [0.9, 1.1]
        self.robot.scale_mass(mass_scale)
    
    # Randomize actuator properties
    if self.cfg.randomize_actuators:
        kp_scale = torch.rand(self.num_envs) * 0.2 + 0.9
        kd_scale = torch.rand(self.num_envs) * 0.2 + 0.9
        self.robot.set_actuator_gains(kp_scale, kd_scale)
```

## 19.5 Training Configuration

### PPO Configuration

```python
from rsl_rl.runners import OnPolicyRunner
from rsl_rl.algorithms import PPO

@configclass
class PPORunnerCfg:
    # Algorithm
    algorithm = PPO
    
    # Policy network
    policy_class_name = "ActorCritic"
    actor_hidden_dims = [512, 256, 128]
    critic_hidden_dims = [512, 256, 128]
    activation = "elu"
    
    # PPO parameters
    learning_rate = 1e-3
    num_learning_epochs = 5
    num_mini_batches = 4
    gamma = 0.99
    lam = 0.95
    entropy_coef = 0.01
    clip_param = 0.2
    value_loss_coef = 1.0
    max_grad_norm = 1.0
    
    # Training
    num_steps_per_env = 24
    max_iterations = 3000
    save_interval = 100
    
    # Logging
    log_dir = "logs/humanoid_walk"
    experiment_name = "ppo_walk"
```

### Training Script

```python
import torch
from omni.isaac.lab.envs import ManagerBasedRLEnv
from rsl_rl.runners import OnPolicyRunner

def train():
    # Create environment
    env_cfg = HumanoidEnvCfg()
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Create runner
    runner_cfg = PPORunnerCfg()
    runner = OnPolicyRunner(env, runner_cfg, device="cuda")
    
    # Train
    runner.learn(num_learning_iterations=runner_cfg.max_iterations)
    
    # Save final policy
    runner.save(runner_cfg.log_dir)
    
    env.close()

if __name__ == "__main__":
    train()
```

### Running Training

```bash
# Launch training
python scripts/rsl_rl/train.py --task Humanoid-Walk-v0 --num_envs 4096

# Monitor with TensorBoard
tensorboard --logdir logs/

# Resume training
python scripts/rsl_rl/train.py --task Humanoid-Walk-v0 --resume --checkpoint logs/model_1000.pt
```

## 19.6 Policy Evaluation

### Evaluation Script

```python
def evaluate(checkpoint_path, num_episodes=100):
    # Load environment (fewer environments for evaluation)
    env_cfg = HumanoidEnvCfg()
    env_cfg.scene.num_envs = 16
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Load policy
    policy = torch.jit.load(checkpoint_path)
    policy.eval()
    
    # Evaluate
    total_rewards = []
    episode_lengths = []
    
    obs = env.reset()
    
    for _ in range(num_episodes):
        done = False
        episode_reward = 0
        episode_length = 0
        
        while not done:
            with torch.no_grad():
                actions = policy(obs)
            
            obs, reward, done, info = env.step(actions)
            episode_reward += reward.mean().item()
            episode_length += 1
        
        total_rewards.append(episode_reward)
        episode_lengths.append(episode_length)
    
    print(f"Mean reward: {np.mean(total_rewards):.2f} ± {np.std(total_rewards):.2f}")
    print(f"Mean length: {np.mean(episode_lengths):.2f}")
    
    env.close()
```

### Visualization

```python
def visualize(checkpoint_path):
    # Create environment with rendering
    env_cfg = HumanoidEnvCfg()
    env_cfg.scene.num_envs = 1
    env_cfg.sim.render_interval = 1
    env = ManagerBasedRLEnv(cfg=env_cfg)
    
    # Load policy
    policy = torch.jit.load(checkpoint_path)
    policy.eval()
    
    obs = env.reset()
    
    while True:
        with torch.no_grad():
            actions = policy(obs)
        
        obs, reward, done, info = env.step(actions)
        
        if done:
            obs = env.reset()
```

## 19.7 Advanced Techniques

### Asymmetric Actor-Critic

Use privileged information during training:

```python
class AsymmetricActorCritic(nn.Module):
    def __init__(self, obs_dim, privileged_dim, action_dim):
        super().__init__()
        
        # Actor only sees regular observations
        self.actor = nn.Sequential(
            nn.Linear(obs_dim, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, action_dim),
        )
        
        # Critic sees privileged information too
        self.critic = nn.Sequential(
            nn.Linear(obs_dim + privileged_dim, 256),
            nn.ELU(),
            nn.Linear(256, 128),
            nn.ELU(),
            nn.Linear(128, 1),
        )
    
    def forward(self, obs):
        return self.actor(obs)
    
    def evaluate(self, obs, privileged_obs):
        combined = torch.cat([obs, privileged_obs], dim=-1)
        return self.critic(combined)
```

### Imitation Learning Pretraining

```python
def pretrain_from_demonstrations(policy, demo_dataset, epochs=100):
    optimizer = torch.optim.Adam(policy.parameters(), lr=1e-3)
    criterion = nn.MSELoss()
    
    for epoch in range(epochs):
        for obs, expert_actions in demo_dataset:
            predicted_actions = policy(obs)
            loss = criterion(predicted_actions, expert_actions)
            
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        
        print(f"Epoch {epoch}: Loss = {loss.item():.4f}")
```

### Multi-Task Learning

```python
class MultiTaskEnv(DirectRLEnv):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.tasks = ["walk", "run", "turn", "stand"]
        self.current_task = torch.zeros(self.num_envs, dtype=torch.long)
    
    def _get_observations(self):
        # Regular observations
        obs = self._get_base_observations()
        
        # Task embedding
        task_one_hot = F.one_hot(self.current_task, len(self.tasks))
        
        return {"policy": torch.cat([obs, task_one_hot], dim=-1)}
    
    def _compute_rewards(self):
        rewards = torch.zeros(self.num_envs, device=self.device)
        
        for i, task in enumerate(self.tasks):
            mask = self.current_task == i
            if task == "walk":
                rewards[mask] = self._walk_reward()[mask]
            elif task == "run":
                rewards[mask] = self._run_reward()[mask]
            elif task == "turn":
                rewards[mask] = self._turn_reward()[mask]
            elif task == "stand":
                rewards[mask] = self._stand_reward()[mask]
        
        return rewards
```

## 19.8 Debugging and Troubleshooting

### Common Issues

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| No learning | Reward too sparse | Add shaping rewards |
| Unstable training | Learning rate too high | Reduce LR, increase batch |
| Robot falls immediately | Bad initialization | Adjust init pose |
| Reward hacking | Missing penalties | Add constraints |
| Sim-to-real gap | No randomization | Add domain randomization |

### Debugging Tools

```python
def debug_rewards(env, policy):
    """Visualize reward components."""
    import matplotlib.pyplot as plt
    
    obs = env.reset()
    reward_components = []
    
    for _ in range(100):
        actions = policy(obs)
        obs, reward, done, info = env.step(actions)
        
        # Log individual reward components
        reward_components.append({
            'forward': info['reward_forward'].mean().item(),
            'upright': info['reward_upright'].mean().item(),
            'energy': info['reward_energy'].mean().item(),
        })
    
    # Plot
    for key in reward_components[0].keys():
        values = [r[key] for r in reward_components]
        plt.plot(values, label=key)
    
    plt.legend()
    plt.show()
```

## 19.9 Summary

In this chapter, you learned:

- **RL fundamentals**: States, actions, rewards, policies
- **Isaac Lab**: Parallel GPU environments for training
- **Reward design**: Components and shaping for locomotion
- **Observations**: Proprioceptive and exteroceptive inputs
- **Training**: PPO configuration and execution
- **Evaluation**: Testing and visualizing policies
- **Advanced techniques**: Asymmetric AC, imitation, multi-task

RL enables humanoid robots to learn complex behaviors that are difficult to hand-engineer. In the next chapter, we'll cover sim-to-real transfer.

## Review Questions

1. What are the key components of an RL problem?
2. Why is parallel simulation important for RL training?
3. What reward components encourage smooth walking?
4. How does domain randomization help sim-to-real transfer?
5. What is the purpose of curriculum learning?

## Hands-On Exercise

1. Set up Isaac Lab with a humanoid robot
2. Design a reward function for walking forward
3. Configure PPO training parameters
4. Train a walking policy for 1000 iterations
5. Evaluate the trained policy
6. Add domain randomization and compare results
