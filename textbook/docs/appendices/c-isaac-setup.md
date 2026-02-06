---
sidebar_position: 3
title: "Appendix C: NVIDIA Isaac Sim Setup"
description: "Installation and configuration guide for NVIDIA Isaac Sim"
---

# Appendix C: NVIDIA Isaac Sim Setup

This appendix covers the installation and configuration of NVIDIA Isaac Sim for robotics simulation.

## C.1 System Requirements

### Hardware Requirements

| Component | Minimum | Recommended |
|-----------|---------|-------------|
| GPU | GeForce RTX 2070 | RTX 4080 / A6000 |
| VRAM | 8 GB | 16+ GB |
| CPU | Intel i7 / AMD Ryzen 7 | i9 / Ryzen 9 |
| RAM | 32 GB | 64 GB |
| Storage | 50 GB SSD | 100 GB NVMe |

### Supported GPUs

- NVIDIA GeForce RTX 20/30/40 series
- NVIDIA Quadro RTX series
- NVIDIA RTX A series
- NVIDIA A100, H100 (datacenter)

## C.2 Installation Methods

### Method 1: Omniverse Launcher (Recommended)

```bash
# 1. Download Omniverse Launcher
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# 2. Make executable
chmod +x omniverse-launcher-linux.AppImage

# 3. Run launcher
./omniverse-launcher-linux.AppImage

# 4. In Launcher: Exchange → Isaac Sim → Install
```

### Method 2: Docker Container

```bash
# Login to NGC
docker login nvcr.io
# Username: $oauthtoken
# Password: <your NGC API key>

# Pull Isaac Sim container
docker pull nvcr.io/nvidia/isaac-sim:2023.1.1

# Run container
docker run --gpus all -it --rm \
    -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache \
    -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov \
    -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip \
    -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache \
    -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache \
    -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs \
    -v ~/docker/isaac-sim/data:/root/.local/share/ov/data \
    -v ~/docker/isaac-sim/documents:/root/Documents \
    --network host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    nvcr.io/nvidia/isaac-sim:2023.1.1
```

### Method 3: pip Install (Isaac Lab)

```bash
# Create conda environment
conda create -n isaaclab python=3.10 -y
conda activate isaaclab

# Install Isaac Lab
pip install isaacsim-rl isaacsim-robot isaacsim-sensor
```

## C.3 Configuration

### Environment Variables

```bash
# Add to ~/.bashrc
export ISAACSIM_PATH="${HOME}/.local/share/ov/pkg/isaac_sim-2023.1.1"
export ISAACSIM_PYTHON="${ISAACSIM_PATH}/python.sh"

# For ROS 2 integration
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

### First Run Setup

```bash
# Launch Isaac Sim
cd $ISAACSIM_PATH
./isaac-sim.sh

# Or headless mode
./isaac-sim.sh --headless
```

### Nucleus Setup

1. Launch Omniverse Launcher
2. Go to Nucleus tab
3. Add local server or connect to cloud
4. Configure paths for assets

## C.4 ROS 2 Bridge Configuration

### Enable ROS 2 Extension

```python
# In Isaac Sim script
from omni.isaac.core.utils.extensions import enable_extension

# Enable ROS 2 bridge
enable_extension("omni.isaac.ros2_bridge")
```

### Bridge Launch

```bash
# Start ROS 2 bridge in Isaac Sim
# Edit → Extensions → ros2_bridge → Enable

# Or via command line
./isaac-sim.sh --ext-folder exts --enable omni.isaac.ros2_bridge
```

### Topic Configuration

```python
# Python script for topic setup
import omni.graph.core as og

og.Controller.edit(
    {"graph_path": "/World/ROS2Graph"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        ],
    }
)
```

## C.5 Performance Optimization

### Graphics Settings

```python
# Reduce rendering quality for performance
import carb.settings

settings = carb.settings.get_settings()
settings.set("/rtx/rendermode", "PathTracing")
settings.set("/rtx/pathtracing/spp", 1)  # Samples per pixel
settings.set("/rtx/post/aa/op", 0)  # Disable anti-aliasing
```

### Physics Settings

```python
from omni.isaac.core import World

# Create world with custom physics settings
world = World(
    physics_dt=1.0/500.0,  # 500 Hz physics
    rendering_dt=1.0/60.0,  # 60 Hz rendering
    stage_units_in_meters=1.0
)
```

### GPU Memory Management

```bash
# Set CUDA memory limit
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:512

# In Isaac Sim settings
# Adjust texture streaming memory
# Window → Settings → Rendering → Texture Streaming
```

## C.6 Troubleshooting

### Common Issues

**Issue: "Failed to create CUDA context"**
```bash
# Check NVIDIA driver
nvidia-smi

# Reinstall driver if needed
sudo apt install nvidia-driver-525
```

**Issue: "Omniverse Nucleus connection failed"**
```bash
# Check firewall
sudo ufw allow 3080
sudo ufw allow 3100
```

**Issue: "Import error for omni modules"**
```bash
# Use Isaac Sim's Python
${ISAACSIM_PATH}/python.sh your_script.py
```

**Issue: "ROS 2 bridge not publishing"**
```bash
# Check ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Verify DDS implementation matches
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## C.7 Standalone Scripts

### Basic Simulation Script

```python
#!/usr/bin/env python3
"""Basic Isaac Sim standalone script."""

from omni.isaac.kit import SimulationApp

# Launch simulation
config = {
    "headless": False,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(config)

# Import after SimulationApp
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid

# Create world
world = World()
world.scene.add_default_ground_plane()

# Add object
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        size=0.1,
        color=np.array([1.0, 0.0, 0.0]),
    )
)

# Initialize
world.reset()

# Run simulation
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

### Running Scripts

```bash
# Using Isaac Sim's Python
${ISAACSIM_PATH}/python.sh my_script.py

# Or from within Isaac Sim
# Window → Script Editor → Load and Run
```
