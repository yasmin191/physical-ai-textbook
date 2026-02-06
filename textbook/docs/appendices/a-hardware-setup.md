---
sidebar_position: 1
title: "Appendix A: Hardware Requirements & Lab Setup"
description: "Hardware specifications and lab environment setup guide"
---

# Appendix A: Hardware Requirements & Lab Setup

This appendix provides comprehensive hardware requirements and setup instructions for the Physical AI & Humanoid Robotics course lab environment.

## A.1 Workstation Requirements

### Development Workstation

| Component | Minimum | Recommended | Notes |
|-----------|---------|-------------|-------|
| CPU | Intel i7-10th gen / AMD Ryzen 7 | Intel i9 / AMD Ryzen 9 | Multi-core for simulation |
| RAM | 32 GB | 64 GB | Isaac Sim is memory-intensive |
| GPU | NVIDIA RTX 3070 | RTX 4080 / A6000 | CUDA compute essential |
| VRAM | 8 GB | 16+ GB | Required for Isaac Sim |
| Storage | 512 GB NVMe SSD | 1 TB NVMe SSD | Fast I/O for simulation |
| OS | Ubuntu 22.04 LTS | Ubuntu 22.04 LTS | ROS 2 Humble compatible |

### GPU Requirements by Task

| Task | Minimum GPU | VRAM |
|------|-------------|------|
| ROS 2 development | Integrated | - |
| Gazebo simulation | GTX 1660 | 6 GB |
| Isaac Sim | RTX 2070 | 8 GB |
| Isaac Lab (RL training) | RTX 3080 | 10 GB |
| Multi-robot simulation | RTX 4090 / A100 | 24 GB |

## A.2 Humanoid Robot Platforms

### Educational Platforms

| Platform | DOF | Height | Price Range | Use Case |
|----------|-----|--------|-------------|----------|
| Robotis OP3 | 20 | 51 cm | $10,000 | Research, education |
| Unitree H1 | 19 | 180 cm | $90,000 | Research |
| Agility Digit | 30+ | 155 cm | Research only | Industry research |
| Boston Dynamics Atlas | 28 | 150 cm | Not for sale | Reference |

### Simulation-Only Options

For courses without physical robots:
- NVIDIA Isaac Sim humanoid models
- Gazebo humanoid URDFs (free)
- MuJoCo humanoid models

## A.3 Sensor Equipment

### Cameras

| Type | Model | Resolution | FPS | Interface |
|------|-------|------------|-----|-----------|
| RGB | Logitech C920 | 1080p | 30 | USB |
| Depth | Intel RealSense D435i | 1280x720 | 90 | USB 3.0 |
| Stereo | ZED 2 | 2K | 100 | USB 3.0 |
| Tracking | Intel T265 | VGA | 200 Hz | USB 3.0 |

### LiDAR

| Model | Range | Points/sec | Price |
|-------|-------|------------|-------|
| Hokuyo UTM-30LX | 30m | 43,200 | $5,000 |
| RPLIDAR A3 | 25m | 16,000 | $600 |
| Velodyne VLP-16 | 100m | 300,000 | $4,000 |

### IMU

| Model | Axes | Interface | Price |
|-------|------|-----------|-------|
| Bosch BNO055 | 9-axis | I2C/UART | $30 |
| VectorNav VN-100 | 9-axis | RS-232/SPI | $1,000 |
| Xsens MTi-3 | 9-axis | USB | $500 |

## A.4 Compute Hardware

### Edge Compute

| Platform | CPU | GPU | Use Case |
|----------|-----|-----|----------|
| NVIDIA Jetson Orin Nano | 6-core ARM | 1024 CUDA | Entry-level |
| NVIDIA Jetson Orin NX | 8-core ARM | 2048 CUDA | Standard |
| NVIDIA Jetson AGX Orin | 12-core ARM | 2048 CUDA | Advanced |

### Development Kits

- **NVIDIA Jetson Orin Developer Kit**: $1,999
- **Intel NUC 12**: $700
- **Raspberry Pi 4 8GB**: $75 (auxiliary tasks only)

## A.5 Lab Environment Setup

### Physical Space

```
┌─────────────────────────────────────────────────────┐
│                    Robot Lab                         │
│                                                     │
│  ┌──────────┐                      ┌──────────┐    │
│  │ Charging │                      │ Storage  │    │
│  │ Station  │                      │          │    │
│  └──────────┘                      └──────────┘    │
│                                                     │
│           ┌─────────────────────┐                  │
│           │                     │                  │
│           │   Testing Arena     │                  │
│           │   (4m x 4m min)     │                  │
│           │                     │                  │
│           └─────────────────────┘                  │
│                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐         │
│  │ Workst.1 │  │ Workst.2 │  │ Workst.3 │         │
│  └──────────┘  └──────────┘  └──────────┘         │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### Safety Equipment

- Emergency stop buttons (multiple locations)
- Safety barriers for robot testing area
- First aid kit
- Fire extinguisher
- Safety glasses
- Anti-fatigue mats

### Network Setup

```
Internet ──► Router ──┬── Wired Network (Workstations)
                      │
                      ├── WiFi AP 1 (Robots - 5GHz)
                      │
                      └── WiFi AP 2 (General - 2.4GHz)
```

Recommended network configuration:
- Dedicated VLAN for robots
- Low-latency switches (< 1ms)
- QoS prioritization for robot traffic

## A.6 Software Environment

### Required Software

| Software | Version | Purpose |
|----------|---------|---------|
| Ubuntu | 22.04 LTS | Operating system |
| ROS 2 | Humble | Robot middleware |
| Gazebo | Fortress | Simulation |
| Isaac Sim | 2023.1+ | Advanced simulation |
| Python | 3.10 | Programming |
| PyTorch | 2.0+ | Machine learning |

### Development Tools

- VS Code with ROS extension
- Git for version control
- Docker for containerization
- Jupyter for experimentation

## A.7 Budget Planning

### Minimal Setup (Simulation Only)

| Item | Cost |
|------|------|
| Development Workstation | $3,000 |
| Monitors (2x) | $600 |
| Total | **$3,600** |

### Standard Lab Setup

| Item | Qty | Cost |
|------|-----|------|
| Workstations | 5 | $15,000 |
| Jetson Orin Kits | 2 | $4,000 |
| RealSense D435i | 3 | $1,000 |
| RPLIDAR A3 | 2 | $1,200 |
| Network Equipment | - | $1,000 |
| Safety Equipment | - | $500 |
| Total | | **$22,700** |

### Advanced Research Lab

| Item | Qty | Cost |
|------|-----|------|
| High-end Workstations | 10 | $50,000 |
| NVIDIA DGX Station | 1 | $50,000 |
| Humanoid Robot | 1 | $50,000+ |
| Motion Capture | 1 | $20,000 |
| Force Plates | 2 | $10,000 |
| Total | | **$180,000+** |

## A.8 Vendor Resources

### Hardware Vendors

- **NVIDIA**: [nvidia.com/en-us/deep-learning-ai/](https://nvidia.com)
- **Intel RealSense**: [intelrealsense.com](https://intelrealsense.com)
- **Robotis**: [robotis.com](https://robotis.com)
- **Unitree**: [unitree.com](https://unitree.com)

### Educational Discounts

Many vendors offer educational discounts:
- NVIDIA Academic Program
- Intel University Program
- Robotis Educational Partnership

Contact vendors directly with institutional credentials for pricing.
