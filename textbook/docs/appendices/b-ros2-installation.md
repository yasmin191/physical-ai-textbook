---
sidebar_position: 2
title: "Appendix B: Ubuntu & ROS 2 Installation"
description: "Step-by-step installation guide for Ubuntu 22.04 and ROS 2 Humble"
---

# Appendix B: Ubuntu 22.04 & ROS 2 Installation

This appendix provides detailed installation instructions for setting up the development environment.

## B.1 Ubuntu 22.04 Installation

### System Requirements

- 64-bit processor
- 4 GB RAM minimum (8 GB recommended)
- 50 GB free disk space
- USB drive (8 GB+) for installation media

### Creating Installation Media

```bash
# Download Ubuntu 22.04 LTS
wget https://releases.ubuntu.com/22.04/ubuntu-22.04.3-desktop-amd64.iso

# Create bootable USB (Linux)
sudo dd bs=4M if=ubuntu-22.04.3-desktop-amd64.iso of=/dev/sdX status=progress oflag=sync

# Or use Rufus on Windows / balenaEtcher on macOS
```

### Installation Steps

1. Boot from USB
2. Select "Install Ubuntu"
3. Choose language and keyboard
4. Select "Normal installation"
5. Enable "Install third-party software"
6. Choose installation type (alongside/replace)
7. Set timezone, username, password
8. Complete installation and reboot

### Post-Installation Setup

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install essential tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    htop \
    python3-pip \
    python3-venv

# Install NVIDIA drivers (if applicable)
sudo ubuntu-drivers autoinstall
sudo reboot
```

## B.2 ROS 2 Humble Installation

### Set Locale

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### Add ROS 2 Repository

```bash
# Enable Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS 2 Humble

```bash
# Update package index
sudo apt update

# Install ROS 2 (Desktop version - recommended)
sudo apt install ros-humble-desktop -y

# Or minimal installation
# sudo apt install ros-humble-ros-base

# Install development tools
sudo apt install ros-dev-tools -y
```

### Environment Setup

```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
```

## B.3 Essential ROS 2 Packages

### Navigation Stack

```bash
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
```

### Simulation

```bash
# Gazebo Fortress
sudo apt install ros-humble-ros-gz -y

# ros2_control
sudo apt install ros-humble-ros2-control -y
sudo apt install ros-humble-ros2-controllers -y

# Robot description
sudo apt install ros-humble-robot-state-publisher -y
sudo apt install ros-humble-joint-state-publisher-gui -y
sudo apt install ros-humble-xacro -y
```

### Perception

```bash
# Image processing
sudo apt install ros-humble-image-transport -y
sudo apt install ros-humble-cv-bridge -y
sudo apt install ros-humble-vision-opencv -y

# Point cloud
sudo apt install ros-humble-pcl-ros -y
```

### Visualization

```bash
sudo apt install ros-humble-rviz2 -y
sudo apt install ros-humble-rqt* -y
```

## B.4 Python Environment

### Virtual Environment

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Create virtual environment
python3 -m venv venv --system-site-packages
source venv/bin/activate

# Install Python packages
pip install --upgrade pip
pip install numpy scipy matplotlib
pip install opencv-python
pip install torch torchvision
pip install transformers
pip install openai
```

### Colcon Build System

```bash
# Install colcon
sudo apt install python3-colcon-common-extensions -y

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## B.5 Verification

### Test ROS 2 Installation

```bash
# Terminal 1: Run talker
ros2 run demo_nodes_cpp talker

# Terminal 2: Run listener
ros2 run demo_nodes_cpp listener
```

### Test Gazebo

```bash
# Launch empty world
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:=empty.sdf
```

### Test RViz2

```bash
rviz2
```

## B.6 Troubleshooting

### Common Issues

**Issue: "ros2: command not found"**
```bash
source /opt/ros/humble/setup.bash
```

**Issue: "Package not found"**
```bash
sudo apt update
sudo apt install ros-humble-<package-name>
```

**Issue: NVIDIA driver issues**
```bash
# Remove existing drivers
sudo apt purge nvidia-*

# Reinstall
sudo ubuntu-drivers autoinstall
sudo reboot
```

**Issue: Permission denied for USB devices**
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

## B.7 Docker Alternative

### Using ROS 2 Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Pull ROS 2 Humble image
docker pull ros:humble

# Run container
docker run -it --rm \
    --network host \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e DISPLAY=$DISPLAY \
    ros:humble \
    bash
```

### Docker Compose for Development

```yaml
# docker-compose.yml
version: '3'
services:
  ros2:
    image: ros:humble
    network_mode: host
    volumes:
      - ./src:/root/ros2_ws/src
      - /tmp/.X11-unix:/tmp/.X11-unix
    environment:
      - DISPLAY=${DISPLAY}
    command: bash
```
