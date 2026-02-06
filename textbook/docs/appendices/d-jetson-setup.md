# Appendix D: Jetson Orin Development Kit Setup

This appendix provides comprehensive instructions for setting up the NVIDIA Jetson Orin development kit for humanoid robotics applications.

## Learning Objectives

By the end of this appendix, you will be able to:
- Set up the Jetson Orin hardware for robotics development
- Install JetPack SDK and configure the development environment
- Deploy ROS 2 and Isaac ROS on Jetson
- Optimize performance for real-time robotics applications

## D.1 Jetson Orin Overview

### D.1.1 Hardware Variants

NVIDIA offers several Jetson Orin variants:

| Module | GPU Cores | CPU | Memory | AI Performance |
|--------|-----------|-----|--------|----------------|
| Orin Nano | 1024 | 6-core Arm | 4-8 GB | 40 TOPS |
| Orin NX | 1024 | 8-core Arm | 8-16 GB | 70-100 TOPS |
| AGX Orin | 2048 | 12-core Arm | 32-64 GB | 200-275 TOPS |

### D.1.2 Developer Kit Contents

The AGX Orin Developer Kit includes:
- Jetson AGX Orin module
- Reference carrier board
- 65W power supply
- USB-C cable for flashing
- Quick start guide

### D.1.3 Additional Hardware Needed

For robotics development, you'll also need:
- MicroSD card (64GB+ recommended)
- USB keyboard and mouse
- HDMI or DisplayPort monitor
- Ethernet cable or WiFi antenna
- USB hub (optional)

## D.2 Initial Hardware Setup

### D.2.1 Physical Assembly

**Step 1: Unpack and Inspect**

```
1. Remove developer kit from packaging
2. Verify all components are present
3. Check for any visible damage
4. Place on anti-static surface
```

**Step 2: Connect Peripherals**

```
                    ┌─────────────────────────────────────┐
                    │        Jetson AGX Orin              │
                    │                                     │
    Power ────────►│█                                   █│◄──── HDMI/DP
                    │                                     │
    USB Hub ──────►│████                             ████│◄──── Ethernet
                    │                                     │
    USB-C ────────►│█                                   █│◄──── Camera (CSI)
    (Flash)         └─────────────────────────────────────┘
```

**Step 3: Power Configuration**

For development, use the included power supply:

```bash
# The developer kit supports multiple power modes
# Default: MAXN (maximum performance)
# Options: 15W, 30W, 50W for power-constrained deployments
```

### D.2.2 First Boot

1. Connect power supply (no switch, powers on automatically)
2. Wait for NVIDIA logo on display
3. Follow Ubuntu setup wizard:
   - Accept license agreement
   - Select language and keyboard
   - Create user account
   - Configure network

## D.3 JetPack SDK Installation

### D.3.1 SDK Manager Method (Recommended)

**On Host Machine (Ubuntu 20.04/22.04):**

```bash
# Download SDK Manager from NVIDIA Developer
# https://developer.nvidia.com/sdk-manager

# Install SDK Manager
sudo dpkg -i sdkmanager_[version]_amd64.deb
sudo apt-get install -f

# Launch SDK Manager
sdkmanager
```

**SDK Manager Workflow:**

```
Step 1: Login
├── NVIDIA Developer account required
└── Accept license agreements

Step 2: Select Target
├── Hardware: Jetson AGX Orin Developer Kit
├── JetPack Version: 6.0 (or latest)
└── Target OS: Linux (Jetson Linux)

Step 3: Select Components
├── Jetson OS (required)
├── Jetson SDK Components
│   ├── CUDA Toolkit
│   ├── cuDNN
│   ├── TensorRT
│   ├── VPI (Vision Programming Interface)
│   ├── Multimedia API
│   └── Developer Tools
└── Additional SDKs (optional)

Step 4: Flash and Install
├── Put Jetson in recovery mode
├── Flash OS image
└── Install SDK components
```

**Recovery Mode:**

```bash
# To enter recovery mode:
# 1. Power off the Jetson
# 2. Hold the middle button (Force Recovery)
# 3. While holding, press power button
# 4. Release both after 2 seconds

# Verify on host machine:
lsusb | grep -i nvidia
# Should show: NVIDIA Corp. APX
```

### D.3.2 Manual Installation Method

If SDK Manager is unavailable:

```bash
# On Jetson (after initial Ubuntu setup)

# Add NVIDIA apt repository
sudo apt-get update
sudo apt-get install -y software-properties-common

# Install JetPack components
sudo apt-get install -y nvidia-jetpack

# This installs:
# - CUDA toolkit
# - cuDNN
# - TensorRT
# - VPI
# - Multimedia API
```

### D.3.3 Verify Installation

```bash
# Check JetPack version
cat /etc/nv_tegra_release
# or
dpkg -l | grep nvidia-jetpack

# Check CUDA
nvcc --version

# Check cuDNN
cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2

# Check TensorRT
dpkg -l | grep tensorrt

# Run CUDA samples
cd /usr/local/cuda/samples/1_Utilities/deviceQuery
sudo make
./deviceQuery
```

## D.4 ROS 2 Installation on Jetson

### D.4.1 ROS 2 Humble Installation

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install ros-dev-tools -y

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### D.4.2 Workspace Setup

```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Initialize workspace
colcon build --symlink-install

# Source workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## D.5 Isaac ROS on Jetson

### D.5.1 Prerequisites

```bash
# Install Docker
sudo apt-get update
sudo apt-get install -y ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
    sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker

# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
    sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### D.5.2 Isaac ROS Installation

```bash
# Clone Isaac ROS common
mkdir -p ~/workspaces/isaac_ros-dev/src
cd ~/workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git

# Set up Isaac ROS
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh

# Inside container, build Isaac ROS packages
cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

### D.5.3 Isaac ROS Packages for Humanoids

```bash
# Clone relevant packages
cd ~/workspaces/isaac_ros-dev/src

# Visual SLAM
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git

# DNN Inference
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_dnn_inference.git

# Image Processing
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

# AprilTag Detection
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git

# Build all packages
cd ~/workspaces/isaac_ros-dev
colcon build --symlink-install
```

## D.6 Camera Setup

### D.6.1 CSI Camera Configuration

```bash
# List available cameras
ls /dev/video*

# Check camera capabilities
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext

# Test camera with GStreamer
gst-launch-1.0 nvarguscamerasrc ! \
    'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
    nvvidconv ! xvimagesink
```

### D.6.2 RealSense Camera Setup

```bash
# Install librealsense
sudo apt-key adv --keyserver keyserver.ubuntu.com \
    --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository \
    "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-utils

# Test RealSense
realsense-viewer

# Install ROS 2 wrapper
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/ros2_ws
colcon build --symlink-install
```

### D.6.3 ZED Camera Setup

```bash
# Download ZED SDK from Stereolabs website
# https://www.stereolabs.com/developers/release

chmod +x ZED_SDK_*.run
./ZED_SDK_*.run

# Install ROS 2 wrapper
cd ~/ros2_ws/src
git clone https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

## D.7 Performance Optimization

### D.7.1 Power Modes

```bash
# List available power modes
sudo nvpmodel -q --verbose

# Set maximum performance mode
sudo nvpmodel -m 0

# Set power-efficient mode (for battery operation)
sudo nvpmodel -m 2

# Check current mode
nvpmodel -q
```

### D.7.2 Clock Configuration

```bash
# Maximize clocks for development
sudo jetson_clocks

# Show current clock frequencies
sudo jetson_clocks --show

# Store current settings
sudo jetson_clocks --store

# Restore saved settings
sudo jetson_clocks --restore
```

### D.7.3 Memory Optimization

```bash
# Check memory usage
free -h

# Configure swap
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Optimize for robotics (reduce GUI memory)
# Consider running headless for deployment
sudo systemctl set-default multi-user.target
```

### D.7.4 Real-Time Configuration

```bash
# Install RT kernel (if available)
sudo apt-get install nvidia-l4t-rt-kernel

# Configure kernel parameters
sudo nano /etc/sysctl.conf
# Add:
# vm.swappiness=10
# kernel.sched_rt_runtime_us=-1

# Apply changes
sudo sysctl -p

# Set process priorities
sudo chrt -f 99 <ros_process>
```

## D.8 Sensor Integration

### D.8.1 IMU Setup

```bash
# Common IMU: BNO055
# Connect via I2C

# Enable I2C
sudo usermod -aG i2c $USER

# Scan I2C bus
i2cdetect -y -r 1

# Install IMU ROS driver
cd ~/ros2_ws/src
git clone https://github.com/flynneva/bno055.git
cd ~/ros2_ws
colcon build --packages-select bno055
```

### D.8.2 LiDAR Integration

```bash
# Example: RPLIDAR
cd ~/ros2_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git -b ros2

# Set USB permissions
sudo cp rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Build and run
cd ~/ros2_ws
colcon build --packages-select rplidar_ros
ros2 launch rplidar_ros rplidar_a1_launch.py
```

### D.8.3 Motor Controllers

```bash
# CAN bus setup for motor controllers
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# Monitor CAN bus
candump can0
```

## D.9 Deployment Considerations

### D.9.1 Creating Deployment Image

```bash
# On host machine with SDK Manager
# Use "Clone" feature to create backup image

# Or manually:
# 1. Boot Jetson in recovery mode
# 2. On host:
cd ~/nvidia/nvidia_sdk/JetPack_6.0_Linux_JETSON_AGX_ORIN_TARGETS/Linux_for_Tegra
sudo ./flash.sh -r -k APP -G backup.img jetson-agx-orin-devkit mmcblk0p1
```

### D.9.2 Headless Operation

```bash
# Disable GUI
sudo systemctl set-default multi-user.target

# Enable SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# Configure static IP
sudo nano /etc/netplan/01-network-manager-all.yaml
```

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

### D.9.3 Startup Services

```bash
# Create systemd service for ROS nodes
sudo nano /etc/systemd/system/humanoid_robot.service
```

```ini
[Unit]
Description=Humanoid Robot ROS 2 Nodes
After=network.target

[Service]
Type=simple
User=nvidia
Environment="ROS_DOMAIN_ID=0"
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && \
    source /home/nvidia/ros2_ws/install/setup.bash && \
    ros2 launch humanoid_bringup robot.launch.py"
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

```bash
# Enable service
sudo systemctl daemon-reload
sudo systemctl enable humanoid_robot
sudo systemctl start humanoid_robot
```

## D.10 Troubleshooting

### Common Issues and Solutions

| Issue | Cause | Solution |
|-------|-------|----------|
| Jetson won't boot | Power supply insufficient | Use official 65W adapter |
| Camera not detected | CSI ribbon loose | Reseat ribbon cable firmly |
| CUDA out of memory | Large model | Reduce batch size, use TensorRT |
| USB devices disconnecting | Power draw | Use powered USB hub |
| Network unstable | WiFi interference | Use Ethernet for robots |

### D.10.1 Diagnostic Commands

```bash
# Check system health
tegrastats

# Monitor GPU
nvidia-smi

# Check thermal status
cat /sys/devices/virtual/thermal/thermal_zone*/temp

# View kernel messages
dmesg | tail -50

# Check ROS 2 nodes
ros2 node list
ros2 topic list
ros2 doctor
```

### D.10.2 Recovery Procedures

```bash
# Factory reset via SDK Manager
# 1. Put Jetson in recovery mode
# 2. Launch SDK Manager
# 3. Select "Flash" without "Skip"

# If SD card corrupted:
# 1. Remove SD card
# 2. Flash new image on host:
sudo dd if=jetson_image.img of=/dev/sdX bs=4M status=progress
```

## Summary

Setting up the Jetson Orin for humanoid robotics involves:

1. **Hardware Setup**: Proper assembly and peripheral connections
2. **JetPack Installation**: Full SDK with CUDA, cuDNN, TensorRT
3. **ROS 2 Configuration**: Humble installation with workspace setup
4. **Isaac ROS**: GPU-accelerated perception packages
5. **Sensor Integration**: Cameras, IMUs, LiDAR, motor controllers
6. **Performance Tuning**: Power modes, clocks, memory optimization
7. **Deployment**: Headless operation with automatic startup

The Jetson Orin provides the computational power necessary for real-time perception, planning, and control in humanoid robots, bridging the gap between simulation and real-world deployment.

## Quick Reference Card

```bash
# Essential commands
nvpmodel -q                    # Check power mode
sudo jetson_clocks             # Max performance
tegrastats                     # System monitor
ros2 doctor                    # ROS 2 diagnostics

# Service management
sudo systemctl status humanoid_robot
sudo journalctl -u humanoid_robot -f
```

## Additional Resources

- [NVIDIA Jetson Documentation](https://docs.nvidia.com/jetson/)
- [JetPack SDK](https://developer.nvidia.com/embedded/jetpack)
- [Isaac ROS Documentation](https://nvidia-isaac-ros.github.io/)
- [Jetson Developer Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)
