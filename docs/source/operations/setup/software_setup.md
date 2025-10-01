# Software Setup Guide

This guide covers the software installation and configuration for the PARTS Common Robotics Platform.

## Prerequisites

- Hardware assembly completed
- Computer with Ubuntu 22.04 or compatible system
- Internet connection for downloads
- Basic familiarity with Linux command line

## ROS2 Installation

### 1. Install ROS2 Humble

```bash
# Add ROS2 repository
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 packages
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop python3-argcomplete python3-colcon-common-extensions python3-rosdep python3-vcstool
```

### 2. Environment Setup

```bash
# Source ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Initialize rosdep
sudo rosdep init
rosdep update
```

## Project Setup

### 1. Clone Repository

```bash
# Clone the project
git clone <repository-url>
cd common_platform

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Build the Workspace

```bash
# Build the workspace
colcon build

# Source the workspace
echo "source ~/common_platform/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Configuration

### 1. Robot Parameters

Edit configuration files in the `config/` directory:

```bash
# Main robot configuration
nano config/ball_tracker_params_robot.yaml

# Navigation parameters
nano config/nav2_params.yaml
```

### 2. Launch Files

Test launch files:

```bash
# Test robot state publisher
ros2 launch common_platform robot_state_publisher.launch.py

# Test simulation
ros2 launch common_platform launch_sim.launch.py
```

## Verification

### 1. Check Installation

```bash
# Verify ROS2 installation
ros2 --version

# Check workspace
colcon list

# Test nodes
ros2 node list
```

### 2. Test Communication

```bash
# Launch robot
ros2 launch common_platform launch_robot.launch.py

# In another terminal, check topics
ros2 topic list
```

## Next Steps

After completing software setup:

1. Proceed to [Calibration](calibration.md)
2. Run initial system tests
3. Review [Daily Operations](../daily_operations/index.md)

---

*For hardware setup, see [Hardware Setup](hardware_setup.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
