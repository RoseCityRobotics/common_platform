# Welcome to Rose City Robotics Common Platform Documentation

Welcome to the comprehensive documentation for the RCR Common Robotics Platform! This documentation provides everything you need to build, operate, and maintain your robotics system.

> 🔗 **GitHub Repository**: [https://github.com/RoseCityRobotics/common_platform](https://github.com/RoseCityRobotics/common_platform) - Source code, hardware designs, and issue tracking

## Quick Start

### Initial Setup

- **[Host Configuration](setup/pi_connection.md)** - Connect to your robot's Raspberry Pi
- **[Network Configuration](setup/host_and_software.md#network-configuration)** - Configure WiFi and network settings
- **[Software Setup](setup/index.md)** - Install ROS2 and configure the development environment
- **[Flash Teensy Firmware](/operations/advanced/firmware_updates.md.md)** - Program the robot's microcontroller

```{toctree}
:maxdepth: 2
:caption: Getting Started

index
intro
setup/index
operations/advanced/firmware_updates
operations/keyboard_teleoperation
lidar_operations
slam/index
camera/camera_operations
data_annotation/label_studio
operations/ros2_joystick
hardware
3dprint
```

```{toctree}
:maxdepth: 1
:caption: Robot Operations

operations/startup_procedure
operations/shutdown_procedure
operations/reboot_procedure
operations/emergency_shutdown
operations/maintenance
```

```{toctree}
:maxdepth: 1
:caption: Testing and Validation

testing
contributing
```

## Getting Help

If you need assistance or have questions:

1. Check the [Operations Manual](operations/index) for step-by-step guides
2. Review the troubleshooting sections
3. Visit our [GitHub repository](https://github.com/RoseCityRobotics/common_platform) for source code and issue tracking
4. Contact the [RCR team](https://rosecityrobotics.com/contact) for additional support

---

*Last updated: October 2025*
*Version: 1.0*
