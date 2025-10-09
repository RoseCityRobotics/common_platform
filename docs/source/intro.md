# Introduction

Welcome to the [Rose City Robotics](https://rosecityrobotics.com) documentation for our fork of the RCR Common Robotics Platform! This project builds on the outstanding work of the PARTS team—many thanks to them for their contributions and for making this platform possible. Here you'll find an introduction to our implementation of the Romi Chassis, a versatile and compact robotic platform, along with details on how we've customized it for our robotics education and development needs.

> 📦 **Source Code**: All code, hardware designs, and documentation for this project are available in our [GitHub repository](https://github.com/RoseCityRobotics/common_platform)

## What is the Romi Chassis?

The Romi Chassis is a compact, versatile robotic platform designed for educational and prototyping applications. Built around a robust frame, it features:

- **Dual DC Motors**: Direct-drive motors with built-in encoders for precise movement control
- **Omnidirectional Capability**: Two wheels and one caster enable smooth movement in any direction (second caster recommended for heavier sensors)
- **Modular Design**: Easy-to-access mounting points for sensors, cameras, and custom electronics
- **Battery Powered**: Rechargeable lithium batteries can be added for extended operation
- **Arduino Compatible**: Built-in carrier board with Arduino-compatible headers for easy programming
- **Expandable**: Multiple I/O pins and mounting options for adding sensors and other peripherals

The Romi Chassis is perfect for learning robotics concepts, prototyping autonomous systems, and developing navigation algorithms. Its compact size (approximately 6" diameter) makes it ideal for indoor testing environments.

## About Rose City Robotics' Implementation

This is Rose City Robotics' fork of the RCR Common Robotics Platform, customized for our specific educational and development needs.

- **Project Goals**: Provide hands-on robotics education, enable rapid prototyping of autonomous systems, and create a standardized platform for our robotics programs and competitions.
- **Custom Features**: Our implementation includes ROS2 integration, custom carrier board modifications, onboard AI inference and enhanced sensor capabilities for advanced navigation and perception tasks.

## Getting Started

Ready to start working with our robotics platform? Here's how to begin:

- **Prerequisites**: Basic understanding of robotics concepts, familiarity with Linux command line, and access to the required hardware components.
- **RCR Foundations**: Check out our [RCR Foundations repository](https://github.com/RoseCityRobotics/rcr-foundations) for fundamental robotics concepts and tutorials.
- **Initial Setup**: Follow our [Hardware Setup](hardware) guide to assemble your robot, then proceed to [Software Setup](software) for the development environment configuration.

## Project Components

Our robotics platform consists of several key components:

- **Hardware Components**: Romi Chassis base, custom carrier board, Raspberry Pi, Teensy microcontroller, LiDAR sensor, camera, IMU, and various mounting hardware. See our [Hardware Guide](hardware) for detailed specifications.
- **Software Stack**: ROS2 Kilted, micro-ROS for Teensy communication, Python, and custom packages for navigation and control. See our [Software Guide](software) for setup instructions.

## How to Use This Documentation

This documentation is organized to help you get up and running quickly:

- **Navigation**: Use the sidebar to browse different sections, or start with the [Quick Start](index) section for an overview of available guides.
- **Getting Started**: New users should begin with the [Introduction](intro) (this page), then follow the [Hardware Setup](hardware) and [Software Setup](software) guides.
- **Daily Operations**: Once your robot is set up, check out our [Robot Operations](operations/index) section for day-to-day usage instructions.

## Contact and Support

Need help or want to contribute to the project?

- **🔗 GitHub Repository**: [https://github.com/RoseCityRobotics/common_platform](https://github.com/RoseCityRobotics/common_platform)
- **Support Channels**: [GitHub Issues](https://github.com/RoseCityRobotics/common_platform/issues) for bug reports and feature requests
- **Community**: Join our discussions and get help from the community
- **Contributing**: We welcome contributions! Check our [GitHub repository](https://github.com/RoseCityRobotics/common_platform) for guidelines on submitting pull requests and improvements

## Conclusion

You're now ready to explore the Rose City Robotics RCR Common Robotics Platform! This documentation will guide you through setup, operation, and advanced usage of our robotics platform. Start with the [Hardware Setup](hardware) guide to begin building your robot, and don't hesitate to reach out if you need assistance along the way.
