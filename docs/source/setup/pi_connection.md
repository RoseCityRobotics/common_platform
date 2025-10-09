# Raspberry Pi Connection Guide

This guide covers how to connect to your robot's Raspberry Pi for development and operation.

## Connection Methods

### Direct Connection (Recommended for Initial Setup)

**Hardware Required:**
- Micro HDMI cable
- USB keyboard
- Monitor with HDMI input
- Power supply for the robot

**Steps:**
- Connect the micro HDMI cable from the Raspberry Pi to your monitor
- Connect a USB keyboard to the Raspberry Pi's USB-A port
- Power on the robot
- Wait for the system to boot

**Login Credentials:**
- **Username:** `rcr`
- **Password:** `siliconforest`

### SSH Connection (Recommended for Development)

**Prerequisites:**
- Robot and development computer on the same network
- SSH client installed on your development computer

**Connection Steps:**
- **Find your robot's IP address:**
  ```bash
  # From the robot's direct connection, check IP
  ip addr show
  ```

- **Connect via SSH:**
  ```bash
  ssh rcr@192.168.1.n
  ```
  Replace `n` with your robot's assigned number.

- **Alternative connection if IP is known:**
  ```bash
  ssh rcr@<robot_ip_address>
  ```

## Network Configuration

### Lab Environment
If using the robot at the Rose City Robotics lab:
- The robot will automatically connect to the lab WiFi
- Discovery server is already configured
- No additional network setup required

### Home/Office Environment
For use outside the lab:
- **Configure WiFi connection** (see [Network Configuration](host_and_software.md#network-configuration))
- **Set up discovery server** (see [Discovery Server Setup](../operations/discovery_server.md))

## Verification

### Test SSH Connection
```bash
# From your development computer
ssh rcr@192.168.1.n "echo 'Connection successful'"
```

### Test ROS2 Communication
```bash
# On the robot
ros2 topic list

# From your development computer (if on same network)
ros2 topic list
```

After establishing connection:
- [Software Setup](host_and_software.md) - Install and configure ROS2
- [Teensy Programming](teensy_programming.md) - Flash robot firmware
- [Testing & Validation](../testing.md) - Verify system functionality
