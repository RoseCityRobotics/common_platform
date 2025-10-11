# Raspberry Pi Connection Guide

This guide covers how to connect to your robot's Raspberry Pi for development and operation.

## Direct Connection (Initial Setup)

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

## SSH Connection (Development)

**Prerequisites:**
- Robot and development computer on the same network
- SSH client installed on your development computer

**Connection Steps:**

<img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> on the Raspberry Pi

**Find your robot's IP address:**
  ```bash
  # From the robot's direct connection, check IP
  ip addr show
  ```

🖥️ On your development computer open a new terminal window 📟

**Connect via SSH:**
  ```bash
  ssh rcr@192.168.1.n
  ```
  Replace `n` with your robot's assigned number.

**Alternative connection if IP is known:**
  ```bash
  ssh rcr@<robot_ip_address>
  ```
