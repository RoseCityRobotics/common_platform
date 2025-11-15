# Raspberry Pi Setup

This procedure walks you through the complete setup of your Raspberry Pi robot, from initial connection to configuring the robot's namespace.

## Overview

You'll complete the following steps:
1. Connect to the Raspberry Pi via SSH
2. Pull the latest code from GitHub
3. Configure host settings
4. Set up network configuration
5. Configure ROS namespacing

---

## Step 1: Connect to the Raspberry Pi

```{include} ../modules/connect-to-raspberry-pi.md
:start-after: "# Connect to Raspberry Pi via SSH"
```

---

## Step 2: Pull Latest Updates

```{include} ../modules/pull-common-platform-updates.md
:start-after: "# Pull the latest changes from GitHub"
```

---

## Step 3: Configure Host Settings

```{include} ../modules/configure-host-settings.md
:start-after: "# Configure host settings on the Raspberry Pi"
```

---

## Step 4: Configure Network

```{include} ../modules/configure-network.md
:start-after: "# Configure network settings on the Raspberry Pi"
```

---

## Step 5: Set ROS Namespacing

```{include} ../modules/set-ros-namespacing.md
:start-after: "# Set ROS namespace for your robot"
```

---

## Next Steps

After completing this setup:
- [Flash the Teensy firmware](firmware-and-teleop.md)
- [Start keyboard teleoperation](keyboard-teleoperation.md)
- [Begin SLAM mapping](slam-mapping.md)

