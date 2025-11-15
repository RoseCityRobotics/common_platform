---
type: module
slug: connect-to-raspberry-pi
title: Connect to Raspberry Pi via SSH
description: SSH into your robot's Raspberry Pi from your development computer.
tags: [setup, ssh, connection]
device: pc
---

# Connect to Raspberry Pi via SSH

{{pc}} **On the development computer**

Both the external device and the robot must be on the same WiFi for SSH to work. Use the static IP address of your Pi to SSH.

## Connect via SSH

```bash
ssh rcr@192.168.1.n
```

Replace `n` with your robot number (e.g., `ssh rcr@192.168.1.9` for robot 9).

## Troubleshooting SSH Connections

If you can't connect, try these steps:

### On the Raspberry Pi

{{pi}} Make sure the IP address is assigned:

```bash
hostname -I
```

Check the host name:

```bash
hostname
```

Ping a website to verify network connectivity:

```bash
ping google.com
```

### On the development computer

{{pc}} Make sure your development machine is on the same network as the robot.

:::{tip}
If you still can't connect, verify the IP address is correct and check that both devices are on the same WiFi network.
:::

