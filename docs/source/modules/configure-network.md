---
type: module
slug: configure-network
title: Configure network settings on the Raspberry Pi
description: Define a static IP address and Wi-Fi credentials using Netplan.
tags: [setup, network, wifi]
device: pi
---

# Configure network settings on the Raspberry Pi

{{pi}} **On the Raspberry Pi**

For reliable communication between your development computer and the robot, assign a static IP address and configure Wi-Fi credentials. The Pi uses Netplan for network management.

## Edit Netplan configuration

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Adjust the file to specify your network settings. For example:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      optional: true
      dhcp4: no
      addresses:
        - 192.168.1.n/24
      routes:
        - to: default
          via: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]
      access-points:
        "robot_overlord_wifi":
          password: "siliconforest"
        "RoseCityRobotics":
          password: "QW260go80.."
```

:::{important}
- Replace `n` in the IP address with your robot number (e.g., `192.168.1.10` for robot 10)
- Configure multiple access points if your robot moves between different Wi-Fi networks
- Add your home WiFi access point as well
:::

Save the file and exit the editor.

## Apply network configuration

```bash
sudo netplan apply
```

## Verify network connectivity

```bash
hostname -I
hostname
ping -c 3 google.com
```

:::{note}
- `hostname -I` prints your new static IP
- `hostname` shows the hostname you configured
- `ping google.com` should succeed, confirming internet access
:::

