---
type: module
slug: configure-host-settings
title: Configure host settings on the Raspberry Pi
description: Set the hostname for your robot to uniquely identify it on the network.
tags: [setup, hostname, config]
device: pi
---

# Configure host settings on the Raspberry Pi

{{pi}} **On the Raspberry Pi**

The host has been preset for you as `rcr00n` with your specific ID in place of "n". To update the hostname:

## Tell cloud-init to preserve your hostname

```bash
sudo nano /etc/cloud/cloud.cfg
```

Set `preserve_hostname: true` and save the file

## Update the hostname for the robot

```bash
sudo nano /etc/hostname
```

Change `rcr001` to `rcr00n` (replace n with your robot number)

## Update the hosts file

```bash
sudo nano /etc/hosts
```

Change `rcr001` to `rcr00n`

## Set the hostname

```bash
sudo hostnamectl set-hostname rcr00n
```

## Reboot to apply changes

```bash
sudo reboot
```

:::{note}
After the reboot, your robot will be identifiable as `rcr00n` on the network.
:::

