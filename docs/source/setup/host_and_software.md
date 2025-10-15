# Host and Network Setup

This section covers the complete setup process for the common platform robot, including host configuration, software installation, and firmware programming.


## Pull the Latest Changes from GitHub

Change to the correct git-enabled directory:

```bash
cd repos/common_platform
```

Stash any changes you have made:

```bash
git stash
```

Update the Git remote for the public repo so you don't need credentials. You should only need to do this step once.

```bash
git remote set-url origin "https://github.com/RoseCityRobotics/common_platform.git"
```

Pull the changes

```bash
git pull origin main
```

Re-apply your stashed changes

```bash
git stash apply
```

## Host Settings
<img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20">
The host has been preset for you as `rcr00_` with your specific ID. To update the hostname:

1. Tell cloud-init to preserve your hostname:
   ```bash
   sudo nano /etc/cloud/cloud.cfg
   ```
   Set `preserve_hostname: true` and save the file

2. Update the hostname for the robot:
   ```bash
   sudo nano /etc/hostname
   ```
   Change `rcr001` to `rcr00n` (replace n with the number)

3. Update the hosts file:
   ```bash
   sudo nano /etc/hosts
   ```
   Change `rcr001` to `rcr00n`

4. Set the hostname:
   ```bash
   sudo hostnamectl set-hostname rcr00n
   ```

5. Reboot to make sure the changes have taken effect:
   ```bash
   sudo reboot
   ```

## Network Configuration
<img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20">

In this section we will set a custom IP address, set up networking and add WiFi access points. The Pi will try access points in order from top to bottom until it is successful connecting.

Edit the netplan configuration:
```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Add the following configuration (replace `n` with your robot number):
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
**Note:** `n` = the robot number. Add your home WiFi access point as well.

Apply the network configuration:
```bash
sudo netplan apply
```

## SSH Connection
**🖥️ On the development computer:** Connect via SSH:
```bash
ssh rcr@192.168.1.n
```
Replace `n` with your robot number.

### Accessing the Pi from your development machine via SSH
<img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> 🖥️
Both the external device and the robot must be on the same WiFi for SSH to work. Use the static IP address of your Pi to SSH. The command will look something like `ssh rcr@192.168.1.n` where `n` is your ID, for example `ssh rcr@192.168.1.9`.

### Troubleshooting SSH Connections
**<img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> On the Pi:** Make sure the IP address is assigned:
```bash
hostname -I
```

Check the host name
```bash
hostname
```

Ping a website to verify network connectivity
```bash
ping google.com
```

Make sure your development machine 🖥️ is on the same network.

## Robot Namespacing Setup

Since we are running a swarm of many robots we need the ROS nodes and topics to be namespaced. To ensure your robot runs under its unique namespace (e.g., `/rcr001`, `/rcr002`, etc.), follow these steps:

### Set Your Namespace in `.profile`

Edit your shell profile to define your robot's namespace:

```bash
sudo nano ~/.profile
```

Add or update this line (replace `n` with your robot number):

```bash
export ROS_NAME=rcr00n
```

Save and close the file. This ensures that your environment variables are set on login.

---

### Set Namespace in `env.list`

Update the environment variable file used for Docker and ROS:

```bash
sudo nano ~/env.list
```

Add or update the line:

```bash
ROS_NAMESPACE=/rcr00n
```

---

### Set Firmware Namespace

In your firmware directory, update the namespace inside the `ros_interface.cpp` file:

```bash
cd ~/repos/common_platform/firmware/closed_loop/
sudo nano RosInterface.cpp
```

Find this line where the node is initialized with a namespace (default should either be empty `""` or `rcr001`). Change it to `rcr00n`:

```cpp
rclc_node_init_default(&node, "micro_ros_arduino_node", "rcr00n", &support));
```

This sets the micro-ROS node namespace properly for communication with ROS2.

### Reload the Firmware
⚠️ **Important:** After updating this firmware, you must re-flash the Teensy. For detailed instructions on flashing the Teensy firmware, see the [Teensy Programming Guide](teensy_programming.md).

<p align="right">(<a href="#readme-top">back to top</a>)</p>
