# Virtual Machine Setup

If you're developing for the common platform robot on Windows or macOS, you'll need to set up a virtual machine (VM) running Ubuntu 24.04. This is necessary because ROS2 runs natively on Ubuntu, and while you can SSH into the robot's Raspberry Pi for many tasks, having a local development environment with RViz and other ROS2 tools is essential for SLAM visualization and real time camera feeds.

There are two recommended approaches:
- **Windows**: Using Windows Subsystem for Linux (WSL)
- **macOS**: Using Parallels Desktop

Both methods will allow you to run Ubuntu 24.04 with full ROS2 Kilted Desktop installation, including RViz for 3D visualization and all the tools you need for robot development.

## Windows: WSL Setup

Windows Subsystem for Linux (WSL) is Microsoft's solution for running a full Linux environment directly on Windows. It provides excellent performance and seamless integration with Windows, making it ideal for ROS2 development.

### Windows Firewall Configuration

**Important:** Windows Defender Firewall can block ROS2 communication between WSL and Windows. If you experience connectivity issues with ROS2 nodes or RViz, you may need to temporarily disable the firewall.

#### Check Firewall Status

Open **PowerShell as Administrator** and run:

```powershell
Get-NetFirewallProfile | Format-Table Name, Enabled
```

This will show which firewall profiles are enabled.

#### Temporarily Disable Firewall (if needed)

If any profiles show `Enabled = True` and you're experiencing ROS2 connectivity issues, disable them:

```powershell
Set-NetFirewallProfile -Profile Domain,Public,Private -Enabled False
```

**⚠️ Security Warning:** Remember to re-enable the firewall after testing. We are working on creating specific firewall rules that allow ROS2 traffic without disabling the entire firewall.

### Installing WSL with Ubuntu 24.04

**Open PowerShell as Administrator** and install WSL with Ubuntu 24.04:
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

**Restart your computer** when prompted.

**Launch Ubuntu** from the Start menu and create your user account when prompted.

**Update your system packages**:
   ```bash
   sudo apt update
   ```

## macOS: Parallels Setup

Parallels Desktop is a powerful virtualization solution for macOS that provides excellent performance for running Ubuntu alongside macOS.

### Installing Parallels and Ubuntu

**Download and install Parallels Desktop** from [parallels.com](https://www.parallels.com)

**Create a new virtual machine**:
   - Open Parallels Desktop
   - Click "+" to create a new VM
   - Select "Install Windows or another OS from a DVD or image file"
   - Download Ubuntu 24.04 LTS Desktop ISO from [ubuntu.com](https://ubuntu.com/download/desktop)
   - Follow the installation wizard to complete setup

**Configure VM resources** (recommended settings):
   - Memory: At least 4GB RAM (8GB recommended)
   - Processors: 2-4 CPU cores
   - Disk space: 40GB or more
   - Graphics: Enable 3D acceleration for better RViz performance

**Install Parallels Tools** for better integration:
   - In Parallels menu, select "Install Parallels Tools"
   - Follow the on-screen instructions
   - Restart the VM when complete

### Optimizing Performance

For the best ROS2 and RViz experience on Parallels:
- Enable "Use Mac GPU for Windows" in VM configuration
- Set graphics to "Better performance" mode
- Allocate sufficient RAM for visualization tasks

## Alternative Options

While WSL and Parallels are our recommended solutions, other virtualization options include:
- **VirtualBox**: Free and open-source, works on both Windows and macOS
- **VMware**: Commercial virtualization software with good performance
- **UTM**: Free virtualization for macOS (especially for Apple Silicon Macs)

**Note:** These alternatives are not officially documented or tested with this platform, but may work for your needs.

## Installing ROS2 Kilted

Once you have Ubuntu 24.04 running (via WSL, Parallels, or another method), you'll need to install ROS2 Kilted Desktop, which includes RViz and all the visualization tools you'll need.

### ROS2 Kilted Desktop Installation

Follow the official ROS2 installation instructions for Ubuntu:

**🔗 Installation Guide**: [https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

### Verifying Your Installation

Check for ROS2
   ```bash
   which ros2
   ```
Should return `/opt/ros/kilted/bin/ros2`

Test RViz
   ```bash
   rviz2
   ```
This should open up RViz tool GUI, if it does you can close this for now and continue setting up the config.

### After Installing ROS

On the VM you will need to set up a few configuration files and environment variables.

#### Edit the .profile
```bash
sudo nano ~/.profile
```

Add the following to the bottom of the .profile, then save and exit the file
```
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SUPER_CLIENT=true
export FASTDDS_DEFAULT_PROFILES_FILE=/home/${USER}/ros2_ws/super_client_configuration_file_rcr.xml
source /home/${USER}/ros2_ws/install/setup.bash
ros2 daemon stop; ros2 daemon start
```
source .profile so the changes take effect in your current session
```bash
source .profile
```

#### Set up the Discovery Server

Now that you have ROS2 on your VM, you will need to point it to the proper discovery server so it can communicate with your robot via ROS. To do this, first create the directory if it doesn't already exist:

```bash
mkdir ~/ros2_ws
```
Then edit or create the discovery server config file referenced in the environment variable above.

```bash
nano ~/ros2_ws/super_client_configuration_file_rcr.xml
```

The config file should look something like this, if not paste the code below into your config file.

```xml
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <participant profile_name="super_client_profile" is_default_profile="true">
    <rtps>
      <builtin>
        <discovery_config>
          <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
          <discoveryServersList>
            <locator>
              <udpv4>
                <address>127.0.0.1</address>
                <port>11811</port>
              </udpv4>
            </locator>
          </discoveryServersList>
        </discovery_config>
      </builtin>
    </rtps>
  </participant>
</profiles>
```
Edit line 11 to set the proper discovery server IP address. If you are running the discovery server on your own Pi, then it should work as is. If you are at the RCR lab, change it to Joe's IP which is `192.168.1.125`. If neither work, figure out what IP address you can ping the discovery server at and use that. See more **[troubleshooting tips here](#troubleshooting-vm-connectivity-to-discovery-server)**.

Save the file above, then stop and start the ros2 daemon

```bash
ros2 daemon stop
ros2 daemon start
```

Testing - check for ROS topics
```bash
ros2 topic list
```

You should now see the full list of ROS topics publishing from the discovery server.

### Troubleshooting VM connectivity to discovery server

If you're having trouble connecting your VM to the discovery server, follow these troubleshooting steps:

#### Network Configuration

Your VM must be configured to use the same network and IP address as your host computer. This requires specific network settings:

**For WSL (Windows):** Configure mirrored networking so your VM shares the same IP address as Windows
- 📖 [WSL Networking Documentation](https://learn.microsoft.com/en-us/windows/wsl/networking)

**For Parallels (macOS):** Set up bridged networking mode
- 📖 [Parallels Network Settings Documentation](https://docs.parallels.com/pdfm-ug-20/parallels-desktop-for-mac-20-users-guide/parallels-desktop-preferences-and-virtual-machine-settings/virtual-machine-settings/hardware-settings/network-settings)

#### Connectivity Tests

Test internet connectivity:
```bash
ping google.com
```

Test connection to your robot (replace `n` with your robot's IP):
```bash
ping 192.168.1.n
```

Test connection to the discovery server:
```bash
ping 192.168.1.125 # for Joe's computer
```

#### Other Common Issues:
- check firewalls - see [Firewall Configuration](#windows-firewall-configuration)
- check your IP address on both your laptop and your VM - they should match. If not, see

## Troubleshooting Graphics Issues

If you encounter graphics or rendering issues when running RViz or other GUI applications in your VM, you may need to enable software rendering.

### Enable Software Rendering

Set this environment variable to force software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

**When to use this:**
- RViz crashes on startup with OpenGL errors
- You see graphics driver warnings
- 3D visualization is not working properly
- Running in a VM without proper GPU acceleration

**Note:** Software rendering will be slower than hardware acceleration, but it ensures compatibility across different VM configurations.

## Next Steps

Now that you have your development environment set up, you can:

- [Visualize sensor data with RViz](rviz_sensor_visualization.md)
- [Set up real-time camera feed](realtime_camera_video.md)
