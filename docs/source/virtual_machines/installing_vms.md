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

1. **Open PowerShell as Administrator** and install WSL with Ubuntu 24.04:
   ```powershell
   wsl --install -d Ubuntu-24.04
   ```

2. **Restart your computer** when prompted.

3. **Launch Ubuntu** from the Start menu and create your user account when prompted.

4. **Update your system packages**:
   ```bash
   sudo apt update
   ```

### Installing GUI Support for WSL

To run graphical applications like RViz in WSL, you'll need to enable GUI support:

1. **Ensure you're running WSL 2** (required for GUI apps):
   ```bash
   wsl --set-version Ubuntu-24.04 2
   ```

2. **Install a Windows X server** such as VcXsrv or use Windows 11's built-in WSLg (available by default on Windows 11).

**Note:** WSLg on Windows 11 provides the best experience with automatic display forwarding. On Windows 10, you may need to configure X server settings manually.

## macOS: Parallels Setup

Parallels Desktop is a powerful virtualization solution for macOS that provides excellent performance for running Ubuntu alongside macOS.

### Installing Parallels and Ubuntu

1. **Download and install Parallels Desktop** from [parallels.com](https://www.parallels.com)

2. **Create a new virtual machine**:
   - Open Parallels Desktop
   - Click "+" to create a new VM
   - Select "Install Windows or another OS from a DVD or image file"
   - Download Ubuntu 24.04 LTS Desktop ISO from [ubuntu.com](https://ubuntu.com/download/desktop)
   - Follow the installation wizard to complete setup

3. **Configure VM resources** (recommended settings):
   - Memory: At least 4GB RAM (8GB recommended)
   - Processors: 2-4 CPU cores
   - Disk space: 40GB or more
   - Graphics: Enable 3D acceleration for better RViz performance

4. **Install Parallels Tools** for better integration:
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

## Installing ROS2 Kilted on Ubuntu 24.04

Once you have Ubuntu 24.04 running (via WSL, Parallels, or another method), you'll need to install ROS2 Kilted Desktop, which includes RViz and all the visualization tools you'll need.

### ROS2 Kilted Desktop Installation

Follow the official ROS2 installation instructions for Ubuntu:

**🔗 Installation Guide**: [https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/kilted/Installation/Ubuntu-Install-Debs.html)

The ROS2 Kilted Desktop installation includes:
- Core ROS2 packages
- RViz for 3D visualization
- RQt tools for debugging and monitoring
- Common ROS2 packages and dependencies

### Verifying Your Installation

After completing the ROS2 installation, verify everything is working:

1. **Source the ROS2 setup file**:
   ```bash
   source /opt/ros/kilted/setup.bash
   ```

2. **Test ROS2**:
   ```bash
   ros2 --version
   ```

3. **Test RViz** (GUI required):
   ```bash
   rviz2
   ```

### Adding ROS2 to Your Shell Profile

To avoid sourcing ROS2 manually every time, add it to your `.profile`:

```bash
echo "source /opt/ros/kilted/setup.bash" >> ~/.profile
source ~/.profile
```

## Troubleshooting Graphics Issues

If you encounter graphics or rendering issues when running RViz or other GUI applications in your VM, you may need to enable software rendering.

### Enable Software Rendering

Set this environment variable to force software rendering:

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

To make this permanent, add it to your `.profile`:

```bash
echo "export LIBGL_ALWAYS_SOFTWARE=1" >> ~/.profile
source ~/.profile
```

**When to use this:**
- RViz crashes on startup with OpenGL errors
- You see graphics driver warnings
- 3D visualization is not working properly
- Running in a VM without proper GPU acceleration

**Note:** Software rendering will be slower than hardware acceleration, but it ensures compatibility across different VM configurations.

## Next Steps

Now that you have your development environment set up, you can:

- Visualize sensor data with RViz (instructions coming soon)
- Real-time camera feed (instructions coming soon)
