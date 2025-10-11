# ROS Discovery Server

The ROS discovery server facilitates communication between different ROS2 nodes. It acts as a central point for nodes to discover each other on the network.

> ⚠️ **Important**
> If you are using your robot at our Rose City Robotics Lab, a discovery server is already running on the lab network. You can skip this step if you have not yet modified this file for your home setup.

## Run your own discovery server
If you want to be able to run your robot on your own, without the RCR discovery server, you will need to follow these steps:

### Note the IP Address

`127.0.0.1` for local host (your own machine)

### Start the Discovery Server

Open a new terminal window 📟

```bash
fastdds discovery --server-id 0
```

This command initiates the Fast DDS discovery server with a server ID of 0.

### Update Configuration File

Open a new terminal window 📟

```bash
nano ~/ros2_ws/super_client_configuration_file_rcr.xml
```

Edit line 11 of the **`super_client_configuration_file_rcr.xml`** file to set the discovery server's address to `127.0.0.1` if running locally.

### Start the Micro ROS Agent

```bash
cd ~
SERIAL_TEENSY_DEVICE=`find /dev/serial/by-id/ -name "usb-Teensyduino*if00"|head -1`
sudo docker run -it --rm -v /dev:/dev --privileged --net=host --env-file ./env.list --name agent microros/micro-ros-agent:kilted serial --dev ${SERIAL_TEENSY_DEVICE} -v4
```

### Update DDS Configuration in Container

Open a new terminal window 📟

```bash
sudo docker cp ~/ros2_ws/super_client_configuration_file_rcr.xml agent:/uros_ws/
sudo docker commit agent microros/micro-ros-agent:kilted
```

Restart the micro ros agent after updating the configuration file inside the container (<CTRL-C> in the terminal window 📟 that is running the agent. You can use the up arrow to repeat the sudo docker run command).

### Verification

To verify the discovery server is working:

```bash
# Check if discovery server is running
ps aux | grep fastdds

# Check network connectivity
ping <discovery_server_ip>
```

## Switch back to RCR discovery server
If you have updated your discovery server settings above to use your robot at home, you will need to follow the steps below

### Note the IP Address

`192.168.1.125` for Joe's laptop at RCR Lab

### Start the Discovery Server

Open a new terminal window 📟

```bash
fastdds discovery --server-id 0
```

This command initiates the Fast DDS discovery server with a server ID of 0.

### Update Configuration File -

Open a new terminal window 📟

```bash
nano ~/ros2_ws/super_client_configuration_file_rcr.xml
```

Edit line 11 of the **`super_client_configuration_file_rcr.xml`** file to set the discovery server's address to `127.0.0.1` if running locally.

### Start the Micro ROS Agent

```bash
cd ~
SERIAL_TEENSY_DEVICE=`find /dev/serial/by-id/ -name "usb-Teensyduino*if00"|head -1`
sudo docker run -it --rm -v /dev:/dev --privileged --net=host --env-file ./env.list --name agent microros/micro-ros-agent:kilted serial --dev ${SERIAL_TEENSY_DEVICE} -v4
```

### Update DDS Configuration in Container

Open a new terminal window 📟

```bash
sudo docker cp ~/ros2_ws/super_client_configuration_file_rcr.xml agent:/uros_ws/
sudo docker commit agent microros/micro-ros-agent:kilted
```

Restart the micro ros agent after updating the configuration file inside the container (<CTRL-C> in the terminal window 📟 that is running the agent. You can use the up arrow to repeat the sudo docker run command).

### Verification

To verify the discovery server is working:

```bash
# Check if discovery server is running
ps aux | grep fastdds

# Check network connectivity
ping <discovery_server_ip>
```

## Common Issues

**Discovery server not starting:**
- Check if port 11811 is available
- Verify Fast DDS is properly installed
- Check network connectivity

**Nodes not discovering each other:**
- Verify the configuration file has the correct IP address
- Check that all nodes are using the same discovery server
- Ensure firewall settings allow ROS2 communication

**Micro ROS agent connection issues:**
- Verify the Teensy device is connected
- Check the serial device path
- Ensure Docker has proper permissions

---

*For keyboard teleoperation setup, see [Keyboard Teleoperation](keyboard_teleoperation.md)*
*For troubleshooting, see [Troubleshooting](../troubleshooting/index.md)*
