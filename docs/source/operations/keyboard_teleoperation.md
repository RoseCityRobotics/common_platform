# Commands for Starting Keyboard Teleoperation of Your Robot

This document outlines essential commands for setting up a ROS2 environment, particularly focusing on micro-ROS integration with a Teensy microcontroller and keyboard teleoperation.

-----

## Prerequisites

Before starting keyboard teleoperation, ensure you have:

1. **Discovery Server Running**: See [Discovery Server Setup](discovery_server.md) for setup instructions
2. **Robot Powered On**: Ensure your both the Pi and Teensy are powered and connected
3. **Network Connectivity**: Verify connection between your computer and the robot

-----

## Power and Log on to Raspberry Pi

The Raspberry Pi will host the micro-ROS agent and other ROS2 nodes. Ensure it's powered on and you're logged in. Required ROS2 environment variables set in the [**`.profile`**](https://github.com/RoseCityRobotics/common_platform/blob/main/.profile) file for automated setup upon login.

You will need a separate terminal window 📟 for each process below:

## Get your Teensy serial number**

Open a new terminal window 📟

```bash
SERIAL_NUM=$(
basename "$(find /dev/serial/by-id/ -name 'usb-Teensyduino*if0[02]' | head -1)" \
| sed -E 's/.*_([0-9A-Za-z]+)-if0[02]/\1/'
)
echo "Teensy serial: $SERIAL_NUM"
```

## Setup a terminal with miniterm to monitor Teensy debug output

Open a new terminal window 📟

```bash
python3 -m serial.tools.miniterm "/dev/serial/by-id/usb-Teensyduino_Dual_Serial_${SERIAL_NUM}-if02" 115200

```

This command starts a `miniterm` session to display debug messages from the Teensy. The path identifies your Teensy and the {SERIAL_NUM} command pulls your serial number, and `115200` sets the baud rate. You should expect to see "**WAITING\_AGENT**" if the Teensy is awaiting a connection.

## Start the micro-ROS Agent

Open a new terminal window 📟

The micro-ROS agent acts as a bridge between the micro-ROS client running on the Teensy and the ROS2 network.

```bash
SERIAL_NUM=$(
basename "$(find /dev/serial/by-id/ -name 'usb-Teensyduino*if0[02]' | head -1)" \
| sed -E 's/.*_([0-9A-Za-z]+)-if0[02]/\1/'
)
echo "Teensy serial: $SERIAL_NUM"
```

```bash
cd ~
sudo docker run -it --rm \
  -v /dev:/dev --privileged --net=host \
  --env-file ./env.list --name agent\
  microros/micro-ros-agent:kilted \
  serial --dev "/dev/serial/by-id/usb-Teensyduino_Dual_Serial_${SERIAL_NUM}-if00" -v4
```

This Docker command starts the micro-ROS agent:

  * **`sudo docker run`**: Executes a Docker container.
  * **`-it`**: Runs in interactive mode with a pseudo-TTY.
  * **`--rm`**: Automatically removes the container when it exits.
  * **`-v /dev:/dev`**: Mounts the host's `/dev` directory into the container, allowing access to serial devices.
  * **`--privileged`**: Grants the container elevated privileges, necessary for direct hardware access.
  * **`--net=host`**: Uses the host's network stack, simplifying network configuration.
  * **`--env-file ./env.list`**: Loads environment variables from the `env.list` file.
  * **`microros/micro-ros-agent:kilted`**: Specifies the Docker image to use.
  * **`serial --dev ...`**: Instructs the agent to connect via a serial port, specifying the Teensy's serial device path.
  * **`-v4`**: Sets the verbosity level for the agent's output.

## Start the Keyboard Monitor Node

Open a new terminal window 📟

This node allows you to control your robot or application using keyboard inputs.

```bash
cd ~/repos/common_platform/common_platform_ws
source install/setup.bash
ros2 launch evdev_teleop evdev_teleop.launch.py
```

This command launches the `evdev_teleop` package, which sets up a ROS2 node to read keyboard input and publish corresponding control commands.

## Keyboard Commands 🎮

> ⚠️ **Important:** Put your robot on the ground before starting teleoperation!

| Command | Key | Action |
|---------|-----|--------|
| ⬆️ | **Forward Arrow** | Move forward |
| ⬇️ | **Backward Arrow** | Move backward |
| ⬅️ | **Left Arrow** | Turn left |
| ➡️ | **Right Arrow** | Turn right |
| ⏸️ | **Spacebar** | Stop/Emergency brake |

-----

## Shutdown Procedure 🔌

When you're finished with teleoperation, properly shut down the system:

```bash
sudo shutdown now
```

This command will safely power down the Raspberry Pi and all connected systems.

> 💡 **Tip:** Always use the proper shutdown command to avoid data corruption and ensure the system starts cleanly next time.
