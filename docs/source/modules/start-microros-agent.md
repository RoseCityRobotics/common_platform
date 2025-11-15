---
type: module
slug: start-microros-agent
title: Start the micro-ROS agent
description: Run the micro-ROS serial agent in a Docker container to bridge the Teensy with ROS 2.
tags: [teensy, microros, agent]
device: pi
---

# Start the micro-ROS agent

{{pi}} **On the Raspberry Pi**

The Teensy communicates with ROS 2 via a micro-ROS agent, which runs in a Docker container. Launch the agent on your Raspberry Pi, mounting `/dev` for serial access, loading environment variables (including `ROS_NAMESPACE`), using the data channel interface (`if00`), and enabling verbose logging.

## Get your Teensy serial number

```bash
SERIAL_NUM=$(
basename "$(find /dev/serial/by-id/ -name 'usb-Teensyduino*if0[02]' | head -1)" \
|| sed -E 's/.*_([0-9A-Za-z]+)-if0[02]/\1/'
)
echo "Teensy serial: $SERIAL_NUM"
```

## Run micro-ROS agent with full configuration

Open a new terminal window {{terminal}}

```bash
cd ~
sudo docker run -it --rm \
  -v /dev:/dev --privileged --net=host \
  --env-file ./env.list --name agent\
  microros/micro-ros-agent:kilted \
  serial --dev "/dev/serial/by-id/usb-Teensyduino_Dual_Serial_${SERIAL_NUM}-if00" -v4
```

:::{note}
**Command breakdown:**
- **`sudo docker run`**: Executes a Docker container
- **`-it`**: Runs in interactive mode with a pseudo-TTY
- **`--rm`**: Automatically removes the container when it exits
- **`-v /dev:/dev`**: Mounts the host's `/dev` directory for serial device access
- **`--privileged`**: Grants elevated privileges for hardware access
- **`--net=host`**: Uses the host's network stack
- **`--env-file ./env.list`**: Loads environment variables including `ROS_NAMESPACE`
- **`-v4`**: Sets verbose logging level
:::

:::{tip}
The agent prints verbose log messages (`-v4`) indicating it has connected to the serial device and is translating micro-ROS messages to DDS. The `--env-file` ensures the agent uses your configured `ROS_NAMESPACE`. Leave this terminal open while using the robot.
:::

