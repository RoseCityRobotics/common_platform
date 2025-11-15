---
type: module
slug: monitor-teensy-serial
title: Monitor Teensy debug output
description: Use miniterm to view debug messages from the Teensy microcontroller.
tags: [teensy, debug, serial]
device: pi
---

# Monitor Teensy debug output

{{pi}} **On the Raspberry Pi**

Open a new terminal window {{terminal}} to monitor debug messages from the Teensy.

## Get your Teensy serial number

```bash
SERIAL_NUM=$(
basename "$(find /dev/serial/by-id/ -name 'usb-Teensyduino*if0[02]' | head -1)" \
|| sed -E 's/.*_([0-9A-Za-z]+)-if0[02]/\1/'
)
echo "Teensy serial: $SERIAL_NUM"
```

## Start miniterm session

```bash
python3 -m serial.tools.miniterm "/dev/serial/by-id/usb-Teensyduino_Dual_Serial_${SERIAL_NUM}-if02" 115200
```

:::{note}
This command starts a `miniterm` session to display debug messages from the Teensy. The path identifies your Teensy using the serial number, and `115200` sets the baud rate.
:::

:::{tip}
You should expect to see "**WAITING_AGENT**" if the Teensy is awaiting a connection from the micro-ROS agent.
:::

