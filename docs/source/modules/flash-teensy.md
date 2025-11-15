---
type: module
slug: flash-teensy
title: Compile and flash the Teensy firmware
description: Build and upload the microcontroller firmware for closed-loop control.
tags: [firmware, teensy, build]
device: pi
---

# Compile and flash the Teensy firmware

{{pi}} **On the Raspberry Pi**

The closed-loop motor controller firmware for the robot runs on a Teensy 4.0 microcontroller. Follow these steps to compile and upload the firmware using `arduino-cli` and `teensy_loader_cli`.

## Clean and prepare build directory

```bash
cd ~/repos/common_platform/firmware/closed_loop
rm -rf build
mkdir build
cd build
```

## Compile firmware with arduino-cli

```bash
arduino-cli compile --fqbn teensy:avr:teensy40 --build-property build.usbtype=USB_DUAL_SERIAL --build-path . ../closed_loop.ino
```

:::{note}
Compilation produces `closed_loop.ino.hex` in the build directory. The `USB_DUAL_SERIAL` build property enables dual serial ports for debugging and communication. Address any compilation errors before proceeding.
:::

## Find the Teensy device

First, power the Teensy by inserting the lower-half batteries.

```bash
SERIAL_TEENSY_DEVICE=$(find /dev/serial/by-id/ -name "usb-Teensyduino*if00" | head -1)
echo "Teensy device: $SERIAL_TEENSY_DEVICE"
```

## Reset Teensy into programming mode

```bash
stty -F $SERIAL_TEENSY_DEVICE 9600
stty -F $SERIAL_TEENSY_DEVICE 134
```

:::{note}
The 9600-baud command opens the port, and the 134-baud command triggers a soft reset that puts the Teensy into bootloader mode.
:::

## Verify bootloader is ready

```bash
lsusb | grep Teensy
```

:::{note}
You should see a Teensy device listed, indicating the bootloader is active and ready to receive new firmware.
:::

## Upload the firmware

```bash
sudo teensy_loader_cli -v --mcu=TEENSY40 closed_loop.ino.hex
```

:::{note}
The loader tool writes the firmware to the Teensy and reports "Download Complete". The microcontroller resets and runs the new code.
:::

