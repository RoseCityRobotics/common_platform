# Flashing the Teensy Microcontroller

This guide covers how to flash firmware to the Teensy 4.0 microcontroller on the RCR Common Robotics Platform.

## Overview

Firmware runs on the Teensy 4.0 microcontroller. It sets up ROS2 nodes that run on the Teensy, but communicate with the larger ROS2 environment via a bridge node called the microros agent that runs on the RPi5.

## Flashing Procedure

### 1. Navigate to the firmware directory and replace the build folder
   ```bash
   cd ~/repos/common_platform/firmware/closed_loop/
   rm -rf build
   mkdir build
   cd build
   ```

### 2. Compile the firmware for Teensy 4.0
   ```bash
   arduino-cli compile --fqbn teensy:avr:teensy40 --build-property build.usbtype=USB_DUAL_SERIAL --build-path . ../closed_loop.ino
   ```

### 3. Find the Teensy device (first power the Teensy - i.e. put in the lower-half batteries)
   ```bash
   SERIAL_TEENSY_DEVICE=`find /dev/serial/by-id/ -name "usb-Teensyduino*if00"|head -1`
   echo "-> Performing soft reset (baud = 134 hack). $SERIAL_TEENSY_DEVICE"
   ```

### 4. Reset Teensy into programming mode
   ```bash
   stty -F $SERIAL_TEENSY_DEVICE 9600
   stty -F $SERIAL_TEENSY_DEVICE 134
   ```

### 5. Verify the Teensy is ready
   ```bash
   lsusb | grep Teensy; echo "-> Should be ready to program if lsusb reports a Bootloader device…"
   ```

### 6. Upload the new firmware
   ```bash
   sudo teensy_loader_cli -v --mcu=TEENSY40 closed_loop.ino.hex
   ```
