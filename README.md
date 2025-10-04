# Rose City Robotics Common Platform

> 📚 **📖 [View Complete Documentation](https://common.rosecityrobotics.com/)** - Comprehensive guides for setup, operation, and development

<!-- ABOUT THE PROJECT -->
## A Fork of the PARTS Common Platform

<br>
<p align="center">
  <img src="github/img/side_image_lidar.jpg" alt="Side view with LiDAR sensor" width="500">
</p>
<br>

This project implements a complete robotics platform built on the Pololu Romi Chassis Kit, featuring a custom carrier PCB that integrates all components into a compact, efficient design. The platform combines hardware, firmware, and software to create a fully functional autonomous robot.

<br>
<p align="center">
  <a href="https://portlandrobotics.org/home.php?link_id=1">
    <img src="github/img/PARTS-board.png" alt="Logo" width="300" height="202">
  </a>
<br>
<br>

**Hardware Components:**
- **Pololu Romi Chassis Kit** - Robust base platform with encoders and caster
- **Custom PARTS CRP Board** - Integrated carrier PCB with all connections
- **Teensy 4.0 Microcontroller** - High-performance ARM Cortex-M7 processor
- **TB9051FTG Motor Drivers** - Dual brushed DC motor control
- **MPU-9250 9DOF IMU** - Nine-axis motion sensing for navigation
- **Raspberry Pi** - Main computer for ROS2 and autonomous functionality
- **Power Management** - 5V regulator and battery voltage monitoring

<br>
<p align="center">
  <img src="github/img/stack_platform_unassembled.jpg" alt="Mobile base platform" width="500">
</p>
<br>

**Current Implementation:**
The platform now features a complete ROS2 integration with micro-ROS running on the Teensy 4.0, enabling real-time communication between the Raspberry Pi and the robot's sensors and actuators. The system supports autonomous navigation, sensor fusion, and closed-loop control, making it an ideal platform for robotics education and development in the Portland Area Robotics Society.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Built With

* [![CPP][cpp]][cpp-url]
* [![C][c]][c-url]
* [![Python 3][python]][python-url]
* [![PlatformIO][platformio]][platformio-url]
* [![Arduino][arduino-ide]][arduino-url]
* [![KiCadEDA][kicad]][kicad-url]
* [![ROS][ros]][ros-url]

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

### <img src="github/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> Direct Connection to Raspberry Pi
Connect your micro HDMI cable to your Pi and monitor, and a keyboard to the USB-A port.

OR

### 🖥️ On the development computer: Connect via SSH:
   ```bash
   ssh rcr@192.168.1.n
   ```
   Replace `n` with your robot number.

### <img src="github/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> Ubuntu Login on Raspberry Pi
- **User:** `rcr`
- **Password:** `siliconforest`

### 🔄 Pull the Latest Changes from GitHub

Change to the correct git-enabled directory:

```bash
cd repos/common_platform
```

Update the Git remote for the public repo so you don’t need credentials:

```bash
git remote set-url origin "https://github.com/RoseCityRobotics/common_platform.git"
```

Pull the changes

```bash
git pull origin main
```

### 🧭 Robot Namespacing Setup

To ensure your robot runs under its unique namespace (e.g., `/rcr001`, `/rcr002`, etc.), follow these steps:

#### 1. Set Your Namespace in `.profile`

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

#### 2. Set Namespace in `env.list`

Update the environment variable file used for Docker and ROS:

```bash
sudo nano ~/env.list
```

Add or update the line:

```bash
ROS_NAMESPACE=/rcr00n
```

---

#### 3. Update Firmware Source Code

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

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### <img src="github/img/raspberry-pi.png" alt="Raspberry Pi" width="20">Flash the Teensy micro-controller with updated firmware

a. Navigate to firmware directory and create build folder:
   ```bash
   cd ~/repos/common_platform/firmware/closed_loop/
   mkdir build
   cd build
   ```

b. Compile the firmware for Teensy 4.0:
   ```bash
   arduino-cli compile --fqbn teensy:avr:teensy40 --build-property build.usbtype=USB_DUAL_SERIAL --build-path . ../closed_loop.ino
   ```

c. Find the Teensy device:
- first power the Teensy - put in the lower-half batteries
- second turn on the switch for the Teensy
- make sure the green LED has turned on

```bash
SERIAL_TEENSY_DEVICE=`find /dev/serial/by-id/ -name "usb-Teensyduino*if00"|head -1`; echo "-> Performing soft reset (baud = 134 hack). $SERIAL_TEENSY_DEVICE"
```

d. Reset Teensy into programming mode:
   ```bash
   stty -F $SERIAL_TEENSY_DEVICE 9600
   stty -F $SERIAL_TEENSY_DEVICE 134
   ```

e. Verify Teensy is ready and upload firmware:
   ```bash
   lsusb | grep Teensy; echo "-> Should be ready to program…"
   ```
   ```bash
   sudo teensy_loader_cli -v --mcu=TEENSY40 closed_loop.ino.hex
   ```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

🎉 Congratulations! You're all caught up for class 2 of the Engineering Boldly: AI Robotics Sprint!

## Class 2

### Attach your camera and battery pack
We will walk you through attaching your wide angle video camera to the robot as well as your battery pack.

When you are ready to proceed, check out the next step on setting up teleoperation: [Keyboard Teleoperation Guide](https://common.rosecityrobotics.com/operations/daily_operations/KeyboardTeleop.html)

## Usage

This section provides practical examples of how to use the Common Robotics Platform effectively. We'll start with a basic test to ensure your setup is functioning correctly.

### Starting with the Basic Test

1. **Flash the Teensy 4.0 Board:**
   Begin by flashing the Teensy 4.0 board with the basic test code found in the repository at `/firmware/basic_test/basic_test.ino`. This initial test is crucial for validating your setup.

2. **Expected Behavior:**
   Upon successful flashing, the robot should exhibit a simple behavior pattern: moving forward, pausing, and then moving backward.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Assembly Images

*Mobile base platform - unassembled components*

<img src="github/img/stack_platform_unassembled.jpg" alt="Mobile base platform" width="500">

*Assembled chassis with controller*

<img src="github/img/stack_with_controller.jpg" alt="Mobile base platform assembly" width="500">

*Custom PCB showing integrated motor drivers and controller connections*

<img src="github/img/stack_ports_plus_controller.jpg" alt="Custom PCB with ports and controller" width="500">

*Completed chassis - top view with Raspberry Pi AI HAT+*

<img src="github/img/stack_top.jpg" alt="Assembled Platform - Top View" width="500">

*Raspberry Pi 5 with AI HAT+*

<img src="github/img/raspberry_pi_ai.jpg" alt="Raspberry Pi 5 with AI HAT+" width="500">

*Side view with LiDAR sensor mounted*

<img src="github/img/side_image_lidar.jpg" alt="Side view with LiDAR sensor" width="500">

<p align="right">(<a href="#readme-top">back to top</a>)</p>

See the [open issues](https://github.com/rosecityrobotics/common_platform/issues) for a full list of proposed features (and known issues).

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Thank you PARTS

<img src="https://raw.githubusercontent.com/rosecityrobotics/common_platform/master/github/img/PARTS.png"
     alt="Portland Area Robotics Society (P.A.R.T.S.)"
     width="300">

*Portland Area RoboTics Society (PARTS)*
Our work is based on the [PARTS Common Platform](https://github.com/portlandrobotics/common_platform)

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the MIT License and the Solderpad Hardware License v2.1. See [LICENSE.txt][license-url] for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/portlandrobotics/common_platform.svg?style=for-the-badge
[contributors-url]: https://github.com/portlandrobotics/common_platform/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/portlandrobotics/common_platform.svg?style=for-the-badge
[forks-url]: https://github.com/portlandrobotics/common_platform/network/members
[stars-shield]: https://img.shields.io/github/stars/portlandrobotics/common_platform.svg?style=for-the-badge
[stars-url]: https://github.com/portlandrobotics/common_platform/stargazers
[issues-shield]: https://img.shields.io/github/issues/portlandrobotics/common_platform.svg?style=for-the-badge
[issues-url]: https://github.com/portlandrobotics/common_platform/issues
[license-shield]: https://img.shields.io/badge/license-mit_%26_solderpad-brightgreen?style=for-the-badge
[license-url]: https://github.com/portlandrobotics/common_platform/blob/master/LICENSE.txt
[arduino-ide]: https://img.shields.io/badge/Arduino_IDE-00979D?style=for-the-badge&logo=arduino&logoColor=white
[arduino-url]: https://www.arduino.cc/en/software
[cpp]: https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white
[cpp-url]: https://en.wikipedia.org/wiki/C%2B%2B
[c]: https://img.shields.io/badge/C-00599C?style=for-the-badge&logo=c&logoColor=white
[c-url]: https://en.wikipedia.org/wiki/C_(programming_language)
[python]: https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white
[python-url]: https://www.python.org/
[platformio]: https://img.shields.io/badge/PlatformIO-F5822A.svg?style=for-the-badge&logo=PlatformIO&logoColor=white
[platformio-url]: https://platformio.org/
[kicad]: https://img.shields.io/badge/KiCad-314CB0.svg?style=for-the-badge&logo=KiCad&logoColor=white
[kicad-url]: https://www.kicad.org/
[ros]: https://img.shields.io/badge/ROS-22314E.svg?style=for-the-badge&logo=ROS&logoColor=white
[ros-url]:https://www.ros.org/
