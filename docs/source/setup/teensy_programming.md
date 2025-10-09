# Teensy Programming - Use Arduino CLI
<img src="/Users/duncanmiller/sites/common_platform/docs/source/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20">

**What we're doing:** We'll set up the Arduino CLI tool on the Raspberry Pi to compile and upload firmware to the Teensy microcontroller. This allows us to program the robot's brain (Teensy) directly from the command line without needing a graphical interface.

**Why Arduino CLI:** The Arduino CLI is the official command-line tool that provides all the functionality of the Arduino IDE but runs entirely in the terminal. This is perfect for our setup since we're working remotely via SSH on the Raspberry Pi.

Arduino provides an official `arduino-cli` tool. You can compile and upload Arduino code for a Teensy entirely from the Ubuntu command line. You will do this from the Raspberry Pi (after connecting via SSH).

#### 1. **Install Arduino CLI:**
```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
mv bin/arduino-cli ~/.local/bin/
```

#### 2. **Configure Arduino CLI:**
```bash
arduino-cli config init
```

#### 3. **Add Teensy board support:**
```bash
arduino-cli core update-index
arduino-cli core install teensy:avr --additional-urls https://www.pjrc.com/teensy/package_teensy_index.json
```

#### 4. Transfer micro-ROS Arduino Library to Raspberry Pi

The required micro-ROS Arduino library is available in the repository: `firmware/libraries/micro_ros_arduino.zip` so we will  pull the repository to Raspberry Pi <img src="/Users/duncanmiller/sites/common_platform/docs/source/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> to access the file.

**What we're doing:** We need to transfer the micro-ROS library GitHub to the Raspberry Pi that's connected to your robot. The Raspberry Pi serves as the main computer that will compile and upload firmware to the Teensy microcontroller.

🖥️ From your development machine
```bash
ssh rcr@192.168.1.n
cd ~/repos/common_platform/
```

**<img src="/Users/duncanmiller/sites/common_platform/docs/source/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> From your Raspberry Pi** in the common_platform directory:

First update your directory with the latest changes from the https://github.com/roseCityRobotics/common_platform repository including the .zip file you'll need next.
```bash
git checkout main
git pull origin main
```

#### 5. Setup micro-ROS Library and Compile Firmware

**What we're doing:** Now we're working directly on the Raspberry Pi to set up the micro-ROS library and compile the robot's firmware. The Raspberry Pi will handle the compilation process and then upload the compiled code to the Teensy microcontroller.

**<img src="/Users/duncanmiller/sites/common_platform/docs/source/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> From the Raspberry Pi (RPi):**

a. Set up Arduino libraries directory:
   ```bash
   cd ~
   mkdir -p Arduino/libraries/
   cp repos/firmware/libraries/micro_ros_arduino.zip Arduino/libraries/
   cd ~/Arduino/libraries/
   ```

b. Install unzip and extract the micro-ROS library:
   ```bash
   sudo apt update
   sudo apt install unzip
   unzip micro_ros_arduino.zip
   ```

c. Navigate to the common_platform repository, update it, and place the arduino-cli config files in the correct place
  ```bash
  cd ~/repos/common_platform/
  git checkout main
  git pull origin main
  scripts/place_arduino-cli_config.sh
  ```
