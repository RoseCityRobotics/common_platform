Of course\! Here is an expanded and nicely formatted markdown version of the text you provided.

# Using a Bluetooth Gamepad to Drive a ROS2 Robot 🎮

This guide walks you through configuring a standard Bluetooth (BT) gamepad to control a ROS2-based robot. The process involves creating a custom ROS2 package to launch and configure the necessary nodes that translate your joystick movements into robot motion commands.

-----

## 🧐 Prerequisites & Setup

Before diving into ROS2, you need to ensure your system is ready.

1.  **Pair the Gamepad**: First, you must pair your Bluetooth gamepad with the Raspberry Pi's operating system. Use the standard Bluetooth graphical interface or command-line tools like `bluetoothctl` to scan for and connect your device.

2.  **Install ROS2 Packages**: You'll need two key packages to get this working. If they aren't already installed in your ROS2 environment (e.g., your Docker container), install them now:

    ```bash
    # Replace ${ROS_DISTRO} with your ROS2 version (e.g. kilted) in case the ROS_DISTRO environment variable is not set
    sudo apt update
    sudo apt install ros-${ROS_DISTRO}-joy ros-${ROS_DISTRO}-teleop-twist-joy
    ```

      * **`joy`**: This package contains the `joy_node`, which connects to your gamepad device (`/dev/input/js0`, for example) and publishes its raw button presses and axis states to a ROS2 topic called `/joy`.
      * **`teleop_twist_joy`**: This package provides the `teleop_node`, which listens to the `/joy` topic and converts the raw joystick data into standard `Twist` messages on the `/cmd_vel` topic, which is what most robot drivers use to control movement.

-----

## 🛠️ Creating the Teleop Package

To keep our configuration organized, we'll create a dedicated ROS2 package.

### 1\. Create a ROS2 Workspace

If you don't already have one, create a new ROS2 workspace. This is the standard directory for developing your own packages.

```bash
# Create the directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

### 2\. Create the New Package

Now, create a new package named `common_robot` inside the `src` directory.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake common_robot
```

### 3\. Create `launch` and `config` Directories

Inside your new package, create subdirectories to hold your launch file and configuration file. This is a common and recommended practice.

```bash
cd common_robot
mkdir launch
mkdir config
```

-----

## ⚙️ Configuration Files

Now, let's create the two files that will define and configure our teleoperation setup.

### 1\. The Launch File (`joystick.launch.py`)

A launch file allows you to start and configure multiple ROS2 nodes with a single command. Inside the `common_robot/launch` directory, create a new file named `joystick.launch.py` and add the following Python code.

```python
# ~/ros2_ws/src/common_robot/launch/joystick.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the path to the joystick configuration file
    joy_params = os.path.join(
        get_package_share_directory('common_robot'),
        'config',
        'joystick.yaml'
    )

    # Node to handle joystick input
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )

    # Node to convert joystick commands to robot motion (Twist messages)
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        # Uncomment the following line to remap the output topic if your robot
        # expects velocity commands on a different topic name
        # remappings=[('/cmd_vel', '/diff_cont/cmd_vel_unstamped')]
    )

    return LaunchDescription([
        joy_node,
        teleop_node
    ])
```

**What this file does:**

  * It defines a path to a YAML configuration file (`joystick.yaml`).
  * It declares a `joy_node` that will read data from your gamepad.
  * It declares a `teleop_node` that will convert the gamepad data into velocity commands.
  * Crucially, it passes the `joy_params` file to **both** nodes, so they share the same configuration.

### 2\. The YAML Config File (`joystick.yaml`)

This file contains all the specific parameters for your gamepad, like which stick controls forward motion and how fast the robot should go. Inside the `common_robot/config` directory, create a file named `joystick.yaml` with the following content.

```yaml
# ~/ros2_ws/src/common_robot/config/joystick.yaml

joy_node:
  ros__parameters:
    device_id: 0         # Corresponds to /dev/input/js0
    deadzone: 0.05       # Ignores small stick movements
    autorepeat_rate: 20.0  # Rate (in Hz) to publish joy messages

teleop_node:
  ros__parameters:
    # Axis mappings for linear (forward/backward) and angular (turning) motion
    axis_linear:
      x: 1  # Vertical axis of the left thumb stick
    scale_linear:
      x: 0.4 # Max linear speed (m/s)
    scale_linear_turbo:
      x: 0.8 # Max linear speed when turbo button is pressed

    axis_angular:
      yaw: 0 # Horizontal axis of the left thumb stick
    scale_angular:
      yaw: 0.5 # Max angular speed (rad/s)
    scale_angular_turbo:
      yaw: 1.0 # Max angular speed when turbo button is pressed

    # Button mappings
    require_enable_button: true   # If true, a button must be held to move the robot (safety feature)
    enable_button: 4              # Left bumper (LB) button
    enable_turbo_button: 5        # Right bumper (RB) button
```

**Understanding these parameters:**

  * `device_id`: The system ID of your joystick. `0` usually corresponds to `/dev/input/js0`.
  * `deadzone`: The amount you need to move the stick from the center before it registers a command. This prevents "drift" from a worn-out stick.
  * `axis_linear`/`axis_angular`: These map a specific joystick axis number to a type of motion. You may need to change these values for your specific gamepad\!
  * `scale_linear`/`scale_angular`: These control the maximum speed of the robot.
  * `enable_button`: This acts as a "dead man's switch." The robot will only move while this button is held down. This is a highly recommended safety feature.
  * `enable_turbo_button`: Allows you to move faster by holding down a second button.

> **💡 Pro Tip:** To find the correct axis and button numbers for your controller, you can use a tool like `jstest-gtk` or simply watch the output of `ros2 run joy joy_node` in a separate terminal while you press buttons.

-----

## 🚀 Building and Running

Finally, let's build the package and launch it.

### 1\. Edit `CMakeLists.txt`

You must tell ROS2 to install your `launch` and `config` directories so it can find your files. Open the `CMakeLists.txt` file in your `common_robot` package and add the following lines just before the final `ament_package()` call.

```cmake
# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
```

### 2\. Build and Source the Workspace

Navigate back to the root of your workspace (`~/ros2_ws`), build your new package, and then source the setup file to make your terminal aware of it.

```bash
cd ~/ros2_ws

# Build all packages in the workspace
colcon build --symlink-install

# Source the setup file to update your environment
source install/setup.bash
```

**Note:** You must `source` the setup file in **every new terminal** where you want to use your package's nodes or launch files.

### 3\. Run Teleop\!

You're all set\! Launch the file you created. Make sure your gamepad is on and connected.

```bash
ros2 launch common_robot joystick.launch.py
```

Now, while holding down your enable button (the left bumper in the config above), move the left thumbstick. Your robot should move\! Press the turbo button (right bumper) to go faster.

### 4\. Verification

To see if it's working, open a **new terminal** (and remember to source `install/setup.bash` again) and use the following command to watch the velocity commands being published:

```bash
ros2 topic echo /cmd_vel
```

You should see messages appear as you move the joystick.

-----

## 📚 Resources

For more details on the concepts used here, this is an excellent resource:

  * [Articulated Robotics: ROS2 C++ Mobile Robot Teleop](https://articulatedrobotics.xyz/mobile-robot-14a-teleop/)

