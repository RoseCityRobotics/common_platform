# Software Guide

### Arduino CLI Setup

For programming the Teensy microcontroller, we use the Arduino CLI tool. This allows you to compile and upload firmware directly from the command line on the Raspberry Pi.

See our detailed guide: [Arduino CLI Setup](software_arduino_cli)

### <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> Host Settings
The host has been preset for you as `rcr00_` with your specific ID. To update the hostname:

1. Tell cloud-init to preserve your hostname:
   ```bash
   sudo nano /etc/cloud/cloud.cfg
   ```
   Set `preserve_hostname: true` and save the file

2. Update the hostname for the robot:
   ```bash
   sudo nano /etc/hostname
   ```
   Change `rcr001` to `rcr00n` (replace n with the number)

3. Update the hosts file:
   ```bash
   sudo nano /etc/hosts
   ```
   Change `rcr001` to `rcr00n`

4. Set the hostname:
   ```bash
   sudo hostnamectl set-hostname rcr00n
   ```

5. Reboot to make sure the changes have taken effect:
   ```bash
   sudo reboot
   ```

### <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> Network Configuration

Set custom IP address and set up networking:

1. Edit the netplan configuration:
   ```bash
   sudo nano /etc/netplan/50-cloud-init.yaml
   ```

2. Add the following configuration (replace `n` with your robot number):
   ```yaml
   network:
     version: 2
     wifis:
       wlan0:
         optional: true
         dhcp4: no
         addresses:
           - 192.168.1.n/24
         routes:
           - to: default
             via: 192.168.1.1
         nameservers:
           addresses: [8.8.8.8, 1.1.1.1]
         access-points:
           "robot_overlord_wifi":
             password: "siliconforest"
           "RoseCityRobotics":
             password: "QW260go80.."
   ```
   **Note:** `n` = the robot number

3. Apply the network configuration:
   ```bash
   sudo netplan apply
   ```

### <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> Testing SSH Connection

1. **<img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> On the Pi:** Make sure the IP address is assigned:
   ```bash
   hostname -I
   ```

2. **🖥️ On the other computer:** Connect via SSH:
   ```bash
   ssh rcr@192.168.1.n
   ```
   Replace `n` with your robot number.

### Accessing the Pi <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="20"> from your development machine 🖥️ via SSH
Both the external device and the robot must be on the same WiFi for SSH to work. Use the static IP address of your Pi to SSH. The command will look something like `ssh rcr@192.168.1.n` where `n` is your ID, for example `ssh rcr@192.168.1.9`.

### Label Studio for Data Annotation

Label Studio is a data labeling tool that students will use to annotate their robot datasets.

#### Setup and Usage

**1. <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Make folders:**
```bash
mkdir -p "$HOME/teleop_data/images"
mkdir -p "$HOME/teleop_data/annotations"
```

**2. <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Start Label Studio:**
```bash
cd ~/LabelMaker
source bin/activate
label-studio start
```

**3. 🖥️ Access Label Studio:**
- Open a web browser and navigate to: `http://192.168.1.n:8800` (replace `n` with your robot number)
- Log in with:
  - Email: `robot@rosecityrobotics.com`
  - Password: `siliconforest`

**4. <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Enable Local File Serving (for accessing images on the Pi):**
```bash
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT=[your path to the images folder on the Pi]
```

**5. <img src="_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> (Optional) Make it persistent for future terminals:**
```bash
grep -q LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT ~/.bashrc || cat >> ~/.bashrc <<'EOF'
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT="$HOME/teleop_data"
EOF
```

**6. 🖥️ Configure Label Studio Storage:**

*Add Source storage (the images to label):*
- Storage type: **Local Files**
- Storage Title: **Teleop Images**
- Absolute local path: `/home/rcr/teleop_data`
- Path: **images** ← (must be a child folder of the absolute path)
- Import method: **Files** ← important for images (not "Tasks")
- Click **Add Storage** → **Sync**

*If you see UnsupportedFileFormatError ... Only .json/.jsonl/.parquet... you picked Tasks. Edit storage and switch Import method to Files.*

*Add Target storage (where your annotations are saved):*
- Storage type: **Local Files**
- Storage Title: **Annotations**
- Absolute local path: `/home/rcr/teleop_data/annotations/`
- Path:
- Click **Add Storage**

*(Later, press Sync on this target storage to export annotations there.)*
