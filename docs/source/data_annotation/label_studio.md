# Label Studio for Data Annotation

Label Studio is a data labeling tool that students will use to annotate their robot datasets. Label Studio allows you to visually annotate your robot's data—such as teleoperation image sequences—so that you can later use it for training machine learning models or conducting analysis. You'll access Label Studio through a browser (while SSH'd to the Pi), connect it to the image data stored on your Raspberry Pi, and annotate that data with bounding boxes and classifications (purple whiffle, green whiffle).

🔗 Full Labeling Guide – [Label Studio Documentation](https://labelstud.io/guide/labeling)

## Setup and Usage

**1. <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Make folders:**
```bash
mkdir -p "$HOME/teleop_data/images"
mkdir -p "$HOME/teleop_data/annotations"
```

**2. <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16">  Enable Local File Serving (for accessing images on the Pi):**
```bash
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
```

**3. <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16">  Start Label Studio:**
```bash
cd ~/LabelMaker
source bin/activate
label-studio start
```

**4. 🖥️ Access Label Studio:**
- Open a web browser and navigate to: `http://192.168.1.n:8080` (replace `n` with your robot number)
- Create a Label Studio Account
- Create a new Label Studio Project

**5. <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16">  (Optional) Make it persistent for future terminals:**
```bash
nano ~/.profile
```

Add the following environment variable to the end of ~/.profile:
```bash
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
```

Then make sure you close and re-open your terminal so the environment variable is picked up.

**6. 🖥️ Configure Label Studio Storage:**

In your browser visit: `http://192.168.1.n:8800`

*Add Source storage (the images to label):*
- Storage type: **Local Files**
- Storage Title: **Teleop Images**
- Absolute local path: `/home/rcr/teleop_data/images/`
<!-- - Path: **images** ← (must be a child folder of the absolute path) -->
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

**7. 🖥️ Label Your Image Data from the Maze:**
- Open your synced image dataset collected from the maze run
- Begin labeling key events, robot actions, or objects of interest by drawing bounding boxes and assigning appropriate labels
- Use overlapping annotations, region duplication, and set up annotation relationships if needed
- For a full overview of labeling features, see the [Label Studio Labeling Guide](https://labelstud.io/guide/)

**8. 🖥️ Upload Your Annotated Dataset:**
- Press **Sync** in the target storage tab in Label Studio to export the annotations to your local `annotations/` folder
- Zip the annotations folder and leave it in the ~/teleop_data/:
  ```bash
  cd /home/rcr/teleop_data
  zip -r annotations.zip annotations/
  ```
- Leave your Pi on and connected to the RCR internet and tell Joe he can grab your annotated dataset from your IP address.

