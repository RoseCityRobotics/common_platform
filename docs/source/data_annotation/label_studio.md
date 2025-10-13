# Label Studio for Data Annotation

Label Studio is the tool students use to annotate robot datasets (e.g., teleop image sequences) for ML training. You’ll access it in a browser, connect it to images on the Pi, and create bounding-box/classification labels (e.g., purple whiffle, green whiffle).

🔗 **[Full Labeling Guide](https://labelstud.io/guide/labeling)**

---

## Setup and Usage

### 1) <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Make folders on the Pi
```bash
mkdir -p "/home/rcr/teleop_data/images"
mkdir -p "/home/rcr/teleop_data/annotations"

# (recommended) sane permissions
chmod -R a+rX "/home/rcr/teleop_data"
chmod -R u+w "/home/rcr/teleop_data/annotations"
```

### 2) <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Set required environment variables for your terminal session
```bash
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT="/home/rcr/teleop_data"
```

### 3) <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> (Optional) Make env vars persistent so you can skp step 2 next time
```bash
nano /home/rcr/.profile
```
Add to the end:
```bash
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT="/home/rcr/teleop_data"
```
Close/reopen terminal (or `source /home/rcr/.profile`) and restart Label Studio.

### 4) <img src="/_static/img/raspberry-pi.png" alt="Raspberry Pi" width="16"> Start Label Studio
```bash
cd /home/rcr/LabelMaker
source bin/activate
label-studio start -p 8080 -b
```

### 5) 🖥️ Open the UI
- In a browser on your laptop: `http://192.168.1.n:8080`
  *(replace `n` with your robot’s number / the Pi’s IP last octet)*
- Create a Label Studio account
- Create a **New Project** (template: **Object Detection with Bounding Boxes**)
- In the template setup for your annotation template, delete the existing class names and add your own.
- In order to merge all the annotated datasets from the whole class, our conventions must align. Add `Purple Ball` as the first class and then `Green Ball` as the second class.
- You can use the color picker to choose an appropriate color for each class. Then click **Save**.

### 6) 🖥️ Configure Storage in the Project

**Add Source storage (images to label):**
Project → Settings → **Cloud Storage** → **Add Source Storage** → **Local files**
- **Storage Title** Anything you want here (required)
- **Absolute Local path:** `/home/rcr/teleop_data/images`
- (Optional) **File mask/regex:** `.*\.(jpg|jpeg|png)$`
- **Test Connection**
- **Next**
- Import method: **Files** ← important for images (not "Tasks")
- Click **Next** → **Save** → **Sync**

> If you see `UnsupportedFileFormatError ... Only .json/.jsonl/.parquet...` you picked Tasks. Edit storage and switch Import method to Files.

**Add Target storage (where annotations are saved):**
Project → Settings → **Cloud Storage** → **Add Target Storage** → **Local files**
- **Storage Title** Anything you want here (required)
- **Absolute Local path:** `/home/rcr/teleop_data/annotations`
- **Path** leave blank
- **Save**

### 7) 🖥️ Label your images
- Go to **Projects → Your Project**
- Select a task, assign labels and draw boxes
- See the [Label Studio labeling guide](https://labelstud.io/guide/) for power tips

### 8) 🖥️ Export / Share your annotations
- If **Sync on submit** is on, results are already in `/home/rcr/teleop_data/annotations`
- Otherwise: go to Target Storage and click **Sync**
- Zip them for handoff:
  ```bash
  cd "/home/rcr/teleop_data"
  zip -r annotations.zip annotations/
  ```

If you are in the RCR lab - let us know and we will grab the annotated zip file from your ip address.
---

## Quick troubleshooting

- **Broken thumbnails:** open DevTools → Network on a task; image URLs should look like
  `/data/local-files/?d=images/<file>`
  - **404** → check `LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT` and path mapping
  - **403** → permissions (ensure read/execute on folders; write on `/home/rcr/teleop_data/annotations`)
- **Cursor opens `localhost` automatically:** that’s just port-forwarding, its possible that port forwarding might cause an issue so you could directly visit this your Pi IP address (`http://192.168.1.n:8080`)
- **Verify env vars before start:**
  ```bash
  env | grep LABEL_STUDIO
  ```

---

## Additional Troubleshooting

- **Broken images not re-syncing**
  If you fixed your configuration but tasks/images still won’t refresh, try deleting all tasks and re-syncing the storage:
  1. Go to **Projects → Your Project**.
  2. In the Data Manager, **Select All** tasks.
  3. In the **Actions** dropdown, choose **Delete Tasks**.
  4. Return to **Settings → Cloud Storage → Source (Local files)** and click **Sync** again.

- **Nuclear option (reset Label Studio state)**
  Label Studio stores app state (including your login) in a local **SQLite** database under `/home/rcr/.local/share/label-studio/`. You can wipe and start fresh if something is corrupted. **This does not delete your image/annotation folders** under `/home/rcr/teleop_data/...`.
  ```bash
  # Stop Label Studio (ignore errors if not running)
  pkill -f label-studio || true

  # Remove LS state (DB + caches) and recreate the folder
  rm -rf /home/rcr/.local/share/label-studio
  mkdir -p /home/rcr/.local/share/label-studio

  # Restart Label Studio with your env vars set (see setup above)
  cd ~/LabelMaker && source bin/activate
  export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
  export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT="/home/rcr/teleop_data"
  label-studio start -p 8080 -b
  ```

---

## Cloud-Based Alternative: Roboflow (if Label Studio is uncooperative)

If Label Studio continues to block you, you can annotate in the cloud with **Roboflow**. First, copy your images off the Pi to your computer, then upload to Roboflow.

Roboflow is a cloud based platform with a free tier which include 1,000 *public* annotations. Robflow also has other powerful integrated ML tools to help automated your data labeling process with predictive labeling which go beyond the capabilities of Label Studio.

### Step A — Copy images from the Pi to your computer

**macOS / Linux / Windows WSL (Terminal):**
```bash
# Replace `n` with your robot/Pi's last IP octet and adjust the destination path if desired
scp -r rcr@192.168.1.n:/home/rcr/teleop_data/images "/Users/$USER/Downloads/teleop_images"
```

**Windows (PowerShell with OpenSSH):**
```powershell
# Replace n with your Pi's last IP octet and YourName with your Windows username
scp -r rcr@192.168.1.n:/home/rcr/teleop_data/images "C:\Users\YourName\Downloads\teleop_images"
```
> Prefer a GUI? Use **WinSCP**: create a new SFTP connection to `rcr@192.168.1.n`, navigate to `/home/rcr/teleop_data/images`, and drag the folder to your PC.

### Step B — Upload & Annotate in Roboflow
- **[Roboflow Annotate Docs](https://docs.roboflow.com/annotate)**
- Create an account at [Roboflow.com](https://app.roboflow.com/login)
- Create a **New Project** (e.g., *Object Detection* for bounding boxes).
- **Upload** your copied images folder.
- Use the browser-based annotator to draw boxes and assign labels (e.g., purple ball, green ball).
- When done, you can **Export** in your preferred training format (e.g., YOLO) from Roboflow’s dataset settings.
