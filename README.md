# COLMAG-seminar-SS26
[TUM Seminar] "Collaborative Robotics and Assistive Technology for Advanced Human-Robot Interaction"

## Before Starting
1. Create a Branch in the Lab Repo with your Name
2. You Must work only on YOUR BRANCH
3. The main branch is used and managed SOLELY by the instructors

## The HOLY RULES
1. Do you think your solution is the best? Maybe not, discuss it with the others and with us; try to improve it! 
2. Keep your code tidy and well documented, not only in all the scripts you produce, but also in the repo's README.MD of your folder
3. Keep the repo clean and updated.

---

## Basic Git Workflow

Git is used to track changes in the project and to collaborate with others.  
Each student must work on their own branch and avoid pushing directly to `main`.

### 1. Clone the Repository

To download the repository to your computer:

```bash
git clone <repository-url>
```

Then enter the repository folder:

```bash
cd COLMAG-seminar-SS26
```

---

### 2. Check the Current Branch

Before working, always check which branch you are using:

```bash
git branch
```

The current branch is marked with `*`.

---

### 3. Create Your Own Branch

Create a branch using your name:

```bash
git checkout -b your-name
```

Example:

```bash
git checkout -b mario-rossi
```

You should work only on your own branch.

---

### 4. Switch Between Branches

To switch to an existing branch:

```bash
git checkout branch-name
```

Example:

```bash
git checkout mario-rossi
```

---

### 5. Check the Status of Your Files

Before committing, check which files were modified:

```bash
git status
```

This shows new, modified, and deleted files.

---

### 6. Add Files to a Commit

To add a specific file:

```bash
git add filename.py
```

To add all modified files:

```bash
git add .
```

Use `git add .` carefully. Make sure you are not adding unnecessary files.

---

### 7. Commit Your Changes

A commit saves a snapshot of your changes.

```bash
git commit -m "Add serial port reading script"
```

Use clear commit messages that explain what was changed.

Good examples:

```bash
git commit -m "Add magnetometer data parser"
git commit -m "Fix serial packet validation"
git commit -m "Update README with setup instructions"
```

Bad examples:

```bash
git commit -m "update"
git commit -m "stuff"
git commit -m "final version"
```

---

### 8. Push Your Branch to GitHub

The first time you push your branch:

```bash
git push -u origin your-name
```

After that, you can simply use:

```bash
git push
```

---

### 9. Pull the Latest Changes

To update your local branch with changes from the remote repository:

```bash
git pull
```

If you need to update your branch with the latest changes from `main`:

```bash
git checkout main
git pull
git checkout your-name
git merge main
```

Resolve conflicts carefully if Git reports any.

---

**For all previous instructions/commands, you can alternatively use the Github Desktop app available here https://desktop.github.com/download/**


### 10. Do Not Work Directly on `main`

The `main` branch is managed only by the instructors.

Before editing files, always check your branch:

```bash
git branch
```

If you are on `main`, switch to your own branch:

```bash
git checkout your-name
```

---

## Recommended Software Setup

### Install Python

Python is required for the programming tasks in this seminar.

Download and install Python from:

```text
https://www.python.org/downloads/
```

During installation on Windows, make sure to select:

```text
Add Python to PATH
```

After installation, check that Python is available:

```bash
python --version
```

or:

```bash
python3 --version
```

You should also check that `pip` is installed:

```bash
pip --version
```

or:

```bash
pip3 --version
```

---

### Create a Python Virtual Environment

It is recommended to use a virtual environment for this project.

Create a virtual environment:

```bash
python -m venv venv
```

or:

```bash
python3 -m venv venv
```

Activate it on Windows:

```bash
venv\Scripts\activate
```

Activate it on Linux or macOS:

```bash
source venv/bin/activate
```

When the virtual environment is active, install the required packages inside it.

Example:

```bash
pip install pyserial matplotlib numpy
```

To deactivate the environment:

```bash
deactivate
```

---

### Install Visual Studio Code

Visual Studio Code is the recommended code editor for this seminar.

Download it from:

```text
https://code.visualstudio.com/
```

Useful VS Code extensions:

1. **Python**  
   Provides Python language support, debugging, and environment selection.

2. **Pylance**  
   Provides improved Python code analysis and autocomplete.

3. **GitLens**  
   Helps visualize Git history and changes.

4. **Jupyter**  
   Useful if you want to work with notebooks.

5. **GitHub Pull Requests**  
   Useful for GitHub-based collaboration.

---

## AI Coding Assistant Extensions

You may use AI coding assistants to help with programming, debugging, and documentation.  
However, you are responsible for understanding the code you submit.

Useful AI coding extensions include:

1. **GitHub Copilot**
2. **ChatGPT (Codex) extension for VS Code**
3. **Claude Code**
4. **Codeium / Windsurf**
5. **Cursor editor**
6. **Antigravity Gemini**

AI tools can help with:

- explaining errors,
- suggesting code structure,
- writing documentation,
- debugging,
- refactoring,
- generating simple examples.

However, do not blindly copy generated code. Always test it and make sure you understand what it does.

---

## Using `.gitignore`

The `.gitignore` file tells Git which files or folders should not be tracked.

This is useful for excluding temporary files, virtual environments, cache files, large generated files, and local configuration files.

### Common Files to Ignore

For a Python project, a typical `.gitignore` should include:

```gitignore
# Python cache files
__pycache__/
*.pyc
*.pyo

# Virtual environments
venv/
.venv/
env/

# VS Code settings
.vscode/

# Jupyter Notebook checkpoints
.ipynb_checkpoints/

# Operating system files
.DS_Store
Thumbs.db

# Logs
*.log

# Temporary files
*.tmp
*.temp

# Build and distribution folders
build/
dist/
*.egg-info/
```

---

### Important Notes About `.gitignore`

`.gitignore` only prevents new files from being tracked.

If a file was already committed before being added to `.gitignore`, Git will still track it.

To stop tracking a file that is already committed:

```bash
git rm --cached filename
```

To stop tracking a folder that is already committed:

```bash
git rm -r --cached foldername
```

Then commit the change:

```bash
git commit -m "Update gitignore"
```

Example:

```bash
git rm -r --cached venv
git commit -m "Remove virtual environment from tracking"
```

---

## Good Repository Practices

Keep your branch clean and organized.

Recommended practices:

1. Commit often, but with meaningful messages.
2. Do not commit generated files, cache files, or virtual environments.
3. Keep your scripts readable and documented.
4. Update the README in your own folder when you add important code.
5. Pull regularly to keep your branch updated.
6. Test your code before pushing.
7. Ask for help when you are unsure about Git conflicts or code structure.

---

## Useful Git Commands Summary

| Command | Description |
|---|---|
| `git clone <url>` | Download the repository |
| `git branch` | Show available branches |
| `git checkout -b <branch>` | Create and switch to a new branch |
| `git checkout <branch>` | Switch to an existing branch |
| `git status` | Show changed files |
| `git add <file>` | Add a file to the next commit |
| `git add .` | Add all changed files |
| `git commit -m "message"` | Commit changes |
| `git push` | Upload changes to GitHub |
| `git pull` | Download latest changes |
| `git merge <branch>` | Merge another branch into the current branch |
| `git log --oneline` | Show commit history |
| `git diff` | Show file differences before committing |

---

## Final Reminder

Before starting any work, always check:

```bash
git branch
```

**Make sure you are working on your own branch, not on `main`.**

---

## Magnetometer Data Reader

This project includes a Python program for reading magnetometer data from a serial port, as required for the Collaborative Robotics seminar.

### Files

- `magnetometer_reader.py` — Main program for reading, visualizing, and classifying magnetometer data
- `csv_visualizer.py` — Script for visualizing recorded CSV data with multiple plot types
- `test_components.py` — Test script to verify program components
- `requirements.txt` — Python dependencies
- `ros/` — ROS 1 Noetic Docker setup and `colmag_ros` catkin package (see ROS section below)

### Setup

1. Create and activate a Python virtual environment:
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

   For ROS WebSocket publishing on Mac (without a native ROS install):
   ```bash
   pip install roslibpy
   ```

### Running the Program

1. Connect your magnetometer hardware to a serial port
2. Run the program (recommended lab command):
   ```bash
   python magnetometer_reader.py --clean --writing-max-z 0.05
   ```

#### Command Line Options

**Input / serial**

- `--sensor, -s`: Sensor number to plot (1-16, default: 1)
- `--baudrate, -b`: Serial baudrate (default: 921600)
- `--input-source`: `serial` for the real sensor or `touchpad` for the Mac trackpad simulator (default: serial)

**CSV / output**

- `--csv, -c`: Explicit continuous raw CSV output filename (default: off unless `--record-data`)
- `--no-csv`: Disable CSV logging for maximum live read rate

**Touchpad simulator**

- `--touchpad-ink-mode`: `velocity` draws from movement (default), `pen` draws only while clicking/holding space
- `--touchpad-magnetic-calibration`: Calibration JSON from `calibrate_touchpad_magnetics.py` for realistic synthetic fields
- `--touchpad-sample-rate`: Samples per second (default: 100)
- `--touchpad-ink-strength`: 0.0–1.0 opacity of touchpad ink (default: 1.0)

**Projection image**

- `--trail-length`: Number of recent pose samples used for the 3D trail and 2D character image (default: 250)
- `--image-size`: Projection image size in pixels (default: 64)
- `--image-dir`: Directory for projected character PNGs (default: digit_images)
- `--z-close-mode`: How Z maps to stroke darkness; `max` means larger Z is darker (default)

**Classifier**

- `--no-classifier`: Run without loading the real-time EasyOCR character classifier
- `--classifier-gpu`: Use GPU for EasyOCR if available
- `--classifier-labels`: Character set: `alphanumeric`, `digits`, `letters`, or a custom subset such as `ABCXLRUD0123` (default: alphanumeric)
- `--classifier-interval`: Minimum seconds between background OCR requests (default: 0.75)
- `--classifier-mode`: `on-idle` runs OCR after a writing pause; `continuous` runs at `--classifier-interval` (default: on-idle)
- `--classifier-idle-seconds`: Writing pause before OCR fires in `on-idle` mode (default: 0.8)
- `--classifier-min-ink-samples`: Minimum ink samples required before live OCR can run (default: 8)
- `--letter-labels`: Label set for letter mode via virtual joystick **L** (default: letters)
- `--digit-labels`: Label set for digit mode via virtual joystick **R** (default: digits)
- `--joystick-dwell-seconds`: Seconds the cursor must dwell inside a virtual button to press it (default: 1.5)

**Writing filter**

- `--no-writing-filter`: Draw every pose sample (no velocity/Z gating)
- `--writing-min-velocity`: Minimum XY pose velocity to count as writing. Auto-set to **0.04** for serial (real sensor) and **0.08** for touchpad if not explicitly passed
- `--writing-max-velocity`: Optional upper velocity cap; faster moves become pen-up (default: disabled)
- `--writing-min-closeness`: Normalized Z closeness gate 0..1 for contact mode (default: disabled)
- `--writing-max-z`: **Do not draw when |pose_z| exceeds this value in meters.** E.g. `0.05` suppresses ink when the magnet is more than 5 cm from the sensor — essential for clean air-writing with the real sensor

**View**

- `--clean`: **Clean view** — show only the drawing canvas + classifier panel; hides raw Bx/By/Bz traces and 3D trajectory graph. Reduces CPU load on the real sensor setup

**ROS**

- `--ros`: Publish sensor data, pose, and OCR results to ROS 1 topics. Uses native `rospy` inside Docker, or `roslibpy` WebSocket bridge when running on Mac

**Mode flags**

- `--record-data`: Explicitly enable saving CSV/PNG/JSON session files (default: live view only)
- `--dry-run`: Skip hardware setup, create output layout + manifest, then exit
- `--validation-mode`: Connect to sensor, print packet health stats for 60 s, no files saved
- `--output-dir`: Base output directory (default: `data/lab_YYYY-MM-DD/`)
- `--run-id`: Optional prefix for session filenames (e.g. `run_001`)

**Mode selection rules:**

| Mode Flag | Sensor Connects? | Files Saved? | Use Case |
|-----------|-----------------|-------------|----------|
| (none) | ✓ | ✗ (unless `--csv`) | Live view, test classifier, practice |
| `--record-data` | ✓ | ✓ | Real data capture for training |
| `--dry-run` | ✗ | layout + manifest only | Verify paths before hardware |
| `--validation-mode` | ✓ | ✗ | Check sensor health |

#### Session Recording Controls

**Serial / real-sensor mode:**

| Key | Action |
|-----|--------|
| `0` – `9` | Start recording that digit |
| `A` – `Z` | Start recording that letter |
| `-` | Start `control_blank` (no magnet, stationary) |
| `=` | Start `control_still` (magnet held still) |
| `s` | Stop & save current session |
| `q` | Quit |

**Touchpad simulator mode (additional controls):**

| Key | Action |
|-----|--------|
| `A` – `Z` (excl. p/s/q) | Start recording that letter |
| `p` / `s` / `q` | Record letter **P** / **S** / **Q** |
| `Shift+P` | Toggle pen on/off |
| `Shift+S` | Stop & save current session |
| `Shift+Q` | Quit |
| `Shift+L` | Switch to letters OCR mode |
| `Shift+D` | Switch to digits OCR mode |
| `Shift+R` | Reset canvas |
| Space / click | Draw (pen mode) |

With `--record-data`, each session creates a matched CSV, grayscale PNG, and JSON sidecar under `data/lab_YYYY-MM-DD/samples/<label>/`. The run-level `manifest.json` records paths, settings, command line, and git metadata.

#### Examples

```bash
# Recommended lab command: clean view, suppress ink when magnet > 5 cm away
python magnetometer_reader.py --clean --writing-max-z 0.05

# Same but also publish classifier results to ROS
python magnetometer_reader.py --clean --writing-max-z 0.05 --ros

# Validation mode — check sensor health, no files saved
python magnetometer_reader.py --validation-mode

# Record data with structured output (auto date-based directory)
python magnetometer_reader.py --record-data --output-dir data/lab_2026-05-20 --run-id run_001

# Record using the touchpad simulator for offline practice
python magnetometer_reader.py --input-source touchpad --record-data --output-dir data/practice_2026-05-20

# Touchpad with click-to-write instead of velocity-to-write
python magnetometer_reader.py --input-source touchpad --touchpad-ink-mode pen

# Dry run — test the data layout without any hardware
python magnetometer_reader.py --dry-run --record-data --run-id dry_test
```

### Session Recording Workflow

**Full recording command** (replace date/run-id as needed):

```bash
python magnetometer_reader.py \
  --input-source serial \
  --clean \
  --writing-max-z 0.05 \
  --record-data \
  --output-dir data/lab_$(date +%Y-%m-%d)/ \
  --run-id run_001
```

**Keys during recording:**

| Key          | Action                                    |
|--------------|-------------------------------------------|
| `0` – `9`   | Start recording that digit                |
| `A` – `Z`   | Start recording that letter               |
| `-`          | Start `control_blank` (no magnet, stationary) |
| `=`          | Start `control_still` (magnet held still) |
| `s`          | Stop & save current session               |
| `q`          | Quit                                      |

**What to record** (38 target labels, aim for ≥ 5 reps each):

```
Digits  : 0 1 2 3 4 5 6 7 8 9
Letters : A B C D E F G H I J K L M
          N O P Q R S T U V W X Y Z
Controls: blank(-) still(=)
```

After every `s` the terminal prints a live checklist:

```
============================================================
RECORDING PROGRESS  (target: 5 reps each, v = done)
  Digits:   0[3 ] 1[5v] 2[2 ] ...
  Letters:  A[5v] B[3 ] C[0 ] ...
  Controls: blank(-)[2 ]  still(=)[0 ]
  Done: 3/38 labels | 15/190 reps total
  Still needed: 0 2 3 ...
============================================================
```

The checklist tracks reps within the current run. If you restart the program for a new run, the counter resets — that is intentional so each run ID is independent.

**Output layout:**

```
data/lab_2026-05-20/
├── manifest.json           ← lists every session with paths, git hash, settings
├── raw/
│   └── run_001_raw.csv     ← continuous magnetic stream (all 48 channels + pose)
└── samples/
    ├── digit_5/
    │   ├── run_001_digit_5_rep001_20260520_143022.csv
    │   ├── run_001_digit_5_rep001_20260520_143022.png
    │   └── run_001_digit_5_rep001_20260520_143022.json
    ├── letter_A/
    │   └── ...
    └── control_blank/
        └── ...
```

Each JSON sidecar includes git metadata, settings snapshot, and classifier prediction.
Control labels are stored as separate folders (`control_blank`, `control_still`), not mixed with characters.
For `control_still`, add `--no-writing-filter` so the stationary samples appear in the PNG projection.

### Touchpad Magnetic Calibration

> **Full guide: see [CALIBRATION_GUIDE.md](CALIBRATION_GUIDE.md)**

The touchpad simulator generates synthetic magnetic fields matching a 4×4 dipole grid.
Calibrating maps that synthetic output onto the real sensor channel scale/offset, so
touchpad recordings are a realistic drop-in for real hardware data.

**Step 1 — Record calibration CSV** (real sensor, ~2 min slow scan):

```bash
python magnetometer_reader.py \
  --input-source serial \
  --csv data/lab_$(date +%Y-%m-%d)/raw/magcal_raw.csv
```

Move the magnet slowly: all four corners, edges, centre, at 2–3 heights. `Ctrl+C` to stop.

**Step 2 — Fit the calibration:**

```bash
python calibrate_touchpad_magnetics.py \
  data/lab_2026-05-20/raw/magcal_raw.csv
```

Prints a per-sensor RMSE table + quality badge (`EXCELLENT / GOOD / ACCEPTABLE / POOR`).
Output: `data/lab_2026-05-20/raw/touchpad_magnetic_calibration.json`

**Step 3 — Use it with the touchpad simulator:**

```bash
python magnetometer_reader.py \
  --input-source touchpad \
  --touchpad-magnetic-calibration data/lab_2026-05-20/raw/touchpad_magnetic_calibration.json \
  --record-data \
  --output-dir data/lab_2026-05-20/ \
  --run-id run_001
```

This calibrates per-channel scale and offset — not a full physics model, but makes touchpad
magnetic rows much more realistic for CSV saving and downstream pipeline testing.

## ROS 1 Integration

The project publishes live sensor data and OCR results to ROS 1 Noetic topics. This allows a robot (or any ROS node) to listen for gesture commands without needing to run the Python visualizer itself.

### Topics published

| Topic | Type | Content |
|-------|------|---------|
| `/colmag/command` | `std_msgs/String` | UI commands (`canvas:reset`, `letter_detection`, `number_detection`, `choice:0` …) |
| `/colmag/classifier` | `std_msgs/String` | Top predicted character label |
| `/colmag/confidence` | `std_msgs/Float64` | Confidence of top prediction (0–1) |
| `/colmag/sensor_data` | `std_msgs/Float64MultiArray` | Raw Bx/By/Bz (48 floats) + pose (6 floats) |
| `/colmag/pose` | `geometry_msgs/PoseStamped` | Computed XYZ position of the magnet |

### Running inside Docker (ROS Noetic — native rospy)

A Docker setup is provided under `ros/` for running a standalone ROS node on any machine (including Apple Silicon).

```bash
cd ros/

# First-time setup — builds the Docker image and catkin workspace
bash docker_setup.sh

# Open a shell in the container
bash docker_connect.sh

# Inside the container — start rosbridge + listen for commands
roslaunch rosbridge_server rosbridge_websocket.launch &
rosrun colmag_ros colmag_listener.py
```

### Running on Mac (roslibpy WebSocket bridge)

If you have no local ROS install, `magnetometer_reader.py` automatically falls back to **roslibpy**, which communicates with the Docker container over a WebSocket:

```bash
pip install roslibpy

# In the Docker container (one terminal):
roslaunch rosbridge_server rosbridge_websocket.launch

# On the Mac (another terminal):
python magnetometer_reader.py --clean --writing-max-z 0.05 --ros
```

The Mac process connects to `localhost:9090` (the rosbridge port, forwarded by OrbStack/Docker). All five topics become available inside the container immediately.

### Testing the listener

`ros/colmag_ros/scripts/colmag_listener.py` is a minimal test subscriber. Run it inside the container to verify the full pipeline:

```bash
rosrun colmag_ros colmag_listener.py
```

It prints a human-readable message whenever a command arrives, e.g.:

```
[INFO] *** CONFIRMED: "A" (94.3%) — robot would execute this command ***
[INFO] Mode switched: letters OCR
[INFO] Canvas reset
```

---

## Data Processing Pipeline

For the current week 4 implementation plan, see `PROJECT_TASKS.md`.

The project implements a complete pipeline for processing magnetometer data and classifying 2D magnet trajectories as characters. The pipeline consists of the following stages:

1. **Data Acquisition**:
   - Reads serial data from the magnetometer hardware at high baudrates (default: 921600).
   - Parses packets containing magnetic field data (48 floats for 16 sensors × 3 axes) and pose data (6 floats for position and orientation).
   - Buffers data in memory for real-time processing.

2. **Real-time Visualization and Logging**:
   - Plots magnetic field components (Bx, By, Bz) for a selected sensor in real-time.
   - In `--record-data` mode, logs the continuous raw stream to `data/lab_YYYY-MM-DD/raw/<run_id>_raw.csv`.
   - Displays 3D pose trajectories and live projections.

3. **Session Recording**:
   - Allows users to record specific character-drawing sessions by pressing digit keys (0-9) or letter keys (A-Z).
   - Captures pose trajectories during the session for later processing.

4. **2D Projection**:
   - Projects 3D pose trails (x, y, z coordinates) onto a 2D grayscale image.
   - Maps Z-values to stroke darkness, creating OCR-ready character images (default: 64x64 pixels).
   - Saves projected images as PNG files next to their session CSV/JSON sidecars under `data/lab_YYYY-MM-DD/samples/<exact_label>/`.

5. **Character Classification**:
   - Uses EasyOCR (pretrained OCR model) to classify the 2D projected images as digits, letters, or a smaller custom command alphabet.
   - Provides real-time predictions with confidence scores and smoothed results.
   - Supports GPU acceleration for faster inference.

6. **Post-Processing Analysis**:
   - Uses `csv_visualizer.py` to analyze recorded CSV data with various plot types (Z-axis overview, sensor analysis, pose data, 3D trajectories, statistics).

This pipeline enables the classification of handwritten characters drawn via magnetometer pose trajectories, facilitating applications in human-robot interaction and assistive technology.

### Character Classifier

The project can use EasyOCR as a pretrained character recognizer for the live 64x64 projection. This avoids collecting a custom training set and avoids training a local model.

Install dependencies:

```bash
pip install -r requirements.txt
```

The first EasyOCR run may download pretrained OCR weights. After that, inference uses the cached model.

Run the magnetometer reader with real-time OCR classification:

```bash
python magnetometer_reader.py --no-csv
```

Do not pass `--no-classifier` when testing predictions. With `--no-classifier`, the virtual joystick still switches modes and queues the active label set, but the right classifier panel will stay disabled because EasyOCR was intentionally not loaded.

EasyOCR is relatively slow, so live OCR runs in a background worker. By default it runs only after you stop writing briefly, which keeps the joystick/cursor responsive and avoids spending CPU while the trajectory is still changing. Use `--classifier-mode continuous` to restore repeated live OCR, and increase `--classifier-interval` if your machine lags.

The live UI uses a blitted 2D update path for the writing surface, classifier panel, and sensor traces. The 3D axis is kept static during blitting because Matplotlib's `mplot3d` artists are backend-fragile. Performance-sensitive pieces include `--display-window`, the incremental OCR canvas, a precomputed brush kernel, throttled sensor-axis limit updates, and cached joystick text.

The live classifier panel shows the top prediction, runner-up, and a top-character confidence chart. By default the classifier accepts digits 0-9 and letters A-Z; use `--classifier-labels digits`, `--classifier-labels letters`, or a custom subset such as `--classifier-labels ABCXLRUD0123` to narrow the recognition set. For dataset recording, that custom subset should be the full union of the characters you plan to record. The projected magnetometer image has a white background and dark strokes; the EasyOCR wrapper upsamples and autocontrasts the image before recognition.

The final robot command vocabulary does not need all 26 letters. Prefer a small set of characters that are reliable in air-writing and map cleanly to robot tasks. Non-OCR shapes such as a star should be handled by a future gesture/template recognizer rather than EasyOCR.

The live interface now includes two virtual joystick groups on the writing surface. Dwelling on **L** switches to letter detection, dwelling on **R** switches to number detection, **D** resets the writing canvas, and **1/2/3/4** confirm one of the four current classifier candidates. The joystick/app-mode logic lives in `colmag/interaction.py` so the UI, OCR, and future robot adapter can stay separate.

The OCR image is filtered before classification: by default a pose sample only becomes ink when the magnet moves fast enough in X/Y. This prevents a resting magnet from becoming a dot while still supporting air-writing. For multi-stroke letters, tune `--writing-max-velocity` so fast repositioning motions are treated as pen-up, then draw actual strokes with slower controlled motion. `--writing-min-closeness` is still available for board/contact experiments, but it is disabled by default.

To skip classification:

```bash
python magnetometer_reader.py --no-classifier
```

To try GPU acceleration if available:

```bash
python magnetometer_reader.py --classifier-gpu
```

### CSV Data Visualization

After recording sessions, use the CSV visualizer to analyze your data:

```bash
python csv_visualizer.py no_magnet_20260424_143022.csv
```

#### Visualization Options

The visualizer provides multiple plot types:
- **Z-axis overview**: Shows Z-component for all 16 sensors
- **Single sensor analysis**: Bx, By, Bz components for one sensor
- **Pose data**: Position and orientation over time
- **3D trajectory**: Spatial movement patterns
- **Statistical analysis**: Mean, std, min/max values

#### Visualizer Examples

```bash
# Visualize all sensors Z-axis
python csv_visualizer.py data.csv --plot z_all

# Analyze single sensor with all components
python csv_visualizer.py data.csv --plot sensor --sensor 5

# Show pose data
python csv_visualizer.py data.csv --plot pose

# Statistical summary
python csv_visualizer.py data.csv --plot stats
```

### Data Format

The CSV file contains rows with the following format:
- **Magnetic field data** (48 values): Bx1, By1, Bz1, ..., Bx16, By16, Bz16 (16 sensors × 3 axes)
- **Pose data** (6 values): x, y, z, mx, my, mz

### Measurement Units

**CRITICAL:** The magnetic field measurement units depend on your specific magnetometer hardware. You must check your sensor's datasheet for exact specifications.

#### Common Magnetometer Units:

| Unit | Symbol | Conversion | Typical Earth Field | Typical Sensor Range |
|------|--------|------------|-------------------|-------------------|
| **Tesla** | T | 1 T = 10,000 G | ~0.00005 T | 0.00001 - 0.0001 T |
| **Gauss** | G | 1 G = 0.0001 T | ~0.5 G | 0.0001 - 0.001 G |
| **milliGauss** | mG | 1 mG = 0.001 G | ~500 mG | 0.1 - 1.0 mG |
| **microTesla** | μT | 1 μT = 0.000001 T | ~50 μT | 10 - 100 μT |

#### How to Identify Your Units:

1. **Check your magnetometer datasheet** - This is the authoritative source
2. **Look for Earth field values** - Earth's magnetic field is ~25-65 μT or ~0.25-0.65 G
3. **Test with known magnets** - Strong magnets should show large changes
4. **Contact sensor manufacturer** if documentation is unclear

#### Coordinate System:

The Bx, By, Bz axes depend on your sensor's orientation. Check the datasheet for:
- Which axis points in which direction
- How the sensor should be mounted
- Any calibration requirements

### Packet Structure

- **Header**: 0xAA (1 byte)
- **Data**: 54 floats (216 bytes, little-endian)
- **Tail**: 0xBB (1 byte)
- **Total packet size**: 218 bytes

### Testing

Run the component tests to verify functionality:

```bash
python test_components.py
```

This tests port listing, packet parsing, and CSV writing without requiring hardware.

### Recent changes

- Corrected the 3D plot scale in `magnetometer_reader.py`: the pose trajectory axis limits now use ±0.05 instead of ±0.5.

### Safety

- Press `Ctrl+C` to safely close the serial port and exit
- The program uses threading locks for thread-safe data access
- Data is buffered in memory for plotting and saved to disk
