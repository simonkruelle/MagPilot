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

- `magnetometer_reader.py` - Main program for reading and processing magnetometer data
- `csv_visualizer.py` - Script for visualizing recorded CSV data with multiple plot types
- `test_components.py` - Test script to verify program components
- `requirements.txt` - Python dependencies

### Setup

1. Activate your conda environment:
   ```bash
   conda activate simon
   ```

2. Install dependencies (if not already installed):
   ```bash
   pip install -r requirements.txt
   ```

### Running the Program

1. Connect your magnetometer hardware to a serial port
2. Run the program with optional parameters:
   ```bash
   python magnetometer_reader.py --sensor 5 --csv experiment1.csv
   ```

#### Command Line Options

- `--sensor, -s`: Sensor number to plot (1-16, default: 1)
- `--baudrate, -b`: Serial baudrate (default: 921600)
- `--csv, -c`: CSV output filename (default: magnetometer_data.csv)
- `--no-csv`: Disable CSV logging for maximum live read rate
- `--trail-length`: Number of recent pose samples used for the 3D trail and 2D character image (default: 250)
- `--image-size`: Projection image size in pixels (default: 64)
- `--image-dir`: Directory for projected character PNGs (default: digit_images)
- `--z-close-mode`: How Z maps to stroke darkness; `max` means larger Z is darker (default)
- `--no-classifier`: Run without loading the real-time EasyOCR character classifier
- `--classifier-gpu`: Use GPU for EasyOCR if available
- `--classifier-labels`: Character set for EasyOCR recognition: `alphanumeric`, `digits`, `letters`, or a custom subset such as `ABCX0123` (default: alphanumeric)
- `--classifier-interval`: Minimum seconds between background OCR requests (default: 0.75)
- `--classifier-mode`: Run live OCR after a writing pause (`on-idle`) or continuously at `--classifier-interval` (default: on-idle)
- `--classifier-idle-seconds`: Writing pause duration before OCR runs in `on-idle` mode (default: 0.8)
- `--classifier-min-ink-samples`: Minimum ink samples required before live OCR can run (default: 8)
- `--letter-labels`: Label set used after holding virtual **L** for letter mode (default: letters)
- `--digit-labels`: Label set used after holding virtual **R** for number mode (default: digits)
- `--joystick-dwell-seconds`: Seconds the cursor must dwell inside a virtual button to press it (default: 2.0)
- `--no-writing-filter`: Disable writing/hover filtering and draw every pose sample into the OCR image
- `--writing-min-velocity`: Minimum XY pose velocity for a sample to count as writing (default: 0.01)
- `--writing-max-velocity`: Optional maximum XY pose velocity; faster moves are treated as repositioning/pen-up (default: disabled)
- `--writing-min-closeness`: Optional normalized Z closeness gate for board/contact mode, 0..1 (default: disabled for air-writing)

#### New Mode Flags (Lab Preparation)

These flags control what happens when you run the program:

- `--record-data`: Explicitly enable saving CSV/PNG/JSON session files. Without this flag, the program runs in **live view only** — sessions can be started/stopped for live classification, but no files are written to disk.
- `--dry-run`: Skip sensor and touchpad hardware setup entirely. Shows the plot layout for 5 seconds and exits. Perfect for verifying the visualization layout works before connecting hardware.
- `--validation-mode`: Connect to the real sensor but do NOT save any files. Prints packet health stats (packet rate, bad packet ratio) for 60 seconds. Run this at the start of every lab session to verify the sensor link is healthy.
- `--output-dir`: Base directory for recorded session data. Defaults to `data/lab_YYYY-MM-DD/` using today's date, so each lab day gets its own folder automatically.
- `--run-id`: Optional prefix for session filenames (e.g., `run_001`). Useful when recording multiple runs in the same lab session.

**Mode selection rules:**
| Mode Flag | Sensor Connects? | Files Saved? | Use Case |
|-----------|-----------------|-------------|----------|
| (none) | ✓ | ✗ | Live view, test classifier, practice |
| `--record-data` | ✓ | ✓ | Real data capture for training/analysis |
| `--dry-run` | ✗ | ✗ | Verify plot layout, test flags |
| `--validation-mode` | ✓ | ✗ | Check sensor health before recording |

#### Session Recording Controls

The program supports recording separate character drawing sessions:

- **0-9**: Start recording that digit
- **A-Z**: Start recording that letter (lowercase **s** and **q** are reserved for controls; uppercase **S** and **Q** record letters)
- **s**: Stop current recording session and save CSV + projected PNG
- **q**: Quit program

Each session creates a separate CSV file, a 64x64 grayscale PNG in `digit_images/`, and a JSON sidecar containing the projection/filter settings used for that image.

#### Examples

```bash
# Plot sensor 3 with default settings
python magnetometer_reader.py -s 3

# Use custom baudrate and output file
python magnetometer_reader.py --sensor 7 --baudrate 115200 --csv test_data.csv

# Default settings (plots sensor 1)
python magnetometer_reader.py

# Dry run — test the plot layout without any hardware
python magnetometer_reader.py --input-source touchpad --dry-run

# Validation mode — check sensor health, no files saved
python magnetometer_reader.py --validation-mode

# Record data with structured output (auto date-based directory)
python magnetometer_reader.py --record-data --output-dir data/lab_2026-05-20 --run-id run_001

# Record with touchpad simulator for offline practice
python magnetometer_reader.py --input-source touchpad --record-data --output-dir data/practice_2026-05-18
```

### Session Recording Workflow

1. Start the program normally
2. Press the digit or letter key you want to draw, e.g. **5** or **A**
3. Press **s** to stop recording
4. Repeat for more samples
5. Press **q** to quit

This creates files under `data/lab_YYYY-MM-DD/` organized by label:

```
data/lab_2026-05-20/
├── samples/
│   ├── digit_5/
│   │   ├── run_001_digit_5_20260520_143022.csv
│   │   ├── run_001_digit_5_20260520_143022_64px.png
│   │   └── run_001_digit_5_20260520_143022_64px.json
│   ├── letter_A/
│   │   ├── run_001_letter_A_20260520_144155.csv
│   │   ├── run_001_letter_A_20260520_144155_64px.png
│   │   └── run_001_letter_A_20260520_144155_64px.json
│   └── digit_0/
│       ├── run_002_digit_0_20260520_150312.csv
│       ├── run_002_digit_0_20260520_150312_64px.png
│       └── run_002_digit_0_20260520_150312_64px.json
```

Each JSON sidecar now includes **git metadata** (branch, commit hash, dirty status) for full reproducibility.

## Data Processing Pipeline

For the current week 4 implementation plan, see `PROJECT_TASKS.md`.

The project implements a complete pipeline for processing magnetometer data and classifying 2D magnet trajectories as characters. The pipeline consists of the following stages:

1. **Data Acquisition**:
   - Reads serial data from the magnetometer hardware at high baudrates (default: 921600).
   - Parses packets containing magnetic field data (48 floats for 16 sensors × 3 axes) and pose data (6 floats for position and orientation).
   - Buffers data in memory for real-time processing.

2. **Real-time Visualization and Logging**:
   - Plots magnetic field components (Bx, By, Bz) for a selected sensor in real-time.
   - Logs all data to CSV for offline analysis.
   - Displays 3D pose trajectories and live projections.

3. **Session Recording**:
   - Allows users to record specific character-drawing sessions by pressing digit keys (0-9) or letter keys (A-Z).
   - Captures pose trajectories during the session for later processing.

4. **2D Projection**:
   - Projects 3D pose trails (x, y, z coordinates) onto a 2D grayscale image.
   - Maps Z-values to stroke darkness, creating OCR-ready character images (default: 64x64 pixels).
   - Saves projected images as PNG files in the `digit_images/` directory.

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

The live classifier panel shows the top prediction, runner-up, and a top-character confidence chart. By default the classifier accepts digits 0-9 and letters A-Z; use `--classifier-labels digits`, `--classifier-labels letters`, or a custom subset such as `--classifier-labels ABCX0123` to narrow the recognition set. The projected magnetometer image has a white background and dark strokes; the EasyOCR wrapper upsamples and autocontrasts the image before recognition.

The final robot command vocabulary does not need all 26 letters. Prefer a small set of characters that are reliable in air-writing and map cleanly to robot tasks. Non-OCR shapes such as a star should be handled by a future gesture/template recognizer rather than EasyOCR.

The live interface now includes two virtual joystick groups on the writing surface. Dwelling on **L** switches to letter detection, dwelling on **R** switches to number detection, and **A/B/C/X/U/D** currently emit robot-command placeholders for the later ROS adapter. The joystick/app-mode logic lives in `colmag/interaction.py` so the UI, OCR, and future robot adapter can stay separate.

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
