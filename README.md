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
2. Run the program:
   ```bash
   python magnetometer_reader.py
   ```

3. The program will:
   - List available serial ports
   - Let you choose the correct port
   - Open the port at 921600 baud
   - Start a reading thread at 100 Hz
   - Parse incoming packets (header 0xAA, 54 floats, tail 0xBB)
   - Display decoded data in terminal
   - Save data to CSV file (`magnetometer_data.csv`)
   - Show real-time plots using matplotlib

### Data Format

The CSV file contains rows with the following format:
- **Magnetic field data** (48 values): Bx1, By1, Bz1, ..., Bx16, By16, Bz16 (16 sensors × 3 axes)
- **Pose data** (6 values): x, y, z, mx, my, mz

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

### Safety

- Press `Ctrl+C` to safely close the serial port and exit
- The program uses threading locks for thread-safe data access
- Data is buffered in memory for plotting and saved to disk