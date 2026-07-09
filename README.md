<div align="center">

# COLMAG

### Air-write a character. Watch the robot move.

Magnetometer-based gesture control and teleoperation for a **Franka FR3 / Panda**
arm — draw letters and digits in the air (or on a trackpad) and the robot acts;
switch to teleop and steer the end-effector with the magnet itself.
Runs fully in **Gazebo simulation** inside Docker, or on the real robot.

![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-11-FF6A00)
![Robot](https://img.shields.io/badge/Franka-FR3%20%2F%20Panda-0A84FF)
![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)

</div>

---

## Setup (once)

Needs Docker and a Linux desktop (X11). NVIDIA GPU optional.

```bash
git clone <this-repo> && cd COLMAG-seminar-SS26
xhost +local:root
INSTALL_GAZEBO=1 bash ros/docker_setup.sh     # builds the image, starts the container
```

---

## 🚀 The Control Center app

Everything is driven from one window — no terminals needed:

```bash
python3 colmag_launcher.py
```

<img alt="Control Center" src="docs/launcher.png" width="600">

| Element | What it does |
|---|---|
| **Simulation / Real robot** | Chooses where the pipeline runs. Real mode asks for the robot IP and shows safety confirmations. |
| **1 · Robot — Start** | Spawns the FR3 in Gazebo (sim) or connects to the real arm (`fr3_real.launch`). |
| **2 · Arm nodes — Start** | Starts teleop + gesture nodes together. Uncheck **live** for a dry run (log only, no motion). |
| **3 · Interface — Start** | Opens the writing/teleop window. Pick `trackpad` (no hardware) or `magnetometer` (real sensor). |
| **Status dots** | Green when that stage is actually running (polled every 2 s). |
| **Stop all / Restart container** | Kill the pipeline · full clean reset. |
| **Log pane** | Live tail of the selected stage's output. |

Start **1 → 2 → 3**, wait for each dot to turn green. The arm homes on startup.

---

## Writing mode (gestures)

Draw a character in the Interface window, then dwell on the confirm button.
The legend beside the canvas shows the mapping:

| Letter | Action | Digit | Action |
|:---:|---|:---:|---|
| `A` | wave 👋 | `1`–`9` | park at slot 1…9 on a left→right line |
| `B` | bow 🙇 | `0` | home / reset |
| `C` | fist pumps 💪 | | *(nine evenly spaced positions at constant depth —* |
| `D` | dab 😎 | | *ideal for checking the digit classifier)* |
| `U` | stretch up 🙆 | | |
| `L` / `R` | point left / right | `X` | home |

## Teleoperation mode

Dwell on **Teleop**. The interface switches to a clean canvas with a taskbar on
top (**Draw** = exit · **Gripper** · **Layer**) and a live magnet gyroscope with
the thresholds and a height gauge. The taskbar strip is button-only — the arm
does not follow the cursor there.

| Magnet input | Controls | Simulate on the trackpad |
|---|---|---|
| move over the board | end-effector X/Y | move the cursor |
| **height** over board (0–5 cm) | end-effector height (0 cm = ground, 5 cm = default) | mouse **scroll wheel** |
| lift **above 5 cm** | teleop pauses — arm holds | scroll to the top |
| **tilt** ≥ 55° / ≤ 25° | gripper close / open | numpad **2** / **8** |
| **twist** (15° steps) | rotate the end-effector | numpad **4** / **6** |
| — | reset magnet upright | numpad **5** |

**Safety exits:** `Shift+E` leaves teleop immediately (trackpad *and* magnet
mode) and the arm returns to its neutral pose. `Ctrl+C` in any stage stops and
holds the arm.

---

## Manual commands (alternative to the app)

<details>
<summary>Run the three stages by hand</summary>

Each terminal: `bash ros/docker_connect.sh`, then:

```bash
# 1 — robot (sim)
roslaunch colmag_ros fr3.launch controller:=effort_joint_trajectory_controller
# 2 — arm nodes (teleop + gestures)
roslaunch colmag_ros colmag_arm_nodes.launch dry_run:=false
# 3 — interface
python3 magnetometer_reader.py --input-source trackpad --ros --classifier-labels ABCXLRUD0123
```

Real sensor instead of trackpad:
`python3 magnetometer_reader.py --clean --writing-max-z 0.05 --ros --classifier-labels ABCXLRUD0123`
</details>

---

## Real robot — staged safety pipeline

Never skip stages. Move on only when the current stage behaves exactly as
expected. Stop immediately on any wrong/repeated command, wrong `arm_id`, or
anyone inside the workspace.

| Stage | What | Moves the real arm? |
|:---:|---|:---:|
| 1 | Everything in **simulation** (the app, Simulation mode) | No |
| 2 | Real sensor + ROS, arm nodes **dry-run** (uncheck *live*) | No |
| 3 | One tiny supervised nudge: `rosrun colmag_ros fr3_simple_move.py _dry_run:=false` | ✳ tiny, supervised |
| 4 | One approved gesture, then the full stack, then teleop | ✳ supervised |

For the real connection use the app's **Real robot** mode (it runs
`fr3_real.launch robot_ip:=…`), or set `ROS_MASTER_URI` / `ROS_IP` for the lab
network inside the container first. Keep the E-stop reachable at all times.

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Stage dot stays gray | Check that stage's log in the app's log pane. |
| *"new node registered with same name"* | Two copies of a stage are running — **Stop all**, then start again. |
| Anything weird / stuck | **Restart container** in the app (bulletproof reset). |
| GUI window doesn't open | `xhost +local:root` on the host once. |
| `python: command not found` in container | Use `python3`. |
| Gripper won't reopen | Fixed via grasp→stop→move; if it recurs, toggle the taskbar Gripper button once. |

---

## Repository map

```
colmag_launcher.py          ← the Control Center app (run on the host)
magnetometer_reader.py      writing/teleop interface (classifier, gyro, taskbar)
ros/colmag_ros/
  launch/fr3.launch              FR3 in Gazebo (+ practice objects)
  launch/fr3_real.launch         real FR3 connection
  launch/colmag_arm_nodes.launch teleop + gesture nodes together
  scripts/colmag_draw_node.py    teleop: cursor→EE, height, rotation, gripper
  scripts/colmag_robot_node.py   gestures: letter tricks, digit line, homing
```

Magnetometer hardware: 218-byte packets (`0xAA` + 48 field floats + 6 pose
floats + `0xBB`) at 921600 baud; pose `mx,my,mz` encode the magnet orientation
(θ = tilt, φ = twist) used for the teleop controls.

---

<div align="center">
<sub>TUM Seminar · Collaborative Robotics and Assistive Technology for Advanced Human-Robot Interaction</sub>
</div>
