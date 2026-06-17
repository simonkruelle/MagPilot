<div align="center">

# COLMAG

### Air-write a character. Watch the robot move.

Magnetometer-based gesture control for a **Franka Emika Panda / FR3** arm —
write letters and digits in the air (or on a trackpad), and a 7-DOF robot
responds. Runs entirely in **Gazebo simulation** inside Docker, or against the
real magnetometer and robot.

![ROS](https://img.shields.io/badge/ROS-Noetic-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-11-FF6A00)
![Robot](https://img.shields.io/badge/Franka-Panda%20%2F%20FR3-0A84FF)
![Docker](https://img.shields.io/badge/Docker-ready-2496ED?logo=docker&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.8%20%2F%203.10-3776AB?logo=python&logoColor=white)

</div>

---

## What it does

```
trackpad / magnetometer  →  pose  →  EasyOCR classifier  →  virtual joystick  →  /colmag/command  →  FR3 in Gazebo
```

You draw a character. The pose trajectory is projected to a 64×64 image,
recognized by EasyOCR, confirmed with a dwell-based virtual joystick, and
published as a ROS command. A robot node maps each character to a motion — write
an **`L`** and the arm rotates its base left; write an **`A`** and it waves.

Everything below runs without any hardware: a touchpad/trackpad stands in for the
magnetometer, and the Panda/FR3 lives in Gazebo.

---

## Quick start · Linux + Docker

**Prerequisites:** Docker and an X11 display (standard on Linux desktops). An
NVIDIA GPU is optional — see [GPU acceleration](#gpu-acceleration).

### 1 · Build the image (once)

```bash
git clone <this-repo> && cd COLMAG-seminar-SS26
INSTALL_GAZEBO=1 bash ros/docker_setup.sh
```

This builds a ROS Noetic image with the CPU-only EasyOCR stack, Gazebo 11, and
the Franka **Panda + FR3** simulation, then starts a container named
`colmag_ros`. The first build downloads a few GB and takes several minutes;
afterwards it is cached.

The Gazebo image builds the Franka stack from source with pinned versions:
`libfranka=0.13.3` and `franka_ros=0.10.2`. That keeps the Docker setup aligned
with an FR3 lab setup on robot system `5.5.0`, where `libfranka 0.13.3` is the
target client library.

### 2 · Run the demo — three terminals

Open each terminal with `bash ros/docker_connect.sh`, then run **in order**
(Terminal 1 starts the ROS master, so it must go first):

```bash
# Terminal 1 — spawn the FR3 in Gazebo with its trajectory controller
roslaunch colmag_ros fr3.launch controller:=effort_joint_trajectory_controller

# Terminal 2 — the robot action node (waits for the arm, then drives it)
rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=fr3

# Terminal 3 — the trackpad writing interface
python3 magnetometer_reader.py --input-source trackpad --ros --classifier-labels ABCXLRUD0123
```

### 3 · Write something

In the Terminal 3 window, **air-write a character** on the trackpad. When the
top prediction is right, **dwell on the confirm target**. The FR3 in Gazebo
performs that character's motion. The action legend is shown to the left of the
canvas.

> Want the Panda instead of the FR3? Use
> `roslaunch franka_gazebo panda.launch controller:=effort_joint_trajectory_controller`
> and `_arm_id:=panda` on the robot node.

---

## Gesture → robot action

| Gesture | Action | Gesture | Action |
|:------:|:-------|:------:|:-------|
| `A` | wave 👋 | `1` | nod — yes ✅ |
| `B` | bow 🙇 | `2` | shake — no ❌ |
| `C` | fist pumps 💪 | `3` | cheer 🙌 |
| `D` | dab 😎 | `L` / `R` | point left / right |
| `U` | stretch up 🙆 | `0` / `X` | home |
| *(other)* | small nod | | |

The map lives in `self._motions` and the `_wave` / `_bow` / `_dab` / … motion
methods in
[`ros/colmag_ros/scripts/colmag_robot_node.py`](ros/colmag_ros/scripts/colmag_robot_node.py).
Each motion is just a list of `(joint_positions, time)` waypoints — add your own,
or replace `execute_command()` with MoveIt `move_group` goals (build with
`INSTALL_MOVEIT=1`) or real-robot API calls. Keep the legend in
`magnetometer_reader.py` (`ROBOT_ACTION_LEGEND`) in sync.

---

## What else you can run

<details>
<summary><b>ROS-only test (no Gazebo, no hardware)</b></summary>

```bash
roslaunch colmag_ros colmag_distributed.launch run_robot:=false
rosrun colmag_ros colmag_listener.py
```
</details>

<details>
<summary><b>Headless smoke test (verify the arm + controllers load)</b></summary>

```bash
roslaunch colmag_ros fr3.launch headless:=true controller:=effort_joint_trajectory_controller
```
</details>

<details>
<summary><b>Real magnetometer + robot (hardware path)</b></summary>

`docker_setup.sh` auto-mounts `/dev/ttyUSB*` / `/dev/ttyACM*`. Use the
distributed launch, where `sensor_node` reads the serial port and the launch's
own classifier/joystick nodes produce commands (do **not** also run the reader):

```bash
roslaunch colmag_ros colmag_distributed.launch run_robot:=true dry_run:=false
```
</details>

<details>
<summary><b>macOS / no local ROS (roslibpy WebSocket bridge)</b></summary>

```bash
pip install roslibpy
# In the container:
roslaunch rosbridge_server rosbridge_websocket.launch
# On the Mac:
python3 magnetometer_reader.py --input-source trackpad --ros
```
The reader connects to `localhost:9090` and publishes all topics over the bridge.
</details>

---

## Simulation details

**Panda vs. FR3.** `franka_gazebo` ships only `panda.launch` (the older Franka
Emika Panda). This repo adds [`fr3.launch`](ros/colmag_ros/launch/fr3.launch) for
the **Franka Research 3** — `franka_ros 0.10.2` includes an FR3-capable
`franka_description`
and both arms share the Gazebo-capable `franka_robot.xacro`, so they simulate
with the same controllers (joints become `fr3_joint1..7`). Both are 7-DOF Franka
arms; match the launch to your target hardware for accurate kinematics.

**Controller.** Motions are sent as `FollowJointTrajectory` goals to
`effort_joint_trajectory_controller` (defined by franka_gazebo). The robot node
reconnects lazily, so launch order doesn't matter — as long as the arm is up by
the time you confirm a character.

**Version pins.** `INSTALL_GAZEBO=1` does not install the unpinned apt Franka
packages. It builds `libfranka` from tag `0.13.3` and `franka_ros` from tag
`0.10.2`, then sources `/opt/franka_ros_ws/devel/setup.bash` before this repo's
catkin workspace. The COLMAG scripts publish ROS messages and trajectory goals,
so no Python script changes are needed for the `libfranka` API version.

### GPU acceleration

The simulation runs fine on CPU; a GPU only makes the Gazebo/RViz 3D view
smooth. `docker_setup.sh` **auto-detects** a working NVIDIA Docker runtime and
enables it (you'll see `GPU: NVIDIA (runtime=nvidia)` in the banner); otherwise
it falls back to software rendering. To enable it:

```bash
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker
```

> On Pop!_OS / some distros, apt may pin an **outdated** toolkit (1.12.x) that
> crashes against new drivers. If the setup banner says the GPU probe failed,
> install the current version explicitly:
> ```bash
> sudo apt-get install -y nvidia-container-toolkit=1.19.1-1 \
>   nvidia-container-toolkit-base=1.19.1-1 \
>   libnvidia-container-tools=1.19.1-1 libnvidia-container1=1.19.1-1
> sudo systemctl restart docker
> ```

EasyOCR uses **CPU-only** PyTorch in the image (to keep it small), so
`--classifier-gpu` falls back to CPU even with GPU passthrough enabled.

---

## ROS topics

| Topic | Type | Content |
|-------|------|---------|
| `/colmag/command` | `std_msgs/String` | UI commands: `choice:0…3`, `canvas:reset`, `letter_detection`, `number_detection`, `symbol_detection` |
| `/colmag/classifier` | `std_msgs/String` | Top predicted character |
| `/colmag/confidence` | `std_msgs/Float64` | Confidence of top prediction (0–1) |
| `/colmag/pose` | `geometry_msgs/PoseStamped` | Magnet XYZ position |
| `/colmag/sensor_data` | `std_msgs/Float64MultiArray` | Raw Bx/By/Bz (48 floats) + pose (6 floats) |

In trackpad mode `magnetometer_reader.py --ros` publishes all of these itself
(it runs its own classifier + joystick), so you only need the **robot node**
downstream — not the full distributed launch.

---

## The writing interface

`magnetometer_reader.py` is the drawing + classification UI. Common options:

| Option | Purpose |
|--------|---------|
| `--input-source serial \| trackpad` | real sensor, or trackpad simulator (`touchpad` is an alias) |
| `--ros` | publish to ROS (native `rospy` in Docker, `roslibpy` bridge on Mac) |
| `--classifier-labels ABCXLRUD0123` | restrict OCR to the characters the robot has actions for |
| `--writing-max-z 0.05` | (real sensor) ignore ink when the magnet is > 5 cm away |
| `--clean` / `--full-view` | canvas + classifier only (default for trackpad) / add raw B-field plots |
| `--no-classifier` | run the UI without loading EasyOCR |

Run `python3 magnetometer_reader.py --help` for the full list (recording,
calibration, touchpad tuning, projection, and writing-filter options).

---

## Repository layout

```
magnetometer_reader.py     Drawing + EasyOCR classification UI (publishes ROS topics)
csv_visualizer.py          Plot recorded CSV sessions
ros/
  Dockerfile               ROS Noetic + EasyOCR + (opt) Gazebo/Panda/FR3 + (opt) MoveIt
  docker_setup.sh          Build image, start container, GPU auto-detect
  docker_connect.sh        Open a shell in the running container
  colmag_ros/              catkin package
    launch/
      fr3.launch                 spawn the FR3 in Gazebo
      colmag_distributed.launch  full pipeline (sensor→classifier→joystick→robot)
    scripts/
      colmag_robot_node.py       maps gestures → arm motion (NAMED_POSES)
      colmag_sensor_node.py      reads the serial magnetometer
      colmag_classifier_node.py  EasyOCR classification node
      colmag_joystick_node.py    dwell-based command confirmation
digit_classifier/          EasyOCR inference wrapper
```

### Build flags

| Flag | Default | Effect |
|------|:------:|--------|
| `INSTALL_EASYOCR` | `1` | CPU-only EasyOCR/PyTorch (set `0` for a light ROS-only image) |
| `INSTALL_GAZEBO` | `0` | Gazebo 11 + `franka_ros` (Panda + FR3) |
| `INSTALL_MOVEIT` | `0` | MoveIt + `panda_moveit_config` for motion planning |
| `LIBFRANKA_VERSION` | `0.13.3` | Source tag used when `INSTALL_GAZEBO=1` |
| `FRANKA_ROS_VERSION` | `0.10.2` | Source tag used when `INSTALL_GAZEBO=1` |
| `COLMAG_ENABLE_GPU` | `auto` | force GPU on (`1`) / off (`0`) |

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `python: command not found` | Use `python3` (the image has no bare `python`). |
| `No trajectory client — skipping motion` | The arm wasn't up yet; restart the robot node (it also reconnects on the next command). |
| Launch fails: *"new node registered with same name"* / *"address already in use"* | A stale `rosmaster`/`gazebo` lingered. Reset cleanly: `docker restart colmag_ros`. |
| Banner shows `GPU: none (software rendering)` | Install `nvidia-container-toolkit` (see [GPU acceleration](#gpu-acceleration)); the sim still works without it. |
| Gazebo GUI won't open a window | Run `xhost +local:root` on the host once. |
| Need to verify the Franka pin | Inside the container: `grep -R PACKAGE_VERSION /usr/local/lib/cmake/Franka/FrankaConfigVersion.cmake`. |

---

## Hardware reference

For the real magnetometer over serial (default 921600 baud):

- **Packet:** `0xAA` header · 54 little-endian floats (216 B) · `0xBB` tail = **218 B**.
- **Payload:** 48 magnetic-field values (16 sensors × Bx/By/Bz) + 6 pose values (x, y, z, mx, my, mz).
- **Units** depend on your sensor — check its datasheet (Earth's field ≈ 25–65 µT ≈ 0.25–0.65 G).

Recorded sessions (`--record-data`) write a CSV + 64×64 PNG + JSON sidecar under
`data/lab_YYYY-MM-DD/samples/<label>/` with a run-level `manifest.json`.

---

<div align="center">
<sub>TUM Seminar · Collaborative Robotics and Assistive Technology for Advanced Human-Robot Interaction</sub>
</div>
