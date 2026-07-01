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

Stages 0-3 below run without any hardware: a touchpad/trackpad stands in for the
magnetometer, and the Panda/FR3 lives in Gazebo.

---

## Tomorrow's safe testing pipeline

Run the same stages in order. Only move to the next stage when the current one
behaves exactly as expected.

| Stage | Environment | Goal | Allowed to move a real robot? |
|-------|-------------|------|:-----------------------------:|
| 0 | Docker only | Build and verify EasyOCR imports | No |
| 1 | Touchpad UI only | Check drawing and OCR without ROS | No |
| 2 | ROS dry-run | Check `/colmag/command` and robot-node logs | No |
| 3 | Gazebo FR3 | Move the simulated robot from touchpad commands | No |
| 4 | Real setup, dry-run | Check magnetometer, ROS networking, command flow | No |
| 5 | Real FR3, tiny smoke test | One small `fr3_simple_move.py` nudge | Yes, supervised only |
| 6 | Real COLMAG pipeline | One approved gesture through the full stack | Yes, supervised only |

Stop immediately if the predicted command is wrong, a command repeats
unexpectedly, the wrong controller/`arm_id` is active, the robot moves in the
wrong direction, or anyone is inside the robot workspace.

### Stage 0 - build Docker once

**Prerequisites:** Docker and an X11 display on Linux. An NVIDIA GPU is optional;
Gazebo physics and EasyOCR both work on CPU.

On the host PC:

```bash
cd COLMAG-seminar-SS26
git checkout Simon
git pull --ff-only
xhost +local:root
INSTALL_GAZEBO=1 bash ros/docker_setup.sh
```

This builds a ROS Noetic image with CPU-only EasyOCR, Gazebo 11, and the Franka
Panda/FR3 simulation, then starts a container named `colmag_ros`. The first build
downloads several GB; later builds are cached.

Open a shell in the container:

```bash
bash ros/docker_connect.sh
```

Optional EasyOCR warm-up inside the container. This downloads the OCR model into
the Docker cache volume, so tomorrow's first classifier run should start faster:

```bash
python3 - <<'PY'
import torch, easyocr
print('torch', torch.__version__, 'cuda', torch.cuda.is_available())
easyocr.Reader(['en'], gpu=False)
print('EasyOCR model ready')
PY
```

### Stage 1 - touchpad + EasyOCR only

Inside the container:

```bash
python3 magnetometer_reader.py --input-source touchpad --classifier-labels ABCXLRUD0123
```

Draw a few simple characters (`A`, `L`, `R`, `0`) and confirm that the top OCR
prediction is stable. This stage does not start ROS and cannot move any robot.

### Stage 2 - ROS dry-run, still no robot

Open three container terminals with `bash ros/docker_connect.sh`.

```bash
# Terminal 1 - ROS master only
roscore

# Terminal 2 - robot node in dry-run mode
rosrun colmag_ros colmag_robot_node.py _dry_run:=true _arm_id:=fr3

# Terminal 3 - touchpad UI publishing ROS topics
python3 magnetometer_reader.py --input-source touchpad --ros --classifier-labels ABCXLRUD0123
```

Expected result: when you dwell-confirm the top prediction, Terminal 2 logs
`CONFIRMED gesture: ... (dry run)`. Nothing moves.

Useful extra check in a fourth terminal:

```bash
rostopic echo /colmag/command
```

The Gazebo image builds the Franka stack from source with pinned versions:
`libfranka=0.13.3` and `franka_ros=0.10.2`. That keeps the Docker setup aligned
with an FR3 lab setup on robot system `5.5.0`, where `libfranka 0.13.3` is the
target client library.

### Stage 3 - FR3 in Gazebo

Open three fresh container terminals. Run them in this order:

```bash
# Terminal 1 - spawn the FR3 in Gazebo with the trajectory controller
roslaunch colmag_ros fr3.launch controller:=effort_joint_trajectory_controller

# Terminal 2 - robot action node, live only for the simulated arm
rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=fr3

# Terminal 3 - touchpad writing interface
python3 magnetometer_reader.py --input-source touchpad --ros --classifier-labels ABCXLRUD0123
```

Expected result: dwell-confirming the top prediction sends a trajectory to the
Gazebo FR3. Test calm commands first: `0`/`X` for home, then `L` or `R`, then
`A`.

Quick health checks:

```bash
rostopic hz /franka_state_controller/franka_states
rostopic list | grep follow_joint_trajectory
rosservice call /controller_manager/list_controllers
```

Want the Panda instead of the FR3? Use:

```bash
roslaunch franka_gazebo panda.launch controller:=effort_joint_trajectory_controller
rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=panda
```

### Stage 4 - real magnetometer + real ROS, dry-run

Only start this after Stage 3 works. Keep the robot node in dry-run while you
check the real sensor, ROS master, and command flow.

On the host, rebuild/restart Docker after plugging in the magnetometer so the
serial device is mounted:

```bash
INSTALL_GAZEBO=1 bash ros/docker_setup.sh
```

Inside the container, set the lab ROS network values given by the supervisor:

```bash
export ROS_MASTER_URI=http://<lab-ros-master-ip>:11311
export ROS_IP=<this-pc-ip-on-the-robot-network>
roslaunch colmag_ros colmag_distributed.launch run_robot:=true dry_run:=true arm_id:=fr3 port:=/dev/ttyUSB0
```

If the magnetometer appears as `/dev/ttyACM0`, use that instead. Expected
result: the sensor, classifier, joystick, and robot node all run, and the robot
node logs confirmed gestures as dry-run messages. The real robot must not move.

### Stage 5 - one tiny supervised real-FR3 command

Do this only after Stage 4 is clean and the supervisor confirms:

- the workspace is clear,
- the emergency stop / enabling device is ready,
- the correct robot and controller are active,
- the first motion is allowed to be small.

This stage does **not** use gestures yet. It uses the dedicated smoke-test
script from the ROS package: it reads the current joint state, offsets one joint
by a tiny amount, and returns to the starting pose.

```bash
# Terminal 1 - connect ROS to the real FR3
roslaunch colmag_ros fr3_real.launch robot_ip:=<FR3-IP>

# Terminal 2 - dry-run first; prints the planned trajectory only
rosrun colmag_ros fr3_simple_move.py _dry_run:=true _arm_id:=fr3

# Terminal 2 - only after the dry-run and supervisor check are clean
rosrun colmag_ros fr3_simple_move.py _dry_run:=false _arm_id:=fr3 _delta:=0.04
```

Stop if anything differs from the dry-run plan.

### Stage 6 - full real COLMAG gesture pipeline

Only start this after Stage 5 succeeds. Keep `fr3_real.launch` running in one
terminal, then run the full pipeline in another. Use the real trajectory
controller from `fr3_real.launch`, not the Gazebo effort controller:

```bash
roslaunch colmag_ros colmag_distributed.launch \
  run_robot:=true \
  dry_run:=false \
  arm_id:=fr3 \
  arm_controller:=position_joint_trajectory_controller \
  port:=/dev/ttyUSB0
```

The first live gesture must be explicitly approved. Review the motion mapped to
that label in `ros/colmag_ros/scripts/colmag_robot_node.py` before running it on
the real arm.

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
<summary><b>Simulation movement smoke test (Gazebo, no gestures yet)</b></summary>

Run this before touching the real robot. It uses the same tiny trajectory script
as the real FR3 smoke test, but targets Gazebo's
`effort_joint_trajectory_controller`.

```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

```bash
# Terminal 1 — spawn the simulated FR3
roslaunch colmag_ros fr3.launch headless:=false controller:=effort_joint_trajectory_controller

# Terminal 2 — dry-run first
rosrun colmag_ros fr3_simple_move.py \
  _dry_run:=true \
  _arm_id:=fr3 \
  _arm_controller:=effort_joint_trajectory_controller

# Terminal 2 — move only the simulated robot
rosrun colmag_ros fr3_simple_move.py \
  _dry_run:=false \
  _arm_id:=fr3 \
  _arm_controller:=effort_joint_trajectory_controller \
  _delta:=0.08
```
</details>

<details>
<summary><b>Real FR3 smoke test (franka_control, no gestures yet)</b></summary>

Use this before the COLMAG gesture pipeline. It starts `franka_control` for the
real FR3, spawns `position_joint_trajectory_controller`, then sends one tiny
joint-space nudge and returns to the measured starting pose.

Rebuild/source once after pulling new ROS package changes:

```bash
cd /catkin_ws
catkin_make
source devel/setup.bash
```

```bash
# Terminal 1 — connect to the real robot controller
roslaunch colmag_ros fr3_real.launch robot_ip:=<FR3-IP>

# Terminal 2 — dry-run first; prints the planned tiny trajectory only
rosrun colmag_ros fr3_simple_move.py _dry_run:=true _arm_id:=fr3

# Terminal 2 — only after the dry-run looks sane and the robot is ready
rosrun colmag_ros fr3_simple_move.py _dry_run:=false _arm_id:=fr3 _delta:=0.04
```

Keep the workspace clear and the stop button reachable. The script reads
`/franka_state_controller/joint_states`, offsets joint 7 by a small amount, and
returns to the start pose. Tune with `_joint_index:=7`, `_delta:=0.05`, or
`_arm_controller:=position_joint_trajectory_controller`.

To watch the real robot state, use RViz in another terminal:

```bash
rviz
```

Add `RobotModel` and `TF` displays. Gazebo is a separate simulated robot, not a
live mirror of the physical FR3, so use Gazebo for rehearsal and RViz for the
real robot.
</details>

<details>
<summary><b>Real magnetometer hardware dry-run shortcut</b></summary>

`docker_setup.sh` auto-mounts `/dev/ttyUSB*` / `/dev/ttyACM*`. Use the
distributed launch, where `sensor_node` reads the serial port and the launch's
own classifier/joystick nodes produce commands (do **not** also run the reader).
Keep `dry_run:=true` until the full Stage 4 checklist passes:

```bash
roslaunch colmag_ros colmag_distributed.launch run_robot:=true dry_run:=true arm_id:=fr3 port:=/dev/ttyUSB0
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
`effort_joint_trajectory_controller` in simulation (defined by franka_gazebo).
Launch Gazebo first during testing because it makes errors easier to see. The
robot node can reconnect lazily as long as the arm is up before you confirm a
character.

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
| `/colmag/command` | `std_msgs/String` | UI commands: `choice:0…3`, `canvas:reset`, `letter_detection`, `number_detection` |
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
      fr3_real.launch            connect to a real FR3 via franka_control
      colmag_distributed.launch  full pipeline (sensor→classifier→joystick→robot)
    scripts/
      colmag_robot_node.py       maps gestures → arm motion (NAMED_POSES)
      fr3_simple_move.py         tiny real/sim trajectory smoke test
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
