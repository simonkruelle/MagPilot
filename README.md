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
| 6 | Real FR3 + touchpad | One approved gesture, touchpad input (known-good from Stage 3) | Yes, supervised only |
| 7 | Real FR3 + magnetometer | The full COLMAG stack on real hardware | Yes, supervised only |
| 8 | Real FR3 teleop | Cursor-following + pick-and-place on the real arm | Yes, supervised only |

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
roslaunch colmag_ros colmag_distributed.launch input:=magnetometer run_robot:=true dry_run:=true arm_id:=fr3 port:=/dev/ttyUSB0
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

### Stage 6 - real FR3 gestures from the touchpad

Only start this after Stage 5 succeeds. This drives the real arm from the
**touchpad** — the same input already proven against Gazebo in Stage 3 — so the
first real-robot gestures do not depend on the magnetometer at all. The
distributed launch defaults to `input:=touchpad`, which starts only the robot
node and rosbridge; the touchpad UI publishes the commands itself.

Keep `fr3_real.launch` running in one terminal and use the real trajectory
controller, not the Gazebo effort controller:

```bash
# Terminal 1 - connect ROS to the real FR3
roslaunch colmag_ros fr3_real.launch robot_ip:=<FR3-IP>

# Terminal 2 - robot node + rosbridge (input defaults to touchpad)
roslaunch colmag_ros colmag_distributed.launch \
  run_robot:=true \
  dry_run:=false \
  arm_id:=fr3 \
  arm_controller:=position_joint_trajectory_controller

# Terminal 3 - touchpad writing interface
python3 magnetometer_reader.py --input-source touchpad --ros --classifier-labels ABCXLRUD0123
```

The first live gesture must be explicitly approved. Review the motion mapped to
that label in `ros/colmag_ros/scripts/colmag_robot_node.py` before running it on
the real arm. Test calm commands first: `0`/`X` for home, then `L` or `R`.

### Stage 7 - full real COLMAG pipeline with the magnetometer

Only start this after Stage 6 succeeds. Same setup, but the gesture input now
comes from the real magnetometer: `input:=magnetometer` starts the serial
sensor, classifier, and joystick nodes instead of expecting the touchpad UI
(do **not** also run `magnetometer_reader.py`).

```bash
# Terminal 1 - connect ROS to the real FR3
roslaunch colmag_ros fr3_real.launch robot_ip:=<FR3-IP>

# Terminal 2 - full pipeline from the serial magnetometer
roslaunch colmag_ros colmag_distributed.launch \
  input:=magnetometer \
  run_robot:=true \
  dry_run:=false \
  arm_id:=fr3 \
  arm_controller:=position_joint_trajectory_controller \
  port:=/dev/ttyUSB0
```

If the magnetometer appears as `/dev/ttyACM0`, use that instead. If a gesture
misclassifies, fall back to Stage 6 (touchpad) to tell apart sensor problems
from robot problems.

### Stage 8 - real FR3 teleop + pick-and-place

Only start after Stage 6 ran clean, and after rehearsing the full pick-and-place
in Gazebo (Stage 3 setup + `pick_objects:=true`). Bring a soft, light
(< 0.5 kg) practice object.

Pre-flight checklist:

- [ ] The teleop workspace box — x 0.30–0.60, y ±0.30, z 0.12–0.68 in the base
      frame — is completely clear: no table edges, monitors, cables, people.
      Remember the fingertips reach ~0.10–0.16 m *below* the flange coordinate.
- [ ] If the arm is mounted on a table, measure the real surface height and
      raise `plane_center_z` / shrink `plane_height` so the box bottom stays
      ~2 cm above the surface for the first session.
- [ ] Gripper mounted and `load_gripper:=true`; object under ~0.5 kg (heavier:
      configure the load mass in Desk first, or a collision reflex will fire).
- [ ] One person holds the E-stop the entire session; a second drives.

```bash
# Terminal 1 - real FR3 with the gripper
roslaunch colmag_ros fr3_real.launch robot_ip:=<FR3-IP> load_gripper:=true

# Terminal 2 - draw node, DRY RUN first (default)
roslaunch colmag_ros colmag_draw.launch arm_controller:=position_joint_trajectory_controller

# Terminal 3 - touchpad UI
python3 magnetometer_reader.py --input-source touchpad --ros
```

Dry-run verification (nothing moves): press Teleop, move the cursor, and watch
Terminal 2 — the logged targets should match where you expect the arm to go and
the IK residuals should stay well under 5 mm. Only then restart Terminal 2 with
`dry_run:=false`.

Expected behaviour going live: on pressing Teleop the arm glides at ~0.10 m/s
from its current pose toward the cursor — slow and predictable. Test in this
order: (1) small circles at mid height, (2) Shift+V height changes, (3) gripper
rotation with the arrow keys, (4) one grasp + lift + set-down of the practice
object. Stop immediately (E-stop or leaving teleop via Letters/Digits) if the
arm does not track the cursor, a reflex fires (`rosrun colmag_ros
fr3_recover.py` to recover), or anything enters the workspace box.

### Stopping safely and recovering from an E-stop

**Ctrl-C:** the robot node and `fr3_simple_move.py` cancel their running
trajectory on shutdown, so the arm stops and holds position instead of playing
the rest of the motion. Stop the pipeline terminal (robot node) first, then
`fr3_real.launch`. When `fr3_real.launch` exits, the arm holds its pose with
motors enabled; use Desk to lock/park it.

**Emergency stop / reflex stop:** pressing the activation device (or tripping a
safety reflex) locks the robot in an error state — it stays locked until the
error is cleared, even after the button is released. To recover:

```bash
# 1. Release the activation device.
# 2. With fr3_real.launch still running:
rosrun colmag_ros fr3_recover.py
```

If the arm stays locked, unlock the joints in the Desk web interface (brakes)
and confirm FCI mode is active, then run the script again. If `fr3_real.launch`
died (its `franka_control` is a required process), just restart the launch — a
fresh connection also clears the error.

---

## Draw-in-the-air mode (Cartesian tracing)

An alternative to the classify-and-gesture pipeline: press the **Teleop** button
(top of the left button pad) and the end-effector **follows the cursor/magnet**
on an invisible plane in front of the robot — by default a **horizontal** plane,
like a tabletop floating in the air (cursor up = further away, cursor right =
right; `plane_orientation:=vertical` gives an upright canvas instead). Pressing
**Letters**, **Digits**, or **Signs** leaves teleop: the arm stops following and
gesture mode works as usual. **Shift+G** in the touchpad UI toggles the
two-finger gripper (grasp with force / release), so you can pick something up
while teleoping. It maps `/colmag/pose` onto the plane and solves inverse
kinematics to stream joint targets to the same trajectory controller used in
Stages 6-7 (no MoveIt or controller swap needed). See
[`colmag_draw_node.py`](ros/colmag_ros/scripts/colmag_draw_node.py).

**Rehearse in Gazebo first** (the IK geometry is validated, but the mapping,
plane placement, and feel should be checked in sim before the real arm):

```bash
# Terminal 1 - simulated FR3 (Gazebo's trajectory controller is effort-based)
roslaunch colmag_ros fr3.launch controller:=effort_joint_trajectory_controller

# Terminal 2 - draw node (dry_run:=false lets it move the *sim* arm)
roslaunch colmag_ros colmag_draw.launch dry_run:=false

# Terminal 3 - touchpad publishing /colmag/pose
python3 magnetometer_reader.py --input-source touchpad --ros
```

To practice pick-and-place, spawn two cubes and a cup inside the workspace:

```bash
roslaunch colmag_ros colmag_pick_objects.launch
# or spawn them together with the arm:
roslaunch colmag_ros fr3.launch controller:=effort_joint_trajectory_controller pick_objects:=true
```

Spawned objects only live for the current Gazebo session — after restarting
`fr3.launch` they are gone until you spawn them again (that is Gazebo behaviour,
not a bug).

Then dwell on the **Teleop** button (top-left) — the arm starts tracing the
cursor. In teleop the canvas shows a **softly fading trail** (~5 s) instead of
accumulating ink. Dwell on **Letters** or **Digits** to stop following and go
back to gestures. **Shift+G** toggles the gripper (grasp force via the
`grasp_force` arg, default 20 N). **Hold the arrow keys ← / →** to rotate the
gripper smoothly around its own axis (~35°/s, release to stop, ±92° range) to
align the fingers with an object before grasping. **Shift+V** switches the control plane: horizontal (cursor drives x/y at the currently held height) ↔
vertical (cursor drives y/z at the currently held depth) — the third coordinate
keeps its last value, so alternating planes positions the end-effector anywhere
in the workspace box (x 0.30–0.60, y ±0.30, z 0.12–0.68 m — the bottom layer
puts the fingertips at about base height, so objects standing next to the robot
can be grasped in vertical mode). The four extreme far-top-edge spots are
outside the arm's reach with the gripper-down orientation; the node skips such
targets safely instead of forcing them. On the **real** robot, run
`fr3_real.launch` in place of `fr3.launch` (with `load_gripper:=true` if the
hand is mounted), pass `arm_controller:=position_joint_trajectory_controller`
to `colmag_draw.launch`, and start with `dry_run:=true` to watch the logged
targets and IK residuals before enabling motion. For the real magnetometer
instead of the touchpad, add `input:=magnetometer port:=/dev/ttyUSB0` and skip
the reader.

Safety and tuning:

- **Gated + dry-run by default.** Nothing moves until `dry_run:=false` *and*
  teleop is enabled (Teleop button, or manually
  `rostopic pub -1 /colmag/draw_enable std_msgs/Bool "data: true"`). Leaving
  teleop, Ctrl-C, or publishing `false` cancels the goal and the arm holds.
  While teleop is active, `colmag_robot_node` ignores confirmed gestures, so the
  two modes never fight over the arm.
- **Speed-capped.** The tool glides toward the cursor at `max_linear_speed`
  (default 0.10 m/s) — flicking the cursor across the pad cannot make the arm
  lunge, and enabling teleop with the arm far from the cursor produces a slow
  approach. Raise the cap (e.g. `max_linear_speed:=0.2`) once you are
  comfortable.
- **Speed cap.** `max_joint_step` (rad per 15 Hz update, default 0.05 → ~0.75
  rad/s) bounds how fast the arm tracks; fast scribbles lag smoothly rather than
  snapping. Raise it for snappier tracing only after sim rehearsal.
- **Workspace placement.** `plane_orientation` (default `horizontal`),
  `plane_center_x/y/z` and `plane_width/depth/height` (default box: 30 cm deep ×
  60 cm wide × 56 cm tall, centred 45 cm in front at 40 cm height — note the
  coordinates are *flange* positions; with the hand mounted the fingertips sit
  ~0.10-0.16 m lower). The defaults were validated point-by-point against the
  FR3/Panda joint limits; targets the IK cannot reach within 5 mm are skipped,
  not forced. The held orientation is gripper-down, ready for grasping.
- The horizontal mapping is oriented for someone standing **in front of** the
  robot: cursor up → the plane's far edge (toward the robot), cursor right →
  the viewer's right. Vertical plane: cursor right → +Y, cursor up → higher
  (+Z). Flip either axis with `map_x_sign` / `map_y_sign` if your viewpoint
  differs.

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
roslaunch colmag_ros colmag_distributed.launch input:=magnetometer run_robot:=false
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
roslaunch colmag_ros colmag_distributed.launch input:=magnetometer run_robot:=true dry_run:=true arm_id:=fr3 port:=/dev/ttyUSB0
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
      colmag_distributed.launch  pipeline; input:=touchpad (default) runs robot node +
                                 rosbridge, input:=magnetometer adds sensor→classifier→joystick
    scripts/
      colmag_robot_node.py       maps gestures → arm motion (NAMED_POSES)
      fr3_simple_move.py         tiny real/sim trajectory smoke test
      fr3_recover.py             clear the Franka error state after E-stop/reflex
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
