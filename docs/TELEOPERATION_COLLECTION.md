# Teleoperation Pipeline

**Data → Teleoperation Pipeline → Start** opens the FR3 in MuJoCo.

- Shared experiment name and participant names from the collection registry.
- **Trackpad:** move the pointer over the scene for X/Y; scroll for height.
- **Magnet board:** board X/Y and calibrated magnet height control X/Y/Z.
- **Enter:** reveal a random target and start timing from the common pose.
- **Hold:** keep the green flange marker inside the blue sphere for **2 seconds**.
- **Loading ring:** fills during the hold; leaving the sphere resets it.
- **Saved automatically:** Enter starts the next target. Escape cancels and logs the attempt.

<img src="teleoperation_data.png" width="560" alt="Data window with a separate Teleoperation Pipeline and named participants">

![MuJoCo target-reaching collection](teleoperation.png)

## Setup

```bash
python3 tools/setup_teleoperation.py
python3 colmag_launcher.py
```

Tk must be available in the Python used for setup. The launcher uses
`.venv/teleoperation/bin/python`; `COLMAG_TELEOP_PYTHON` selects another runtime.
Trackpad trials run locally. Board trials reuse the existing Docker serial-port
setup; close other board recordings before starting.

Standalone trackpad demo:

```bash
.venv/teleoperation/bin/python tools/teleoperation_demo.py
```

## Pilot defaults

| Setting | Value |
| --- | --- |
| Position reference | Measured FR3 flange, in the robot base frame |
| Common start | X 0.45 / Y 0.00 / Z 0.40 m |
| Task workspace | Existing 24 cm cube in front of the robot |
| Random targets | 10, seed 0; target spheres fit inside the cube |
| Target radius | 25 mm; full 3D distance |
| Continuous hold / timeout | 2 / 60 seconds |
| Orientation | Fixed; position-only task |

The cube constrains both target generation and controls in this demo. Board
axes and nonlinear height follow the existing flight deck mapping, scaled to
this cube. Pick-and-place objects provide fixed visual landmarks.

## Saved data

Each Start creates `data_collection/teleoperation/Pxx/Sxx/` (gitignored).

- **manifest.json:** participant, experiment, input, magnet count, seed and settings.
- **summary.csv:** completion time, first entry, endpoint error, path length and status.
- **trial_NNN_trajectory.csv:** measured and commanded XYZ, both clocks, target error and input snapshots.
- **trial_NNN_result.json:** frozen trial result and protocol settings.

Time runs from Enter through the completed hold. Failed/cancelled trials retain
their status and have no successful completion time. Stationary commands alone
cannot earn a hold: fresh measured physics feedback is required. Input snapshots
include the latest board pose and 48 magnetic channels; they are sampled with the
robot trace rather than a full-rate sensor archive.

Practice first. Keep seed, targets, tolerance, hold and control mapping identical
when comparing trackpad and magnet conditions; plan condition order and repetitions
before collecting participant comparisons. The existing Gazebo observer remains
available under **Virtual task**: [earlier pilot](VIRTUAL_TASK_PILOT.md).

The FR3 model is vendored from MuJoCo Menagerie; see its
[source and license](../assets/teleoperation/franka_fr3/SOURCE.md).
