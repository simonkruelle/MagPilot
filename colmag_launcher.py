#!/usr/bin/env python3
"""
colmag_launcher.py — one-window control center for the COLMAG pipeline.

Run on the HOST (not inside Docker):

    python3 colmag_launcher.py

Buttons start each pipeline stage inside the `colmag_simon` Docker container via
`docker exec`, so you never need more than this window plus the GUIs that the
stages open themselves (Gazebo, the trackpad interface).
The Data window also starts an independent host MuJoCo Teleoperation Pipeline.
Its trackpad input works without Docker; board input uses the container bridge.

    1. Robot     — Gazebo FR3 (sim) or franka_control (real, needs robot IP)
    2. Arm nodes — teleop draw node + gesture robot node (one launch)
    3. Interface — trackpad UI or real magnetometer reader

Status lights poll the container every 2 s. STOP ALL stops the owned host
simulation and the pipeline processes inside the container.
Logs of each stage are written inside the container to /tmp/colmag_gui_*.log
and tailed in the bottom pane.
"""

import json
import math
import os
import re
import shlex
import shutil
import signal
import subprocess
import threading
import time
import tkinter as tk
import tkinter.font as tkfont
from datetime import date, datetime
from tkinter import messagebox, ttk

from colmag import dataset
from colmag.target_reaching import TrialSettings, default_targets
from colmag.action_mapping import (
    ACTION_CATALOG,
    DEFAULT_ACTION_MAP_PATH,
    DIGIT_LABELS,
    LETTER_LABELS,
    default_action_mapping,
    load_action_mapping,
    save_action_mapping,
)

CONTAINER = 'colmag_simon'
LEGACY_CONTAINERS = ('colmag_ros',)
REPO_DIR = os.path.dirname(os.path.abspath(__file__))
# Recorded participant data; gitignored. The container sees it as
# /colmag/data_collection because the whole repo is mounted there.
DATA_DIR = os.path.join(REPO_DIR, dataset.DATA_DIR_NAME)
ROS_SETUP = ('source /opt/ros/noetic/setup.bash; '
             '[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash; ')

# ── MagPilot sky palette: white cards floating like clouds on light blue ────
BG = '#e9f3fc'
CARD = '#ffffff'
TEXT = '#1d1d1f'
SUBTLE = '#7b8b99'
BLUE = '#0a84ff'
BLUE_DARK = '#0060df'
GREEN = '#34c759'
RED = '#ff3b30'
AMBER = '#ff9f0a'
BORDER = '#d9e6f2'
TRACK = '#dcebf7'
DOT_OFF = '#c6d6e3'

STAGES = ('robot', 'nodes', 'interface')
DETACHED_TAGS = STAGES + ('window', 'pilot')
SIMULATION_ROS_PACKAGES = (
    'gazebo_ros',
    'franka_description',
    'franka_gazebo',
    'controller_manager',
)
FRANKA_ROBOT_MODES = {
    0: 'Other',
    1: 'Idle',
    2: 'Move',
    3: 'Guiding',
    4: 'Reflex',
    5: 'User stopped',
    6: 'Automatic error recovery',
}
TRACKED_ROS_NODES = (
    '/franka_control',
    '/gazebo',
    '/colmag_draw_node',
    '/colmag_robot_node',
)
INTERFACE_ROS_NODES = (
    '/colmag_node',
    '/colmag_sensor_node',
    '/colmag_classifier_node',
    '/colmag_joystick_node',
    '/colmag_listener',
)
ARM_ROS_NODES = (
    '/colmag_draw_node',
    '/colmag_robot_node',
)
ROBOT_ROS_NODES = (
    '/gazebo_gui',
    '/gazebo',
    '/position_joint_trajectory_controller_spawner',
    '/state_controller_spawner',
    '/joint_state_publisher',
    '/robot_state_publisher',
    '/franka_gripper',
    '/franka_control',
)
PIPELINE_ROS_NODES = (
    INTERFACE_ROS_NODES + ARM_ROS_NODES + ROBOT_ROS_NODES)
WIDTH = 780


def pick_font(candidates, fallback='TkDefaultFont'):
    try:
        fams = set(tkfont.families())
    except Exception:
        return fallback
    for name in candidates:
        if name in fams:
            return name
    return fallback


def sh(cmd, timeout=10):
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                             timeout=timeout)
        return out.returncode == 0, (out.stdout + out.stderr).strip()
    except subprocess.TimeoutExpired:
        return False, '(timeout)'


def validate_project_mount(mounts_json, repo_dir):
    """Check that /colmag is bound to the checkout running this launcher."""
    try:
        mounts = json.loads(mounts_json)
    except (TypeError, json.JSONDecodeError):
        return False, 'Docker returned unreadable mount metadata.'
    if not isinstance(mounts, list) or not all(isinstance(m, dict) for m in mounts):
        return False, 'Docker returned invalid mount metadata; expected a list of mounts.'
    project_mounts = [m for m in mounts if m.get('Destination') == '/colmag']
    if not project_mounts:
        return False, 'The container has no project bind mount at /colmag.'
    if len(project_mounts) != 1:
        return False, 'Docker returned multiple project mounts at /colmag.'
    project_mount = project_mounts[0]
    source = project_mount.get('Source')
    if project_mount.get('Type') != 'bind':
        return False, '/colmag must be a bind mount; current source: {!r}.'.format(source)
    if not isinstance(source, str) or not source or not os.path.isabs(source):
        return False, 'Docker returned an invalid source for the /colmag bind mount.'
    expected = os.path.realpath(repo_dir)
    actual = os.path.realpath(source)
    if actual != expected:
        return False, ('The container is using a different project at /colmag.\n'
                       'Current source: {}\nExpected checkout: {}').format(source, expected)
    return True, ''


def probe_container_project_mount(repo_dir):
    """Inspect the existing container without changing it or its mounts."""
    ok, output = sh('docker inspect --format {} {}'.format(
        shlex.quote('{{json .Mounts}}'), shlex.quote(CONTAINER)), timeout=8)
    if not ok:
        return False, 'Could not inspect the container project mount: {}'.format(
            output or 'docker inspect failed')
    return validate_project_mount(output, repo_dir)


def stop_conflicting_colmag_containers():
    """Stop other COLMAG containers whose host-network ROS nodes leak in."""
    ok, output = sh(
        'docker ps --format "{{.Names}}|{{.Image}}"', timeout=5)
    if not ok:
        return False, output or 'could not list Docker containers'

    candidates = []
    for line in output.splitlines():
        name, _, image = line.partition('|')
        name = name.strip()
        if not name or name == CONTAINER:
            continue
        if (name in LEGACY_CONTAINERS
                or 'colmag' in name.lower()
                or 'colmag' in image.lower()):
            candidates.append(name)

    stopped = []
    for name in candidates:
        ok, detail = sh(
            'docker stop -t 5 {}'.format(shlex.quote(name)), timeout=12)
        if not ok:
            ok, detail = sh(
                'docker kill {}'.format(shlex.quote(name)), timeout=8)
        if not ok:
            return False, (
                'could not stop conflicting container {}: {}'.format(
                    name, detail or 'unknown Docker error'))
        stopped.append(name)
    return True, ', '.join(stopped)


def _stage_path(tag, suffix):
    if tag not in DETACHED_TAGS:
        raise ValueError('Unknown launcher stage: {}'.format(tag))
    return '/tmp/colmag_gui_{}.{}'.format(tag, suffix)


def build_detached_inner(tag, command):
    """Build a detached stage runner whose complete process group we own."""
    pid_file = _stage_path(tag, 'pid')
    log_file = _stage_path(tag, 'log')
    runner = (
        'echo $$ > {pid}; '
        'cleanup() {{ rm -f {pid}; }}; trap cleanup EXIT; '
        'printf "[launcher] starting {tag}\\n"; '
        '{command}; status=$?; '
        'printf "[launcher] {tag} exited with status %s\\n" "$status"; '
        'exit "$status"'
    ).format(pid=shlex.quote(pid_file), tag=tag, command=command)
    return (
        'export PYTHONUNBUFFERED=1; {setup}'
        'rm -f {pid}; '
        'exec setsid bash -lc {runner} > {log} 2>&1'
    ).format(
        setup=ROS_SETUP,
        pid=shlex.quote(pid_file),
        runner=shlex.quote(runner),
        log=shlex.quote(log_file),
    )


def in_container_detached(tag, command):
    # The setsid runner gives each stage its own process group. This lets a new
    # launcher stop jobs that survived an earlier launcher crash.
    inner = build_detached_inner(tag, command)
    return sh('docker exec -d {} bash -lc {}'.format(CONTAINER, shlex.quote(inner)))


def in_container(command, timeout=8):
    return sh('docker exec {} bash -lc {}'.format(
        CONTAINER, shlex.quote(ROS_SETUP + command)), timeout=timeout)


def build_ros_package_probe_command(packages):
    package_words = ' '.join(shlex.quote(package) for package in packages)
    return (
        'for package in {packages}; do '
        'rospack find "$package" >/dev/null 2>&1 || echo "$package"; '
        'done'
    ).format(packages=package_words)


def missing_container_ros_packages(packages):
    ok, output = in_container(
        build_ros_package_probe_command(packages), timeout=8)
    if not ok:
        return None
    return tuple(line.strip() for line in output.splitlines() if line.strip())


def _managed_stage_signal(tag, signal):
    pid_file = _stage_path(tag, 'pid')
    return (
        '(pid_file={pid}; '
        'if [ -s "$pid_file" ]; then '
        'stage_pid=$(cat "$pid_file"); '
        'case "$stage_pid" in ""|*[!0-9]*) rm -f "$pid_file";; '
        '*) if ps -o args= -p "$stage_pid" 2>/dev/null | '
        'grep -Fq "$pid_file"; then '
        'kill -{signal} -- "-$stage_pid" 2>/dev/null || '
        'kill -{signal} "$stage_pid" 2>/dev/null || true; '
        'else rm -f "$pid_file"; fi;; esac; fi)'
    ).format(pid=shlex.quote(pid_file), signal=signal)


def build_stage_probe_command(tag):
    pid_file = _stage_path(tag, 'pid')
    legacy_probe = {
        'robot': (
            "pgrep -f '[f]r3(_real)?[.]launch|[f]ranka_control_node' "
            '>/dev/null || pgrep -x gzserver >/dev/null'),
        'nodes': (
            "pgrep -f '[c]olmag_arm_nodes[.]launch|[c]olmag_draw_node.py|"
            "[c]olmag_robot_node.py' >/dev/null"),
        'interface': (
            "pgrep -f '[m]agnetometer_reader.py' >/dev/null"),
        'window': 'pgrep -x gzclient >/dev/null',
        'pilot': "pgrep -f '[t]arget_reaching_pilot.py' >/dev/null",
    }[tag]
    return (
        'managed=false; '
        'pid_file={pid}; '
        'if [ -s "$pid_file" ]; then '
        'stage_pid=$(cat "$pid_file"); '
        'if ps -o args= -p "$stage_pid" 2>/dev/null | '
        'grep -Fq "$pid_file"; then managed=true; '
        'else rm -f "$pid_file"; fi; fi'
        '; if $managed || {legacy}; then echo running; fi'
    ).format(pid=shlex.quote(pid_file), legacy=legacy_probe)


def _legacy_process_signals(signal, groups=('interface', 'nodes', 'robot')):
    # Bracketed patterns deliberately cannot match this cleanup shell's own
    # command line. The old "pkill -f roslaunch" did, aborting Stop All early.
    by_group = {
        'interface': ('[m]agnetometer_reader.py', '[r]ecord_tracking_error.py',
                      '[t]arget_reaching_pilot.py', '[t]eleoperation_serial_stream.py'),
        'nodes': (
            '[c]olmag_arm_nodes[.]launch',
            '[c]olmag_draw_node.py',
            '[c]olmag_robot_node.py',
            '[r]ostopic.*[/]colmag/',
        ),
        'robot': (
            '[f]r3_real[.]launch',
            '[f]r3[.]launch',
            '[f]ranka_control_node',
            '[f]ranka_gripper_node',
            '[c]ontroller_manager/spawner',
            '[r]obot_state_publisher',
            '[j]oint_state_publisher',
            '[r]oslaunch',
            '[r]osmaster',
            '[r]oscore',
            '[r]osout',
        ),
    }
    patterns = tuple(
        pattern for group in groups for pattern in by_group[group])
    commands = [
        "pkill -{} -f '{}' 2>/dev/null || true".format(
            signal, pattern)
        for pattern in patterns
    ]
    if 'robot' in groups:
        commands.extend(
            'pkill -{} -x {} 2>/dev/null || true'.format(signal, process)
            for process in ('gzclient', 'gzserver')
        )
    return commands


def build_pipeline_probe_command():
    patterns = (
        '[m]agnetometer_reader.py',
        '[t]arget_reaching_pilot.py',
        '[t]eleoperation_serial_stream.py',
        '[c]olmag_arm_nodes[.]launch',
        '[c]olmag_draw_node.py',
        '[c]olmag_robot_node.py',
        '[r]ostopic.*[/]colmag/',
        '[f]r3(_real)?[.]launch',
        '[f]ranka_control_node',
        '[f]ranka_gripper_node',
        '[c]ontroller_manager/spawner',
        '[r]obot_state_publisher',
        '[j]oint_state_publisher',
        '[r]oslaunch',
        '[r]osmaster',
        '[r]oscore',
        '[r]osout',
    )
    pid_files = ' '.join(
        shlex.quote(_stage_path(tag, 'pid')) for tag in DETACHED_TAGS)
    return (
        'for pid_file in {pid_files}; do '
        'if [ -s "$pid_file" ]; then stage_pid=$(cat "$pid_file"); '
        'if ps -o args= -p "$stage_pid" 2>/dev/null | '
        'grep -Fq "$pid_file"; then echo running; exit 0; '
        'else rm -f "$pid_file"; fi; fi; done; '
        "if pgrep -f '{patterns}' >/dev/null || pgrep -x gzclient >/dev/null || "
        'pgrep -x gzserver >/dev/null; then echo running; fi'
    ).format(pid_files=pid_files, patterns='|'.join(patterns))


def build_clear_stale_launcher_state_command():
    paths = []
    for tag in DETACHED_TAGS:
        paths.extend((_stage_path(tag, 'pid'), _stage_path(tag, 'log')))
    return 'rm -f {}'.format(
        ' '.join(shlex.quote(path) for path in paths))


def build_ros_node_shutdown_command(nodes):
    quoted_nodes = ' '.join(shlex.quote(node) for node in nodes)
    return (
        'listed=$(rosnode list 2>/dev/null) || listed=""; '
        'for node in {nodes}; do '
        'printf "%s\\n" "$listed" | grep -Fxq "$node" || continue; '
        'timeout 3 rosnode kill "$node" >/dev/null 2>&1 || true; '
        'done'
    ).format(nodes=quoted_nodes)


def build_ros_pipeline_shutdown_command():
    commands = [
        build_ros_node_shutdown_command(INTERFACE_ROS_NODES),
        build_ros_node_shutdown_command(ARM_ROS_NODES),
        # Let the arm nodes cancel their trajectories before stopping the
        # controller/backend nodes on a real robot.
        'sleep 1',
        build_ros_node_shutdown_command(ROBOT_ROS_NODES),
        'true',
    ]
    return '; '.join(commands)


def build_stop_all_command():
    """Stop local managed jobs plus processes from older launcher versions."""
    stop_order = ('pilot', 'interface', 'nodes', 'window', 'robot')
    commands = [_managed_stage_signal('pilot', 'TERM'),
                _managed_stage_signal('interface', 'TERM')]
    commands.extend(_legacy_process_signals('TERM', ('interface',)))
    commands.append(_managed_stage_signal('nodes', 'TERM'))
    commands.extend(_legacy_process_signals('TERM', ('nodes',)))
    # Give the arm nodes time to cancel/empty their trajectories and leave the
    # hardware controller holding position before franka_control is stopped.
    commands.append('sleep 1')
    commands.extend(
        _managed_stage_signal(tag, 'TERM') for tag in ('window', 'robot'))
    commands.extend(_legacy_process_signals('TERM', ('robot',)))
    commands.append('sleep 2')
    commands.extend(_managed_stage_signal(tag, 'KILL') for tag in stop_order)
    commands.extend(_legacy_process_signals('KILL'))
    commands.extend(
        'rm -f {}'.format(shlex.quote(_stage_path(tag, 'pid')))
        for tag in DETACHED_TAGS
    )
    commands.append('true')
    return '; '.join(commands)


def detect_robot_backend(ros_nodes):
    nodes = set(line.strip() for line in ros_nodes.splitlines())
    real = '/franka_control' in nodes
    simulation = '/gazebo' in nodes
    if real and simulation:
        return 'conflict'
    if real:
        return 'real'
    if simulation:
        return 'sim'
    return None


def build_live_ros_nodes_command(nodes=TRACKED_ROS_NODES):
    nodes = ' '.join(shlex.quote(node) for node in nodes)
    return (
        'listed=$(rosnode list 2>/dev/null) || exit 0; '
        'for node in {nodes}; do '
        'printf "%s\\n" "$listed" | grep -Fxq "$node" || continue; '
        'timeout 1 rosnode ping -c 1 "$node" >/dev/null 2>&1 && '
        'printf "%s\\n" "$node"; '
        'done'
    ).format(nodes=nodes)


def build_local_stage_status_command():
    """Report pipeline stages backed by processes in this container."""
    return (
        "real=false; sim=false; "
        "pgrep -f '[f]ranka_control_node' >/dev/null && real=true || true; "
        "pgrep -x gzserver >/dev/null && sim=true || true; "
        'if $real && $sim; then echo robot:conflict; '
        'elif $real; then echo robot:real; '
        'elif $sim; then '
        'if pgrep -x gzclient >/dev/null; then echo robot:sim; '
        'else echo robot:sim-nowin; fi; fi; '
        "if pgrep -f '[c]olmag_draw_node.py' >/dev/null && "
        "pgrep -f '[c]olmag_robot_node.py' >/dev/null; "
        'then echo nodes:up; fi'
    )


def parse_local_stage_status(output):
    """Convert build_local_stage_status_command output to launcher lights."""
    lines = set(line.strip() for line in output.splitlines())
    if 'robot:conflict' in lines:
        robot = 'conflict'
    elif 'robot:sim-nowin' in lines:
        robot = 'nowin'
    elif 'robot:real' in lines or 'robot:sim' in lines:
        robot = True
    else:
        robot = False
    return robot, 'nodes:up' in lines


def controller_is_running(service_output, controller):
    current_name = None
    states = {}
    for line in service_output.splitlines():
        field = line.strip()
        if field.startswith('name:'):
            current_name = field.split(':', 1)[1].strip().strip('"\'')
        elif field.startswith('state:') and current_name:
            states[current_name] = field.split(':', 1)[1].strip().strip('"\'')
    return states.get(controller) == 'running'


def parse_franka_robot_mode(topic_output):
    for line in topic_output.splitlines():
        field = line.strip()
        if field.startswith('robot_mode:'):
            field = field.split(':', 1)[1].strip()
        if field.isdigit():
            value = int(field)
            if value in FRANKA_ROBOT_MODES:
                return value
    return None


def build_interface_command(input_source, serial_port=''):
    args = ['python3', 'magnetometer_reader.py']
    if input_source == 'trackpad':
        args.extend(['--input-source', 'trackpad'])
    elif input_source == 'magnetometer':
        if not serial_port:
            raise ValueError('A serial port is required for magnetometer input.')
        args.extend([
            '--input-source', 'serial',
            '--port', serial_port,
            '--clean',
            '--writing-max-z', '0.05',
        ])
    else:
        raise ValueError('Unknown input source: {}'.format(input_source))

    args.extend(['--ros', '--classifier-labels', 'ABCXLRUD0123'])
    return 'cd /colmag && {}'.format(' '.join(shlex.quote(arg) for arg in args))


def build_record_command(serial_port, participant_id, session_id, height_mm,
                         experiment_name='', demo=False):
    """Record participant characters from the board or a separate mouse demo.

    Unlike the Interface stage this never passes --ros: a recording must not
    send commands to the robot.
    """
    if not demo and not serial_port:
        raise ValueError('A serial port is required for recording.')
    if not dataset.PARTICIPANT_ID.match(participant_id or ''):
        raise ValueError('Participant IDs look like P01, got {!r}.'.format(
            participant_id))
    if not dataset.SESSION_ID.match(session_id or ''):
        raise ValueError('Session IDs look like S01, got {!r}.'.format(
            session_id))
    args = [
        'python3', 'magnetometer_reader.py',
        '--input-source', 'trackpad' if demo else 'serial',
    ]
    if demo:
        args.extend(['--touchpad-ink-mode', 'pen', '--writing-min-velocity', '0'])
    else:
        args.extend(['--port', serial_port])
    args.extend([
        '--clean',
        '--record-data',
        '--no-classifier',
        '--output-dir', dataset.session_output_dir(participant_id, session_id,
                                                   demo=demo),
        '--participant-id', participant_id,
        '--session-id', session_id,
        '--height-mm', '{:g}'.format(height_mm),
        '--target-reps', str(dataset.TARGET_REPS),
    ])
    if experiment_name.strip():
        args.extend(['--experiment-name', experiment_name.strip()])
    if not demo and height_mm == 0:
        # Same pen-up filter as the Interface stage. Raised heights skip it,
        # because it would blank the saved picture above 5 cm.
        args.extend(['--writing-max-z', '0.05'])
    return 'cd /colmag && {}'.format(' '.join(shlex.quote(arg) for arg in args))


def parse_teleoperation_settings(seed, trials, tolerance_mm, dwell_seconds,
                                 magnet_count=None):
    """Validate the separate MuJoCo collection form before creating a run."""
    try:
        seed, trials = int(seed), int(trials)
        tolerance_mm, dwell_seconds = float(tolerance_mm), float(dwell_seconds)
        count = None if magnet_count in (None, '', 'Not specified') else int(magnet_count)
    except (TypeError, ValueError):
        raise ValueError('Seed and trials must be integers; tolerance and dwell must be numbers.')
    if trials < 1 or trials > 1000:
        raise ValueError('Choose between 1 and 1000 trials.')
    if not math.isfinite(tolerance_mm) or not 0 < tolerance_mm < 120:
        raise ValueError('Tolerance must be greater than 0 and less than 120 mm.')
    if not math.isfinite(dwell_seconds) or not 0 < dwell_seconds < 60:
        raise ValueError('Dwell must be greater than 0 and less than 60 seconds.')
    if count not in (None, 1, 2, 3):
        raise ValueError('Choose 1, 2 or 3 magnets, or leave the count unspecified.')
    from colmag.teleoperation_task import TeleoperationSettings, random_targets
    protocol = TeleoperationSettings(tolerance_m=tolerance_mm / 1000,
                                    dwell_s=dwell_seconds)
    # The tolerance sphere and required navigation distance must both fit in
    # the same workspace that the simulator samples, before opening a window.
    random_targets(1, seed, protocol)
    return dict(seed=seed, trials=trials, tolerance_mm=tolerance_mm,
                dwell_seconds=dwell_seconds, magnet_count=count)


def teleoperation_python():
    """Use the dedicated host environment rather than the ROS container."""
    override = os.environ.get('COLMAG_TELEOP_PYTHON', '').strip()
    if override:
        return os.path.abspath(os.path.expanduser(override)) if os.path.sep in override else (shutil.which(override) or override)
    return os.path.join(REPO_DIR, '.venv', 'teleoperation', 'bin', 'python')


def probe_teleoperation_runtime(python):
    if not os.path.isfile(python):
        return False, 'Python environment not found: {}'.format(python)
    try:
        result = subprocess.run(
            [python, '-c', 'import mujoco, numpy, PIL, tkinter; print(mujoco.__version__)'],
            cwd=REPO_DIR, capture_output=True, text=True, timeout=20)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, str(exc)
    return result.returncode == 0, (result.stdout + result.stderr).strip()


def next_teleoperation_session(data_dir, participant_id):
    if not dataset.PARTICIPANT_ID.fullmatch(participant_id or ''):
        raise ValueError('Participant IDs look like P01.')
    folder = os.path.join(data_dir, 'teleoperation', participant_id)
    numbers = [0]
    if os.path.isdir(folder):
        numbers.extend(int(name[1:]) for name in os.listdir(folder)
                       if dataset.SESSION_ID.fullmatch(name))
    return 'S{:02d}'.format(max(numbers) + 1)


def build_teleoperation_argv(python, participant_id, session_id, input_source='trackpad',
                             serial_port='', participant_name='', experiment_name='',
                             seed=0, trials=10, tolerance_mm=25, dwell_seconds=2,
                             magnet_count=None, baudrate=921600):
    """Structured host argv: names remain literal values, without a shell."""
    if not dataset.PARTICIPANT_ID.fullmatch(participant_id or ''):
        raise ValueError('Participant IDs look like P01.')
    if not dataset.SESSION_ID.fullmatch(session_id or ''):
        raise ValueError('Session IDs look like S01.')
    if input_source not in ('trackpad', 'serial'):
        raise ValueError('Teleoperation input must be trackpad or serial.')
    if input_source == 'serial' and not serial_port:
        raise ValueError('A serial port is required for sensor-board teleoperation.')
    settings = parse_teleoperation_settings(seed, trials, tolerance_mm, dwell_seconds, magnet_count)
    output = os.path.join(dataset.DATA_DIR_NAME, 'teleoperation', participant_id, session_id)
    args = [python, os.path.join(REPO_DIR, 'tools', 'teleoperation_demo.py'),
            '--input-source', input_source, '--output-dir', output,
            '--participant-id', participant_id, '--session-id', session_id,
            '--participant-name', participant_name, '--experiment-name', experiment_name,
            '--seed', str(settings['seed']), '--trials', str(settings['trials']),
            '--tolerance-mm', '{:g}'.format(settings['tolerance_mm']),
            '--dwell-seconds', '{:g}'.format(settings['dwell_seconds'])]
    if input_source == 'serial':
        args.extend(['--port', serial_port, '--baudrate', str(baudrate)])
    if settings['magnet_count'] is not None:
        args.extend(['--magnet-count', str(settings['magnet_count'])])
    return args


def parse_tracking_settings(magnet, magnet_offset_mm, heights_mm):
    """Check the Data window's tracking-error fields; return clean values."""
    magnet = magnet.strip()
    if not magnet:
        raise ValueError('Enter the magnet, e.g. 12x12mm_stack.')
    try:
        offset = float(magnet_offset_mm)
    except ValueError:
        raise ValueError('The magnet offset must be a number in mm.')
    try:
        heights = [float(h) for h in heights_mm.split(',') if h.strip()]
    except ValueError:
        raise ValueError('Heights must be numbers in mm, e.g. 10,50,100,150,200.')
    if not heights or min(heights) < 0 or offset < 0:
        raise ValueError('Enter at least one height; heights and offset '
                         'cannot be negative.')
    return magnet, offset, heights


def build_tracking_command(serial_port, magnet, magnet_offset_mm, heights_mm,
                           run_id):
    """Run tools/record_tracking_error.py for one run (it prompts per target)."""
    if not serial_port:
        raise ValueError('A serial port is required for tracking error.')
    args = [
        'python3', 'tools/record_tracking_error.py',
        '--port', serial_port,
        '--magnet', magnet,
        '--magnet-offset-mm', '{:g}'.format(magnet_offset_mm),
        '--heights-mm', ','.join('{:g}'.format(h) for h in heights_mm),
        '--run-id', run_id,
        '--output-dir', dataset.tracking_output_dir(run_id),
    ]
    return 'cd /colmag && {}'.format(' '.join(shlex.quote(arg) for arg in args))


def build_virtual_task_command(run_id, participant_id, condition, input_source,
                               tolerance_mm=20.0, dwell_s=0.5, repetitions=1,
                               magnet_count=None, notes='', participant_name='',
                               experiment_name=''):
    """Observe Gazebo TF in a separate window while the existing UI controls it."""
    TrialSettings(tolerance_m=tolerance_mm / 1000, dwell_s=dwell_s)
    default_targets(repetitions)
    if not participant_id.strip() or not condition.strip():
        raise ValueError('Enter a participant ID and condition label.')
    if magnet_count not in (None, 1, 2, 3):
        raise ValueError('The magnet stack must contain 1, 2, or 3 magnets.')
    args = ['python3', 'tools/target_reaching_pilot.py', '--run-id', run_id,
            '--participant-id', participant_id, '--condition', condition,
            '--input-source', 'trackpad' if input_source == 'trackpad' else 'serial',
            '--tolerance-mm', str(tolerance_mm), '--dwell-s', str(dwell_s),
            '--repetitions', str(repetitions), '--notes', notes]
    if magnet_count is not None:
        args.extend(['--magnet-count', str(magnet_count)])
    if participant_name:
        args.extend(['--participant-name', participant_name])
    if experiment_name:
        args.extend(['--experiment-name', experiment_name])
    return 'cd /colmag && {}'.format(' '.join(shlex.quote(arg) for arg in args))


def parse_gazebo_pilot_settings(condition, magnet_count, tolerance_mm,
                                dwell_s, repetitions, notes=''):
    condition = condition.strip()
    if not condition:
        raise ValueError('Enter a condition label, for example practice.')
    try:
        tolerance_mm, dwell_s = float(tolerance_mm), float(dwell_s)
        repetitions = int(repetitions)
        magnets = None if magnet_count in (None, '', 'Not specified') else int(magnet_count)
    except (TypeError, ValueError):
        raise ValueError('Margin and hold must be numbers; repetitions must be an integer.')
    if magnets not in (None, 1, 2, 3):
        raise ValueError('Choose 1, 2 or 3 magnets, or leave the count unspecified.')
    TrialSettings(tolerance_m=tolerance_mm / 1000, dwell_s=dwell_s)
    default_targets(repetitions)
    return dict(condition=condition, magnet_count=magnets, tolerance_mm=tolerance_mm,
                dwell_s=dwell_s, repetitions=repetitions, notes=notes)


def active_teleop_input_source(process_output):
    """Identify the actual reader process; selector changes do not restart it."""
    readers = []
    for line in process_output.splitlines():
        try:
            tokens = shlex.split(line)
        except ValueError:
            continue
        # pgrep joins argv with spaces: a bash -lc wrapper may therefore have
        # an unquoted reader name later in its text. Only accept a Python
        # executable whose script argument is the reader itself.
        if (len(tokens) < 3 or not tokens[0].isdigit()
                or re.fullmatch(r'python(?:\d+(?:\.\d+)*)?', os.path.basename(tokens[1])) is None):
            continue
        index = 2
        while index < len(tokens) and tokens[index] in ('-u', '-B', '-O', '-OO', '-E', '-s', '-S', '-I', '-q'):
            index += 1
        if index >= len(tokens) or os.path.basename(tokens[index]) != 'magnetometer_reader.py':
            continue
        args = tokens[index + 1:]
        if '--record-data' in args or '--ros' not in args:
            raise ValueError('The active reader is a collection window. Start the teleop Interface first.')
        source = 'serial'
        for i, token in enumerate(args):
            if token == '--input-source' and i + 1 < len(args):
                source = args[i + 1]
            elif token.startswith('--input-source='):
                source = token.split('=', 1)[1]
        if source not in ('serial', 'trackpad', 'touchpad'):
            raise ValueError('Could not verify the active Interface input source.')
        readers.append('serial' if source == 'serial' else 'trackpad')
    if len(readers) != 1:
        raise ValueError('Start exactly one teleop Interface before opening the virtual task.')
    return readers[0]


# Host terminals that can run an interactive command, with the flag that
# precedes it. The tracking-error recorder asks for Enter at every target, so it
# needs a real terminal instead of a detached launcher stage.
TERMINALS = (
    ('gnome-terminal', ['--']),
    ('konsole', ['-e']),
    ('xfce4-terminal', ['-x']),
    ('xterm', ['-e']),
    ('x-terminal-emulator', ['-e']),
)


def find_terminal():
    for name, _ in TERMINALS:
        if shutil.which(name):
            return name
    return None


def build_terminal_argv(terminal, container_command):
    """Open `terminal` running container_command interactively in Docker."""
    docker = 'docker exec -it {} bash -lc {}'.format(
        CONTAINER, shlex.quote(container_command))
    # Keep the window open afterwards so the summary can be read.
    host = '{}; echo; read -r -p "Finished. Press Enter to close."'.format(docker)
    return [terminal] + dict(TERMINALS)[terminal] + ['bash', '-lc', host]


def serial_port_label(port):
    return port[5:] if port.startswith('/host/dev/') else port


def resolve_serial_port(selection, available_ports):
    selection = selection.strip()
    if not selection:
        raise ValueError('Enter a port number from the Interface log.')
    if not selection.isdigit():
        raise ValueError('The port selection must be a number from the log.')

    index = int(selection) - 1
    if not available_ports:
        raise ValueError('No serial ports are visible inside Docker.')
    if index < 0 or index >= len(available_ports):
        choices = '\n'.join(
            '{}: {}'.format(i + 1, serial_port_label(port))
            for i, port in enumerate(available_ports))
        raise ValueError(
            'Port number {} is not available. Detected ports:\n{}'.format(
                selection, choices))
    return available_ports[index]


def format_serial_port_list(ports):
    if not ports:
        return ('Available serial ports inside {}:\n'
                '  (none found)\n\n'
                'Reconnect the sensor, then select magnetometer again.'.format(
                    CONTAINER))
    choices = '\n'.join(
        '  {}: {}'.format(index + 1, serial_port_label(port))
        for index, port in enumerate(ports))
    return ('Available serial ports inside {}:\n{}\n\n'
            'Enter a port number in the port # field, then click Start.'.format(
                CONTAINER, choices))


def container_serial_ports():
    script = (
        'import glob, os, serial.tools.list_ports as p; '
        'devices=[port.device for port in p.comports()]; '
        'mapped=[("/host"+device if os.path.exists("/host"+device) else device) '
        'for device in devices]; '
        'mapped=glob.glob("/host/dev/ttyACM*")+'
        'glob.glob("/host/dev/ttyUSB*")+mapped; '
        'mapped=list(dict.fromkeys(path for path in mapped if os.path.exists(path))); '
        'print("\\n".join(mapped))')
    ok, output = in_container(
        'python3 -c {}'.format(shlex.quote(script)), timeout=10)
    if not ok:
        return None
    return [line.strip() for line in output.splitlines() if line.strip()]


def probe_container_serial_port(port):
    script = ('import os, sys; '
              'fd=os.open(sys.argv[1], os.O_RDWR | os.O_NONBLOCK); '
              'os.close(fd)')
    return in_container(
        'python3 -c {} {}'.format(shlex.quote(script), shlex.quote(port)),
        timeout=10)


def round_rect(canvas, x1, y1, x2, y2, r, **kw):
    pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
           x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
    return canvas.create_polygon(pts, smooth=True, **kw)


# ── macOS-style custom widgets ───────────────────────────────────────────────

class Pill(tk.Canvas):
    """Rounded push button."""

    def __init__(self, parent, text, command, kind='primary',
                 width=96, height=32, font=None, parent_bg=CARD):
        super().__init__(parent, width=width, height=height, bg=parent_bg,
                         highlightthickness=0, bd=0, cursor='hand2')
        self._command = command
        self._kind = kind
        fills = {'primary': (BLUE, 'white', BLUE),
                 'danger': ('#ffffff', RED, '#f2cfcc'),
                 'plain': ('#ffffff', TEXT, BORDER)}
        self._fill, fg, outline = fills[kind]
        self._shape = round_rect(self, 1, 1, width - 1, height - 1,
                                 height // 2 - 1, fill=self._fill,
                                 outline=outline)
        self.create_text(width // 2, height // 2, text=text, fill=fg, font=font)
        self.bind('<Button-1>', lambda e: self._command())
        self.bind('<Enter>', lambda e: self.itemconfigure(
            self._shape, fill={'primary': BLUE_DARK, 'danger': '#fff0ef',
                               'plain': '#f5f5f7'}[self._kind]))
        self.bind('<Leave>', lambda e: self.itemconfigure(
            self._shape, fill=self._fill))


class Segmented(tk.Canvas):
    """macOS segmented control bound to a StringVar."""

    def __init__(self, parent, variable, options, command=None,
                 width=300, height=32, font=None, parent_bg=BG):
        super().__init__(parent, width=width, height=height, bg=parent_bg,
                         highlightthickness=0, bd=0, cursor='hand2')
        self._var, self._command, self._font = variable, command, font
        self._opts = options          # [(value, label), ...]
        self._wseg = (width - 4) // len(options)
        round_rect(self, 0, 0, width, height, height // 2, fill=TRACK,
                   outline=TRACK)
        self._redraw()
        self.bind('<Button-1>', self._click)

    def _redraw(self):
        self.delete('seg')
        h = int(self['height'])
        for i, (value, label) in enumerate(self._opts):
            x1 = 2 + i * self._wseg
            if value == self._var.get():
                round_rect(self, x1 + 1, 3, x1 + self._wseg - 1, h - 3,
                           (h - 6) // 2, fill=CARD, outline='#d8d8dd',
                           tags='seg')
            self.create_text(x1 + self._wseg // 2, h // 2, text=label,
                             fill=TEXT, font=self._font, tags='seg')

    def _click(self, event):
        idx = max(0, min(len(self._opts) - 1,
                         (event.x - 2) // self._wseg))
        value = self._opts[idx][0]
        if value != self._var.get():
            self._var.set(value)
            self._redraw()
            if self._command:
                self._command()


class Toggle(tk.Canvas):
    """macOS switch bound to a BooleanVar."""

    def __init__(self, parent, variable, parent_bg=CARD, command=None):
        w, h = 46, 27
        super().__init__(parent, width=w, height=h, bg=parent_bg,
                         highlightthickness=0, bd=0, cursor='hand2')
        self._var, self._command = variable, command
        self._redraw()
        self.bind('<Button-1>', self._flip)

    def _redraw(self):
        self.delete('all')
        on = bool(self._var.get())
        round_rect(self, 1, 1, 45, 26, 12, fill=GREEN if on else TRACK,
                   outline='' if on else '#dcdce1')
        x = 32 if on else 13
        self.create_oval(x - 10, 3, x + 10, 23, fill='white',
                         outline='#e6e6e6')

    def _flip(self, _):
        self._var.set(not self._var.get())
        self._redraw()
        if self._command:
            self._command()


class Selector(tk.Canvas):
    """Rounded value cycler (click to switch between options)."""

    def __init__(self, parent, variable, options, width=132, height=30,
                 font=None, parent_bg=CARD):
        super().__init__(parent, width=width, height=height, bg=parent_bg,
                         highlightthickness=0, bd=0, cursor='hand2')
        self._var, self._opts, self._font = variable, list(options), font
        self._shape = round_rect(self, 1, 1, width - 1, height - 1,
                                 height // 2 - 1, fill='#ffffff',
                                 outline=BORDER)
        self._wdt, self._hgt = width, height
        self._redraw()
        self.bind('<Button-1>', self._next)
        self.bind('<Enter>', lambda e: self.itemconfigure(self._shape,
                                                          fill='#f5f5f7'))
        self.bind('<Leave>', lambda e: self.itemconfigure(self._shape,
                                                          fill='#ffffff'))

    def _redraw(self):
        self.delete('txt')
        self.create_text(self._wdt // 2 - 6, self._hgt // 2,
                         text=self._var.get(), fill=TEXT, font=self._font,
                         tags='txt')
        self.create_text(self._wdt - 16, self._hgt // 2, text='⌄',
                         fill=SUBTLE, font=self._font, tags='txt')

    def _next(self, _):
        idx = self._opts.index(self._var.get())
        self._var.set(self._opts[(idx + 1) % len(self._opts)])
        self._redraw()


class RoundEntry(tk.Canvas):
    """Entry inside a rounded border."""

    def __init__(self, parent, variable, width=130, height=30, font=None,
                 parent_bg=BG):
        super().__init__(parent, width=width, height=height, bg=parent_bg,
                         highlightthickness=0, bd=0)
        round_rect(self, 1, 1, width - 1, height - 1, height // 2 - 1,
                   fill='#ffffff', outline=BORDER)
        self.entry = tk.Entry(self, textvariable=variable, relief='flat',
                              bd=0, highlightthickness=0, font=font,
                              fg=TEXT, bg='#ffffff', justify='center',
                              disabledbackground='#ffffff',
                              disabledforeground='#c7c7cc')
        self.create_window(width // 2, height // 2, window=self.entry,
                           width=width - 22, height=height - 10)


class Card(tk.Canvas):
    """Rounded white card; children go into .inner."""

    def __init__(self, parent, height, width=WIDTH):
        super().__init__(parent, width=width, height=height, bg=BG,
                         highlightthickness=0, bd=0)
        round_rect(self, 1, 2, width - 1, height - 1, 14,
                   fill=CARD, outline=BORDER)
        self.inner = tk.Frame(self, bg=CARD)
        self.create_window(width // 2, height // 2, window=self.inner,
                           width=width - 28, height=height - 18)


class ActionMappingEditor(tk.Toplevel):
    """Modal editor for the validated character-to-action JSON file."""

    def __init__(self, parent, path=DEFAULT_ACTION_MAP_PATH, on_saved=None):
        super().__init__(parent)
        self.parent = parent
        self.path = path
        self.on_saved = on_saved
        self.title('Robot Action Mapping')
        self.configure(bg=BG)
        self.resizable(False, False)
        self.transient(parent)
        self.protocol('WM_DELETE_WINDOW', self.destroy)

        try:
            mapping = load_action_mapping(path)
            load_error = None
        except FileNotFoundError:
            mapping = default_action_mapping()
            load_error = None
        except Exception as exc:
            mapping = default_action_mapping()
            load_error = str(exc)

        self._action_name_to_id = {
            display_name: action_id
            for action_id, display_name in ACTION_CATALOG
        }
        self._action_names = tuple(self._action_name_to_id)
        action_display = dict(ACTION_CATALOG)
        self._variables = {
            label: tk.StringVar(value=action_display[mapping[label]])
            for label in DIGIT_LABELS + LETTER_LABELS
        }
        self._mode = tk.StringVar(value='letters')
        self._frames = {}
        combo_style = ttk.Style(self)
        combo_style.configure(
            'Action.TCombobox', padding=(6, 3), fieldbackground='#ffffff',
            background='#ffffff', foreground=TEXT)
        combo_style.map(
            'Action.TCombobox',
            fieldbackground=[('readonly', '#ffffff')],
            foreground=[('readonly', TEXT)],
            selectbackground=[('readonly', '#ffffff')],
            selectforeground=[('readonly', TEXT)])
        self._build()
        self._show_mode()
        self.update_idletasks()
        x = parent.winfo_rootx() + max(
            0, (parent.winfo_width() - self.winfo_reqwidth()) // 2)
        y = parent.winfo_rooty() + 34
        self.geometry('+{}+{}'.format(x, y))
        self.grab_set()
        self.focus_set()

        if load_error:
            self.after_idle(
                lambda: messagebox.showwarning(
                    'Mapping file needs repair',
                    'The current mapping could not be loaded:\n\n{}\n\n'
                    'The editor is showing the original defaults. Saving will '
                    'repair the file; until then, the robot keeps its last '
                    'valid mapping.'.format(load_error),
                    parent=self))

    def _build(self):
        heading = tk.Frame(self, bg=BG)
        heading.pack(fill='x', padx=28, pady=(22, 12))
        tk.Label(heading, text='Robot action mapping', bg=BG, fg=TEXT,
                 font=self.parent.f_title).pack(anchor='w')
        tk.Label(
            heading,
            text='Assign recognized characters to the tested motion library.',
            bg=BG, fg=SUBTLE, font=self.parent.f_body).pack(
                anchor='w', pady=(3, 0))

        Segmented(
            self, self._mode,
            [('letters', 'Letters A-Z'), ('digits', 'Digits 0-9')],
            command=self._show_mode, width=300, height=32,
            font=self.parent.f_body, parent_bg=BG).pack(
                anchor='w', padx=28, pady=(0, 12))

        holder = tk.Frame(self, bg=CARD, highlightbackground=BORDER,
                          highlightthickness=1, width=664, height=480)
        holder.pack(padx=28)
        holder.pack_propagate(False)
        self._frames['letters'] = self._mapping_frame(
            holder, LETTER_LABELS, columns=2)
        self._frames['digits'] = self._mapping_frame(
            holder, DIGIT_LABELS, columns=2)

        note = tk.Frame(self, bg=BG)
        note.pack(fill='x', padx=28, pady=(12, 6))
        tk.Label(
            note,
            text='Applies to the next confirmed character in simulation and '
                 'real mode. Running motions are not interrupted.',
            bg=BG, fg=SUBTLE, font=self.parent.f_small).pack(anchor='w')

        controls = tk.Frame(self, bg=BG)
        controls.pack(fill='x', padx=28, pady=(5, 22))
        Pill(
            controls, 'Restore defaults', self._restore_defaults,
            kind='plain', width=142, font=self.parent.f_body,
            parent_bg=BG).pack(side='left')
        Pill(
            controls, 'Cancel', self.destroy, kind='plain', width=90,
            font=self.parent.f_body, parent_bg=BG).pack(
                side='right', padx=(10, 0))
        Pill(
            controls, 'Save mapping', self._save, kind='primary', width=132,
            font=self.parent.f_btn, parent_bg=BG).pack(side='right')

    def _mapping_frame(self, parent, labels, columns):
        frame = tk.Frame(parent, bg=CARD)
        rows = (len(labels) + columns - 1) // columns
        for index, label in enumerate(labels):
            column_group = index // rows
            row = index % rows
            left = column_group * 2
            tk.Label(
                frame, text=label, width=3, bg='#eef4fb', fg=TEXT,
                font=self.parent.f_h).grid(
                    row=row, column=left, padx=(18, 8), pady=4, sticky='w')
            combo = ttk.Combobox(
                frame, textvariable=self._variables[label],
                values=self._action_names, state='readonly', width=23,
                font=self.parent.f_body, style='Action.TCombobox')
            combo.grid(
                row=row, column=left + 1, padx=(0, 18), pady=4, sticky='w')
        return frame

    def _show_mode(self):
        for frame in self._frames.values():
            frame.pack_forget()
        self._frames[self._mode.get()].pack(fill='both', expand=True)

    def _restore_defaults(self):
        defaults = default_action_mapping()
        display_names = dict(ACTION_CATALOG)
        for label, action_id in defaults.items():
            self._variables[label].set(display_names[action_id])

    def _save(self):
        mapping = {
            label: self._action_name_to_id[self._variables[label].get()]
            for label in DIGIT_LABELS + LETTER_LABELS
        }
        try:
            save_action_mapping(self.path, mapping)
        except Exception as exc:
            messagebox.showerror(
                'Could not save mapping', str(exc), parent=self)
            return
        if self.on_saved:
            self.on_saved()
        self.destroy()


class DataPanel(tk.Toplevel):
    """Participants, recorded coverage and data-collection starts.

    A normal window beside the launcher (not modal, not kept on top), so the
    launcher's port field, log and Stop all stay usable while it is open.
    Reading and writing files happens in colmag/dataset.py.
    """

    EMPTY = TRACK          # no takes yet
    PARTIAL = '#ffefd2'    # some takes
    COMPLETE = '#dcf5e2'   # TARGET_REPS or more
    TABLE_ROWS = 8         # visible participant rows before the table scrolls
    RESCAN_MS = 5000       # how often new takes are picked up while open

    def __init__(self, parent, data_dir=DATA_DIR):
        super().__init__(parent)
        self.parent = parent
        self.data_dir = data_dir
        self.title('MagPilot Data')
        self.configure(bg=BG)
        self.resizable(False, False)
        self.protocol('WM_DELETE_WINDOW', self.destroy)
        os.makedirs(data_dir, exist_ok=True)

        self.participants = []
        self.samples = []
        self.participants_error = None
        self.collection_settings_error = None
        self.record = None  # participant shown in the form

        self.experiment_name = tk.StringVar(value='')
        try:
            settings = dataset.load_collection_settings(data_dir)
            self.experiment_name.set(settings['experiment_name'])
        except ValueError as exc:
            self.collection_settings_error = str(exc)

        # Form of the selected participant
        self.pid = tk.StringVar()
        self.name = tk.StringVar()
        self.hand = tk.StringVar(value=dataset.HANDS[0])
        self.age = tk.StringVar(value=dataset.AGE_BANDS[0])
        self.consent = tk.BooleanVar(value=False)
        self.excluded = tk.BooleanVar(value=False)
        self.notes = tk.StringVar()
        self.pipeline = tk.StringVar(value='characters')
        self.teleop_source = tk.StringVar(value='trackpad')
        self.teleop_mode = tk.StringVar(value='mujoco')
        self.teleop_seed = tk.StringVar(value='0')
        self.teleop_trials = tk.StringVar(value='10')
        self.teleop_tolerance = tk.StringVar(value='25')
        self.teleop_dwell = tk.StringVar(value='2')
        self.teleop_magnets = tk.StringVar(value='Not specified')
        self.pilot_condition = tk.StringVar(value='practice')
        self.pilot_tolerance = tk.StringVar(value='20')
        self.pilot_dwell = tk.StringVar(value='0.5')
        self.pilot_repetitions = tk.StringVar(value='1')
        self.pilot_notes = tk.StringVar(value='')
        # Table switches
        self.source = tk.StringVar(value='serial')
        self.char_set = tk.StringVar(value='digits')
        self.height = tk.StringVar(value='0')
        # Tracking-error settings
        self.magnet = tk.StringVar(value='12x12mm_stack')
        self.magnet_offset = tk.StringVar(value='0')
        self.tracking_heights = tk.StringVar(
            value=','.join(str(h) for h in dataset.HEIGHTS_MM if h > 0))

        self._build()
        if self.collection_settings_error:
            messagebox.showerror(
                'Collection settings',
                '{}\n\nFix or restore the file before saving an experiment '
                'or starting a recording.'.format(self.collection_settings_error),
                parent=self)
        self.refresh()
        if self.participants:
            self.load_participant(self.participants[0]['participant_id'])
        else:
            self.new_participant()
        runs = dataset.list_tracking_runs(data_dir)
        if runs:  # start from the settings of the last tracking run
            self.magnet.set(runs[0]['magnet'] or self.magnet.get())
            self.magnet_offset.set('{:g}'.format(runs[0]['magnet_offset_mm'] or 0))
        # Open beside the launcher when there is room.
        self.update_idletasks()
        x = parent.winfo_rootx() + parent.winfo_width() + 12
        x = max(0, min(x, self.winfo_screenwidth() - self.winfo_reqwidth()))
        y = max(0, min(parent.winfo_rooty(),
                       self.winfo_screenheight() - self.winfo_reqheight() - 64))
        self.geometry('+{}+{}'.format(x, y))
        self.focus_set()
        self.after(self.RESCAN_MS, self._rescan)

    # ── Layout ──────────────────────────────────────────────────────────────

    def _build(self):
        f = self.parent
        heading = tk.Frame(self, bg=BG)
        heading.pack(fill='x', padx=28, pady=(22, 10))
        Pill(heading, 'Refresh', self.refresh, kind='plain', width=90,
             font=f.f_body, parent_bg=BG).pack(side='right')
        tk.Label(heading, text='Data collection', bg=BG, fg=TEXT,
                 font=f.f_title).pack(anchor='w')
        self.summary = tk.Label(heading, bg=BG, fg=SUBTLE, font=f.f_body)
        self.summary.pack(anchor='w', pady=(3, 0))
        row = tk.Frame(heading, bg=BG)
        row.pack(fill='x', pady=(10, 0))
        self._pipeline_selector = Segmented(row, self.pipeline,
                  [('characters', 'Character Pipeline'), ('teleoperation', 'Teleoperation Pipeline')],
                  command=self._pipeline_changed, width=490, height=34, font=f.f_body,
                  parent_bg=BG)
        self._pipeline_selector.pack(side='left')

        experiment = self._box('Experiment')
        row = tk.Frame(experiment, bg=CARD)
        row.pack(fill='x')
        tk.Label(row, text='name', bg=CARD, fg=SUBTLE, font=f.f_body).pack(
            side='left', padx=(0, 8))
        RoundEntry(row, self.experiment_name, width=425, height=30,
                   font=f.f_body, parent_bg=CARD).pack(side='left')
        Pill(row, 'Save', self._save_experiment, kind='primary', width=80,
             font=f.f_btn).pack(side='right')
        tk.Label(experiment, text='Included in character and teleoperation sessions.',
                 bg=CARD, fg=SUBTLE, font=f.f_small).pack(anchor='w', pady=(8, 0))

        # Participant form
        self.participant_form_panel = tk.Frame(self, bg=BG)
        self.participant_form_panel.pack(fill='x')
        form = self._box('Participant', parent=self.participant_form_panel)
        row = tk.Frame(form, bg=CARD)
        row.pack(fill='x', pady=(0, 8))
        tk.Label(row, textvariable=self.pid, width=5, bg='#eef4fb', fg=TEXT,
                 font=f.f_h).pack(side='left')
        self._caption(row, 'name')
        RoundEntry(row, self.name, width=270, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')

        row = tk.Frame(form, bg=CARD)
        row.pack(fill='x', pady=(0, 8))
        self._caption(row, 'hand')
        self._hand = Segmented(
            row, self.hand, [('right', 'Right'), ('left', 'Left')],
            width=150, height=30, font=f.f_body, parent_bg=CARD)
        self._hand.pack(side='left')
        self._caption(row, 'age')
        self._age = Selector(row, self.age, dataset.AGE_BANDS, width=120,
                             height=30, font=f.f_body)
        self._age.pack(side='left')
        self._caption(row, 'consent')
        self._consent = Toggle(row, self.consent)
        self._consent.pack(side='left')

        row = tk.Frame(form, bg=CARD)
        row.pack(fill='x')
        tk.Label(row, text='notes', bg=CARD, fg=SUBTLE, font=f.f_body).pack(
            side='left', padx=(0, 8))
        RoundEntry(row, self.notes, width=250, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')
        self._caption(row, 'excluded')
        self._excluded = Toggle(row, self.excluded)
        self._excluded.pack(side='left')
        Pill(row, 'Save', self.save_participant, kind='primary', width=80,
             font=f.f_btn).pack(side='right')
        Pill(row, 'New', self.new_participant, kind='plain', width=70,
             font=f.f_body).pack(side='right', padx=(0, 8))
        self.form_status = tk.Label(
            form, bg=CARD, fg=SUBTLE, font=f.f_small,
            text='Names are shown here; recording folders keep the participant ID.')
        self.form_status.pack(anchor='w', pady=(8, 0))

        # Teleoperation keeps the shared name/consent fields compact so the
        # participant Start list stays visible beside the inline pilot options.
        self.participant_compact_panel = tk.Frame(self, bg=BG)
        compact = self._box('Participant', parent=self.participant_compact_panel)
        row = tk.Frame(compact, bg=CARD)
        row.pack(fill='x')
        tk.Label(row, textvariable=self.pid, width=5, bg='#eef4fb', fg=TEXT,
                 font=f.f_h).pack(side='left')
        self._caption(row, 'name')
        RoundEntry(row, self.name, width=230, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')
        self._caption(row, 'consent')
        self._compact_consent = Toggle(row, self.consent)
        self._compact_consent.pack(side='left')
        Pill(row, 'Save', self.save_participant, kind='primary', width=80,
             font=f.f_btn).pack(side='right')
        Pill(row, 'New', self.new_participant, kind='plain', width=70,
             font=f.f_body).pack(side='right', padx=(0, 8))

        self.pipeline_body = tk.Frame(self, bg=BG)
        self.pipeline_body.pack(fill='x')
        self.character_panel = tk.Frame(self.pipeline_body, bg=BG)
        self.character_panel.pack(fill='x')
        row = tk.Frame(self.character_panel, bg=BG)
        row.pack(fill='x', padx=28, pady=(14, 0))
        tk.Label(row, text='Character source', bg=BG, fg=SUBTLE,
                 font=f.f_body).pack(side='left', padx=(0, 12))
        Segmented(row, self.source,
                  [('serial', 'Sensor board'), ('touchpad', 'Demo (mouse)')],
                  command=self.refresh, width=310, height=30, font=f.f_body,
                  parent_bg=BG).pack(side='left')
        self.source_notice = tk.Label(self.character_panel, bg=BG, fg=SUBTLE,
                                      font=f.f_small, anchor='w', justify='left')
        self.source_notice.pack(anchor='w', padx=28, pady=(6, 0))

        # Which part of the dataset the table shows
        switches = tk.Frame(self.character_panel, bg=BG)
        switches.pack(fill='x', padx=28, pady=(14, 8))
        Segmented(
            switches, self.char_set,
            [('digits', 'Digits 0-9'), ('letters', 'Letters A-J')],
            command=self.show_table, width=230, height=30, font=f.f_body,
            parent_bg=BG).pack(side='left')
        Segmented(
            switches, self.height,
            [(str(h), '{} mm'.format(h)) for h in dataset.HEIGHTS_MM],
            command=self.refresh, width=390, height=30, font=f.f_small,
            parent_bg=BG).pack(side='right')
        tk.Label(switches, text='height', bg=BG, fg=SUBTLE,
                 font=f.f_body).pack(side='right', padx=(0, 8))

        # Coverage table: one row per participant, one column per character.
        # It sits in a canvas so it can scroll when there are many people.
        holder = tk.Frame(self.character_panel, bg=CARD, highlightbackground=BORDER,
                          highlightthickness=1)
        holder.pack(fill='x', padx=28)
        self.table_view = tk.Canvas(holder, bg=CARD, highlightthickness=0,
                                    width=WIDTH - 40)
        self.table_scroll = tk.Scrollbar(holder, orient='vertical',
                                         command=self.table_view.yview)
        self.table_view.configure(yscrollcommand=self.table_scroll.set)
        self.table_view.pack(side='left', fill='both', expand=True)
        self.table = tk.Frame(self.table_view, bg=CARD)
        self.table_view.create_window(0, 0, window=self.table, anchor='nw')
        for event, step in (('<Button-4>', -1), ('<Button-5>', 1)):
            self.bind(event, lambda event, step=step: self._scroll_data_content(event, step))
        self.bind('<MouseWheel>', lambda event: self._scroll_data_content(
            event, -1 if event.delta > 0 else 1))
        self.problems = tk.Label(self.character_panel, bg=BG, fg=SUBTLE, font=f.f_small,
                                 justify='left')
        self.problems.pack(anchor='w', padx=28, pady=(4, 0))

        # Tracking error
        tracking = self._box('Tracking error (sensor board)', parent=self.character_panel)
        row = tk.Frame(tracking, bg=CARD)
        row.pack(fill='x')
        tk.Label(row, text='magnet', bg=CARD, fg=SUBTLE, font=f.f_body).pack(
            side='left', padx=(0, 8))
        RoundEntry(row, self.magnet, width=165, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')
        self._caption(row, 'offset mm')
        RoundEntry(row, self.magnet_offset, width=60, height=30,
                   font=f.f_body, parent_bg=CARD).pack(side='left')
        self._caption(row, 'heights mm')
        RoundEntry(row, self.tracking_heights, width=185, height=30,
                   font=f.f_body, parent_bg=CARD).pack(side='left')
        Pill(row, 'Start', self.start_tracking, kind='primary', width=80,
             font=f.f_btn).pack(side='right')
        self.tracking_runs = tk.Label(tracking, bg=CARD, fg=SUBTLE,
                                      font=f.f_small, justify='left')
        self.tracking_runs.pack(anchor='w', pady=(8, 0))

        self.teleoperation_panel = tk.Frame(self.pipeline_body, bg=BG)
        mode_row = tk.Frame(self.teleoperation_panel, bg=BG)
        mode_row.pack(fill='x', padx=28, pady=(14, 0))
        self._teleop_mode_selector = Segmented(mode_row, self.teleop_mode,
                  [('mujoco', 'MuJoCo demo'), ('gazebo', 'Gazebo pilot')],
                  command=self._teleop_mode_changed, width=340, height=32,
                  font=f.f_body, parent_bg=BG)
        self._teleop_mode_selector.pack(side='left')
        self._caption(mode_row, 'magnets')
        Selector(mode_row, self.teleop_magnets, ['Not specified', '1', '2', '3'],
                 width=135, height=30, font=f.f_body, parent_bg=BG).pack(side='left')

        options = tk.Frame(self.teleoperation_panel, bg=BG)
        options.pack(fill='x')
        self.teleop_options_view = tk.Canvas(options, bg=BG, highlightthickness=0,
                                            width=WIDTH - 40, height=180,
                                            yscrollincrement=28)
        self.teleop_options_scroll = tk.Scrollbar(options, orient='vertical',
                                                 command=self.teleop_options_view.yview)
        self.teleop_options_view.pack(side='left', fill='both', expand=True)
        self.teleop_options_view.configure(yscrollcommand=self.teleop_options_scroll.set)
        self.teleop_options_content = tk.Frame(self.teleop_options_view, bg=BG)
        self.teleop_options_view.create_window(0, 0, window=self.teleop_options_content,
                                              anchor='nw', width=WIDTH - 40)
        self.teleop_options_content.bind('<Configure>', self._fit_teleop_options)
        self.mujoco_options = tk.Frame(self.teleop_options_content, bg=BG)
        self.mujoco_options.pack(fill='x')
        task = self._box('MuJoCo · random targets', parent=self.mujoco_options)
        row = tk.Frame(task, bg=CARD)
        row.pack(fill='x')
        tk.Label(row, text='input', bg=CARD, fg=SUBTLE, font=f.f_body).pack(side='left', padx=(0, 12))
        Segmented(row, self.teleop_source,
                  [('trackpad', 'Trackpad / mouse'), ('serial', 'Sensor board')],
                  command=self._update_teleop_summary, width=330, height=30,
                  font=f.f_body, parent_bg=CARD).pack(side='left')
        row = tk.Frame(task, bg=CARD)
        row.pack(fill='x', pady=(12, 0))
        for caption, variable, width in (
                ('seed', self.teleop_seed, 65), ('trials', self.teleop_trials, 65),
                ('margin mm', self.teleop_tolerance, 65), ('hold seconds', self.teleop_dwell, 65)):
            self._caption(row, caption)
            RoundEntry(row, variable, width=width, height=30, font=f.f_body,
                       parent_bg=CARD).pack(side='left')
        tk.Label(task, text='Enter starts each trial. Reach the blue target and hold for the selected duration.\n'
                 'The circle fills while the measured flange stays inside the margin; completion saves automatically.',
                 bg=CARD, fg=SUBTLE, font=f.f_small, justify='left', wraplength=690).pack(anchor='w', pady=(12, 0))

        self.gazebo_options = tk.Frame(self.teleop_options_content, bg=BG)
        task = self._box('Gazebo · cube-corner pilot', parent=self.gazebo_options)
        row = tk.Frame(task, bg=CARD)
        row.pack(fill='x')
        tk.Label(row, text='condition', bg=CARD, fg=SUBTLE, font=f.f_body).pack(side='left', padx=(0, 8))
        RoundEntry(row, self.pilot_condition, width=220, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')
        self._caption(row, 'repetitions')
        RoundEntry(row, self.pilot_repetitions, width=65, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')
        row = tk.Frame(task, bg=CARD)
        row.pack(fill='x', pady=(8, 0))
        for caption, variable in (('margin mm', self.pilot_tolerance), ('hold seconds', self.pilot_dwell)):
            self._caption(row, caption)
            RoundEntry(row, variable, width=65, height=30, font=f.f_body,
                       parent_bg=CARD).pack(side='left')
        row = tk.Frame(task, bg=CARD)
        row.pack(fill='x', pady=(8, 0))
        tk.Label(row, text='notes', bg=CARD, fg=SUBTLE, font=f.f_body).pack(side='left', padx=(0, 8))
        RoundEntry(row, self.pilot_notes, width=440, height=30, font=f.f_body,
                   parent_bg=CARD).pack(side='left')
        tk.Label(task, text='Uses the running Interface input and observes actual Gazebo flange feedback.\n'
                 'Condition is a log label. Robot controls remain in the Interface window.',
                 bg=CARD, fg=SUBTLE, font=f.f_small, justify='left', wraplength=630).pack(anchor='w', pady=(10, 0))
        people = self._box('Choose a participant', parent=self.teleoperation_panel)
        list_holder = tk.Frame(people, bg=CARD)
        list_holder.pack(fill='x')
        self.teleop_view = tk.Canvas(list_holder, bg=CARD, highlightthickness=0,
                                    width=WIDTH - 100, height=100, yscrollincrement=34)
        self.teleop_scroll = tk.Scrollbar(list_holder, orient='vertical', command=self.teleop_view.yview)
        self.teleop_view.configure(yscrollcommand=self.teleop_scroll.set)
        self.teleop_view.pack(side='left', fill='both', expand=True)
        self.teleop_participants = tk.Frame(self.teleop_view, bg=CARD)
        self.teleop_view.create_window(0, 0, window=self.teleop_participants, anchor='nw', width=WIDTH - 100)
        Pill(people, 'Stop', self.stop_teleoperation,
             kind='plain', width=80, height=28, font=f.f_body).pack(anchor='e', pady=(8, 0))
        self.teleop_status = tk.Label(people, bg=CARD, fg=SUBTLE, font=f.f_small,
                                     justify='left', wraplength=690)
        self.teleop_status.pack(anchor='w', pady=(10, 0))

        footer = tk.Frame(self, bg=BG)
        footer.pack(fill='x', padx=28, pady=(12, 22))
        self.recording_help = tk.Label(
            footer,
            bg=BG, fg=SUBTLE, font=f.f_small, justify='left')
        self.recording_help.pack(side='left')
        Pill(footer, 'Close', self.destroy, kind='plain', width=80,
             font=f.f_body, parent_bg=BG).pack(side='right')

    def _box(self, title, parent=None):
        parent = self if parent is None else parent
        tk.Label(parent, text=title, bg=BG, fg=TEXT, font=self.parent.f_h).pack(
            anchor='w', padx=28, pady=(12, 4))
        box = tk.Frame(parent, bg=CARD, highlightbackground=BORDER,
                       highlightthickness=1)
        box.pack(fill='x', padx=28)
        inner = tk.Frame(box, bg=CARD)
        inner.pack(fill='x', padx=14, pady=12)
        return inner

    def _pipeline_changed(self):
        self._pipeline_selector._redraw()
        if self.pipeline.get() == 'teleoperation':
            self.participant_form_panel.pack_forget()
            self.participant_compact_panel.pack(fill='x', before=self.pipeline_body)
            self.character_panel.pack_forget()
            self.teleoperation_panel.pack(fill='x')
            self._update_teleop_summary()
        else:
            self.participant_compact_panel.pack_forget()
            self.participant_form_panel.pack(fill='x', before=self.pipeline_body)
            self.teleoperation_panel.pack_forget()
            self.character_panel.pack(fill='x')
            self.refresh()
        self.update_idletasks()
        if self.pipeline.get() == 'teleoperation':
            self._fit_teleop_options()

    def _teleop_mode_changed(self):
        self._teleop_mode_selector._redraw()
        if self.teleop_mode.get() == 'gazebo':
            self.mujoco_options.pack_forget()
            self.gazebo_options.pack(fill='x')
        else:
            self.gazebo_options.pack_forget()
            self.mujoco_options.pack(fill='x')
        self._update_teleop_summary()
        self._fit_teleop_options()
        self.teleop_options_view.yview_moveto(0)

    def _fit_teleop_options(self, _event=None):
        """Scroll settings while keeping the participant Start buttons visible."""
        if self.pipeline.get() != 'teleoperation':
            return
        self.update_idletasks()
        height = self.teleop_options_content.winfo_reqheight()
        current = self.teleop_options_view.winfo_reqheight()
        people_current = self.teleop_view.winfo_reqheight()
        fixed = self.winfo_reqheight() - current - people_current
        budget = self.winfo_screenheight() - fixed - 96
        people_height = self.teleop_participants.winfo_reqheight()
        wanted_people = min(people_height, 3 * 34)
        people_visible = min(wanted_people, max(34, int((budget - 80) // 34) * 34))
        self.teleop_view.configure(height=people_visible)
        if people_height > people_visible:
            self.teleop_scroll.pack(side='right', fill='y')
        else:
            self.teleop_scroll.pack_forget()
        available = max(80, budget - people_visible)
        visible = min(height, available)
        self.teleop_options_view.configure(height=visible,
            scrollregion=(0, 0, WIDTH - 40, height))
        if height > visible:
            self.teleop_options_scroll.pack(side='right', fill='y')
        else:
            self.teleop_options_scroll.pack_forget()

    def _scroll_data_content(self, event, step):
        if self.pipeline.get() == 'characters':
            self.table_view.yview_scroll(step, 'units')
        elif (self.teleop_view.winfo_rooty() <= event.y_root <
              self.teleop_view.winfo_rooty() + self.teleop_view.winfo_height()):
            self.teleop_view.yview_scroll(step, 'units')
        else:
            self.teleop_options_view.yview_scroll(step, 'units')
        return 'break'

    def _update_teleop_summary(self):
        if self.teleop_mode.get() == 'gazebo':
            self.summary.configure(text='Gazebo FR3 · cube corners · measured end-effector feedback')
            self.recording_help.configure(text='Start Robot → Arm nodes → Interface in Simulation first.\n'
                                          'Enter starts a trial; controls stay in the Interface window.')
            running = self.parent.__dict__.get('_virtual_task_running', False)
            self.teleop_status.configure(text='Gazebo pilot running. Close its window or use Stop to finish recording.'
                                        if running else 'Saved pilot runs: data_collection/virtual_task/<run-id>/')
            return
        self.summary.configure(text='MuJoCo FR3 · random positions · measured end-effector feedback')
        self.recording_help.configure(text='Start opens the simulation. Enter begins a trial; hold inside the target to save.\n'
                                      'Trackpad / mouse runs without Docker. Sensor board uses the main window\'s port #.')
        status = self.parent.__dict__.get('_teleoperation_process')
        running = status is not None and status.poll() is None
        self.teleop_status.configure(text='Simulation running. Close its window or use Stop to finish the session.'
                                    if running else 'Saved trials: data_collection/teleoperation/<participant>/<session>/')

    def _show_teleop_participants(self):
        for child in self.teleop_participants.winfo_children():
            child.destroy()
        if not self.participants:
            tk.Label(self.teleop_participants, text='Add a name in the participant form and press Save.',
                     bg=CARD, fg=SUBTLE, font=self.parent.f_body).pack(anchor='w')
        for participant in self.participants:
            row = tk.Frame(self.teleop_participants, bg=CARD)
            row.pack(fill='x', pady=3)
            label = tk.Label(row, text=dataset.participant_label(participant),
                             bg=CARD, fg=BLUE, font=self.parent.f_body, cursor='hand2')
            label.pack(side='left')
            pid = participant['participant_id']
            label.bind('<Button-1>', lambda _, pid=pid: self.load_participant(pid))
            Pill(row, 'Start', lambda pid=pid: self.start_teleoperation(pid),
                 kind='primary', width=80, height=28, font=self.parent.f_body).pack(side='right')
        self.teleop_participants.update_idletasks()
        height = self.teleop_participants.winfo_reqheight()
        visible = min(height, 3 * 34)
        self.teleop_view.configure(height=visible,
            scrollregion=(0, 0, WIDTH - 100, height))
        if height > visible:
            self.teleop_scroll.pack(side='right', fill='y')
        else:
            self.teleop_scroll.pack_forget()

    def _caption(self, parent, text):
        tk.Label(parent, text=text, bg=CARD, fg=SUBTLE,
                 font=self.parent.f_body).pack(side='left', padx=(16, 8))

    # ── Reading the data folder ─────────────────────────────────────────────

    def refresh(self):
        try:
            self.participants = dataset.load_participants(self.data_dir)
            self.participants_error = None
        except ValueError as exc:
            self.participants = []
            self.participants_error = str(exc)
            messagebox.showerror(
                'participants.json',
                '{}\n\nFix or restore the file. Saving participants is '
                'blocked until then.'.format(exc), parent=self)
        demo = self.source.get() == 'touchpad'
        collection_dir = dataset.collection_data_dir(self.data_dir, demo=demo)
        self.samples, problems = dataset.scan_samples(
            collection_dir, input_source=self.source.get())

        active = [p for p in self.participants if not p.get('excluded')]
        wanted = len(active) * (len(dataset.DIGITS) + len(dataset.LETTERS)) \
            * dataset.TARGET_REPS
        active_ids = {p['participant_id'] for p in active}
        height_mm = int(self.height.get())
        done = dataset.progress(self.samples, active_ids, height_mm)
        folder = dataset.collection_data_dir(dataset.DATA_DIR_NAME, demo=demo)
        self.summary.configure(text='{} · {}/ · {} participants · {} / {} '
                               'takes at {} mm'.format(
                                   'DEMO' if demo else 'Sensor board', folder,
                                   len(active), done, wanted, height_mm))
        self.source_notice.configure(
            text=('DEMO: draw with the mouse. No sensor board or robot is required; '
                  'demo takes are saved separately.' if demo else
                  'Sensor-board recordings count toward the participant dataset.'),
            fg=BLUE if demo else SUBTLE)
        self.recording_help.configure(text=(
            'DEMO: select 0-9 or A-J; Enter starts and saves a take.\n'
            'Move the mouse to draw while recording; Enter stops the ink and saves.'
            if demo else
            'Start uses the selected height and the main window\'s port #.\n'
            'Select 0-9 or A-J, then press Enter to start and Enter to save.'))
        self.problems.configure(text='Check: ' + ' · '.join(
            '{} {}'.format(n, reason) for reason, n in sorted(problems.items()))
            if problems else '')

        runs = dataset.list_tracking_runs(self.data_dir)[:3]
        self.tracking_runs.configure(text='\n'.join(
            '{} · {} · heights {} mm · {} of {} targets'.format(
                run['run_id'], run['magnet'],
                ','.join('{:g}'.format(h) for h in run['heights_mm']),
                run['captured'], run['targets'])
            for run in runs) or 'No tracking-error runs yet.')
        self.show_table()
        self._show_teleop_participants()
        if self.pipeline.get() == 'teleoperation':
            self._update_teleop_summary()
            self._fit_teleop_options()

    def show_table(self):
        for child in self.table.winfo_children():
            child.destroy()
        f = self.parent
        chars = (dataset.DIGITS if self.char_set.get() == 'digits'
                 else dataset.LETTERS)
        coverage = dataset.count_coverage(
            self.samples, chars, int(self.height.get()))
        registered = [p['participant_id'] for p in self.participants]
        rows = list(self.participants) + [
            {'participant_id': pid} for pid in sorted(coverage)
            if pid not in registered]

        headers = ['Participant', 'Hand'] + list(chars) + ['Total']
        for column, text in enumerate(headers):
            tk.Label(self.table, text=text, bg=CARD, fg=SUBTLE,
                     font=f.f_small).grid(row=0, column=column, padx=3,
                                          pady=(8, 2))
        if not rows:
            tk.Label(self.table, bg=CARD, fg=SUBTLE, font=f.f_body,
                     text='No participants yet. Fill in the form above and '
                          'press Save.').grid(
                row=1, column=0, columnspan=len(headers) + 1, pady=12)
            self._fit_table()
            return

        totals = dict.fromkeys(chars, 0)
        for row, participant in enumerate(rows, start=1):
            pid = participant['participant_id']
            counts = coverage.get(pid, {})
            known = pid in registered
            counted = known and not participant.get('excluded')
            label = tk.Label(self.table, text=dataset.participant_label(participant),
                             width=18, wraplength=155, anchor='w', justify='left',
                             bg=CARD, fg=BLUE if known else SUBTLE, font=f.f_body,
                             cursor='hand2' if known else '')
            label.grid(row=row, column=0, padx=(10, 3), pady=2)
            if known:
                label.bind('<Button-1>',
                           lambda _, pid=pid: self.load_participant(pid))
            hand = participant.get('handedness', '')[:1].upper()
            tk.Label(self.table, text=hand, bg=CARD, fg=SUBTLE,
                     font=f.f_body).grid(row=row, column=1, padx=3)
            for column, char in enumerate(chars, start=2):
                n = counts.get(char, 0)
                if counted:
                    totals[char] += n
                tk.Label(self.table, text=str(n), width=3,
                         bg=self._cell_colour(n), fg=TEXT if counted else SUBTLE,
                         font=f.f_body).grid(row=row, column=column, padx=2,
                                             pady=2)
            tk.Label(self.table, text=str(sum(counts.values())), bg=CARD,
                     fg=TEXT, font=f.f_h).grid(row=row, column=len(chars) + 2,
                                               padx=(6, 3))
            if known:
                Pill(self.table, 'Start',
                     lambda pid=pid: self.start_recording(pid),
                     kind='primary', width=64, height=24,
                     font=f.f_small).grid(row=row, column=len(chars) + 3,
                                          padx=(6, 10))

        last = len(rows) + 1
        tk.Label(self.table, text='All', bg=CARD, fg=SUBTLE,
                 font=f.f_h).grid(row=last, column=0, padx=(10, 3),
                                  pady=(4, 8))
        for column, char in enumerate(chars, start=2):
            tk.Label(self.table, text=str(totals[char]), bg=CARD, fg=SUBTLE,
                     font=f.f_body).grid(row=last, column=column, pady=(4, 8))
        tk.Label(self.table, text=str(sum(totals.values())), bg=CARD,
                 fg=SUBTLE, font=f.f_h).grid(row=last, column=len(chars) + 2,
                                             pady=(4, 8))
        self._fit_table()

    def _fit_table(self):
        """Fit participant rows while keeping the form and footer on screen."""
        self.table.update_idletasks()
        width, height = self.table.winfo_reqwidth(), self.table.winfo_reqheight()
        rows = max(1, self.table.grid_size()[1])
        visible = int(height * min(1.0, (self.TABLE_ROWS + 2) / rows))
        self.table_view.configure(height=visible)
        self.update_idletasks()
        fixed_height = self.winfo_reqheight() - visible
        available = max(40, self.winfo_screenheight() - fixed_height - 80)
        visible = min(visible, available)
        self.table_view.configure(scrollregion=(0, 0, width, height),
                                  height=visible)
        if visible < height:
            self.table_scroll.pack(side='right', fill='y')
        else:
            self.table_scroll.pack_forget()
            self.table_view.yview_moveto(0)

    def _rescan(self):
        """Pick up new takes while the window is open (not participants.json)."""
        if not self.winfo_exists():
            return
        collection_dir = dataset.collection_data_dir(
            self.data_dir, demo=self.source.get() == 'touchpad')
        samples, _ = dataset.scan_samples(
            collection_dir, input_source=self.source.get())
        if samples != self.samples:
            self.refresh()
        elif self.pipeline.get() == 'teleoperation':
            self._update_teleop_summary()
        self.after(self.RESCAN_MS, self._rescan)

    def _cell_colour(self, n):
        if n >= dataset.TARGET_REPS:
            return self.COMPLETE
        return self.PARTIAL if n else self.EMPTY

    # ── Participant form ────────────────────────────────────────────────────

    def _fill_form(self, record):
        self.record = dict(record)
        self.pid.set(record['participant_id'])
        self.name.set(record.get('name', ''))
        self.hand.set(record.get('handedness', dataset.HANDS[0]))
        self.age.set(record.get('age_band', dataset.AGE_BANDS[0]))
        self.consent.set(bool(record.get('consent')))
        self.excluded.set(bool(record.get('excluded')))
        self.notes.set(record.get('notes', ''))
        for widget in (self._hand, self._age, self._consent, self._excluded, self._compact_consent):
            widget._redraw()

    def new_participant(self):
        # Never reuse an ID that already has recordings on disk.
        seen = {s['participant_id'] for s in self.samples}
        for demo in (False, True):
            characters_dir = os.path.join(
                dataset.collection_data_dir(self.data_dir, demo=demo),
                dataset.CHARACTERS_DIR)
            if os.path.isdir(characters_dir):
                seen.update(os.listdir(characters_dir))
        teleoperation_dir = os.path.join(self.data_dir, 'teleoperation')
        if os.path.isdir(teleoperation_dir):
            seen.update(os.listdir(teleoperation_dir))
        pid = dataset.next_participant_id(self.participants, seen)
        self._fill_form(dataset.new_participant(pid))
        self.form_status.configure(
            text='{} is new and not saved yet. Add a name and press Save.'.format(pid))

    def load_participant(self, participant_id):
        for record in self.participants:
            if record['participant_id'] == participant_id:
                self._fill_form(record)
                self.form_status.configure(
                    text='{} saved on {}.'.format(
                        dataset.participant_label(record), record.get('created_at', '?')))
                return

    def save_participant(self):
        if self.participants_error:
            messagebox.showerror('participants.json', self.participants_error,
                                 parent=self)
            return
        record = dict(self.record)
        record.update({
            'name': self.name.get().strip(),
            'handedness': self.hand.get(),
            'age_band': self.age.get(),
            'consent': bool(self.consent.get()),
            'excluded': bool(self.excluded.get()),
            'notes': self.notes.get().strip(),
        })
        if not record['consent']:
            record['consent_date'] = None
        elif not record.get('consent_date'):
            record['consent_date'] = date.today().isoformat()
        others = [p for p in self.participants
                  if p['participant_id'] != record['participant_id']]
        try:
            dataset.save_participants(self.data_dir, others + [record])
        except OSError as exc:
            messagebox.showerror('Could not save participant', str(exc),
                                 parent=self)
            return
        self.refresh()
        self.load_participant(record['participant_id'])

    # ── Starting recordings ─────────────────────────────────────────────────

    def _save_experiment(self):
        experiment_name = self.experiment_name.get().strip()
        try:
            # Re-read before saving so a damaged file cannot be overwritten.
            dataset.load_collection_settings(self.data_dir)
            dataset.save_collection_settings(self.data_dir, experiment_name)
        except (OSError, ValueError) as exc:
            self.collection_settings_error = str(exc)
            messagebox.showerror('Could not save experiment', str(exc), parent=self)
            return False
        self.collection_settings_error = None
        self.experiment_name.set(experiment_name)
        return True

    def start_recording(self, participant_id):
        record = next(p for p in self.participants
                      if p['participant_id'] == participant_id)
        demo = self.source.get() == 'touchpad'
        if record.get('excluded'):
            messagebox.showwarning(
                'Excluded', '{} is marked as excluded.'.format(
                    dataset.participant_label(record)),
                parent=self)
            return
        if not demo and not record.get('consent'):
            messagebox.showwarning(
                'No consent',
                'No consent is recorded for {}. Switch on consent in the form '
                'and press Save first.'.format(dataset.participant_label(record)),
                parent=self)
            return
        if not self._save_experiment():
            return
        self.parent.start_character_recording(
            participant_id, int(self.height.get()), self.experiment_name.get(),
            demo=demo)

    def start_tracking(self):
        self.parent.start_tracking_error(
            self.magnet.get(), self.magnet_offset.get(),
            self.tracking_heights.get())

    def start_teleoperation(self, participant_id):
        participant = next(p for p in self.participants if p['participant_id'] == participant_id)
        if participant.get('excluded'):
            messagebox.showwarning('Excluded', '{} is marked as excluded.'.format(
                dataset.participant_label(participant)), parent=self)
            return
        gazebo = self.teleop_mode.get() == 'gazebo'
        source = None if gazebo else self.teleop_source.get()
        if not gazebo and source == 'serial' and not participant.get('consent'):
            messagebox.showwarning('No consent', 'Save consent for {} before collecting with the sensor board.'.format(
                dataset.participant_label(participant)), parent=self)
            return
        try:
            if gazebo:
                settings = parse_gazebo_pilot_settings(self.pilot_condition.get(),
                    self.teleop_magnets.get(), self.pilot_tolerance.get(), self.pilot_dwell.get(),
                    self.pilot_repetitions.get(), self.pilot_notes.get())
            else:
                settings = parse_teleoperation_settings(
                    self.teleop_seed.get(), self.teleop_trials.get(), self.teleop_tolerance.get(),
                    self.teleop_dwell.get(), self.teleop_magnets.get())
        except ValueError as exc:
            messagebox.showerror('Teleoperation Pipeline', str(exc), parent=self)
            return
        if not self._save_experiment():
            return
        if gazebo:
            self.parent.start_virtual_task(participant_id,
                participant_name=participant.get('name', ''),
                experiment_name=self.experiment_name.get(), **settings)
        else:
            self.parent.start_teleoperation(
                participant_id, input_source=source, participant_name=participant.get('name', ''),
                experiment_name=self.experiment_name.get(), **settings)
        self._update_teleop_summary()

    def stop_teleoperation(self):
        process = self.parent.__dict__.get('_teleoperation_process')
        mujoco_running = process is not None and process.poll() is None
        gazebo_running = self.parent.__dict__.get('_virtual_task_running', False)
        if self.teleop_mode.get() == 'gazebo' or gazebo_running:
            self.parent.stop_virtual_task()
        if self.teleop_mode.get() == 'mujoco' or mujoco_running:
            self.parent.stop_teleoperation()


# ── The app ──────────────────────────────────────────────────────────────────

class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('MagPilot Control Center')
        self.configure(bg=BG)
        self.resizable(False, False)
        self._pipeline_notice = None
        self._stopping = False
        self._recovering = False
        self._closing = False
        self._close_after_stop = False
        self._teleoperation_process = None
        self._teleoperation_log_path = None
        self._teleoperation_log_file = None
        self._virtual_task_running = False
        self._fonts()
        self._build_ui()
        self._poll_running = True
        threading.Thread(target=self._poll_loop, daemon=True).start()
        self.protocol('WM_DELETE_WINDOW', self._on_close)
        self._install_signal_handlers()
        self.after_idle(self._cleanup_on_startup)

    def _fonts(self):
        ui = pick_font(['SF Pro Text', 'SF Pro Display', 'Helvetica Neue',
                        'Fira Sans', 'Inter', 'Roboto', 'Ubuntu', 'DejaVu Sans'])
        mono = pick_font(['SF Mono', 'Menlo', 'Fira Mono', 'JetBrains Mono',
                          'Ubuntu Mono', 'DejaVu Sans Mono'])
        self.f_title = (ui, 20, 'bold')
        self.f_h = (ui, 12, 'bold')
        self.f_body = (ui, 11)
        self.f_small = (ui, 9)
        self.f_btn = (ui, 11, 'bold')
        self.f_mono = (mono, 9)

    def _build_ui(self):
        head = tk.Frame(self, bg=BG)
        head.pack(fill='x', padx=26, pady=(20, 0))
        title_row = tk.Frame(head, bg=BG)
        title_row.pack(anchor='w')
        logo_path = os.path.join(REPO_DIR, 'docs', 'logo_small.png')
        self._logo_img = None
        if os.path.exists(logo_path):
            try:
                self._logo_img = tk.PhotoImage(file=logo_path)
                tk.Label(title_row, image=self._logo_img, bg=BG).pack(
                    side='left', padx=(0, 12))
            except tk.TclError:
                self._logo_img = None
        text_col = tk.Frame(title_row, bg=BG)
        text_col.pack(side='left')
        tk.Label(text_col, text='MagPilot', bg=BG, fg=TEXT,
                 font=self.f_title).pack(anchor='w')
        tk.Label(text_col, text='Pilot a Franka arm with a magnet — the whole '
                                'pipeline in one window, no terminals.',
                 bg=BG, fg=SUBTLE, font=self.f_body).pack(anchor='w',
                                                          pady=(3, 0))

        # Mode row: segmented control + IP field
        mode_row = tk.Frame(self, bg=BG)
        mode_row.pack(fill='x', padx=26, pady=(14, 8))
        self.mode = tk.StringVar(value='sim')
        Segmented(mode_row, self.mode,
                  [('sim', 'Simulation'), ('real', 'Real robot')],
                  command=self._mode_changed, width=280, height=32,
                  font=self.f_body, parent_bg=BG).pack(side='left')
        self._detected_serial_ports = []
        self._interface_notice = None
        self.sensor_port = tk.StringVar(value='')
        tk.Label(mode_row, text='port #', bg=BG, fg=SUBTLE,
                 font=self.f_body).pack(side='left', padx=(20, 8))
        self._sensor_port = RoundEntry(
            mode_row, self.sensor_port, width=112, height=30,
            font=self.f_body, parent_bg=BG)
        self._sensor_port.pack(side='left')
        self._sensor_port.entry.configure(state='disabled')
        ip_group = tk.Frame(mode_row, bg=BG)
        ip_group.pack(side='right')
        self.robot_ip = tk.StringVar(value='172.16.0.2')
        tk.Label(ip_group, text='robot IP', bg=BG, fg=SUBTLE,
                 font=self.f_body).pack(side='left', padx=(0, 8))
        self._ip = RoundEntry(ip_group, self.robot_ip, width=126, height=30,
                              font=self.f_body, parent_bg=BG)
        self._ip.pack(side='left')

        # Stage cards
        self.lights = {}
        inner = self._stage(
            'robot', '1 · Robot',
            'Gazebo FR3 + controllers (sim) · franka_control (real)',
            self.start_robot)
        Pill(inner, 'Recover', self.recover_robot, kind='plain', width=82,
             font=self.f_small).pack(side='right', padx=(0, 8))
        inner = self._stage('nodes', '2 · Arm nodes',
                            'Teleop (draw) + gestures (robot), one launch',
                            self.start_nodes)
        self.live = tk.BooleanVar(value=True)
        Toggle(inner, self.live).pack(side='right', padx=(0, 10))
        tk.Label(inner, text='live', bg=CARD, fg=TEXT,
                 font=self.f_body).pack(side='right', padx=(0, 8))
        inner = self._stage('interface', '3 · Interface',
                            'Writing / teleop UI (opens its own window)',
                            self.start_interface)
        self.input_src = tk.StringVar(value='trackpad')
        self.input_src.trace_add('write', self._input_changed)
        self._input_selector = Selector(
            inner, self.input_src, ('trackpad', 'magnetometer'),
            width=138, height=30, font=self.f_body)
        self._input_selector.pack(side='right', padx=(0, 12))

        # Control row
        row = tk.Frame(self, bg=BG)
        row.pack(fill='x', padx=26, pady=(14, 4))
        Pill(row, 'Stop all', self.stop_all, kind='danger', width=104,
             font=self.f_btn, parent_bg=BG).pack(side='left')
        Pill(row, 'Restart container', self.restart_container, kind='plain',
             width=150, font=self.f_body, parent_bg=BG
             ).pack(side='left', padx=12)
        Pill(row, 'Action mapping', self.open_action_mapping, kind='plain',
             width=142, font=self.f_body, parent_bg=BG
             ).pack(side='left')
        Pill(row, 'Data', self.open_data_panel, kind='plain', width=80,
             font=self.f_body, parent_bg=BG).pack(side='left', padx=12)
        self.container_light = tk.Label(row, text='●  container', bg=BG,
                                        fg=DOT_OFF, font=self.f_body)
        self.container_light.pack(side='right')

        # Log card
        log_card = Card(self, height=196)
        log_card.pack(padx=26, pady=(10, 20))
        top = tk.Frame(log_card.inner, bg=CARD)
        top.pack(fill='x')
        tk.Label(top, text='Log', bg=CARD, fg=SUBTLE,
                 font=self.f_small).pack(side='left')
        self.log_choice = tk.StringVar(value='interface')
        self._log_selector = Selector(
            top, self.log_choice, STAGES + ('pilot',), width=118, height=26,
            font=self.f_small)
        self._log_selector.pack(side='right')
        Pill(top, 'Copy', self.copy_log, kind='plain', width=64, height=26,
             font=self.f_small).pack(side='right', padx=(0, 8))
        self.log = tk.Text(log_card.inner, height=9, bg='#fbfbfd', fg=TEXT,
                           font=self.f_mono, relief='flat', wrap='none')
        self.log.pack(fill='both', expand=True, pady=(5, 0))
        self.log.bind('<Key>', lambda _: 'break')
        self.log.bind('<Control-c>', self._copy_log_selection)
        self.log.bind('<Control-C>', self._copy_log_selection)
        self.log.bind('<Control-a>', self._select_log_all)
        self.log.bind('<Control-A>', self._select_log_all)
        self.log.bind('<<Cut>>', lambda _: 'break')
        self.log.bind('<<Paste>>', lambda _: 'break')
        self.log.bind('<Button-2>', lambda _: 'break')
        self._log_menu = tk.Menu(self, tearoff=False)
        self._log_menu.add_command(
            label='Copy', command=lambda: self._copy_log_selection())
        self._log_menu.add_command(label='Copy all', command=self.copy_log)
        self.log.bind('<Button-3>', self._show_log_menu)

    def _stage(self, tag, title, subtitle, command):
        card = Card(self, height=68)
        card.pack(padx=26, pady=5)
        inner = card.inner
        light = tk.Label(inner, text='●', bg=CARD, fg=DOT_OFF,
                         font=(self.f_body[0], 15))
        light.pack(side='left', padx=(4, 12))
        self.lights[tag] = light
        col = tk.Frame(inner, bg=CARD)
        col.pack(side='left')
        tk.Label(col, text=title, bg=CARD, fg=TEXT, font=self.f_h
                 ).pack(anchor='w')
        tk.Label(col, text=subtitle, bg=CARD, fg=SUBTLE, font=self.f_small
                 ).pack(anchor='w')
        Pill(inner, 'Start', command, kind='primary', width=92,
             font=self.f_btn).pack(side='right', padx=(0, 2))
        return inner

    # ── Actions ─────────────────────────────────────────────────────────────

    def open_action_mapping(self):
        existing = getattr(self, '_action_mapping_editor', None)
        if existing is not None and existing.winfo_exists():
            existing.lift()
            existing.focus_set()
            return
        self._action_mapping_editor = ActionMappingEditor(
            self, on_saved=self._action_mapping_saved)

    def open_data_panel(self):
        existing = getattr(self, '_data_panel', None)
        if existing is not None and existing.winfo_exists():
            existing.refresh()
            existing.lift()
            existing.focus_set()
            return
        self._data_panel = DataPanel(self)

    def open_virtual_task(self):
        """Compatibility entry point: the pilot controls live inside Data."""
        self.open_data_panel()
        panel = self._data_panel
        panel.pipeline.set('teleoperation')
        panel.teleop_mode.set('gazebo')
        panel._teleop_mode_changed()
        panel._pipeline_changed()

    def start_virtual_task(self, participant_id, condition, magnet_count,
                           tolerance_mm, dwell_s, repetitions, notes='',
                           participant_name='', experiment_name=''):
        if self.mode.get() != 'sim':
            messagebox.showerror('Virtual task', 'Select Simulation mode for the virtual task.')
            return
        if not self._pipeline_action_ready() or not self._ensure_container():
            return
        if detect_robot_backend(self._running_ros_nodes()) != 'sim':
            messagebox.showerror('Virtual task', 'Start Robot in Simulation mode first, '
                                 'then Arm nodes and Interface.')
            return
        run_id = datetime.now().strftime('pilot_%Y%m%d_%H%M%S_%f')
        try:
            ok, processes = in_container(
                "pgrep -af '[m]agnetometer_reader.py' || true", timeout=5)
            if not ok:
                raise ValueError('Could not inspect the active Interface input source.')
            source = active_teleop_input_source(processes)
            command = build_virtual_task_command(
                run_id, participant_id, condition, source,
                tolerance_mm, dwell_s, repetitions, magnet_count, notes,
                participant_name, experiment_name)
        except (TypeError, ValueError) as exc:
            messagebox.showerror('Virtual task', str(exc))
            return
        if self._launch_stage('pilot', command):
            self._virtual_task_running = True

    def stop_virtual_task(self):
        def stop():
            command = _managed_stage_signal('pilot', 'TERM') + "; pkill -TERM -f '[t]arget_reaching_pilot.py' 2>/dev/null || true"
            in_container(command, timeout=8)
            self._virtual_task_running = False
        threading.Thread(target=stop, daemon=True).start()

    def _action_mapping_saved(self):
        self._pipeline_notice = (
            'Action mapping saved. It applies to the next confirmed character; '
            'no ROS node restart is needed.')
        self._replace_log(self._pipeline_notice)
        self.after(5000, self._clear_pipeline_notice, self._pipeline_notice)

    def _mode_changed(self):
        real = self.mode.get() == 'real'
        if real:
            messagebox.showwarning(
                'Real robot mode',
                'REAL ROBOT selected.\n\nFollow the staged pipeline in the '
                'README: dry-run first, supervisor present, E-stop reachable.')

    def _input_changed(self, *_):
        self._input_selector._redraw()  # input_src may be set from code
        magnetometer = self.input_src.get() == 'magnetometer'
        state = 'normal' if magnetometer else 'disabled'
        self._sensor_port.entry.configure(state=state)
        if magnetometer:
            self.sensor_port.set('')
            self._detected_serial_ports = []
            self._show_interface_notice('Scanning serial ports inside {}...'.format(
                CONTAINER))
            self.after_idle(self.refresh_serial_ports)
        else:
            self._interface_notice = None

    def _replace_log(self, text):
        if (not self.log.tag_ranges('sel')
                and self.log.get('1.0', 'end-1c') != text):
            self.log.delete('1.0', 'end')
            self.log.insert('end', text)
            self.log.see('end')

    def _show_interface_notice(self, text):
        self._interface_notice = text
        self.log_choice.set('interface')
        self._log_selector._redraw()
        self._replace_log(text)

    def _select_stage_log(self, tag, notice):
        self._pipeline_notice = None
        self.log_choice.set(tag)
        self._log_selector._redraw()
        self._replace_log(notice)

    def _running_ros_nodes(self):
        ok, nodes = in_container(build_live_ros_nodes_command(), timeout=6)
        return nodes if ok else ''

    def _pipeline_action_ready(self):
        # Only an in-progress cleanup gates a start, and only briefly. The
        # launcher does not otherwise second-guess what is running: you pick a
        # mode and start the stages you want.
        if self._stopping:
            messagebox.showinfo(
                'Pipeline cleanup',
                'Please wait for the current pipeline cleanup to finish.')
            return False
        return True

    def _franka_robot_mode(self):
        ok, output = in_container(
            'timeout 4 rostopic echo -n 1 '
            '/franka_state_controller/franka_states/robot_mode 2>/dev/null',
            timeout=6)
        return parse_franka_robot_mode(output) if ok else None

    def _launch_stage(self, tag, command):
        _, active = in_container(build_stage_probe_command(tag), timeout=5)
        if active.strip() == 'running':
            self._select_stage_log(
                tag, '{} is already being started or is running.'.format(tag))
            messagebox.showwarning(
                'Stage already running',
                'The {} stage already has a launcher-owned process.\n\n'
                'Press Stop all before starting another copy.'.format(tag))
            return False
        self._select_stage_log(
            tag, 'Starting {}... output will appear here.'.format(tag))
        started, error = in_container_detached(tag, command)
        if not started:
            detail = error or 'docker exec failed'
            self._replace_log('Could not start {}:\n{}'.format(tag, detail))
            messagebox.showerror(
                'Start failed',
                'Could not start the {} stage.\n\n{}'.format(tag, detail))
        return started

    def refresh_serial_ports(self):
        if self.input_src.get() != 'magnetometer':
            return
        if self._stopping:
            if not self._closing:
                self.after(500, self.refresh_serial_ports)
            return
        if not self._ensure_container():
            self._show_interface_notice(
                'Could not start the {} Docker container.'.format(CONTAINER))
            return

        ports = container_serial_ports()
        if ports is None:
            self._detected_serial_ports = []
            self._show_interface_notice(
                'Could not list serial ports inside {}.'.format(CONTAINER))
            return

        self._detected_serial_ports = ports
        self._show_interface_notice(format_serial_port_list(ports))
        if ports:
            self._sensor_port.entry.focus_set()

    def _copy_log_selection(self, _=None):
        try:
            text = self.log.get('sel.first', 'sel.last')
        except tk.TclError:
            return 'break'
        self.clipboard_clear()
        self.clipboard_append(text)
        return 'break'

    def _select_log_all(self, _=None):
        self.log.tag_add('sel', '1.0', 'end-1c')
        return 'break'

    def copy_log(self):
        text = self.log.get('1.0', 'end-1c')
        self.clipboard_clear()
        self.clipboard_append(text)

    def _show_log_menu(self, event):
        try:
            self._log_menu.tk_popup(event.x_root, event.y_root)
        finally:
            self._log_menu.grab_release()

    def _ensure_container(self):
        ok, _ = sh('docker ps --format "{{.Names}}" | grep -qx %s' % CONTAINER)
        if not ok:
            started, _ = sh('docker start %s' % CONTAINER, timeout=30)
            if not started:
                messagebox.showerror('Docker', 'Container "%s" not found.\n'
                                     'Run: bash ros/docker_setup.sh' % CONTAINER)
                return False
        mounted, error = probe_container_project_mount(REPO_DIR)
        if not mounted:
            repair = 'cd {} && COLMAG_SKIP_BUILD=1 bash ros/docker_setup.sh'.format(
                shlex.quote(os.path.realpath(REPO_DIR)))
            messagebox.showerror(
                'Docker project mount',
                '{}\n\nThe launcher needs this checkout mounted at /colmag.\n'
                'Recreate the container from the current project:\n\n{}'.format(
                    error, repair))
            return False
        return True

    def start_robot(self):
        if not self._pipeline_action_ready():
            return
        if not self._ensure_container():
            return
        if self.mode.get() == 'sim':
            # Never start a SECOND fr3.launch while a sim is up: its controller
            # spawner fights the first one and leaves the arm controller STOPPED
            # (arm ignores all motion). If a sim is already running, just
            # (re)open the Gazebo window. This is a direct process check, not a
            # ROS-node sniff, so a stale /gazebo registration cannot confuse it.
            _, sim_up = in_container(
                'pgrep -x gzserver >/dev/null && echo up || true')
            if 'up' in sim_up:
                _, window = in_container(
                    'pgrep -x gzclient >/dev/null && echo open || true')
                if 'open' in window:
                    self._select_stage_log(
                        'robot', 'Simulation and Gazebo are already running.')
                else:
                    self._select_stage_log(
                        'robot',
                        'Simulation is already running; opening Gazebo.')
                    _, active = in_container(
                        build_stage_probe_command('window'), timeout=5)
                    if active.strip() != 'running':
                        in_container_detached('window', 'gzclient')
                return
            missing = missing_container_ros_packages(
                SIMULATION_ROS_PACKAGES)
            if missing is None:
                messagebox.showerror(
                    'Simulation check failed',
                    'Could not inspect ROS packages inside {}.\n\n'
                    'Check that the container is running, then try again.'
                    .format(CONTAINER))
                return
            if missing:
                package_list = ', '.join(missing)
                notice = (
                    'Simulation is unavailable in this Docker image.\n\n'
                    'Missing ROS packages: {}\n\n'
                    'Rebuild the simulation-capable image once from the '
                    'repository root:\n'
                    'INSTALL_GAZEBO=1 bash ros/docker_setup.sh\n\n'
                    'Do not use COLMAG_SKIP_BUILD=1 for this rebuild.'
                ).format(package_list)
                self._select_stage_log('robot', notice)
                messagebox.showerror('Simulation dependencies missing', notice)
                return
            cmd = ('roslaunch colmag_ros fr3.launch '
                   'controller:=effort_joint_trajectory_controller')
        else:
            ip = self.robot_ip.get().strip()
            if not ip:
                messagebox.showerror('Real robot', 'Enter the robot IP first.')
                return
            stack_ok, _ = in_container('rospack find franka_control')
            if not stack_ok:
                messagebox.showerror(
                    'Real robot',
                    'This Docker image does not contain franka_control, so it '
                    'can run the interface but cannot drive the FR3.\n\n'
                    'Rebuild the full robot image with:\n'
                    'INSTALL_GAZEBO=1 bash ros/docker_setup.sh')
                return
            if not messagebox.askokcancel(
                    'Real robot',
                    'Connect to the REAL FR3 at %s?\n\nWorkspace clear, '
                    'E-stop reachable, supervisor present?' % ip):
                return
            cmd = ('roslaunch colmag_ros fr3_real.launch robot_ip:=%s '
                   'load_gripper:=true' % ip)
        self._launch_stage('robot', cmd)

    def recover_robot(self):
        if not self._pipeline_action_ready():
            return
        if not self._ensure_container():
            return
        backend = detect_robot_backend(self._running_ros_nodes())
        if self.mode.get() != 'real' or backend != 'real':
            messagebox.showinfo(
                'FR3 recovery',
                'Recovery is available when the Control Center is in Real '
                'robot mode and franka_control is running.')
            return
        if self._recovering:
            return
        mode = self._franka_robot_mode()
        mode_name = FRANKA_ROBOT_MODES.get(mode, 'unknown')
        if not messagebox.askokcancel(
                'Recover real FR3',
                'Current FR3 mode: {}\n\nRelease the E-stop/activation device, '
                'unlock the joints in Franka Desk, and confirm FCI is active.\n\n'
                'Send the explicit error-recovery request?'.format(mode_name)):
            return
        self._recovering = True
        self._pipeline_notice = 'Requesting FR3 error recovery...'
        self._replace_log(self._pipeline_notice)
        threading.Thread(target=self._recover_robot_worker, daemon=True).start()

    def _recover_robot_worker(self):
        ok, detail = in_container(
            'rosrun colmag_ros fr3_recover.py', timeout=20)
        mode = self._franka_robot_mode()
        try:
            self.after(0, self._finish_robot_recovery, ok, detail, mode)
        except tk.TclError:
            pass

    def _finish_robot_recovery(self, ok, detail, mode):
        self._recovering = False
        mode_name = FRANKA_ROBOT_MODES.get(mode, 'unknown')
        recovered = ok and mode in (None, 1, 2)
        if recovered:
            notice = 'FR3 recovery succeeded.'
            if mode is not None:
                notice += ' Robot mode: {}.'.format(mode_name)
        else:
            notice = (
                'FR3 recovery did not reach Idle/Move mode (mode: {}).\n\n{}'
                .format(mode_name, detail or 'No recovery response.'))
        self._pipeline_notice = notice
        self._replace_log(notice)
        if not recovered:
            messagebox.showerror('FR3 recovery', notice)
        self.after(5000, self._clear_pipeline_notice, notice)

    def start_nodes(self):
        if not self._pipeline_action_ready():
            return
        if not self._ensure_container():
            return
        live = self.live.get()
        # Moving the REAL arm is the one irreversible action here, so it keeps
        # its explicit confirmation. Everything else just starts.
        if live and self.mode.get() == 'real':
            if not messagebox.askokcancel(
                    'Real robot — LIVE',
                    'Arm nodes will MOVE THE REAL ARM (dry_run:=false).\n'
                    'Continue?'):
                return
        cmd = ('roslaunch colmag_ros colmag_arm_nodes.launch '
               'dry_run:=%s arm_id:=fr3' % ('false' if live else 'true'))
        if self.mode.get() == 'real':
            # fr3_real.launch spawns the position controller (franka_ros
            # default for real hardware); the nodes must target the same one,
            # not the Gazebo effort controller they default to.
            cmd += ' arm_controller:=position_joint_trajectory_controller'
            # Do NOT auto-home the real arm the instant the node starts: that
            # lurch to HOME (and the reflex it can trip on the FR3) is exactly
            # the "it locks the end-effector when I start the node" surprise.
            # The arm stays put until you command a gesture or teleop; send a
            # "0"/"X" gesture to home deliberately.
            cmd += ' home_on_start:=false'
        self._launch_stage('nodes', cmd)

    def start_interface(self):
        if not self._pipeline_action_ready():
            return
        if not self._ensure_container():
            return
        busy = self._sensor_in_use()
        if busy == 'interface':
            self._select_stage_log(
                'interface', 'The MagPilot interface is already running.')
            messagebox.showwarning(
                'Interface already running',
                'The interface process is already active. Press Stop all '
                'before starting another copy.')
            return
        if busy:
            self._warn_sensor_busy(busy)
            return
        input_source = self.input_src.get()
        port = ''
        if input_source == 'magnetometer':
            port = self._checked_sensor_port('Magnetometer')
            if not port:
                return
        cmd = build_interface_command(input_source, port)
        started = self._launch_stage('interface', cmd)
        if started:
            self._interface_notice = None

    def _checked_sensor_port(self, title):
        """Return the Docker path of the port typed in 'port #', or None."""
        if self.input_src.get() != 'magnetometer':
            # Switching the Interface card to magnetometer lists the ports in
            # the log, so the user can type the right number.
            self.input_src.set('magnetometer')
            messagebox.showinfo(
                title,
                'The Interface card is now set to magnetometer and the log of '
                'the main window lists the serial ports.\n\nType the sensor '
                'board\'s number into "port #" there, then press Start again. '
                '(Switch the card back to trackpad for trackpad demos.)')
            return None
        port = self.sensor_port.get().strip()
        if not port:
            messagebox.showerror(
                title,
                'Enter one of the port numbers shown in the Interface log.')
            return None
        try:
            port = resolve_serial_port(port, self._detected_serial_ports)
        except ValueError as exc:
            messagebox.showerror(title, str(exc))
            return None
        ok, error = probe_container_serial_port(port)
        if not ok:
            messagebox.showerror(
                title,
                'Docker cannot open serial port "{}".\n\nReconnect the '
                'sensor and recreate the container once with:\n'
                'COLMAG_SKIP_BUILD=1 bash ros/docker_setup.sh\n\n{}'.format(
                    serial_port_label(port), error))
            return None
        return port

    def _sensor_in_use(self):
        """Report each process that can own the board's serial stream."""
        _, processes = in_container(
            "pgrep -af '[m]agnetometer_reader.py|[r]ecord_tracking_error.py|[t]eleoperation_serial_stream.py' "
            "|| true", timeout=5)
        if 'teleoperation_serial_stream' in processes:
            return 'teleoperation'
        if 'record_tracking_error' in processes:
            return 'tracking'
        if 'magnetometer_reader' in processes:
            return 'interface'
        return None

    def _warn_sensor_busy(self, busy):
        if busy == 'teleoperation':
            text = ('The Teleoperation Pipeline is using the sensor board. '
                    'Close its simulation window or press Stop in Data first.')
        elif busy == 'tracking':
            text = ('The tracking-error recorder is still open in its '
                    'terminal. Type q and Enter there, or press Stop all.')
        else:
            text = ('A character recording or the Interface '
                    '(magnetometer_reader.py) is running. Close its window '
                    'or press Stop all first.')
        messagebox.showwarning('Sensor board busy', text)

    def start_teleoperation(self, participant_id, input_source='trackpad', participant_name='',
                            experiment_name='', seed=0, trials=10, tolerance_mm=25,
                            dwell_seconds=2, magnet_count=None):
        """Data window: launch an isolated host MuJoCo collection process."""
        if not self._pipeline_action_ready():
            return False
        process = self.__dict__.get('_teleoperation_process')
        if process is not None and process.poll() is None:
            messagebox.showwarning('Teleoperation Pipeline', 'The simulation is already running. Close its window or press Stop first.')
            return False
        port = ''
        if input_source == 'serial':
            if not self._ensure_container():
                return False
            busy = self._sensor_in_use()
            if busy:
                self._warn_sensor_busy(busy)
                return False
            port = self._checked_sensor_port('Teleoperation Pipeline')
            if not port:
                return False
        python = teleoperation_python()
        ready, detail = probe_teleoperation_runtime(python)
        if not ready:
            messagebox.showerror('Teleoperation environment',
                '{}\n\nCreate the host simulation environment:\n'
                'python3 tools/setup_teleoperation.py\n\n'
                'Alternatively set COLMAG_TELEOP_PYTHON to an existing compatible Python.'.format(detail))
            return False
        try:
            session_id = next_teleoperation_session(DATA_DIR, participant_id)
            args = build_teleoperation_argv(python, participant_id, session_id, input_source,
                    serial_port=port, participant_name=participant_name, experiment_name=experiment_name,
                    seed=seed, trials=trials, tolerance_mm=tolerance_mm,
                    dwell_seconds=dwell_seconds, magnet_count=magnet_count)
            folder = os.path.join(DATA_DIR, 'teleoperation', participant_id)
            os.makedirs(folder, exist_ok=True)
            log_path = os.path.join(folder, '{}_launcher.log'.format(session_id))
            log_file = open(log_path, 'a', encoding='utf-8')
            try:
                process = subprocess.Popen(args, cwd=REPO_DIR, stdout=log_file,
                    stderr=subprocess.STDOUT, start_new_session=True)
            except BaseException:
                log_file.close()
                raise
        except (OSError, ValueError) as exc:
            messagebox.showerror('Teleoperation Pipeline', str(exc))
            return False
        previous_log = self.__dict__.get('_teleoperation_log_file')
        if previous_log is not None:
            previous_log.close()
        self._teleoperation_process = process
        self._teleoperation_stop_requested = False
        self._teleoperation_finalized_process = None
        self._teleoperation_log_path = log_path
        self._teleoperation_log_file = log_file
        self._pipeline_notice = 'Teleoperation {} / {} opened in MuJoCo. Enter starts each trial.'.format(participant_id, session_id)
        self._replace_log(self._pipeline_notice)
        if 'log_choice' in self.__dict__:
            self.log_choice.set('interface')
            self._log_selector._redraw()
        self.after(3500, self._clear_pipeline_notice, self._pipeline_notice)
        self.after(1500, self._watch_teleoperation, process)
        return True

    def _watch_teleoperation(self, process):
        if process is not self.__dict__.get('_teleoperation_process'):
            return
        if process is self.__dict__.get('_teleoperation_finalized_process'):
            return
        if process.poll() is None:
            if not self._closing:
                self.after(1500, self._watch_teleoperation, process)
            return
        self._teleoperation_finalized_process = process
        log_file = self.__dict__.get('_teleoperation_log_file')
        if log_file is not None:
            log_file.close()
            self._teleoperation_log_file = None
        if (process.returncode and not self._closing and not self._stopping
                and not self.__dict__.get('_teleoperation_stop_requested', False)):
            path = self.__dict__.get('_teleoperation_log_path')
            try:
                with open(path, encoding='utf-8') as stream:
                    detail = stream.read()[-6000:]
            except (OSError, TypeError):
                detail = 'Read the session launcher log for details.'
            messagebox.showerror('Teleoperation Pipeline stopped', detail)

    def _stop_teleoperation_worker(self):
        self._teleoperation_stop_requested = True
        process = self.__dict__.get('_teleoperation_process')
        log_file = self.__dict__.get('_teleoperation_log_file')
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=4)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2)
                except (OSError, subprocess.TimeoutExpired):
                    pass
            except OSError:
                pass
        if log_file is not None:
            log_file.close()
            if log_file is self.__dict__.get('_teleoperation_log_file'):
                self._teleoperation_log_file = None

    def stop_teleoperation(self):
        threading.Thread(target=self._stop_teleoperation_worker, daemon=True).start()

    def start_character_recording(self, participant_id, height_mm,
                                  experiment_name='', demo=False):
        """Data window: record digits and letters for one participant."""
        if not self._pipeline_action_ready() or not self._ensure_container():
            return
        port = ''
        if not demo:
            busy = self._sensor_in_use()
            if busy:
                self._warn_sensor_busy(busy)
                return
            port = self._checked_sensor_port('Data collection')
            if not port:
                return
        collection_dir = dataset.collection_data_dir(DATA_DIR, demo=demo)
        session_id = dataset.next_session_id(collection_dir, participant_id)
        try:
            participants = dataset.load_participants(DATA_DIR)
        except ValueError as exc:
            messagebox.showerror('Data collection', str(exc))
            return
        participant = next((p for p in participants
                            if p['participant_id'] == participant_id),
                           {'participant_id': participant_id})
        where = ('with mouse input' if demo else
                 'on the cover (0 mm)' if height_mm == 0 else
                 'at {0:g} mm. Fit the {0:g} mm spacer first'.format(height_mm))
        experiment = ('Experiment: {}\n\n'.format(experiment_name.strip())
                      if experiment_name.strip() else '')
        if not messagebox.askokcancel(
                'DEMO character recording' if demo else 'Record characters',
                '{}{}Record {} session {} {}?'.format(
                    'DEMO — no sensor board or robot required.\n\n' if demo else '',
                    experiment, dataset.participant_label(participant),
                    session_id, where)):
            return
        # Creating the folder now reserves the session number, even if the
        # recorder stops before its first take.
        try:
            os.makedirs(os.path.join(
                REPO_DIR,
                dataset.session_output_dir(participant_id, session_id, demo=demo)),
                exist_ok=True)
        except OSError as exc:
            messagebox.showerror('Data collection', str(exc))
            return
        cmd = build_record_command(port, participant_id, session_id, height_mm,
                                   experiment_name, demo=demo)
        if self._launch_stage('interface', cmd):
            self._interface_notice = None

    def start_tracking_error(self, magnet, magnet_offset_mm, heights_mm):
        """Data window: run the tracking-error recorder in a terminal."""
        try:
            magnet, offset, heights = parse_tracking_settings(
                magnet, magnet_offset_mm, heights_mm)
        except ValueError as exc:
            messagebox.showerror('Tracking error', str(exc))
            return
        # docker exec takes a moment before the recorder shows up in pgrep,
        # so ignore a second click right after a start.
        if time.monotonic() - getattr(self, '_tracking_started_at', -60) < 10:
            return
        if not self._pipeline_action_ready() or not self._ensure_container():
            return
        busy = self._sensor_in_use()
        if busy:
            self._warn_sensor_busy(busy)
            return
        port = self._checked_sensor_port('Tracking error')
        if not port:
            return
        run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        cmd = build_tracking_command(port, magnet, offset, heights, run_id)
        terminal = find_terminal()
        if terminal is None:
            messagebox.showerror(
                'No terminal found',
                'Run this in a terminal instead:\n\ndocker exec -it {} bash '
                '-lc {}'.format(CONTAINER, shlex.quote(cmd)))
            return
        try:
            os.makedirs(os.path.join(
                REPO_DIR, dataset.tracking_output_dir(run_id)), exist_ok=True)
            subprocess.Popen(build_terminal_argv(terminal, cmd),
                             start_new_session=True)
        except OSError as exc:
            messagebox.showerror('Tracking error', str(exc))
            return
        self._tracking_started_at = time.monotonic()
        self._pipeline_notice = (
            'Tracking-error run {} opened in a new {} window. Follow the '
            'prompts there; Stop all also ends it.'.format(run_id, terminal))
        self._replace_log(self._pipeline_notice)
        self.after(8000, self._clear_pipeline_notice, self._pipeline_notice)

    def stop_all(self):
        self._begin_pipeline_cleanup('manual')

    def _cleanup_on_startup(self):
        self._begin_pipeline_cleanup('startup')

    def _begin_pipeline_cleanup(self, reason):
        if self._stopping:
            if reason == 'close':
                self._close_after_stop = True
                self._pipeline_notice = (
                    'Closing Control Center after pipeline cleanup...')
            return
        self._stopping = True
        if reason == 'close':
            self._close_after_stop = True
        self._interface_notice = None
        self._pipeline_notice = {
            'startup': (
                'Startup safety check: clearing stale MagPilot processes...'),
            'manual': (
                'Stopping interface, arm nodes, controllers, and robot backend...'),
            'close': (
                'Stopping the MagPilot pipeline before closing...'),
        }[reason]
        self._replace_log(self._pipeline_notice)
        threading.Thread(
            target=self._stop_all_worker, args=(reason,), daemon=True).start()

    def _stop_all_worker(self, reason):
        self._stop_teleoperation_worker()
        # Simple, forceful, best-effort cleanup that NEVER blocks the UI.
        # 1) Stop the legacy host-network container: its ROS nodes register on
        #    the shared localhost:11311 master but cannot be pkilled from
        #    colmag_simon, so they would otherwise linger as phantom nodes.
        stop_conflicting_colmag_containers()
        # 2) Kill every pipeline process inside colmag_simon. build_stop_all_
        #    command TERMs then KILLs roslaunch, rosmaster/roscore, gazebo,
        #    franka_control, the colmag nodes and the interface — so the ROS
        #    master itself dies and no stale registration (e.g. a phantom
        #    /gazebo that made "real" think a simulation was up) can survive.
        running, _ = sh(
            'docker ps --format "{{.Names}}" | grep -qx %s' % CONTAINER,
            timeout=5)
        if running:
            in_container(build_stop_all_command(), timeout=25)
            if reason == 'startup':
                in_container(
                    build_clear_stale_launcher_state_command(), timeout=5)
        try:
            self.after(0, self._finish_stop_all, reason)
        except tk.TclError:
            pass

    def _finish_stop_all(self, reason):
        self._stopping = False
        if self._close_after_stop:
            self.destroy()
            return
        notice = (
            'Ready — nothing is running. Start stages manually.'
            if reason == 'startup'
            else 'All MagPilot pipeline processes stopped.')
        self._pipeline_notice = notice
        self._replace_log(notice)
        self.after(3500, self._clear_pipeline_notice, notice)

    def _clear_pipeline_notice(self, notice):
        if self._pipeline_notice == notice:
            self._pipeline_notice = None

    def restart_container(self):
        if self._stopping:
            messagebox.showinfo(
                'Pipeline cleanup',
                'Please wait for the current pipeline cleanup to finish.')
            return
        if messagebox.askokcancel('Restart', 'Restart the container? '
                                  'All pipeline processes stop.'):
            ok, detail = sh('docker restart %s' % CONTAINER, timeout=60)
            if ok:
                self._begin_pipeline_cleanup('manual')
            else:
                messagebox.showerror(
                    'Restart failed', detail or 'Docker restart failed.')

    # ── Status polling ───────────────────────────────────────────────────────

    def _poll_loop(self):
        import time as _t
        while self._poll_running:
            self._poll_once()
            _t.sleep(2.0)

    def _poll_once(self):
        ok, _ = sh('docker ps --format "{{.Names}}" | grep -qx %s' % CONTAINER,
                   timeout=5)
        states = {'robot': False, 'nodes': False, 'interface': False}
        tail = '(container not running)'
        self._virtual_task_running = False
        if ok:
            # Status LEDs describe this container, not every process visible
            # through the host-network ROS master. Otherwise an external or
            # stale /franka_control registration can light Robot at startup.
            local_ok, local_status = in_container(
                build_local_stage_status_command(), timeout=5)
            if local_ok:
                states['robot'], states['nodes'] = parse_local_stage_status(
                    local_status)
            # [m] trick: don't match this pgrep's own bash wrapper
            _, procs = in_container("pgrep -af '[m]agnetometer_reader.py|[t]arget_reaching_pilot.py' || true",
                                    timeout=5)
            states['interface'] = 'magnetometer_reader.py' in procs
            self._virtual_task_running = 'target_reaching_pilot.py' in procs
            _, tail = in_container(
                'tail -n 60 /tmp/colmag_gui_%s.log 2>/dev/null || true'
                % self.log_choice.get(), timeout=5)
        process = self.__dict__.get('_teleoperation_process')
        log_path = self.__dict__.get('_teleoperation_log_path')
        if process is not None and log_path and self.log_choice.get() == 'interface':
            try:
                with open(log_path, encoding='utf-8') as stream:
                    tail = stream.read()[-6000:] or 'MuJoCo Teleoperation Pipeline is open.'
            except OSError:
                pass
        if self._pipeline_notice is not None:
            tail = self._pipeline_notice
        elif (self.log_choice.get() == 'interface'
              and self._interface_notice is not None):
            tail = self._interface_notice
        try:
            self.after(0, self._apply_status, ok, states, tail)
        except tk.TclError:
            pass  # window closed mid-poll

    def _apply_status(self, container_ok, states, tail):
        self.container_light.configure(fg=GREEN if container_ok else RED)
        for tag, light in self.lights.items():
            value = states.get(tag)
            light.configure(fg=GREEN if value is True
                            else (AMBER if value == 'nowin'
                                  else (RED if value == 'conflict' else DOT_OFF)))
        self._replace_log(tail or '(no log yet)')

    def _install_signal_handlers(self):
        for name in ('SIGINT', 'SIGTERM', 'SIGHUP'):
            sig = getattr(signal, name, None)
            if sig is not None:
                try:
                    signal.signal(sig, self._on_signal)
                except ValueError:
                    pass  # Tk launcher was embedded outside the main thread.

    def _on_signal(self, _signum, _frame):
        try:
            self.after(0, self._on_close)
        except tk.TclError:
            pass

    def _on_close(self):
        if self._closing:
            return
        self._closing = True
        self._poll_running = False
        self._begin_pipeline_cleanup('close')


def _ensure_good_tk():
    """Conda/miniconda Tk often lacks fontconfig and only sees ~20 X11 bitmap
    fonts, which makes the UI look ancient. If we detect that and the system
    python has a proper Tk, re-exec ourselves with it."""
    if os.environ.get('COLMAG_LAUNCHER_REEXEC'):
        return
    try:
        root = tk.Tk()
        root.withdraw()
        n = len(tkfont.families())
        root.destroy()
    except Exception:
        return
    if n >= 50:
        return
    sys_py = '/usr/bin/python3'
    if not os.path.exists(sys_py):
        return
    probe = subprocess.run([sys_py, '-c', 'import tkinter'],
                           capture_output=True)
    if probe.returncode == 0:
        os.environ['COLMAG_LAUNCHER_REEXEC'] = '1'
        os.execv(sys_py, [sys_py, os.path.abspath(__file__)])


def main():
    _ensure_good_tk()
    Launcher().mainloop()


if __name__ == '__main__':
    main()
