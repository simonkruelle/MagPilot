#!/usr/bin/env python3
"""
colmag_launcher.py — one-window control center for the COLMAG pipeline.

Run on the HOST (not inside Docker):

    python3 colmag_launcher.py

Buttons start each pipeline stage inside the `colmag_simon` Docker container via
`docker exec`, so you never need more than this window plus the GUIs that the
stages open themselves (Gazebo, the trackpad interface).

    1. Robot     — Gazebo FR3 (sim) or franka_control (real, needs robot IP)
    2. Arm nodes — teleop draw node + gesture robot node (one launch)
    3. Interface — trackpad UI or real magnetometer reader

Status lights poll the container every 2 s. STOP ALL kills the pipeline
processes inside the container; "Restart container" is the bulletproof reset.
Logs of each stage are written inside the container to /tmp/colmag_gui_*.log
and tailed in the bottom pane.
"""

import os
import shlex
import signal
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox

CONTAINER = 'colmag_simon'
LEGACY_CONTAINERS = ('colmag_ros',)
REPO_DIR = os.path.dirname(os.path.abspath(__file__))
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
DETACHED_TAGS = STAGES + ('window',)
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
        'interface': ('[m]agnetometer_reader.py',),
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
    stop_order = ('interface', 'nodes', 'window', 'robot')
    commands = [_managed_stage_signal('interface', 'TERM')]
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
        Selector(inner, self.input_src, ('trackpad', 'magnetometer'),
                 width=138, height=30, font=self.f_body
                 ).pack(side='right', padx=(0, 12))

        # Control row
        row = tk.Frame(self, bg=BG)
        row.pack(fill='x', padx=26, pady=(14, 4))
        Pill(row, 'Stop all', self.stop_all, kind='danger', width=104,
             font=self.f_btn, parent_bg=BG).pack(side='left')
        Pill(row, 'Restart container', self.restart_container, kind='plain',
             width=160, font=self.f_body, parent_bg=BG
             ).pack(side='left', padx=12)
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
            top, self.log_choice, STAGES, width=118, height=26,
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

    def _mode_changed(self):
        real = self.mode.get() == 'real'
        if real:
            messagebox.showwarning(
                'Real robot mode',
                'REAL ROBOT selected.\n\nFollow the staged pipeline in the '
                'README: dry-run first, supervisor present, E-stop reachable.')

    def _input_changed(self, *_):
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
        if ok:
            return True
        ok2, _ = sh('docker start %s' % CONTAINER, timeout=30)
        if not ok2:
            messagebox.showerror('Docker', 'Container "%s" not found.\n'
                                 'Run: bash ros/docker_setup.sh' % CONTAINER)
        return ok2

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
            # not the Gazebo effort controller they default to. This is the
            # only sim/real difference the nodes stage needs.
            cmd += ' arm_controller:=position_joint_trajectory_controller'
        self._launch_stage('nodes', cmd)

    def start_interface(self):
        if not self._pipeline_action_ready():
            return
        if not self._ensure_container():
            return
        _, processes = in_container(
            "pgrep -f '[m]agnetometer_reader.py' || true", timeout=5)
        if processes.strip():
            self._select_stage_log(
                'interface', 'The MagPilot interface is already running.')
            messagebox.showwarning(
                'Interface already running',
                'The interface process is already active. Press Stop all '
                'before starting another copy.')
            return
        input_source = self.input_src.get()
        port = self.sensor_port.get().strip()
        if input_source == 'magnetometer':
            if not port:
                messagebox.showerror(
                    'Magnetometer',
                    'Enter one of the port numbers shown in the Interface log.')
                return
            try:
                port = resolve_serial_port(
                    port, self._detected_serial_ports)
            except ValueError as exc:
                messagebox.showerror('Magnetometer', str(exc))
                return
            ok, error = probe_container_serial_port(port)
            if not ok:
                messagebox.showerror(
                    'Magnetometer',
                    'Docker cannot open serial port "{}".\n\nReconnect the '
                    'sensor and recreate the container once with:\n'
                    'COLMAG_SKIP_BUILD=1 bash ros/docker_setup.sh\n\n{}'.format(
                        serial_port_label(port), error))
                return
        cmd = build_interface_command(input_source, port)
        started = self._launch_stage('interface', cmd)
        if started:
            self._interface_notice = None

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
        if ok:
            ok2, nodes = in_container(
                build_live_ros_nodes_command(), timeout=6)
            if ok2:
                if '/franka_control' in nodes:
                    states['robot'] = True
                elif '/gazebo' in nodes:
                    # Sim backend up; amber when the Gazebo window is closed
                    # (Start then reopens just the window, never a 2nd sim).
                    _, win = in_container(
                        'pgrep -x gzclient >/dev/null && echo w || true',
                        timeout=5)
                    states['robot'] = True if 'w' in win else 'nowin'
                states['nodes'] = ('/colmag_draw_node' in nodes
                                   and '/colmag_robot_node' in nodes)
            # [m] trick: don't match this pgrep's own bash wrapper
            _, procs = in_container("pgrep -f '[m]agnetometer_reader' || true",
                                    timeout=5)
            states['interface'] = procs.strip() != ''
            _, tail = in_container(
                'tail -n 60 /tmp/colmag_gui_%s.log 2>/dev/null || true'
                % self.log_choice.get(), timeout=5)
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
                            else (AMBER if value == 'nowin' else DOT_OFF))
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
