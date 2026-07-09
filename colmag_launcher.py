#!/usr/bin/env python3
"""
colmag_launcher.py — one-window control center for the COLMAG pipeline.

Run on the HOST (not inside Docker):

    python3 colmag_launcher.py

Buttons start each pipeline stage inside the `colmag_ros` Docker container via
`docker exec`, so you never need more than this window plus the GUIs that the
stages open themselves (Gazebo, the trackpad interface). Simulation and real
robot mode share the same three steps:

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
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox, ttk

CONTAINER = 'colmag_ros'
REPO_DIR = os.path.dirname(os.path.abspath(__file__))
ROS_SETUP = ('source /opt/ros/noetic/setup.bash; '
             '[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash; ')

# ── Apple-ish palette ────────────────────────────────────────────────────────
BG = '#f5f5f7'
CARD = '#ffffff'
TEXT = '#1d1d1f'
SUBTLE = '#86868b'
BLUE = '#0a84ff'
BLUE_DARK = '#0060df'
GREEN = '#34c759'
RED = '#ff3b30'
BORDER = '#e3e3e8'
DOT_OFF = '#d2d2d7'

STAGES = ('robot', 'nodes', 'interface')
WIDTH = 760


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
    """Run a host command, return (ok, output)."""
    try:
        out = subprocess.run(cmd, shell=True, capture_output=True, text=True,
                             timeout=timeout)
        return out.returncode == 0, (out.stdout + out.stderr).strip()
    except subprocess.TimeoutExpired:
        return False, '(timeout)'


def in_container_detached(tag, command):
    """Start a long-running command inside the container, detached, logged."""
    inner = '{}{} > /tmp/colmag_gui_{}.log 2>&1'.format(ROS_SETUP, command, tag)
    return sh('docker exec -d {} bash -lc {}'.format(CONTAINER, shlex.quote(inner)))


def in_container(command, timeout=8):
    return sh('docker exec {} bash -lc {}'.format(
        CONTAINER, shlex.quote(ROS_SETUP + command)), timeout=timeout)


def round_rect(canvas, x1, y1, x2, y2, r, **kw):
    pts = [x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r, x2, y2 - r, x2, y2,
           x2 - r, y2, x1 + r, y2, x1, y2, x1, y2 - r, x1, y1 + r, x1, y1]
    return canvas.create_polygon(pts, smooth=True, **kw)


class Pill(tk.Canvas):
    """A rounded, macOS-style button."""

    def __init__(self, parent, text, command, kind='primary',
                 width=96, height=32, font=None, parent_bg=CARD):
        super().__init__(parent, width=width, height=height, bg=parent_bg,
                         highlightthickness=0, bd=0, cursor='hand2')
        self._command = command
        self._kind = kind
        fills = {'primary': (BLUE, 'white', BLUE),
                 'danger': ('#ffffff', RED, '#f0d3d1'),
                 'plain': ('#ffffff', TEXT, BORDER)}
        self._fill, fg, outline = fills[kind]
        self._shape = round_rect(self, 1, 1, width - 1, height - 1,
                                 height // 2 - 1, fill=self._fill,
                                 outline=outline)
        self._label = self.create_text(width // 2, height // 2, text=text,
                                       fill=fg, font=font)
        self.bind('<Button-1>', lambda e: self._command())
        self.bind('<Enter>', self._hover_on)
        self.bind('<Leave>', self._hover_off)

    def _hover_on(self, _):
        hover = {'primary': BLUE_DARK, 'danger': '#fff0ef',
                 'plain': '#f5f5f7'}[self._kind]
        self.itemconfigure(self._shape, fill=hover)

    def _hover_off(self, _):
        self.itemconfigure(self._shape, fill=self._fill)


class Card(tk.Canvas):
    """Rounded white card; children go into .inner."""

    def __init__(self, parent, height, width=WIDTH):
        super().__init__(parent, width=width, height=height, bg=BG,
                         highlightthickness=0, bd=0)
        round_rect(self, 1, 1, width - 1, height - 1, 12,
                   fill=CARD, outline=BORDER)
        self.inner = tk.Frame(self, bg=CARD)
        self.create_window(width // 2, height // 2, window=self.inner,
                           width=width - 24, height=height - 16)


class Launcher(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('COLMAG Control Center')
        self.configure(bg=BG)
        self.resizable(False, False)
        self._fonts()
        self._build_ui()
        self._poll_running = True
        threading.Thread(target=self._poll_loop, daemon=True).start()
        self.protocol('WM_DELETE_WINDOW', self._on_close)

    def _fonts(self):
        ui = pick_font(['SF Pro Text', 'SF Pro Display', 'Helvetica Neue',
                        'Fira Sans', 'Inter', 'Roboto', 'Ubuntu', 'DejaVu Sans'])
        mono = pick_font(['SF Mono', 'Menlo', 'Fira Mono', 'JetBrains Mono',
                          'Ubuntu Mono', 'DejaVu Sans Mono'])
        self.f_title = (ui, 19, 'bold')
        self.f_h = (ui, 12, 'bold')
        self.f_body = (ui, 11)
        self.f_small = (ui, 9)
        self.f_btn = (ui, 11, 'bold')
        self.f_mono = (mono, 9)
        s = ttk.Style(self)
        try:
            s.theme_use('clam')
        except tk.TclError:
            pass
        s.configure('.', background=BG, foreground=TEXT, font=self.f_body)
        s.configure('TRadiobutton', background=BG, font=self.f_body)
        s.map('TRadiobutton', background=[('active', BG)])
        s.configure('Card.TCheckbutton', background=CARD, font=self.f_body)
        s.map('Card.TCheckbutton', background=[('active', CARD)])
        s.configure('TEntry', padding=4)
        s.configure('TCombobox', padding=3)

    # ── UI ──────────────────────────────────────────────────────────────────

    def _build_ui(self):
        head = tk.Frame(self, bg=BG)
        head.pack(fill='x', padx=22, pady=(16, 0))
        tk.Label(head, text='COLMAG Control Center', bg=BG, fg=TEXT,
                 font=self.f_title).pack(anchor='w')
        tk.Label(head, text='Runs the whole pipeline inside the colmag_ros '
                            'container — no extra terminals.',
                 bg=BG, fg=SUBTLE, font=self.f_body).pack(anchor='w', pady=(2, 0))

        # Mode row
        mode_row = tk.Frame(self, bg=BG)
        mode_row.pack(fill='x', padx=22, pady=(12, 6))
        self.mode = tk.StringVar(value='sim')
        ttk.Radiobutton(mode_row, text='Simulation (Gazebo)', value='sim',
                        variable=self.mode, command=self._mode_changed
                        ).pack(side='left')
        ttk.Radiobutton(mode_row, text='Real robot', value='real',
                        variable=self.mode, command=self._mode_changed
                        ).pack(side='left', padx=(18, 10))
        tk.Label(mode_row, text='robot IP', bg=BG, fg=SUBTLE,
                 font=self.f_body).pack(side='left')
        self.robot_ip = tk.StringVar(value='172.16.0.2')
        self.ip_entry = ttk.Entry(mode_row, textvariable=self.robot_ip,
                                  width=13, font=self.f_body)
        self.ip_entry.pack(side='left', padx=(6, 0))
        self.ip_entry.configure(state='disabled')

        # Stage cards
        self.lights = {}
        self._stage('robot', '1 · Robot',
                    'Gazebo FR3 + controllers (sim) · franka_control (real)',
                    self.start_robot)
        inner = self._stage('nodes', '2 · Arm nodes',
                            'Teleop (draw) + gestures (robot), one launch',
                            self.start_nodes)
        self.live = tk.BooleanVar(value=True)
        ttk.Checkbutton(inner, text='live (moves the arm)', variable=self.live,
                        style='Card.TCheckbutton').pack(side='right', padx=(0, 12))
        inner = self._stage('interface', '3 · Interface',
                            'Writing / teleop UI (opens its own window)',
                            self.start_interface)
        self.input_src = tk.StringVar(value='trackpad')
        ttk.Combobox(inner, textvariable=self.input_src, width=12,
                     state='readonly', font=self.f_body,
                     values=('trackpad', 'magnetometer')
                     ).pack(side='right', padx=(0, 12))

        # Control row
        row = tk.Frame(self, bg=BG)
        row.pack(fill='x', padx=22, pady=(12, 4))
        Pill(row, '■  Stop all', self.stop_all, kind='danger', width=118,
             font=self.f_btn, parent_bg=BG).pack(side='left')
        Pill(row, 'Restart container', self.restart_container, kind='plain',
             width=165, font=self.f_body, parent_bg=BG
             ).pack(side='left', padx=12)
        self.container_light = tk.Label(row, text='●  container', bg=BG,
                                        fg=DOT_OFF, font=self.f_body)
        self.container_light.pack(side='right')

        # Log card
        log_card = Card(self, height=190)
        log_card.pack(padx=22, pady=(10, 18))
        top = tk.Frame(log_card.inner, bg=CARD)
        top.pack(fill='x')
        tk.Label(top, text='Log', bg=CARD, fg=SUBTLE,
                 font=self.f_small).pack(side='left')
        self.log_choice = tk.StringVar(value='interface')
        ttk.Combobox(top, textvariable=self.log_choice, width=10,
                     state='readonly', values=STAGES,
                     font=self.f_small).pack(side='right')
        self.log = tk.Text(log_card.inner, height=9, bg='#fbfbfd', fg=TEXT,
                           font=self.f_mono, relief='flat', state='disabled',
                           wrap='none')
        self.log.pack(fill='both', expand=True, pady=(4, 0))

    def _stage(self, tag, title, subtitle, command):
        card = Card(self, height=66)
        card.pack(padx=22, pady=5)
        inner = card.inner
        light = tk.Label(inner, text='●', bg=CARD, fg=DOT_OFF,
                         font=(self.f_body[0], 15))
        light.pack(side='left', padx=(6, 10))
        self.lights[tag] = light
        col = tk.Frame(inner, bg=CARD)
        col.pack(side='left')
        tk.Label(col, text=title, bg=CARD, fg=TEXT, font=self.f_h
                 ).pack(anchor='w')
        tk.Label(col, text=subtitle, bg=CARD, fg=SUBTLE, font=self.f_small
                 ).pack(anchor='w')
        Pill(inner, 'Start', command, kind='primary', width=92,
             font=self.f_btn).pack(side='right', padx=(0, 4))
        return inner

    # ── Actions ─────────────────────────────────────────────────────────────

    def _mode_changed(self):
        real = self.mode.get() == 'real'
        self.ip_entry.configure(state='normal' if real else 'disabled')
        if real:
            messagebox.showwarning(
                'Real robot mode',
                'REAL ROBOT selected.\n\nFollow the staged pipeline in the '
                'README: dry-run first, supervisor present, E-stop reachable.')

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
        if not self._ensure_container():
            return
        if self.mode.get() == 'sim':
            cmd = ('roslaunch colmag_ros fr3.launch '
                   'controller:=effort_joint_trajectory_controller')
        else:
            ip = self.robot_ip.get().strip()
            if not ip:
                messagebox.showerror('Real robot', 'Enter the robot IP first.')
                return
            if not messagebox.askokcancel(
                    'Real robot',
                    'Connect to the REAL FR3 at %s?\n\nWorkspace clear, '
                    'E-stop reachable, supervisor present?' % ip):
                return
            cmd = 'roslaunch colmag_ros fr3_real.launch robot_ip:=%s' % ip
        in_container_detached('robot', cmd)

    def start_nodes(self):
        if not self._ensure_container():
            return
        live = self.live.get()
        if live and self.mode.get() == 'real':
            if not messagebox.askokcancel(
                    'Real robot — LIVE',
                    'Arm nodes will MOVE THE REAL ARM (dry_run:=false).\n'
                    'Continue?'):
                return
        cmd = ('roslaunch colmag_ros colmag_arm_nodes.launch '
               'dry_run:=%s arm_id:=fr3' % ('false' if live else 'true'))
        in_container_detached('nodes', cmd)

    def start_interface(self):
        if not self._ensure_container():
            return
        if self.input_src.get() == 'trackpad':
            cmd = ('cd /colmag && python3 magnetometer_reader.py '
                   '--input-source trackpad --ros '
                   '--classifier-labels ABCXLRUD0123')
        else:
            cmd = ('cd /colmag && python3 magnetometer_reader.py '
                   '--clean --writing-max-z 0.05 --ros '
                   '--classifier-labels ABCXLRUD0123')
        in_container_detached('interface', cmd)

    def stop_all(self):
        in_container('pkill -f roslaunch; pkill -f magnetometer_reader; '
                     'sleep 2; pkill -f gzserver; pkill -f gzclient; '
                     'pkill -f rosmaster; true', timeout=20)

    def restart_container(self):
        if messagebox.askokcancel('Restart', 'Restart the container? '
                                  'All pipeline processes stop.'):
            sh('docker restart %s' % CONTAINER, timeout=60)

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
            ok2, nodes = in_container('rosnode list 2>/dev/null', timeout=5)
            if ok2:
                states['robot'] = '/gazebo' in nodes or '/franka_control' in nodes
                states['nodes'] = ('/colmag_draw_node' in nodes
                                   and '/colmag_robot_node' in nodes)
            # [m] trick: don't match this pgrep's own bash wrapper
            _, procs = in_container("pgrep -f '[m]agnetometer_reader' || true",
                                    timeout=5)
            states['interface'] = procs.strip() != ''
            _, tail = in_container(
                'tail -n 60 /tmp/colmag_gui_%s.log 2>/dev/null || true'
                % self.log_choice.get(), timeout=5)
        try:
            self.after(0, self._apply_status, ok, states, tail)
        except tk.TclError:
            pass  # window closed mid-poll

    def _apply_status(self, container_ok, states, tail):
        self.container_light.configure(fg=GREEN if container_ok else RED)
        for tag, light in self.lights.items():
            light.configure(fg=GREEN if states.get(tag) else DOT_OFF)
        self.log.configure(state='normal')
        self.log.delete('1.0', 'end')
        self.log.insert('end', tail or '(no log yet)')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _on_close(self):
        self._poll_running = False
        self.destroy()


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
