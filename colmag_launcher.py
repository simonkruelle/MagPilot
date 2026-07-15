#!/usr/bin/env python3
"""
colmag_launcher.py — one-window control center for the COLMAG pipeline.

Run on the HOST (not inside Docker):

    python3 colmag_launcher.py

Buttons start each pipeline stage inside the `colmag_ros` Docker container via
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
import subprocess
import threading
import tkinter as tk
import tkinter.font as tkfont
from tkinter import messagebox

CONTAINER = 'colmag_ros'
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


def in_container_detached(tag, command):
    # PYTHONUNBUFFERED: python ROS nodes block-buffer stdout when redirected
    # to a file, which left the log pane empty until the process exited.
    inner = 'export PYTHONUNBUFFERED=1; {}{} > /tmp/colmag_gui_{}.log 2>&1'.format(
        ROS_SETUP, command, tag)
    return sh('docker exec -d {} bash -lc {}'.format(CONTAINER, shlex.quote(inner)))


def in_container(command, timeout=8):
    return sh('docker exec {} bash -lc {}'.format(
        CONTAINER, shlex.quote(ROS_SETUP + command)), timeout=timeout)


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
        self.robot_ip = tk.StringVar(value='172.16.0.2')
        tk.Label(mode_row, text='robot IP', bg=BG, fg=SUBTLE,
                 font=self.f_body).pack(side='left', padx=(22, 8))
        self._ip = RoundEntry(mode_row, self.robot_ip, width=132, height=30,
                              font=self.f_body, parent_bg=BG)
        self._ip.pack(side='left')
        self._ip.entry.configure(state='disabled')

        # Stage cards
        self.lights = {}
        self._stage('robot', '1 · Robot',
                    'Gazebo FR3 + controllers (sim) · franka_control (real)',
                    self.start_robot)
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
        Selector(top, self.log_choice, STAGES, width=118, height=26,
                 font=self.f_small).pack(side='right')
        self.log = tk.Text(log_card.inner, height=9, bg='#fbfbfd', fg=TEXT,
                           font=self.f_mono, relief='flat', state='disabled',
                           wrap='none')
        self.log.pack(fill='both', expand=True, pady=(5, 0))

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
        self._ip.entry.configure(state='normal' if real else 'disabled')
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
            # Never start a SECOND fr3.launch while the sim is up: its
            # controller spawner fights the first one and leaves the arm
            # controller STOPPED (arm ignores all motion). If only the
            # Gazebo window was closed, just reopen the window.
            ok, out = in_container(
                'pgrep -x gzserver >/dev/null && echo up || echo down')
            if ok and 'up' in out:
                in_container_detached('window', 'gzclient')
                return
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
        if self.mode.get() == 'real':
            # fr3_real.launch spawns the position controller (franka_ros
            # default for real hardware); the nodes must target the same one,
            # not the Gazebo effort controller they default to.
            cmd += ' arm_controller:=position_joint_trajectory_controller'
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
