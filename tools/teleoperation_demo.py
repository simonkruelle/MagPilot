#!/usr/bin/env python3
"""Operator-started MuJoCo target-reaching collection, without ROS output."""

import argparse
from datetime import datetime
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from colmag.robot_targets import DIGIT_CUBE_CENTER_M
from colmag.teleoperation_control import magnet_position, pointer_position
from colmag.teleoperation_task import (
    TeleoperationRun, TeleoperationSettings, TeleoperationTrial, random_targets,
)


def parser():
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument('--input-source', choices=('trackpad', 'serial'), default='trackpad')
    result.add_argument('--port', default='/dev/ttyACM0')
    result.add_argument('--baudrate', type=int, default=921600)
    result.add_argument('--container', default='colmag_simon')
    result.add_argument('--output-dir', type=Path)
    result.add_argument('--participant-id', default='DEMO')
    result.add_argument('--participant-name', default='')
    result.add_argument('--session-id', default='')
    result.add_argument('--experiment-name', default='Virtual task pilot')
    result.add_argument('--seed', type=int, default=0)
    result.add_argument('--trials', type=int, default=10)
    result.add_argument('--tolerance-mm', type=float, default=25.0)
    result.add_argument('--dwell-seconds', type=float, default=2.0)
    result.add_argument('--timeout-seconds', type=float, default=60.0)
    result.add_argument('--magnet-count', type=int, choices=(1, 2, 3))
    return result


class TeleoperationWindow:
    BG = '#f5f7fb'
    BLUE = '#0a84ff'
    GREEN = '#24ad64'
    TEXT = '#202631'

    def __init__(self, root, args):
        import tkinter as tk
        from tkinter import font as tkfont
        from PIL import Image, ImageTk
        from colmag.mujoco_robot import MuJoCoRobot

        self.tk, self.Image, self.ImageTk = tk, Image, ImageTk
        self.root, self.args = root, args
        families = set(tkfont.families(root))
        self.font_family = next((family for family in ('Fira Sans', 'Arial', 'DejaVu Sans')
                                 if family in families), 'TkDefaultFont')
        self.settings = TeleoperationSettings(
            tolerance_m=args.tolerance_mm / 1000, dwell_s=args.dwell_seconds,
            timeout_s=args.timeout_seconds)
        self.targets = random_targets(args.trials, args.seed, self.settings)
        self.robot = MuJoCoRobot()
        self.source = None
        if args.input_source == 'serial':
            from colmag.teleoperation_input import SerialInput
            self.source = SerialInput(args.port, args.baudrate, args.container)
        session = args.session_id or datetime.now().strftime('S%Y%m%d_%H%M%S_%f')
        if args.output_dir is None and any(
                not value or value in ('.', '..') or '/' in value or '\\' in value
                for value in (args.participant_id, session)):
            self.robot.close()
            if self.source:
                self.source.close()
            raise ValueError('Participant and session IDs must be single folder names.')
        output = args.output_dir or ROOT / 'data_collection' / 'teleoperation' / args.participant_id / session
        try:
            self.run = TeleoperationRun(
                output, args.participant_id, session, self.settings, self.targets,
                experiment_name=args.experiment_name,
                participant_name=args.participant_name, input_source=args.input_source,
                magnet_count=args.magnet_count, seed=args.seed,
                metadata=dict(backend=self.robot.backend_name,
                              position_control='fixed-orientation flange, XYZ only',
                              pointer_mapping='up=-X, right=+Y, scroll=Z; absolute cube mapping',
                              board_input_extent_m=0.05, board_z_bias_m=0.010,
                              board_height_range_m=[0.007, 0.150], board_height_gain=2.0,
                              purpose='pilot; no robot hardware commands'))
        except Exception:
            if self.source:
                self.source.close()
            self.robot.close()
            raise
        self.trial = None
        self.pending_save = None
        self.index = 0
        self.u = self.v = 0.0
        self.height = DIGIT_CUBE_CENTER_M[2]
        self.closed = False
        self.enter_down = False
        self.enter_release_job = None
        self.last_tick = time.monotonic()
        self.last_render = 0.0
        self._photo = None
        self._build()
        self.root.protocol('WM_DELETE_WINDOW', self.close)
        self.root.bind('<KeyPress-Return>', self._enter)
        self.root.bind('<KeyRelease-Return>', self._release_enter)
        self.root.bind('<Escape>', lambda event: self.cancel())
        self.root.after(20, self.tick)
        print('MuJoCo collection ready: {}'.format(self.run.output_dir), flush=True)

    def _label(self, parent, text='', size=11, bold=False, **kwargs):
        return self.tk.Label(parent, text=text, bg=parent.cget('bg'), fg=self.TEXT,
                             font=(self.font_family, size, 'bold' if bold else 'normal'), **kwargs)

    def _build(self):
        tk = self.tk
        self.root.title('MagPilot · Teleoperation collection · MuJoCo')
        self.root.geometry('1220x820')
        self.root.minsize(1000, 740)
        self.root.configure(bg=self.BG)
        top = tk.Frame(self.root, bg=self.BG)
        top.pack(fill='x', padx=24, pady=(18, 10))
        self._label(top, 'Teleoperation Pipeline', 22, True).pack(anchor='w')
        identity = '{} · {} · {}'.format(
            self.args.experiment_name,
            self.args.participant_name or self.args.participant_id,
            'Trackpad' if self.source is None else 'Magnet board')
        self._label(top, identity, 11).pack(anchor='w', pady=(5, 0))
        content = tk.Frame(self.root, bg=self.BG)
        content.pack(fill='both', expand=True, padx=20)
        sidebar = tk.Frame(content, bg='white', width=248)
        sidebar.pack(side='right', fill='y', padx=(12, 0))
        sidebar.pack_propagate(False)
        self._label(sidebar, 'Reach. Hold. Saved.', 17, True).pack(pady=(22, 4))
        self._label(sidebar, 'Stay inside for {:.1f} seconds'.format(self.settings.dwell_s)).pack()
        self._label(sidebar, 'Target radius · {:g} mm'.format(self.args.tolerance_mm)).pack(pady=(4, 8))
        self.ring = tk.Canvas(sidebar, width=210, height=160, bg='white', highlightthickness=0)
        self.ring.pack()
        self.ring.create_oval(47, 15, 163, 131, outline='#e7edf5', width=9)
        self.arc = self.ring.create_arc(47, 15, 163, 131, start=90, extent=0,
                                       style='arc', outline=self.GREEN, width=9)
        self.percent = self.ring.create_text(105, 73, text='0%', fill=self.BLUE,
                                             font=(self.font_family, 24, 'bold'))
        self.progress_text = self._label(sidebar, 'Waiting for Enter', 12, True)
        self.progress_text.pack()
        self.metrics = self._label(sidebar, 'Distance —\nElapsed —', 13, justify='left')
        self.metrics.pack(pady=16)
        self.coordinates = self._label(sidebar, 'Target revealed on Enter', 10, justify='left')
        self.coordinates.pack(pady=(0, 12))
        self._label(sidebar, 'MOVE IN THE ROBOT WORKSPACE', 9, True).pack(pady=(12, 8))
        if self.source is None:
            instructions = 'Move pointer over the scene\n↑ toward base · → right\nScroll ↑ / ↓ to change height\nNo click or held key needed'
        else:
            instructions = 'Move magnet over the board\nBoard X / Y → robot plane\nRaise / lower → robot height\nLift above 15 cm to stop'
        self._label(sidebar, instructions, 10, justify='left').pack(padx=10)
        self._label(sidebar, 'HEIGHT · Z', 9, True).pack(pady=(18, 0))
        self.height_label = self._label(sidebar, '40.0 cm', 16, True)
        self.height_label.pack(pady=5)
        self.feedback = self._label(sidebar, '', 10, justify='center', wraplength=222)
        self.feedback.pack(padx=10, pady=(15, 0))
        self.canvas = tk.Canvas(content, bg='#e9edf3', highlightthickness=0)
        self.canvas.pack(side='left', fill='both', expand=True)
        self.scene_image = self.canvas.create_image(0, 0, anchor='nw')
        # Screen overlays keep the target rim and measured flange visible even
        # when the robot mesh occludes their 3D geometry.
        self.target_rim = self.canvas.create_oval(0, 0, 0, 0, outline=self.BLUE, width=2, state='hidden')
        self.target_arc = self.canvas.create_arc(0, 0, 0, 0, start=90, extent=0,
                                                 style='arc', outline=self.GREEN, width=4, state='hidden')
        self.flange_dot = self.canvas.create_oval(0, 0, 0, 0, fill=self.GREEN,
                                                 outline='white', width=1)
        self.canvas.bind('<Motion>', self._motion)
        self.canvas.bind('<MouseWheel>', self._scroll)
        self.canvas.bind('<Button-4>', lambda event: self._height_change(0.003))
        self.canvas.bind('<Button-5>', lambda event: self._height_change(-0.003))
        footer = tk.Frame(self.root, bg=self.BG)
        footer.pack(fill='x', padx=24, pady=14)
        self.status = self._label(footer, 'Ready · Enter reveals a target and starts timing', 11)
        self.status.pack(side='left')
        self.start_button = tk.Button(footer, text='Start trial  ↵', command=self.start_trial,
                                      bg=self.BLUE, fg='white', relief='flat', padx=18, pady=8)
        self.start_button.pack(side='right')
        tk.Button(footer, text='Cancel · Esc', command=self.cancel,
                  relief='flat', bg='white', padx=14, pady=8).pack(side='right', padx=8)

    def _enter(self, event):
        if self.enter_release_job is not None:
            self.root.after_cancel(self.enter_release_job)
            self.enter_release_job = None
        if not self.enter_down:
            self.enter_down = True
            self.start_trial()
        return 'break'

    def _release_enter(self, event):
        # X11 auto-repeat emits adjacent release/press events. A deferred
        # release distinguishes those from an actual key-up.
        def released():
            self.enter_down = False
            self.enter_release_job = None
        self.enter_release_job = self.root.after(40, released)

    def _motion(self, event):
        if self.source is None and self.trial and self.trial.status == 'running':
            self.u = 2 * event.x / max(1, self.canvas.winfo_width()) - 1
            self.v = 1 - 2 * event.y / max(1, self.canvas.winfo_height())

    def _scroll(self, event):
        if event.delta:
            self._height_change(0.003 if event.delta > 0 else -0.003)
        return 'break'

    def _height_change(self, delta):
        if self.source is None and self.trial and self.trial.status == 'running':
            self.height = max(0.28, min(0.52, self.height + delta))

    def input_sample(self, now):
        if self.source is None:
            command = pointer_position(self.u, self.v, self.height)
            return command, dict(u=self.u, v=self.v, height_m=self.height)
        sample = self.source.latest()
        if self.source.error:
            raise ValueError(self.source.error)
        if (not sample or now - sample['received_monotonic_s'] + sample.get('source_age_s', 0.0)
                > self.settings.max_feedback_gap_s):
            raise ValueError('Waiting for fresh magnet-board packets.')
        return magnet_position(sample['pose']), sample

    def start_trial(self):
        if self.pending_save:
            self.save_result()
            return
        if self.closed or (self.trial and self.trial.status == 'running'):
            return
        if self.index >= len(self.targets):
            return
        try:
            self.input_sample(time.monotonic())
            # Establish the same actual starting pose before the timing clock.
            self.robot.reset()
            self.u = self.v = 0.0
            self.height = DIGIT_CUBE_CENTER_M[2]
            self.trial = TeleoperationTrial(
                self.targets[self.index], self.settings, time.monotonic(),
                self.robot.sim_time, self.robot.position)
            self.last_tick = self.trial.start_wall_s
            self.status.configure(text='Trial {} / {} · navigate to the blue sphere'.format(
                self.index + 1, len(self.targets)))
            self.start_button.configure(state='disabled', text='Recording…')
            self.feedback.configure(text='')
            self.coordinates.configure(text='Target · robot base frame\nX {:.3f}   Y {:.3f}   Z {:.3f} m'.format(
                *self.trial.target['position_m']))
        except Exception as exc:
            self.feedback.configure(text=str(exc), fg='#c74736')

    def cancel(self):
        if self.trial and self.trial.status == 'running':
            self.trial.finish('cancelled', time.monotonic(), self.robot.sim_time)
            self.robot.hold()
            self.save_result()

    def save_result(self):
        self.pending_save = self.trial
        try:
            row = self.run.save_trial(self.trial)
        except Exception as exc:
            self.status.configure(text='Save failed · retry before continuing')
            self.feedback.configure(text=str(exc), fg='#c74736')
            self.start_button.configure(state='normal', text='Retry save')
            return
        self.pending_save = None
        completed = row['status'] == 'completed'
        # A cancelled/failed attempt remains logged and repeats the same target.
        if completed or row['status'] == 'timed_out':
            self.index += 1
        self.feedback.configure(
            text=('Saved · {:.2f} s\nEndpoint error {:.1f} mm'.format(
                row['completion_time_s'], row['endpoint_error_m'] * 1000)
                  if completed else 'Saved · {}\n{}'.format(row['status'].replace('_', ' '),
                      'Enter retries this target' if row['status'] != 'timed_out' else 'Enter starts the next target')),
            fg=self.GREEN if completed else '#c74736')
        finished = self.index >= len(self.targets)
        self.status.configure(text='Session complete · {} trials saved'.format(len(self.run.completed))
                              if finished else 'Saved · Enter starts the next trial')
        self.start_button.configure(state='disabled' if finished else 'normal',
                                    text='Session complete' if finished else 'Next trial  ↵')
        print('Saved trial {}: {}'.format(row['trial'], row['status']), flush=True)

    def tick(self):
        if self.closed:
            return
        now = time.monotonic()
        dt, self.last_tick = min(0.1, max(0.001, now - self.last_tick)), now
        active = self.trial and self.trial.status == 'running'
        if active:
            try:
                command, sample = self.input_sample(now)
                self.robot.command_cartesian(command)
                self.robot.step(dt)
                self.trial.update(time.monotonic(), self.robot.sim_time,
                                  self.robot.position, self.robot.command_position,
                                  input_sample=sample)
            except Exception as exc:
                self.trial.finish('feedback_lost', time.monotonic(), self.robot.sim_time)
                self.feedback.configure(text=str(exc), fg='#c74736')
                print('Feedback stopped: {}'.format(exc), flush=True)
            if self.trial.status != 'running':
                self.robot.hold()
                self.save_result()
        progress = (self.trial.dwell_progress(now) if self.trial and
                    self.trial.status in ('running', 'completed') else 0)
        self.ring.itemconfigure(self.arc, extent=-max(0.01, 359.99 * progress))
        self.ring.itemconfigure(self.percent, text='{:d}%'.format(round(progress * 100)))
        self.progress_text.configure(text=('Target held · saving' if self.pending_save else 'Target held · saved') if progress >= 1 else
                                     ('Hold steady…' if progress > 0 else 'Move to target' if active else 'Waiting for Enter'))
        if self.trial:
            self.metrics.configure(text='Distance  {:.1f} mm\nElapsed    {:.2f} s'.format(
                self.trial.error_m * 1000,
                (self.trial.last_wall_s if self.trial.status != 'running' else now) - self.trial.start_wall_s))
        self.height_label.configure(text='{:.1f} cm'.format(self.robot.position[2] * 100))
        if now - self.last_render >= 1 / 25:
            try:
                target = self.trial.target['position_m'] if self.trial else None
                pixels = self.robot.render(width=640, height=426, target=target, tolerance=self.settings.tolerance_m,
                                           dwell_progress=progress)
                image = self.Image.fromarray(pixels)
                scale = min(max(1, self.canvas.winfo_width()) / image.width,
                            max(1, self.canvas.winfo_height()) / image.height)
                image = image.resize((max(1, round(image.width * scale)),
                                      max(1, round(image.height * scale))))
                self._photo = self.ImageTk.PhotoImage(image)
                self.canvas.itemconfigure(self.scene_image, image=self._photo)
                actual_pixel = self.robot.project_world(self.robot.position)
                if actual_pixel:
                    x, y = [coordinate * scale for coordinate in actual_pixel]
                    self.canvas.coords(self.flange_dot, x - 5, y - 5, x + 5, y + 5)
                if target is not None:
                    centre = self.robot.project_world(target)
                    edge = self.robot.project_world((target[0], target[1], target[2] + self.settings.tolerance_m))
                    if centre and edge:
                        x, y = [coordinate * scale for coordinate in centre]
                        radius = max(12, scale * ((edge[0] - centre[0]) ** 2 + (edge[1] - centre[1]) ** 2) ** .5)
                        for item in (self.target_rim, self.target_arc):
                            self.canvas.coords(item, x - radius, y - radius, x + radius, y + radius)
                            self.canvas.itemconfigure(item, state='normal')
                        self.canvas.itemconfigure(self.target_arc, extent=-max(.01, 359.99 * progress))
                self.canvas.tag_raise(self.flange_dot)
                self.last_render = now
            except Exception as exc:
                print('Renderer error: {}'.format(exc), flush=True)
                self.status.configure(text='Renderer unavailable · {}'.format(exc))
                if self.trial and self.trial.status == 'running':
                    self.trial.finish('simulation_stopped', time.monotonic(), self.robot.sim_time)
                    self.robot.hold()
                    self.save_result()
        self.root.after(20, self.tick)

    def close(self):
        if self.closed:
            return
        self.cancel()
        if self.pending_save:
            from tkinter import messagebox
            messagebox.showerror('Trial not saved', 'Saving failed. Retry the save before closing.', parent=self.root)
            return
        self.closed = True
        if self.source:
            self.source.close()
        self.robot.close()
        self.root.destroy()


def main(argv=None):
    args = parser().parse_args(argv)
    import tkinter as tk
    root = tk.Tk()
    try:
        app = TeleoperationWindow(root, args)
    except Exception as exc:
        root.destroy()
        print('Teleoperation could not start: {}'.format(exc), file=sys.stderr)
        return 1
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda number, frame: root.after(0, app.close))
    root.mainloop()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
