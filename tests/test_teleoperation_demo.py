"""Control axes and a full measured-physics GUI collection rehearsal."""

import csv
import importlib.util
import math
import os
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest import mock

from colmag.teleoperation_control import magnet_position, pointer_position, wheel_height_delta


class InputMappingTests(unittest.TestCase):
    def test_robot_plane_normalization_and_board_cube_limits(self):
        self.assertEqual(pointer_position(0, 0, .4), (.45, 0, .4))
        self.assertAlmostEqual(pointer_position(1, 1, .8)[0], .33)
        self.assertEqual(pointer_position(1, 1, .8)[1:], (.12, .52))
        mapped = magnet_position((.05, .05, .017, 0, 0, 1))
        self.assertAlmostEqual(mapped[0], .33)
        self.assertAlmostEqual(mapped[1], .12)
        self.assertAlmostEqual(mapped[2], .28)
        raised = magnet_position((0, 0, .160, 0, 0, 1))
        self.assertAlmostEqual(raised[2], .52)

    def test_invalid_or_lifted_board_input_cannot_drive_the_robot(self):
        for pose in ((0, 0, .161, 0, 0, 1), (0, 0, math.nan, 0, 0, 1), (0, 0, .02),
                     (.116, .191, -.118, 0, 0, 1), (.05001, 0, .02, 0, 0, 1)):
            with self.assertRaises(ValueError):
                magnet_position(pose)
        with self.assertRaises(ValueError):
            pointer_position(math.inf, 0, .4)

    def test_platform_wheel_sign_magnitude_and_fine_deltas(self):
        self.assertEqual(wheel_height_delta(button=4), .001)
        self.assertEqual(wheel_height_delta(button=5), -.001)
        self.assertEqual(wheel_height_delta(120, window_system='win32'), .001)
        self.assertEqual(wheel_height_delta(-240, window_system='win32'), -.002)
        self.assertEqual(wheel_height_delta(30, window_system='win32'), .00025)
        self.assertEqual(wheel_height_delta(-1, window_system='aqua'), -.001)
        self.assertEqual(wheel_height_delta(3, window_system='aqua'), .003)
        self.assertEqual(wheel_height_delta(), 0)
        with self.assertRaises(ValueError):
            wheel_height_delta(math.nan)


@unittest.skipUnless(os.environ.get('DISPLAY') and importlib.util.find_spec('mujoco'),
                     'GUI rehearsal needs MuJoCo and a display (use xvfb-run).')
class GuiCollectionTests(unittest.TestCase):
    def render_ready(self, root, app):
        deadline = time.monotonic() + 3
        while app.scene_image_size is None and time.monotonic() < deadline:
            root.update()
            time.sleep(.01)
        self.assertIsNotNone(app.scene_image_size)

    def point_at(self, root, app, position):
        self.render_ready(root, app)
        x, y = app.robot.project_world(position)
        width, height = app.scene_image_size
        app.canvas.event_generate('<Motion>', x=round(x * width / app.RENDER_SIZE[0]),
                                  y=round(y * height / app.RENDER_SIZE[1]))
        root.update()

    def test_visible_cube_corners_are_reachable_through_real_pointer_events(self):
        from itertools import product
        import tkinter as tk
        from tools.teleoperation_demo import parser, TeleoperationWindow

        with tempfile.TemporaryDirectory() as folder:
            args = parser().parse_args(['--output-dir', str(Path(folder) / 'session')])
            root = tk.Tk()
            app = TeleoperationWindow(root, args)
            try:
                app.start_trial()
                self.render_ready(root, app)
                self.assertEqual(app.height_slider.get(), 40)
                for x, y, z in product((.33, .57), (-.12, .12), (.28, .52)):
                    app.height_slider.set(z * 100)
                    root.update()
                    self.point_at(root, app, (x, y, z))
                    command, sample = app.input_sample(time.monotonic())
                    self.assertLess(math.dist(command, (x, y, z)), .0015,
                                    'goal {}, command {}, selected height {}'.format((x, y, z), command, app.height))
                    self.assertEqual(sample['mapping'], 'scene_height_plane_v1')
                    deadline = time.monotonic() + 4
                    while math.dist(app.robot.position, (x, y, z)) > .002 and time.monotonic() < deadline:
                        root.update()
                        time.sleep(.01)
                    self.assertLess(math.dist(app.robot.position, (x, y, z)), .002)
                # Letterboxing and the reference view must leave XY unchanged.
                previous = (app.u, app.v)
                app._motion(SimpleNamespace(x=app.scene_image_size[0] + 5, y=50))
                left, top, right, bottom = app.reference_bounds
                app._motion(SimpleNamespace(x=(left + right) / 2, y=(top + bottom) / 2))
                self.assertEqual((app.u, app.v), previous)
                self.assertEqual(app.trial.status, 'running')
            finally:
                app.close()

    def test_height_changes_keep_the_pointer_ray_and_resize_preserves_commands(self):
        import tkinter as tk
        from tools.teleoperation_demo import parser, TeleoperationWindow

        with tempfile.TemporaryDirectory() as folder:
            args = parser().parse_args(['--output-dir', str(Path(folder) / 'session')])
            root = tk.Tk()
            app = TeleoperationWindow(root, args)
            try:
                app.start_trial()
                self.point_at(root, app, (.45, 0, .4))
                app._height_change(.01)
                command, unused = app.input_sample(time.monotonic())
                projected = app.robot.project_world(command)
                source_pixel = tuple(fraction * size for fraction, size in
                                     zip(app.pointer_image_uv, app.RENDER_SIZE))
                self.assertLess(math.dist(projected, source_pixel), .01)
                before = (app.u, app.v)
                width, height = app.scene_image_size
                app._motion(SimpleNamespace(x=app.pointer_image_uv[0] * width,
                                            y=app.pointer_image_uv[1] * height))
                self.assertEqual((app.u, app.v), before)
                root.geometry('1000x740')
                root.update()
                app._apply_scene_pointer()
                self.assertEqual((app.u, app.v), before)
            finally:
                app.close()

    def test_sidebar_wheel_and_slider_reach_exact_heights_with_measured_feedback(self):
        import tkinter as tk
        from tools.teleoperation_demo import parser, TeleoperationWindow

        with tempfile.TemporaryDirectory() as folder:
            args = parser().parse_args(['--output-dir', str(Path(folder) / 'session')])
            root = tk.Tk()
            app = TeleoperationWindow(root, args)
            try:
                root.update()
                app.height_label.event_generate('<Button-5>')
                root.update()
                self.assertEqual(app.height, .4)  # ready pose is locked
                app.start_trial()
                self.assertEqual(app.height_slider.cget('state'), 'normal')
                for unused in range(30):
                    # Real Tk events on the sidebar must bubble to the window.
                    app.height_label.event_generate('<Button-5>')
                self.assertAlmostEqual(app.height, .37)
                for height_cm in (37, 39, 40, 49.2):
                    if height_cm != 37:
                        app.height_slider.set(height_cm)
                        root.update()
                    self.assertAlmostEqual(app.height, height_cm / 100)
                    deadline = time.monotonic() + 5
                    while abs(app.robot.position[2] - height_cm / 100) > .0015 and time.monotonic() < deadline:
                        root.update()
                        time.sleep(.01)
                    self.assertLess(abs(app.robot.position[2] - height_cm / 100), .0015)
                    self.assertIn('Measured', app.height_label.cget('text'))
                    self.assertIn('Command', app.command_height_label.cget('text'))
                self.assertEqual(app.height_control, 'slider')
                self.assertEqual(app.trial.status, 'running')
                app.cancel()
                self.assertEqual(app.height_slider.cget('state'), 'disabled')
                root.geometry('1000x740')
                root.update()
                for widget in (app.height_slider, app.feedback):
                    self.assertTrue(widget.winfo_ismapped())
                    self.assertLessEqual(widget.winfo_y() + widget.winfo_height(),
                                         widget.master.winfo_height())
                self.assertEqual(app.run.manifest['protocol'], 'mujoco_random_target_reaching_v2')
                self.assertEqual(app.run.manifest['end_effector_reference'], 'fingertip_midpoint')
            finally:
                app.close()

    def test_lifted_and_stale_board_estimates_pause_then_resume_a_full_hold(self):
        import tkinter as tk
        from tools.teleoperation_demo import parser, TeleoperationWindow

        class Board:
            error = ''
            stale = False
            pose = [0, 0, .050, 0, 0, 1]
            closed = False

            def latest(self):
                return dict(pose=self.pose, magnetic_fields=[1] * 48,
                            received_monotonic_s=time.monotonic() - (1 if self.stale else 0))

            def close(self):
                self.closed = True

        board = Board()
        with tempfile.TemporaryDirectory() as folder:
            args = parser().parse_args(['--input-source', 'serial', '--output-dir',
                                       str(Path(folder) / 'session'), '--magnet-count', '2'])
            root = tk.Tk()
            with mock.patch('colmag.teleoperation_input.SerialInput', return_value=board):
                app = TeleoperationWindow(root, args)
            try:
                root.update()
                app.start_trial()
                self.assertEqual(app.height_slider.cget('state'), 'disabled')
                x, y, z = app.trial.target['position_m']
                fraction = (z - .28) / .24
                linear = -math.log1p(-fraction * (1 - math.exp(-2))) / 2
                board.pose = [y / .12 * .05, (.45 - x) / .12 * .05,
                              .010 + .007 + linear * (.150 - .007), 0, 0, 1]
                deadline = time.monotonic() + 8
                while app.trial.progress < .2 and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertGreaterEqual(app.trial.progress, .2)
                valid_pose = board.pose
                board.pose = [0, 0, .3, 0, 0, 1]
                deadline = time.monotonic() + 2
                while app.trial.rows[-1]['input_valid'] and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertEqual(app.trial.status, 'running')
                self.assertEqual(app.trial.progress, 0)
                self.assertFalse(app.trial.rows[-1]['input_valid'])
                self.assertIn('15 cm', app.trial.rows[-1]['input_sample_json'])
                self.assertEqual(app.index, 0)
                self.assertEqual(app.run.completed, [])
                board.pose = valid_pose
                board.stale = True
                deadline = time.monotonic() + 2
                while 'fresh magnet-board packets' not in app.feedback.cget('text') and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertEqual(app.trial.status, 'running')
                self.assertEqual(app.run.completed, [])
                self.assertEqual(app.ring.itemcget(app.percent, 'text'), '0%')
                self.assertIn('magnetic_fields', app.trial.rows[-1]['input_sample_json'])
                board.stale = False
                recovered = time.monotonic()
                deadline = recovered + 8
                while not app.run.completed and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                result = app.run.completed[0]
                self.assertEqual(result['status'], 'completed')
                self.assertGreaterEqual(result['finished_wall_monotonic_s'] - recovered, 2)
                self.assertGreaterEqual(result['held_duration_s'], 2)
                self.assertTrue(app.trial.rows[-1]['input_valid'])
                self.assertEqual(app.run.manifest['magnet_count'], 2)
            finally:
                app.close()
            self.assertTrue(board.closed)

    def test_invalid_board_cannot_start_and_connection_failure_retains_the_reason(self):
        import tkinter as tk
        from tools.teleoperation_demo import parser, TeleoperationWindow

        class Board:
            error = ''
            pose = [.116, .191, -.118, 0, 0, 1]

            def latest(self):
                return dict(pose=self.pose, magnetic_fields=[1] * 48,
                            received_monotonic_s=time.monotonic())

            def close(self):
                pass

        board = Board()
        with tempfile.TemporaryDirectory() as folder:
            args = parser().parse_args(['--input-source', 'serial', '--output-dir',
                                       str(Path(folder) / 'session')])
            root = tk.Tk()
            with mock.patch('colmag.teleoperation_input.SerialInput', return_value=board):
                app = TeleoperationWindow(root, args)
            try:
                root.update()
                app.start_trial()
                self.assertIsNone(app.trial)
                self.assertIn('input area', app.feedback.cget('text'))
                board.pose = [0, 0, .05, 0, 0, 1]
                app._show_board_readiness(time.monotonic())
                self.assertIn('Board ready', app.feedback.cget('text'))
                app.start_trial()
                board.error = 'serial port disconnected'
                deadline = time.monotonic() + 2
                while not app.run.completed and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertEqual(app.run.completed[0]['status'], 'feedback_lost')
                self.assertIn('serial port disconnected', app.feedback.cget('text'))
            finally:
                app.close()

    def test_enter_pointer_hold_save_cancel_and_close(self):
        import tkinter as tk
        from tools.teleoperation_demo import parser, TeleoperationWindow

        with tempfile.TemporaryDirectory() as folder:
            args = parser().parse_args(['--output-dir', str(Path(folder) / 'session'),
                                       '--trials', '2', '--participant-name', 'Software test',
                                       '--experiment-name', 'GUI rehearsal'])
            root = tk.Tk()
            app = TeleoperationWindow(root, args)
            try:
                root.update()
                # Motion outside a trial must not move the common starting pose.
                app._motion(SimpleNamespace(x=0, y=0))
                self.assertEqual((app.u, app.v), (0, 0))
                app._enter(None)
                first_trial = app.trial
                app._enter(None)  # held Enter cannot reset/restart the clock
                self.assertIs(app.trial, first_trial)
                app.enter_down = False
                target = app.trial.target['position_m']
                # Point at the actual rendered target, not a precomputed
                # position on an invisible full-window control pad.
                self.point_at(root, app, target)
                # Same height increment used by wheel callbacks, without a held button.
                steps = round((target[2] - .4) / .001)
                for unused in range(abs(steps)):
                    app._height_change(.001 if steps > 0 else -.001)
                deadline = time.monotonic() + 12
                while not app.run.completed and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertEqual(len(app.run.completed), 1)
                result = app.run.completed[0]
                self.assertEqual(result['status'], 'completed')
                self.assertGreaterEqual(result['held_duration_s'], 2)
                self.assertGreater(result['completion_time_s'], 2)
                self.assertLessEqual(result['endpoint_error_m'], .025)
                with open(Path(app.run.output_dir) / result['trajectory_csv']) as stream:
                    rows = list(csv.DictReader(stream))
                self.assertTrue(any(abs(float(row['actual_x_m']) - float(row['commanded_x_m'])) > .001
                                    for row in rows if row['commanded_x_m']))
                self.assertIn('height_m', rows[-1]['input_sample_json'])
                self.assertIn('scene_height_plane_v1', rows[-1]['input_sample_json'])
                self.assertIn('pointer_image_uv', rows[-1]['input_sample_json'])
                app.start_trial()
                self.assertEqual(app.trial.status, 'running')
                self.assertLess(math.dist(app.robot.position, (.45, 0, .4)), .001)
                app.close()  # unfinished trial must be retained as cancelled
                self.assertEqual(app.run.completed[-1]['status'], 'cancelled')
                self.assertTrue(app.closed)
            finally:
                if not app.closed:
                    app.close()


if __name__ == '__main__':
    unittest.main()
