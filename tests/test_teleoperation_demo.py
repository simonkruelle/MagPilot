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

from colmag.teleoperation_control import magnet_position, pointer_position


class InputMappingTests(unittest.TestCase):
    def test_pointer_and_board_share_axes_and_cube_limits(self):
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
        for pose in ((0, 0, .161, 0, 0, 1), (0, 0, math.nan, 0, 0, 1), (0, 0, .02)):
            with self.assertRaises(ValueError):
                magnet_position(pose)
        with self.assertRaises(ValueError):
            pointer_position(math.inf, 0, .4)


@unittest.skipUnless(os.environ.get('DISPLAY') and importlib.util.find_spec('mujoco'),
                     'GUI rehearsal needs MuJoCo and a display (use xvfb-run).')
class GuiCollectionTests(unittest.TestCase):
    def test_board_snapshots_are_logged_and_stale_input_cancels_the_hold(self):
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
                board.stale = True
                deadline = time.monotonic() + 2
                while not app.run.completed and time.monotonic() < deadline:
                    root.update()
                    time.sleep(.01)
                self.assertEqual(app.run.completed[0]['status'], 'feedback_lost')
                self.assertEqual(app.run.completed[0]['completion_time_s'], '')
                self.assertEqual(app.ring.itemcget(app.percent, 'text'), '0%')
                self.assertIn('magnetic_fields', app.trial.rows[-1]['input_sample_json'])
                self.assertEqual(app.run.manifest['magnet_count'], 2)
            finally:
                app.close()
            self.assertTrue(board.closed)

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
                u, v = target[1] / .12, (.45 - target[0]) / .12
                app._motion(SimpleNamespace(
                    x=(u + 1) / 2 * app.canvas.winfo_width(),
                    y=(1 - v) / 2 * app.canvas.winfo_height()))
                # Same height increment used by wheel callbacks, without a held button.
                steps = round((target[2] - .4) / .003)
                for unused in range(abs(steps)):
                    app._height_change(.003 if steps > 0 else -.003)
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
