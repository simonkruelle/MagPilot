"""Checks against the real MuJoCo FR3 dynamics, independent of the GUI."""

import importlib.util
import os
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from colmag.mujoco_robot import MuJoCoRobot
from colmag.robot_targets import DIGIT_CUBE_CENTER_M, digit_cube_target


@unittest.skipUnless(importlib.util.find_spec('mujoco'), 'optional MuJoCo dependency is not installed')
class MuJoCoRobotTests(unittest.TestCase):
    def setUp(self):
        self.robot = MuJoCoRobot()

    def tearDown(self):
        self.robot.close()

    def test_default_start_uses_measured_fr3_attachment_site(self):
        self.assertEqual(self.robot.model.nq, 7)
        self.assertEqual(self.robot.model.nu, 7)
        np.testing.assert_allclose(self.robot.position, DIGIT_CUBE_CENTER_M, atol=.001)
        self.assertEqual(self.robot.sim_time, 0)

    def test_commands_do_not_substitute_for_feedback(self):
        initial = self.robot.position
        target = np.asarray(digit_cube_target(6))
        residual = self.robot.command_cartesian(target)
        self.assertLess(residual, .001)
        np.testing.assert_allclose(self.robot.command_position, target)
        np.testing.assert_allclose(self.robot.position, initial)
        self.robot.step(.02)
        self.assertGreater(np.linalg.norm(self.robot.position - target), .05)
        self.assertGreater(self.robot.sim_time, 0)
        for _ in range(200):
            self.robot.step(.01)
        self.assertLess(np.linalg.norm(self.robot.position - target), .003)

    def test_all_existing_cube_corners_are_reached_by_dynamics(self):
        for digit in range(1, 10):
            target = np.asarray(digit_cube_target(digit))
            residual = self.robot.command_cartesian(target)
            self.assertLess(residual, .001, 'IK at digit {}'.format(digit))
            for _ in range(240):
                self.robot.step(.01)
            self.assertLess(np.linalg.norm(self.robot.position - target), .003, 'actual flange at digit {}'.format(digit))
            self.assertTrue(np.all(np.isfinite(self.robot.joint_positions)))

    def test_goal_bounds_and_reset_hold_actual_center(self):
        self.robot.command_cartesian((5, -3, 2))
        np.testing.assert_allclose(self.robot.command_position, (.57, -.12, .52))
        self.robot.step(.03)
        self.robot.reset()
        for _ in range(200):
            self.robot.step(.01)
        np.testing.assert_allclose(self.robot.position, DIGIT_CUBE_CENTER_M, atol=.001)
        self.robot.hold()
        for _ in range(50):
            self.robot.step(.01)
        np.testing.assert_allclose(self.robot.position, DIGIT_CUBE_CENTER_M, atol=.001)
        with self.assertRaises(ValueError):
            self.robot.command_cartesian((float('nan'), 0, 0))
        with self.assertRaises(ValueError):
            self.robot.step(-1)

    @unittest.skipUnless(os.environ.get('COLMAG_TEST_MUJOCO_RENDER') == '1', 'set COLMAG_TEST_MUJOCO_RENDER=1 for the renderer check')
    def test_target_and_loading_ring_render_in_actual_scene(self):
        target = (.50, .08, .40)
        empty = self.robot.render(width=640, height=480, target=target, dwell_progress=0)
        loaded = self.robot.render(width=640, height=480, target=target, dwell_progress=.75)
        self.assertEqual(loaded.shape, (480, 640, 3))
        self.assertEqual(loaded.dtype, np.uint8)
        self.assertGreater(np.count_nonzero(empty != loaded), 30)
        self.assertGreater(float(loaded.std()), 20)
        point = self.robot.project_world(target)
        self.assertIsNotNone(point)
        self.assertTrue(0 < point[0] < 640 and 0 < point[1] < 480)


if __name__ == '__main__':
    unittest.main()
