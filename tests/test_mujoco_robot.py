"""Checks against the real MuJoCo FR3 dynamics, independent of the GUI."""

import importlib.util
from itertools import product
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

    def test_default_start_uses_measured_fingertip_midpoint(self):
        self.assertEqual(self.robot.model.nq, 7)
        self.assertEqual(self.robot.model.nu, 7)
        self.assertEqual(self.robot.end_effector_site, 'gripper_center')
        self.assertEqual(self.robot.end_effector_reference, 'fingertip_midpoint')
        np.testing.assert_allclose(self.robot.position, DIGIT_CUBE_CENTER_M, atol=.001)
        np.testing.assert_allclose(self.robot.position, self.robot.fingertip_positions.mean(axis=0), atol=1e-12)
        self.assertAlmostEqual(self.robot.gripper_opening_m, .040, places=10)
        self.assertEqual(self.robot.flange_to_gripper_offset_frame, 'flange_local')
        np.testing.assert_allclose(self.robot.flange_to_gripper_offset_m, (0, 0, .1029), atol=1e-10)
        self.assertGreater(np.linalg.norm(self.robot.flange_position - self.robot.position), .10)
        self.assertEqual(self.robot.sim_time, 0)

    def test_fingertip_sites_are_on_opposing_contact_pad_faces(self):
        for name, tip in zip(('left_finger', 'right_finger'), self.robot.fingertip_positions):
            body_id = self.robot.mujoco.mj_name2id(self.robot.model, self.robot.mujoco.mjtObj.mjOBJ_BODY, name)
            pads = [index for index in range(self.robot.model.ngeom)
                    if self.robot.model.geom_bodyid[index] == body_id
                    and self.robot.model.geom_type[index] == self.robot.mujoco.mjtGeom.mjGEOM_BOX
                    and np.allclose(self.robot.model.geom_size[index], (.0085, .004, .0085))]
            self.assertEqual(len(pads), 1)
            pad = pads[0]
            rotation = self.robot.data.geom_xmat[pad].reshape(3, 3)
            inner_face = self.robot.data.geom_xpos[pad] - rotation[:, 1] * self.robot.model.geom_size[pad, 1]
            np.testing.assert_allclose(tip, inner_face, atol=1e-12)

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
            self.assertLess(np.linalg.norm(self.robot.position - target), .003, 'actual gripper centre at digit {}'.format(digit))
            np.testing.assert_allclose(self.robot.position, self.robot.fingertip_positions.mean(axis=0), atol=1e-12)
            self.assertTrue(np.all(np.isfinite(self.robot.joint_positions)))

    def test_difficult_heights_and_random_three_dimensional_goals_are_reachable(self):
        # Reproduce the reported 37--40 cm difficulty across the entire cube,
        # including its edges, and the target shown in the user's screenshot.
        goals = [(x, y, z) for z in (.37, .39, .40)
                 for x, y in product(np.linspace(.33, .57, 7), np.linspace(-.12, .12, 7))]
        goals.extend(np.random.default_rng(0).uniform((.33, -.12, .28), (.57, .12, .52), size=(40, 3)))
        goals.append((.403, .078, .492))
        for goal in goals:
            with self.subTest(goal=tuple(goal)):
                self.assertLess(self.robot.command_cartesian(goal), .001)
                for _ in range(240):
                    self.robot.step(.01)
                self.assertLess(np.linalg.norm(self.robot.position - goal), .003)
                np.testing.assert_allclose(self.robot.position, self.robot.fingertip_positions.mean(axis=0), atol=1e-12)
                self.assertAlmostEqual(self.robot.gripper_opening_m, .040, places=10)

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
    def test_faint_target_volume_renders_in_actual_scene(self):
        target = (.50, -.08, .40)
        empty = self.robot.render(width=640, height=480).copy()
        target_frame = self.robot.render(width=640, height=480, target=target)
        self.assertEqual(target_frame.shape, (480, 640, 3))
        self.assertEqual(target_frame.dtype, np.uint8)
        self.assertGreater(np.count_nonzero(empty != target_frame), 30)
        self.assertGreater(float(target_frame.std()), 20)
        point = self.robot.project_world(target)
        self.assertIsNotNone(point)
        self.assertTrue(0 < point[0] < 640 and 0 < point[1] < 480)

    @unittest.skipUnless(os.environ.get('COLMAG_TEST_MUJOCO_RENDER') == '1', 'set COLMAG_TEST_MUJOCO_RENDER=1 for the renderer check')
    def test_projected_overlay_matches_rendered_marker_pixels(self):
        # Render an isolated coloured marker and compare its actual pixels to
        # overlay coordinates. This catches stereo-eye offsets and aspect errors.
        for width, height in ((640, 426), (960, 640), (640, 480)):
            for target in ((.403, .078, .492), (.33, -.12, .37), (.57, .12, .40), (.45, 0, .40)):
                with self.subTest(size=(width, height), target=target):
                    self.robot.render(width, height)
                    scene = self.robot._renderer.scene
                    scene.ngeom = 0
                    marker = self.robot._add_geom(scene, self.robot.mujoco.mjtGeom.mjGEOM_SPHERE,
                                                  (.012, 0, 0), target, (1, 0, 0, 1))
                    marker.emission = 1
                    frame = self.robot._renderer.render()
                    rows, columns = np.nonzero((frame[:, :, 0] > 180) & (frame[:, :, 1] < 80) & (frame[:, :, 2] < 80))
                    self.assertGreater(len(rows), 30)
                    pixel_centre = (np.mean(columns + .5), np.mean(rows + .5))
                    self.assertLess(np.linalg.norm(np.asarray(pixel_centre) - self.robot.project_world(target)), .75)
                    actual_radius = (rows.max() - rows.min() + 1) / 2
                    self.assertLess(abs(actual_radius - self.robot.projected_radius(target, .012)), 1)

        camera = self.robot._mono_camera()
        behind = np.asarray(camera.pos) - np.asarray(camera.forward)
        self.assertIsNone(self.robot.project_world(behind))
        self.assertIsNone(self.robot.projected_radius(behind, .025))
        with self.assertRaises(ValueError):
            self.robot.projected_radius(DIGIT_CUBE_CENTER_M, 0)


if __name__ == '__main__':
    unittest.main()
