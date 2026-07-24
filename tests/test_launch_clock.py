#!/usr/bin/env python3
"""Regression tests for simulation versus real-robot ROS clock selection."""

import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(ROOT, 'ros', 'colmag_ros', 'launch')


class LaunchClockTests(unittest.TestCase):
    def test_simulation_enables_simulated_time(self):
        root = ET.parse(os.path.join(LAUNCH_DIR, 'fr3.launch')).getroot()
        clock_args = [
            element for element in root.iter('arg')
            if element.get('name') == 'use_sim_time'
        ]

        self.assertTrue(clock_args)
        self.assertTrue(any(arg.get('value') == 'true' for arg in clock_args))

    def test_real_robot_forces_wall_clock_before_franka_control(self):
        root = ET.parse(os.path.join(LAUNCH_DIR, 'fr3_real.launch')).getroot()
        children = list(root)
        clock_indices = [
            index for index, element in enumerate(children)
            if (element.tag == 'param'
                and element.get('name') == '/use_sim_time'
                and element.get('value') == 'false')
        ]
        franka_indices = [
            index for index, element in enumerate(children)
            if (element.tag == 'include'
                and 'franka_control' in element.get('file', ''))
        ]

        self.assertEqual(len(clock_indices), 1)
        self.assertEqual(len(franka_indices), 1)
        self.assertLess(clock_indices[0], franka_indices[0])


if __name__ == '__main__':
    unittest.main()
