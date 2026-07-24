#!/usr/bin/env python3
"""Regression tests for consistent responsive MagPilot launch defaults."""

import ast
import os
import unittest
import xml.etree.ElementTree as ET


ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(ROOT, 'ros', 'colmag_ros', 'launch')
DRAW_NODE = os.path.join(
    ROOT, 'ros', 'colmag_ros', 'scripts', 'colmag_draw_node.py')
EXPECTED = {
    'target_smoothing_tau': '0.04',
    'height_smoothing_tau': '0.25',
    'height_smoothing_tau_far': '0.65',
    'max_linear_speed': '0.12',
    'max_linear_acceleration': '0.30',
    'max_linear_jerk': '1.50',
    's_curve_min_duration': '0.50',
}


class MagPilotDefaultsTests(unittest.TestCase):
    def test_draw_node_fallbacks_match_launch_defaults(self):
        with open(DRAW_NODE, 'r') as source:
            tree = ast.parse(source.read(), DRAW_NODE)
        constants = {}
        for node in tree.body:
            if not isinstance(node, ast.Assign) or len(node.targets) != 1:
                continue
            target = node.targets[0]
            if isinstance(target, ast.Name) and target.id.startswith('DEFAULT_'):
                constants[target.id] = ast.literal_eval(node.value)

        expected_constants = {
            'DEFAULT_' + name.upper(): float(value)
            for name, value in EXPECTED.items()
        }
        for name, expected in expected_constants.items():
            self.assertEqual(constants.get(name), expected, name)

    def test_launch_files_share_responsive_defaults(self):
        for filename in ('colmag_arm_nodes.launch', 'colmag_draw.launch'):
            root = ET.parse(os.path.join(LAUNCH_DIR, filename)).getroot()
            defaults = {
                element.get('name'): element.get('default')
                for element in root.findall('arg')
            }
            params = {
                element.get('name'): element.get('value')
                for element in root.iter('param')
            }

            for name, expected in EXPECTED.items():
                self.assertEqual(defaults.get(name), expected, filename)
                self.assertEqual(params.get(name), '$(arg {})'.format(name),
                                 filename)


if __name__ == '__main__':
    unittest.main()
