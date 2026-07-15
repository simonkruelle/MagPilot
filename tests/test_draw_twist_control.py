#!/usr/bin/env python3
"""Focused tests for absolute magnet-twist commands in the draw node."""

import math
import os
import sys
from types import SimpleNamespace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS = os.path.join(ROOT, 'ros', 'colmag_ros', 'scripts')
for path in (ROOT, SCRIPTS):
    if path not in sys.path:
        sys.path.insert(0, path)

from colmag_draw_node import ColmagDrawNode


def _node(yaw=0.2):
    node = ColmagDrawNode.__new__(ColmagDrawNode)
    node.pen_yaw = yaw
    node._magnet_twist_base_yaw = 0.0
    node.max_pen_yaw = math.radians(92.0)
    node.R_pen0 = np.eye(3)
    node.R_pen = node.R_pen0.copy()
    return node


def test_repeated_absolute_twist_target_does_not_accumulate():
    node = _node()
    node._on_command(SimpleNamespace(data='rotate:reference'))

    node._on_command(SimpleNamespace(data='rotate:set:10.0'))
    first = node.pen_yaw
    node._on_command(SimpleNamespace(data='rotate:set:10.0'))

    expected = 0.2 + math.radians(10.0)
    assert math.isclose(first, expected)
    assert math.isclose(node.pen_yaw, expected)


def test_absolute_twist_targets_can_reverse_without_step_accumulation():
    node = _node(yaw=0.0)
    node._on_command(SimpleNamespace(data='rotate:reference'))

    node._on_command(SimpleNamespace(data='rotate:set:12.0'))
    node._on_command(SimpleNamespace(data='rotate:set:-12.0'))

    assert math.isclose(node.pen_yaw, math.radians(-12.0))


def test_height_noise_gate_holds_stationary_robot_target():
    node = ColmagDrawNode.__new__(ColmagDrawNode)
    node.height_sensor_min = 0.007
    node.height_sensor_max = 0.150
    node.height_smoothing_tau = 0.35
    node.height_noise_deadband = 0.001
    node.update_rate = 30.0
    node._height_filtered_m = None

    assert math.isclose(node._robot_height(0.050), 0.050)
    for measured in (0.0508, 0.0492, 0.0506, 0.0494):
        assert math.isclose(node._robot_height(measured), 0.050)


def test_height_filter_follows_real_motion_smoothly():
    node = ColmagDrawNode.__new__(ColmagDrawNode)
    node.height_sensor_min = 0.007
    node.height_sensor_max = 0.150
    node.height_smoothing_tau = 0.35
    node.height_noise_deadband = 0.001
    node.update_rate = 30.0
    node._height_filtered_m = None

    node._robot_height(0.050)
    moved = node._robot_height(0.070)

    assert 0.050 < moved < 0.070


def test_height_filter_resets_when_lift_gate_resumes():
    node = ColmagDrawNode.__new__(ColmagDrawNode)
    node.lift_gate_z = 0.150
    node.lift_reengage_z = 0.140
    node._lift_paused = True
    node.height_sensor_min = 0.007
    node.height_sensor_max = 0.150
    node._height_filtered_m = 0.120
    node._desired_filt = np.array([0.45, 0.0, 0.50])
    node.latest_joints = None

    paused = node._lift_gate_paused(SimpleNamespace(z=0.007))

    assert paused is False
    assert math.isclose(node._height_filtered_m, 0.007)
    assert node._desired_filt is None


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items())
             if name.startswith('test_') and callable(value)]
    for test in tests:
        test()
        print('PASS', test.__name__)
