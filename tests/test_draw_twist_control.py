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

from colmag_draw_node import ColmagDrawNode, _s_curve_step


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
    node._cart_velocity = np.array([0.0, 0.0, 0.08])
    node._cart_acceleration = np.array([0.0, 0.0, 0.20])
    node._qd_filt = np.ones(7)
    node.latest_joints = None

    paused = node._lift_gate_paused(SimpleNamespace(z=0.007))

    assert paused is False
    assert math.isclose(node._height_filtered_m, 0.007)
    assert node._desired_filt is None
    assert np.allclose(node._cart_velocity, 0.0)
    assert np.allclose(node._cart_acceleration, 0.0)
    assert np.allclose(node._qd_filt, 0.0)


def test_s_curve_step_is_monotonic_and_respects_derivative_limits():
    dt = 1.0 / 30.0
    position = np.zeros(3)
    velocity = np.zeros(3)
    acceleration = np.zeros(3)
    desired = np.array([0.0, 0.0, 0.10])
    previous_z = position[2]

    for _ in range(150):
        previous_acceleration = acceleration.copy()
        position, velocity, acceleration = _s_curve_step(
            position, velocity, acceleration, desired, dt,
            min_duration=0.75, max_speed=0.10,
            max_acceleration=0.20, max_jerk=0.80)

        jerk = (acceleration - previous_acceleration) / dt
        assert position[2] >= previous_z - 1e-12
        assert np.linalg.norm(velocity) <= 0.10 + 1e-12
        assert np.linalg.norm(acceleration) <= 0.20 + 1e-12
        assert np.linalg.norm(jerk) <= 0.80 + 1e-12
        previous_z = position[2]

    assert math.isclose(position[2], 0.10, abs_tol=1e-12)
    assert np.linalg.norm(velocity) < 0.002
    assert np.linalg.norm(acceleration) < 0.01


def test_s_curve_reverses_height_without_breaking_derivative_limits():
    dt = 1.0 / 30.0
    position = np.zeros(3)
    velocity = np.zeros(3)
    acceleration = np.zeros(3)

    for index in range(240):
        desired_z = 0.10 if index < 45 else 0.0
        previous_acceleration = acceleration.copy()
        position, velocity, acceleration = _s_curve_step(
            position, velocity, acceleration,
            np.array([0.0, 0.0, desired_z]), dt,
            min_duration=0.75, max_speed=0.10,
            max_acceleration=0.20, max_jerk=0.80)

        jerk = (acceleration - previous_acceleration) / dt
        assert np.linalg.norm(velocity) <= 0.10 + 1e-12
        assert np.linalg.norm(acceleration) <= 0.20 + 1e-12
        assert np.linalg.norm(jerk) <= 0.80 + 1e-12

    assert math.isclose(position[2], 0.0, abs_tol=1e-12)
    assert np.allclose(velocity, 0.0)
    assert np.allclose(acceleration, 0.0)


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items())
             if name.startswith('test_') and callable(value)]
    for test in tests:
        test()
        print('PASS', test.__name__)
