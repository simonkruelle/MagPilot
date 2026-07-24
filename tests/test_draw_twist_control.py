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
    node.height_smoothing_tau_far = 0.90
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
    node.height_smoothing_tau_far = 0.90
    node.height_noise_deadband = 0.001
    node.update_rate = 30.0
    node._height_filtered_m = None

    node._robot_height(0.050)
    moved = node._robot_height(0.070)

    assert 0.050 < moved < 0.070


def test_height_filter_smooths_harder_far_from_the_board():
    def make_node():
        node = ColmagDrawNode.__new__(ColmagDrawNode)
        node.height_sensor_min = 0.007
        node.height_sensor_max = 0.150
        node.height_smoothing_tau = 0.35
        node.height_smoothing_tau_far = 0.90
        node.height_noise_deadband = 0.001
        node.update_rate = 30.0
        node._height_filtered_m = None
        return node

    # Same 10 mm step, applied near the board vs. high up. The far filter is
    # heavier, so a single update should advance a smaller fraction of the step.
    near = make_node()
    near._robot_height(0.020)
    near_progress = near._robot_height(0.030) - 0.020

    far = make_node()
    far._robot_height(0.130)
    far_progress = far._robot_height(0.140) - 0.130

    assert 0.0 < far_progress < near_progress


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
    node._qd_cmd = np.ones(7)
    node.latest_joints = None

    paused = node._lift_gate_paused(SimpleNamespace(z=0.007))

    assert paused is False
    assert math.isclose(node._height_filtered_m, 0.007)
    assert node._desired_filt is None
    assert np.allclose(node._cart_velocity, 0.0)
    assert np.allclose(node._cart_acceleration, 0.0)
    assert np.allclose(node._qd_cmd, 0.0)


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


def test_analytic_jacobian_matches_finite_differences():
    from colmag_draw_node import _Q_MAX, _Q_MIN, _fk, _jacobian

    def finite_difference_jacobian(q, eps=1e-6):
        T0 = _fk(q)
        p0, R0 = T0[:3, 3], T0[:3, :3]
        J = np.zeros((6, 7))
        for i in range(7):
            dq = q.copy()
            dq[i] += eps
            Ti = _fk(dq)
            J[:3, i] = (Ti[:3, 3] - p0) / eps
            dR = (Ti[:3, :3] - R0) @ R0.T
            J[3:, i] = np.array([dR[2, 1], dR[0, 2], dR[1, 0]]) / eps
        return J

    rng = np.random.default_rng(1)
    for _ in range(25):
        q = rng.uniform(_Q_MIN, _Q_MAX)
        assert np.allclose(_jacobian(q), finite_difference_jacobian(q),
                           atol=1e-5)


def _stamp_node(update_rate=30.0):
    node = ColmagDrawNode.__new__(ColmagDrawNode)
    node.update_rate = update_rate
    node._stream_stamp = None
    return node


def test_stream_stamps_form_a_uniform_grid_despite_send_jitter():
    import rospy

    node = _stamp_node()
    period = 1.0 / 30.0
    jitters = [0.0, 0.008, -0.006, 0.011, 0.002, -0.009, 0.013]
    stamps = []
    for index, jitter in enumerate(jitters):
        now = rospy.Time.from_sec(100.0 + index * period + jitter)
        stamps.append(node._next_stream_stamp(now))

    deltas = [(later - earlier).to_sec()
              for earlier, later in zip(stamps, stamps[1:])]
    assert all(math.isclose(delta, period, abs_tol=1e-9) for delta in deltas)


def test_stream_stamps_never_snap_backward_when_grid_leads():
    import rospy

    node = _stamp_node()
    period = 1.0 / 30.0
    # An early-firing timer leaves the grid ahead of the wall clock. The next
    # stamp must stay on the monotonic grid, never jump back to the earlier
    # wall time (which would place a segment before its predecessor).
    first = node._next_stream_stamp(rospy.Time.from_sec(100.0))
    second = node._next_stream_stamp(rospy.Time.from_sec(100.0 + 0.2 * period))
    assert second > first
    assert math.isclose((second - first).to_sec(), period, abs_tol=1e-9)


def test_stream_stamps_reanchor_after_skipped_ticks():
    import rospy

    node = _stamp_node()
    period = 1.0 / 30.0
    node._next_stream_stamp(rospy.Time.from_sec(100.0))
    node._next_stream_stamp(rospy.Time.from_sec(100.0 + period))

    # A pause (lift gate, converged idle) leaves the grid behind; the next
    # send must re-anchor to the wall clock instead of stamping in the past.
    resumed = rospy.Time.from_sec(103.0)
    assert node._next_stream_stamp(resumed) == resumed


def test_s_curve_lookahead_prediction_respects_derivative_limits():
    dt = 1.0 / 30.0
    lookahead = 0.10
    position = np.zeros(3)
    velocity = np.zeros(3)
    acceleration = np.zeros(3)
    desired = np.array([0.0, 0.0, 0.10])

    for _ in range(150):
        predicted, predicted_velocity, _ = _s_curve_step(
            position, velocity, acceleration, desired, lookahead,
            min_duration=0.75, max_speed=0.10,
            max_acceleration=0.20, max_jerk=0.80)
        assert np.linalg.norm(predicted - position) <= 0.10 * lookahead + 1e-9
        assert np.linalg.norm(predicted_velocity) <= 0.10 + 1e-12

        position, velocity, acceleration = _s_curve_step(
            position, velocity, acceleration, desired, dt,
            min_duration=0.75, max_speed=0.10,
            max_acceleration=0.20, max_jerk=0.80)


if __name__ == '__main__':
    tests = [value for name, value in sorted(globals().items())
             if name.startswith('test_') and callable(value)]
    for test in tests:
        test()
        print('PASS', test.__name__)
