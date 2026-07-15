#!/usr/bin/env python3
"""Unit tests for the magnet-orientation teleop control logic in
magnetometer_reader._magnet_control_decisions().

This is the real-sensor-only path (tilt theta -> gripper, twist phi -> wrist)
that cannot be exercised with the trackpad, so it is verified here in
isolation — no ROS, no hardware, no display.

Run:  python3 test_magnet_teleop.py
"""

import math
import os as _os, sys as _sys
from types import SimpleNamespace
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)
import os
import sys

os.environ.setdefault('MPLBACKEND', 'Agg')

from magnetometer_reader import MagnetometerReader


def _reader():
    """A bare reader with just the fields _magnet_control_decisions needs."""
    r = MagnetometerReader.__new__(MagnetometerReader)
    r.magnet_gripper_close_deg = 25.0
    r.magnet_gripper_open_deg = 55.0
    r.magnet_gripper_hold_s = 0.30
    r.magnet_rotate_step_deg = 70.0
    r.magnet_twist_min_tilt_deg = 10.0
    r.magnet_twist_filter_tau_s = 0.35
    r.magnet_twist_control_gain = 0.60
    r.magnet_twist_command_deadband_deg = 3.0
    r.magnet_twist_max_rate_deg_s = 30.0
    r._magnet_grip_closed = False
    r._magnet_grip_pending = None
    r._magnet_twist_ref_phi = None
    r._magnet_robot_phi_filtered = None
    r._magnet_robot_phi_filter_at = None
    r._magnet_robot_phi_reference = None
    r._magnet_robot_target_deg = 0.0
    r._magnet_robot_target_sent_deg = 0.0
    r._magnet_robot_target_at = None
    r.magnet_height_sensor_bias_m = 0.010
    r.magnet_height_min_m = 0.007
    return r


def test_gripper_requires_sustained_upright_hold():
    r = _reader()
    assert r._magnet_control_decisions(10.0, 0.0, 0.00) == []            # upright, not held
    assert r._magnet_control_decisions(10.0, 0.0, 0.20) == []            # still holding
    assert r._magnet_control_decisions(10.0, 0.0, 0.35) == ['gripper:close']  # held >= 0.3s
    assert r._magnet_control_decisions(10.0, 0.0, 0.60) == []            # stays closed
    assert r._magnet_grip_closed is True


def test_gripper_opens_when_tilted():
    r = _reader()
    r._magnet_grip_closed = True
    assert r._magnet_control_decisions(40.0, 0.0, 0.0) == []             # dead band, no change
    assert r._magnet_control_decisions(60.0, 0.0, 0.0) == []             # tilted, start hold
    assert r._magnet_control_decisions(60.0, 0.0, 0.4) == ['gripper:open']
    assert r._magnet_grip_closed is False


def test_gripper_brief_tilt_ignored():
    r = _reader()
    assert r._magnet_control_decisions(10.0, 0.0, 0.0) == []
    # Magnet leaves the close cone before the hold elapses.
    assert r._magnet_control_decisions(40.0, 0.0, 0.1) == []
    assert r._magnet_grip_closed is False
    assert r._magnet_grip_pending is None


def test_rotate_steps_on_twist():
    r = _reader()
    assert r._magnet_control_decisions(40.0, 0.0, 0.0) == []                # sets ref phi = 0
    assert r._magnet_control_decisions(40.0, 40.0, 0.1) == []               # 40 < 70
    assert r._magnet_control_decisions(40.0, 80.0, 0.2) == ['rotate:cw']    # +80 >= 70 (cw)
    assert r._magnet_control_decisions(40.0, 120.0, 0.3) == []              # +40 from new ref
    assert r._magnet_control_decisions(40.0, 0.0, 0.4) == ['rotate:ccw']    # -120 shortest way


def test_rotate_twist_wraps_shortest_way():
    r = _reader()
    r._magnet_twist_ref_phi = 170.0
    # 170 -> -160 is +30 the short way (not -330), below the step -> no step
    assert r._magnet_control_decisions(40.0, -160.0, 0.0) == []


def test_rotate_is_disabled_inside_upright_singularity():
    r = _reader()
    r._magnet_twist_ref_phi = 0.0
    assert r._magnet_control_decisions(5.0, 120.0, 0.0) == []
    assert r._magnet_twist_ref_phi is None
    # Leaving the cone establishes a fresh reference instead of rotating.
    assert r._magnet_control_decisions(10.0, 120.0, 0.1) == []
    assert r._magnet_twist_ref_phi == 120.0


def test_robot_twist_filter_smooths_raw_wrap_without_jump():
    r = _reader()
    first = r._filtered_robot_twist_phi(179.0, 40.0, 0.00)
    second = r._filtered_robot_twist_phi(-179.0, 40.0, 0.02)

    assert abs(first - 1.0) < 1e-9
    assert -1.0 < second < first


def test_robot_twist_filter_resets_inside_singular_cone():
    r = _reader()
    r._filtered_robot_twist_phi(60.0, 40.0, 0.00)
    reset = r._filtered_robot_twist_phi(-40.0, 5.0, 0.02)

    assert abs(reset + 40.0) < 1e-9
    assert r._magnet_robot_phi_filtered is None
    assert r._magnet_robot_phi_filter_at is None
    assert r._filtered_robot_twist_phi(30.0, 40.0, 0.04) == 30.0


def test_stationary_twist_noise_does_not_send_rotation_targets():
    r = _reader()
    commands = r._robot_twist_target_commands(0.0, 40.0, 0.00)
    for index, phi in enumerate((4.0, -4.0, 3.0, -3.0, 2.0, -2.0), 1):
        commands.extend(r._robot_twist_target_commands(
            phi, 40.0, index * 0.05))

    assert commands == ['rotate:reference']


def test_real_twist_uses_rate_limited_absolute_targets():
    r = _reader()
    assert r._robot_twist_target_commands(
        0.0, 40.0, 0.00) == ['rotate:reference']
    command = r._robot_twist_target_commands(30.0, 40.0, 0.10)

    assert command == ['rotate:set:3.00']
    assert not any(item in ('rotate:cw', 'rotate:ccw') for item in command)


def test_serial_magpilot_uses_full_flight_deck_aspect():
    class Axis:
        def __init__(self):
            self.box_aspect = 'unset'
            self.aspect = None

        def set_box_aspect(self, value):
            self.box_aspect = value

        def set_aspect(self, *args, **kwargs):
            self.aspect = (args, kwargs)

    r = _reader()
    r.input_source = 'serial'
    axis = Axis()

    r._set_canvas_aspect(axis, True)
    assert axis.box_aspect is None
    assert axis.aspect == (('auto',), {})

    r._set_canvas_aspect(axis, False)
    assert axis.aspect == (('equal',), {'adjustable': 'box'})


def test_magpilot_aircraft_remains_visible_at_edges():
    r = _reader()
    r.projection_extent = 0.05

    assert r._teleop_cursor_display_xy(-0.05, 0.05) == (-0.048, 0.048)
    assert r._teleop_cursor_display_xy(0.0, 0.0) == (0.0, 0.0)


def test_real_sample_updates_normalized_angles_and_height_without_ros():
    r = _reader()
    r.input_source = 'serial'
    r.ros_bridge = None
    r._last_real_theta = 0.0
    r._last_real_phi = 0.0
    r._last_real_height_m = 0.007

    r.update_magnet_teleop_controls(2.0, 0.0, 0.0, height_m=0.08)

    assert abs(r._last_real_theta - 90.0) < 1e-9
    assert abs(r._last_real_phi - 90.0) < 1e-9
    assert abs(r._last_real_height_m - 0.07) < 1e-9


def test_physical_pose_data_corrects_height_without_mutating_raw_pose():
    r = _reader()
    raw = [0.01, -0.02, 0.017, 0.0, 0.0, 1.0]

    physical = r.physical_pose_data(raw)

    assert raw[2] == 0.017
    assert math.isclose(physical[2], 0.007)


def test_magpilot_mode_latches_draw_enable_state():
    class Bridge:
        def __init__(self):
            self.commands = []
            self.enabled = []

        def publish_command(self, command):
            self.commands.append(command)

        def publish_draw_enable(self, enabled):
            self.enabled.append(enabled)

    r = _reader()
    r.ros_bridge = Bridge()
    r._set_key_feedback = lambda *args, **kwargs: None
    button = SimpleNamespace(name='MagPilot')

    r.handle_interface_event(SimpleNamespace(
        classifier_labels=None, command='robot:teleop', button=button))
    r.handle_interface_event(SimpleNamespace(
        classifier_labels=None, command='letter_detection', button=button))

    assert r.ros_bridge.commands == ['robot:teleop', 'letter_detection']
    assert r.ros_bridge.enabled == [True, False]


def test_real_visualizer_keeps_raw_phi_while_robot_uses_folded_phi():
    class Bridge:
        def publish_command(self, command):
            pass

    r = _reader()
    r.input_source = 'serial'
    r.ros_bridge = Bridge()
    r._last_real_theta = 0.0
    r._last_real_phi = 0.0
    r._last_real_height_m = 0.007
    theta = math.radians(40.0)
    phi = math.radians(-179.0)
    mx = math.sin(theta) * math.sin(phi)
    my = math.sin(theta) * math.cos(phi)
    mz = math.cos(theta)

    r.update_magnet_teleop_controls(mx, my, mz, height_m=0.08)

    assert abs(r._last_real_phi + 179.0) < 1e-9
    assert abs(r._magnet_robot_phi_filtered + 1.0) < 1e-9


if __name__ == '__main__':
    import traceback
    tests = [v for k, v in sorted(globals().items())
             if k.startswith('test_') and callable(v)]
    failed = 0
    for t in tests:
        try:
            t()
            print(f'PASS  {t.__name__}')
        except Exception:
            failed += 1
            print(f'FAIL  {t.__name__}')
            traceback.print_exc()
    print(f'\n{len(tests) - failed}/{len(tests)} passed')
    sys.exit(1 if failed else 0)
