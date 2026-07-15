#!/usr/bin/env python3
"""Unit tests for the magnet-orientation teleop control logic in
magnetometer_reader._magnet_control_decisions().

This is the real-sensor-only path (tilt theta -> gripper, twist phi -> wrist)
that cannot be exercised with the trackpad, so it is verified here in
isolation — no ROS, no hardware, no display.

Run:  python3 test_magnet_teleop.py
"""

import os as _os, sys as _sys
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
    r._magnet_grip_closed = False
    r._magnet_grip_pending = None
    r._magnet_twist_ref_phi = None
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
    assert abs(r._last_real_height_m - 0.08) < 1e-9


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
