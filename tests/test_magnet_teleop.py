#!/usr/bin/env python3
"""Unit tests for the magnet-orientation teleop control logic in
magnetometer_reader._magnet_control_decisions().

This is the real-sensor-only path (tilt theta -> gripper, twist phi -> control
layer) that cannot be exercised with the trackpad, so it is verified here in
isolation — no ROS, no hardware, no display.

Run:  python3 test_magnet_teleop.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ.setdefault('MPLBACKEND', 'Agg')

from magnetometer_reader import MagnetometerReader


def _reader():
    """A bare reader with just the fields _magnet_control_decisions needs."""
    r = MagnetometerReader.__new__(MagnetometerReader)
    r.magnet_gripper_close_deg = 55.0
    r.magnet_gripper_open_deg = 25.0
    r.magnet_gripper_hold_s = 0.30
    r.magnet_rotate_step_deg = 70.0
    r._magnet_grip_closed = False
    r._magnet_grip_pending = None
    r._magnet_twist_ref_phi = None
    return r


def test_gripper_requires_sustained_tilt():
    r = _reader()
    assert r._magnet_control_decisions(10.0, 0.0, 0.00) == []            # upright
    assert r._magnet_control_decisions(60.0, 0.0, 0.00) == []            # tilted, not held
    assert r._magnet_control_decisions(60.0, 0.0, 0.20) == []            # still holding
    assert r._magnet_control_decisions(60.0, 0.0, 0.35) == ['gripper:close']  # held >= 0.3s
    assert r._magnet_control_decisions(60.0, 0.0, 0.60) == []            # stays closed
    assert r._magnet_grip_closed is True


def test_gripper_hysteresis_open():
    r = _reader()
    r._magnet_grip_closed = True
    assert r._magnet_control_decisions(40.0, 0.0, 0.0) == []             # dead band, no change
    assert r._magnet_control_decisions(10.0, 0.0, 0.0) == []             # below open, start hold
    assert r._magnet_control_decisions(10.0, 0.0, 0.4) == ['gripper:open']
    assert r._magnet_grip_closed is False


def test_gripper_brief_tilt_ignored():
    r = _reader()
    assert r._magnet_control_decisions(60.0, 0.0, 0.0) == []
    # magnet returns upright before the hold elapses -> no command, no latch
    assert r._magnet_control_decisions(10.0, 0.0, 0.1) == []
    assert r._magnet_grip_closed is False
    assert r._magnet_grip_pending is None


def test_rotate_steps_on_twist():
    r = _reader()
    assert r._magnet_control_decisions(10.0, 0.0, 0.0) == []                # sets ref phi = 0
    assert r._magnet_control_decisions(10.0, 40.0, 0.1) == []               # 40 < 70
    assert r._magnet_control_decisions(10.0, 80.0, 0.2) == ['rotate:cw']    # +80 >= 70 (cw)
    assert r._magnet_control_decisions(10.0, 120.0, 0.3) == []              # +40 from new ref
    assert r._magnet_control_decisions(10.0, 0.0, 0.4) == ['rotate:ccw']    # -120 the short way (ccw)


def test_rotate_twist_wraps_shortest_way():
    r = _reader()
    r._magnet_twist_ref_phi = 170.0
    # 170 -> -160 is +30 the short way (not -330), below the step -> no step
    assert r._magnet_control_decisions(10.0, -160.0, 0.0) == []


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
