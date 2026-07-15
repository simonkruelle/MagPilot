#!/usr/bin/env python3
"""Tests for the digit cube geometry and FR3/Panda reachability."""

import os as _os
import sys as _sys

import numpy as np

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)
_SCRIPTS = _os.path.join(_ROOT, 'ros', 'colmag_ros', 'scripts')
if _SCRIPTS not in _sys.path:
    _sys.path.insert(0, _SCRIPTS)

from colmag.robot_targets import (  # noqa: E402
    DIGIT_CUBE_CENTER_M,
    DIGIT_CUBE_EDGE_M,
    digit_cube_target,
)
from colmag_robot_node import (  # noqa: E402
    HOME_POSE,
    _Q_MAX,
    _Q_MIN,
    _fk,
    _solve_ik,
)


def test_digits_one_through_eight_are_all_cube_corners():
    center = np.array(DIGIT_CUBE_CENTER_M)
    half = 0.5 * DIGIT_CUBE_EDGE_M
    targets = [digit_cube_target(digit) for digit in range(1, 9)]

    assert len(set(targets)) == 8
    for target in targets:
        assert np.allclose(np.abs(np.array(target) - center), half)


def test_digit_nine_is_cube_center():
    assert digit_cube_target(9) == DIGIT_CUBE_CENTER_M


def test_default_cube_is_reachable_with_shared_joint_limits():
    home = np.array(HOME_POSE, dtype=float)
    orientation = _fk(home)[:3, :3]
    for digit in range(1, 10):
        target = np.array(digit_cube_target(digit))
        joints, error = _solve_ik(target, orientation, home)
        assert error < 0.001
        assert np.all(joints >= _Q_MIN)
        assert np.all(joints <= _Q_MAX)


if __name__ == '__main__':
    import traceback
    tests = [value for name, value in sorted(globals().items())
             if name.startswith('test_') and callable(value)]
    failed = 0
    for test in tests:
        try:
            test()
            print('PASS ', test.__name__)
        except Exception:
            failed += 1
            print('FAIL ', test.__name__)
            traceback.print_exc()
    print('\n{}/{} passed'.format(len(tests) - failed, len(tests)))
    _sys.exit(1 if failed else 0)
