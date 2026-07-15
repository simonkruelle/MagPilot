#!/usr/bin/env python3
"""Tests for shared magnet orientation and height mappings."""

import math
import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

from colmag.control_mapping import (
    EE_HEIGHT_MAX_M,
    EE_HEIGHT_MIN_M,
    MAGNET_HEIGHT_MAX_M,
    MAGNET_HEIGHT_MIN_M,
    magnet_angles_degrees,
    map_magnet_height,
)


def test_orientation_vector_is_normalized():
    theta, phi = magnet_angles_degrees(2.0, 0.0, 0.0)
    assert math.isclose(theta, 90.0)
    assert math.isclose(phi, 90.0)


def test_height_endpoints_include_board_and_workspace_limits():
    assert math.isclose(
        map_magnet_height(MAGNET_HEIGHT_MIN_M), EE_HEIGHT_MIN_M)
    assert math.isclose(
        map_magnet_height(MAGNET_HEIGHT_MAX_M), EE_HEIGHT_MAX_M)


def test_height_gain_reduces_sensitivity_near_uncertain_top():
    low_delta = map_magnet_height(0.017) - map_magnet_height(0.007)
    high_delta = map_magnet_height(0.150) - map_magnet_height(0.140)
    assert low_delta > high_delta


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
