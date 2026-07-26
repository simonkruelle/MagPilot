#!/usr/bin/env python3
"""Tests for the digit cube geometry and FR3/Panda reachability."""

import os as _os
import sys as _sys
import tempfile
from unittest import mock

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
from colmag.action_mapping import (  # noqa: E402
    ReloadableActionMapping,
    default_action_mapping,
    save_action_mapping,
)
import colmag_robot_node as robot_module  # noqa: E402
from colmag_robot_node import (  # noqa: E402
    ColmagRobotNode,
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


def test_robot_dispatch_reloads_only_valid_action_maps():
    calls = []
    with tempfile.TemporaryDirectory() as directory:
        path = _os.path.join(directory, 'robot_actions.json')
        mapping = default_action_mapping()
        save_action_mapping(path, mapping)

        node = ColmagRobotNode.__new__(ColmagRobotNode)
        node._action_mapping = ReloadableActionMapping(path)
        node._reported_action_map_error = None
        node._action_registry = {
            'wave': lambda: calls.append('wave'),
            'cheer': lambda: calls.append('cheer'),
            'nod_yes': lambda: calls.append('nod_yes'),
        }

        with mock.patch.object(robot_module.rospy, 'loginfo'), \
                mock.patch.object(robot_module.rospy, 'logwarn'):
            node.execute_command('A')

            mapping['A'] = 'cheer'
            save_action_mapping(path, mapping)
            node.execute_command('A')

            with open(path, 'w', encoding='utf-8') as handle:
                handle.write('{"schema_version": 1, "bindings": {}}')
            node.execute_command('A')

    assert calls == ['wave', 'cheer', 'cheer']


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
