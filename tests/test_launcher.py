#!/usr/bin/env python3
"""Tests for launcher command construction."""

import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

import unittest

from colmag_launcher import build_interface_command, resolve_serial_port


class InterfaceCommandTests(unittest.TestCase):
    def test_trackpad_command_does_not_include_serial_port(self):
        command = build_interface_command('trackpad', '/dev/ttyUSB0')

        self.assertIn('--input-source trackpad', command)
        self.assertNotIn('--port', command)

    def test_magnetometer_command_includes_quoted_serial_port(self):
        command = build_interface_command(
            'magnetometer', '/dev/serial/by-id/my sensor')

        self.assertIn('--input-source serial', command)
        self.assertIn("--port '/dev/serial/by-id/my sensor'", command)
        self.assertIn('--clean --writing-max-z 0.05', command)

    def test_magnetometer_command_requires_serial_port(self):
        with self.assertRaisesRegex(ValueError, 'serial port'):
            build_interface_command('magnetometer')

    def test_numbered_port_selection_uses_detected_order(self):
        self.assertEqual(
            resolve_serial_port('2', ['/dev/ttyUSB0', '/dev/ttyACM0']),
            '/dev/ttyACM0')

    def test_device_path_is_accepted_directly(self):
        self.assertEqual(
            resolve_serial_port('/dev/ttyUSB3', []), '/dev/ttyUSB3')

    def test_invalid_port_number_lists_available_choices(self):
        with self.assertRaisesRegex(ValueError, '1: /dev/ttyUSB0'):
            resolve_serial_port('2', ['/dev/ttyUSB0'])


if __name__ == '__main__':
    unittest.main()
