#!/usr/bin/env python3
"""Tests for launcher command construction."""

import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

import unittest
from unittest import mock

from colmag_launcher import (
    Launcher,
    build_clear_stale_launcher_state_command,
    build_detached_inner,
    build_interface_command,
    build_live_ros_nodes_command,
    build_pipeline_probe_command,
    build_ros_pipeline_shutdown_command,
    build_ros_node_shutdown_command,
    build_stage_probe_command,
    build_stop_all_command,
    controller_is_running,
    detect_robot_backend,
    format_serial_port_list,
    parse_franka_robot_mode,
    resolve_serial_port,
    serial_port_label,
    stop_conflicting_colmag_containers,
)


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
        self.assertNotIn('--enable-magnet-twist', command)

    def test_magnetometer_command_requires_serial_port(self):
        with self.assertRaisesRegex(ValueError, 'serial port'):
            build_interface_command('magnetometer')

    def test_numbered_port_selection_uses_detected_order(self):
        self.assertEqual(
            resolve_serial_port('2', ['/dev/ttyUSB0', '/dev/ttyACM0']),
            '/dev/ttyACM0')

    def test_port_selection_must_be_a_number(self):
        with self.assertRaisesRegex(ValueError, 'must be a number'):
            resolve_serial_port('/dev/ttyUSB3', ['/dev/ttyUSB3'])

    def test_invalid_port_number_lists_available_choices(self):
        with self.assertRaisesRegex(ValueError, '1: /dev/ttyUSB0'):
            resolve_serial_port('2', ['/host/dev/ttyUSB0'])

    def test_port_list_is_numbered_for_the_launcher_log(self):
        text = format_serial_port_list(
            ['/host/dev/ttyACM0', '/host/dev/ttyUSB0'])

        self.assertIn('1: /dev/ttyACM0', text)
        self.assertIn('2: /dev/ttyUSB0', text)
        self.assertIn('port # field', text)

    def test_host_device_mount_is_hidden_from_display_name(self):
        self.assertEqual(
            serial_port_label('/host/dev/ttyACM0'), '/dev/ttyACM0')


class ProcessLifecycleTests(unittest.TestCase):
    def test_detached_stage_owns_a_process_group_and_log(self):
        command = build_detached_inner('nodes', 'roslaunch example demo.launch')

        self.assertIn('setsid bash -lc', command)
        self.assertIn('/tmp/colmag_gui_nodes.pid', command)
        self.assertIn('/tmp/colmag_gui_nodes.log', command)
        self.assertIn('roslaunch example demo.launch', command)

    def test_detached_stage_rejects_unknown_tag(self):
        with self.assertRaisesRegex(ValueError, 'Unknown launcher stage'):
            build_detached_inner('anything', 'sleep 1')

    def test_stage_probe_validates_pid_owner(self):
        command = build_stage_probe_command('robot')

        self.assertIn('/tmp/colmag_gui_robot.pid', command)
        self.assertIn('ps -o args=', command)
        self.assertIn('[f]ranka_control_node', command)
        self.assertIn('echo running', command)

    def test_stop_all_handles_managed_and_legacy_processes(self):
        command = build_stop_all_command()

        for tag in ('robot', 'nodes', 'interface', 'window'):
            self.assertIn('/tmp/colmag_gui_{}.pid'.format(tag), command)
        self.assertIn("'[r]oslaunch'", command)
        self.assertNotIn("pkill -TERM -f 'roslaunch'", command)
        self.assertNotIn('rosnode kill', command)
        self.assertIn('sleep 1', command)
        self.assertIn('sleep 2', command)
        self.assertIn('pkill -KILL', command)

    def test_pipeline_probe_covers_robot_nodes_and_cannot_match_itself(self):
        command = build_pipeline_probe_command()

        self.assertIn('[f]ranka_control_node', command)
        self.assertIn('[c]olmag_draw_node.py', command)
        self.assertIn('[m]agnetometer_reader.py', command)
        self.assertIn('/tmp/colmag_gui_robot.pid', command)
        self.assertNotIn("'roslaunch'", command)

    def test_startup_clear_removes_only_launcher_pid_and_log_files(self):
        command = build_clear_stale_launcher_state_command()

        self.assertIn('/tmp/colmag_gui_robot.log', command)
        self.assertIn('/tmp/colmag_gui_nodes.pid', command)
        self.assertNotIn('*', command)

    @mock.patch('colmag_launcher.sh')
    def test_conflicting_host_network_containers_are_stopped(self, run):
        run.side_effect = [
            (True, 'colmag_simon|colmag_ros:noetic\n'
                   'colmag_ros|old-image\n'
                   'test_box|colmag_ros:noetic\n'
                   'database|postgres:latest'),
            (True, ''),
            (True, ''),
        ]

        ok, stopped = stop_conflicting_colmag_containers()

        self.assertTrue(ok)
        self.assertEqual(stopped, 'colmag_ros, test_box')
        self.assertIn(
            'docker ps --format "{{.Names}}|{{.Image}}"',
            run.call_args_list[0].args[0])
        self.assertIn('docker stop -t 5 colmag_ros', run.call_args_list[1].args[0])
        self.assertIn('docker stop -t 5 test_box', run.call_args_list[2].args[0])

    def test_remote_ros_shutdown_is_scoped_to_named_nodes(self):
        command = build_ros_node_shutdown_command(
            ('/colmag_draw_node', '/gazebo'))

        self.assertIn('/colmag_draw_node', command)
        self.assertIn('/gazebo', command)
        self.assertIn('rosnode kill "$node"', command)

    def test_remote_shutdown_precedes_robot_backend(self):
        command = build_ros_pipeline_shutdown_command()

        self.assertIn('/colmag_draw_node', command)
        self.assertIn('/gazebo', command)
        self.assertIn('/franka_control', command)
        self.assertLess(
            command.index('/colmag_draw_node'), command.index('/franka_control'))


class RobotReadinessTests(unittest.TestCase):
    def test_backend_status_requires_live_node_ping(self):
        command = build_live_ros_nodes_command()

        self.assertIn('rosnode ping -c 1', command)
        self.assertIn('/gazebo', command)
        self.assertIn('/franka_control', command)
        self.assertNotIn('rosnode cleanup', command)

    def test_backend_detection(self):
        self.assertEqual(
            detect_robot_backend('/rosout\n/franka_control\n'), 'real')
        self.assertEqual(detect_robot_backend('/gazebo\n/rosout\n'), 'sim')
        self.assertIsNone(detect_robot_backend('/rosout\n'))
        self.assertEqual(
            detect_robot_backend('/gazebo\n/franka_control\n'), 'conflict')

    def test_controller_must_have_running_state(self):
        response = """
controller:
  -
    name: "position_joint_trajectory_controller"
    state: "running"
    type: "position_controllers/JointTrajectoryController"
  -
    name: "effort_joint_trajectory_controller"
    state: "stopped"
"""
        self.assertTrue(controller_is_running(
            response, 'position_joint_trajectory_controller'))
        self.assertFalse(controller_is_running(
            response, 'effort_joint_trajectory_controller'))
        self.assertFalse(controller_is_running(response, 'missing_controller'))

    def test_franka_robot_mode_parsing(self):
        self.assertEqual(parse_franka_robot_mode('4\n---\n'), 4)
        self.assertEqual(parse_franka_robot_mode('robot_mode: 1\n'), 1)
        self.assertIsNone(parse_franka_robot_mode('robot_mode: unknown\n'))


class LauncherCleanupHookTests(unittest.TestCase):
    def test_startup_requests_pipeline_cleanup(self):
        launcher = Launcher.__new__(Launcher)
        launcher._begin_pipeline_cleanup = mock.Mock()

        launcher._cleanup_on_startup()

        launcher._begin_pipeline_cleanup.assert_called_once_with('startup')

    def test_failed_cleanup_does_not_block_pipeline_actions(self):
        # A cleanup that could not prove the pipeline idle must never brick the
        # Control Center: survivors it cannot reach (a leftover host-network
        # container, or a gazebo/franka_control ignoring rosnode kill) would
        # otherwise disable every Start button permanently. Only an in-progress
        # cleanup may gate a start.
        launcher = Launcher.__new__(Launcher)
        launcher._stopping = False
        launcher._cleanup_error = 'live ROS nodes remain: /gazebo'

        self.assertTrue(launcher._pipeline_action_ready())

    def test_window_close_stops_pipeline_before_destroy(self):
        launcher = Launcher.__new__(Launcher)
        launcher._closing = False
        launcher._poll_running = True
        launcher._begin_pipeline_cleanup = mock.Mock()

        launcher._on_close()

        self.assertTrue(launcher._closing)
        self.assertFalse(launcher._poll_running)
        launcher._begin_pipeline_cleanup.assert_called_once_with('close')


if __name__ == '__main__':
    unittest.main()
