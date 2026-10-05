#!/usr/bin/env python3
"""Tests for launcher command construction."""

import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

import json
import shlex
import tempfile
import unittest
from unittest import mock

from colmag_launcher import (
    DataPanel,
    Launcher,
    build_clear_stale_launcher_state_command,
    build_detached_inner,
    build_interface_command,
    build_record_command,
    build_terminal_argv,
    build_tracking_command,
    build_teleoperation_argv,
    build_local_stage_status_command,
    build_live_ros_nodes_command,
    build_pipeline_probe_command,
    build_ros_pipeline_shutdown_command,
    build_ros_package_probe_command,
    build_ros_node_shutdown_command,
    build_stage_probe_command,
    build_stop_all_command,
    controller_is_running,
    detect_robot_backend,
    format_serial_port_list,
    missing_container_ros_packages,
    parse_local_stage_status,
    parse_franka_robot_mode,
    parse_tracking_settings,
    parse_teleoperation_settings,
    next_teleoperation_session,
    teleoperation_python,
    probe_container_project_mount,
    resolve_serial_port,
    serial_port_label,
    stop_conflicting_colmag_containers,
    validate_project_mount,
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

    def test_ros_package_probe_reports_each_missing_dependency(self):
        command = build_ros_package_probe_command(
            ('gazebo_ros', 'franka_gazebo'))

        self.assertIn('rospack find "$package"', command)
        self.assertIn('gazebo_ros franka_gazebo', command)
        self.assertIn('echo "$package"', command)

    @mock.patch('colmag_launcher.in_container')
    def test_missing_ros_packages_are_parsed(self, run):
        run.return_value = (True, 'gazebo_ros\nfranka_gazebo\n')

        missing = missing_container_ros_packages(
            ('gazebo_ros', 'franka_gazebo'))

        self.assertEqual(missing, ('gazebo_ros', 'franka_gazebo'))

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

    def test_status_probe_uses_local_processes_not_shared_ros_nodes(self):
        command = build_local_stage_status_command()

        self.assertIn("[f]ranka_control_node", command)
        self.assertIn('pgrep -x gzserver', command)
        self.assertIn("[c]olmag_draw_node.py", command)
        self.assertIn("[c]olmag_robot_node.py", command)
        self.assertNotIn('rosnode', command)

    def test_local_status_parser_distinguishes_backend_states(self):
        self.assertEqual(
            parse_local_stage_status('robot:real\nnodes:up\n'),
            (True, True))
        self.assertEqual(
            parse_local_stage_status('robot:sim-nowin\n'),
            ('nowin', False))
        self.assertEqual(
            parse_local_stage_status('robot:conflict\n'),
            ('conflict', False))
        self.assertEqual(parse_local_stage_status(''), (False, False))

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


class ProjectMountTests(unittest.TestCase):
    def test_current_checkout_mount_accepts_symlink_aliases(self):
        with tempfile.TemporaryDirectory() as temporary_dir:
            checkout = _os.path.join(temporary_dir, 'MagPilot')
            alias = _os.path.join(temporary_dir, 'workspace alias')
            _os.mkdir(checkout)
            _os.symlink(checkout, alias)
            metadata = json.dumps([{
                'Type': 'bind', 'Destination': '/colmag', 'Source': alias,
            }])

            self.assertEqual(validate_project_mount(metadata, checkout), (True, ''))
            self.assertEqual(validate_project_mount(metadata, alias), (True, ''))

    def test_other_checkout_reports_actual_and_expected_source(self):
        metadata = json.dumps([{
            'Type': 'bind', 'Destination': '/colmag', 'Source': '/tmp/old-colmag',
        }])

        ok, error = validate_project_mount(metadata, '/tmp/current-magpilot')

        self.assertFalse(ok)
        self.assertIn('/tmp/old-colmag', error)
        self.assertIn('/tmp/current-magpilot', error)

    def test_missing_project_mount_is_rejected(self):
        ok, error = validate_project_mount(json.dumps([{
            'Type': 'bind', 'Destination': '/catkin_ws', 'Source': '/tmp/catkin',
        }]), '/tmp/current-magpilot')

        self.assertFalse(ok)
        self.assertIn('no project bind mount at /colmag', error)

    def test_malformed_mount_metadata_is_rejected_clearly(self):
        for metadata in ('not json', '{}', '[null]', json.dumps([{
                'Type': 'bind', 'Destination': '/colmag', 'Source': ['wrong'],
        }])):
            with self.subTest(metadata=metadata):
                ok, error = validate_project_mount(metadata, '/tmp/current-magpilot')
                self.assertFalse(ok)
                self.assertTrue(error)

    def test_named_volume_is_not_a_workspace_bind(self):
        ok, error = validate_project_mount(json.dumps([{
            'Type': 'volume', 'Destination': '/colmag', 'Source': '/tmp/current-magpilot',
        }]), '/tmp/current-magpilot')

        self.assertFalse(ok)
        self.assertIn('must be a bind mount', error)

    @mock.patch('colmag_launcher.sh', return_value=(False, 'Docker permission denied'))
    def test_failed_docker_inspection_is_rejected(self, inspect):
        ok, error = probe_container_project_mount('/tmp/current-magpilot')

        self.assertFalse(ok)
        self.assertIn('Could not inspect', error)
        self.assertIn('Docker permission denied', error)
        self.assertIn("docker inspect --format '{{json .Mounts}}' colmag_simon",
                      inspect.call_args.args[0])

    @mock.patch('colmag_launcher.messagebox.showerror')
    @mock.patch('colmag_launcher.os.makedirs')
    @mock.patch('colmag_launcher.dataset.next_session_id')
    @mock.patch('colmag_launcher.sh')
    def test_wrong_mount_blocks_demo_before_creating_a_session(
            self, run, next_session, mkdir, showerror):
        run.side_effect = [(True, ''), (True, json.dumps([{
            'Type': 'bind', 'Destination': '/colmag', 'Source': '/tmp/old-colmag',
        }]))]
        launcher = Launcher.__new__(Launcher)
        launcher._pipeline_action_ready = mock.Mock(return_value=True)
        launcher._launch_stage = mock.Mock()

        launcher.start_character_recording('P01', 0, 'Lab meeting', demo=True)

        next_session.assert_not_called()
        mkdir.assert_not_called()
        launcher._launch_stage.assert_not_called()
        message = showerror.call_args.args[1]
        self.assertIn('/tmp/old-colmag', message)
        self.assertIn('cd ', message)
        self.assertIn('COLMAG_SKIP_BUILD=1 bash ros/docker_setup.sh', message)

    @mock.patch('colmag_launcher.REPO_DIR', '/tmp/current-magpilot')
    @mock.patch('colmag_launcher.sh')
    def test_stopped_container_mount_is_checked_after_start(self, run):
        run.side_effect = [(False, ''), (True, ''), (True, json.dumps([{
            'Type': 'bind', 'Destination': '/colmag', 'Source': '/tmp/current-magpilot',
        }]))]
        launcher = Launcher.__new__(Launcher)

        self.assertTrue(launcher._ensure_container())

        self.assertEqual(run.call_args_list[1].args[0], 'docker start colmag_simon')
        self.assertIn('docker inspect', run.call_args_list[2].args[0])


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


class LauncherSimulationPreflightTests(unittest.TestCase):
    @mock.patch('colmag_launcher.messagebox.showerror')
    @mock.patch(
        'colmag_launcher.missing_container_ros_packages',
        return_value=('gazebo_ros', 'franka_gazebo'))
    @mock.patch('colmag_launcher.in_container', return_value=(True, ''))
    def test_missing_gazebo_stops_before_roslaunch(
            self, _run, _missing, showerror):
        launcher = Launcher.__new__(Launcher)
        launcher._pipeline_action_ready = mock.Mock(return_value=True)
        launcher._ensure_container = mock.Mock(return_value=True)
        launcher.mode = mock.Mock()
        launcher.mode.get.return_value = 'sim'
        launcher._select_stage_log = mock.Mock()
        launcher._launch_stage = mock.Mock()

        launcher.start_robot()

        launcher._launch_stage.assert_not_called()
        launcher._select_stage_log.assert_called_once()
        message = showerror.call_args.args[1]
        self.assertIn('gazebo_ros, franka_gazebo', message)
        self.assertIn(
            'INSTALL_GAZEBO=1 bash ros/docker_setup.sh', message)
        self.assertIn('Do not use COLMAG_SKIP_BUILD=1', message)

    def test_setup_builds_simulation_stack_by_default(self):
        setup_path = _os.path.join(_ROOT, 'ros', 'docker_setup.sh')
        with open(setup_path, 'r', encoding='utf-8') as handle:
            setup = handle.read()

        self.assertIn(
            'INSTALL_GAZEBO="${INSTALL_GAZEBO:-1}"', setup)


class DataCollectionCommandTests(unittest.TestCase):
    def test_record_command_tags_the_participant_and_never_uses_ros(self):
        command = build_record_command('/host/dev/ttyACM0', 'P03', 'S01', 0)

        self.assertTrue(command.startswith('cd /colmag && python3 magnetometer_reader.py'))
        self.assertIn('--input-source serial --port /host/dev/ttyACM0', command)
        self.assertIn('--record-data', command)
        self.assertIn('--no-classifier', command)
        self.assertIn('--output-dir data_collection/characters/P03/S01', command)
        self.assertIn('--participant-id P03 --session-id S01 --height-mm 0', command)
        self.assertIn('--target-reps 10', command)
        self.assertIn('--writing-max-z 0.05', command)
        self.assertNotIn('--ros', command)
        self.assertNotIn('--classifier-labels', command)
        self.assertNotIn('--experiment-name', command)

    def test_experiment_name_is_one_quoted_metadata_argument(self):
        experiment_name = "Pen pilot: Simon's $(touch /tmp/never) `echo nope`; v2"
        command = build_record_command(
            '/host/dev/ttyACM0', 'P03', 'S01', 0, experiment_name)
        args = shlex.split(command.partition(' && ')[2])

        self.assertEqual(args[args.index('--experiment-name') + 1], experiment_name)
        self.assertEqual(args[args.index('--participant-id') + 1], 'P03')
        self.assertEqual(args[args.index('--output-dir') + 1],
                         'data_collection/characters/P03/S01')

    def test_blank_experiment_name_keeps_existing_command_shape(self):
        self.assertEqual(
            build_record_command('/dev/ttyACM0', 'P01', 'S01', 0, '  '),
            build_record_command('/dev/ttyACM0', 'P01', 'S01', 0))

    def test_demo_records_without_serial_or_ros_into_a_separate_folder(self):
        command = build_record_command('', 'P03', 'S01', 0, 'Lab demo', demo=True)
        args = shlex.split(command.partition(' && ')[2])

        self.assertEqual(args[args.index('--input-source') + 1], 'trackpad')
        self.assertEqual(args[args.index('--touchpad-ink-mode') + 1], 'pen')
        self.assertEqual(args[args.index('--writing-min-velocity') + 1], '0')
        self.assertEqual(args[args.index('--output-dir') + 1],
                         'data_collection/demo/characters/P03/S01')
        self.assertIn('--record-data', args)
        self.assertIn('--no-classifier', args)
        self.assertIn('--clean', args)
        self.assertNotIn('--port', args)
        self.assertNotIn('--ros', args)
        self.assertNotIn('--writing-max-z', args)

    def test_raised_heights_skip_the_pen_up_filter(self):
        command = build_record_command('/host/dev/ttyACM0', 'P01', 'S02', 50)

        self.assertIn('--height-mm 50', command)
        self.assertNotIn('--writing-max-z', command)

    def test_record_command_rejects_bad_ids_and_missing_port(self):
        with self.assertRaisesRegex(ValueError, 'serial port'):
            build_record_command('', 'P01', 'S01', 0)
        with self.assertRaisesRegex(ValueError, 'Participant'):
            build_record_command('/dev/ttyACM0', 'Simon', 'S01', 0)
        with self.assertRaisesRegex(ValueError, 'Session'):
            build_record_command('/dev/ttyACM0', 'P01', '../x', 0)

    def test_tracking_settings_are_checked(self):
        self.assertEqual(
            parse_tracking_settings(' 12x12mm_stack ', '6', '10, 50,100'),
            ('12x12mm_stack', 6.0, [10.0, 50.0, 100.0]))
        for magnet, offset, heights in (('', '0', '10'), ('m', 'x', '10'),
                                        ('m', '0', '10,abc'), ('m', '0', ''),
                                        ('m', '-1', '10')):
            with self.assertRaises(ValueError):
                parse_tracking_settings(magnet, offset, heights)

    def test_tracking_command_writes_into_data_collection(self):
        command = build_tracking_command(
            '/host/dev/ttyACM0', '12x12mm_stack', 6.0, [10.0, 50.0], 'run_x')

        self.assertIn('python3 tools/record_tracking_error.py', command)
        self.assertIn('--magnet 12x12mm_stack --magnet-offset-mm 6', command)
        self.assertIn('--heights-mm 10,50 --run-id run_x', command)
        self.assertIn('--output-dir data_collection/tracking_error/run_x', command)

    def test_terminal_runs_docker_interactively_and_stays_open(self):
        argv = build_terminal_argv('gnome-terminal', 'cd /colmag && echo hi')

        self.assertEqual(argv[:4], ['gnome-terminal', '--', 'bash', '-lc'])
        self.assertIn('docker exec -it colmag_simon bash -lc', argv[4])
        self.assertIn("'cd /colmag && echo hi'", argv[4])
        self.assertIn('Press Enter to close', argv[4])
        self.assertEqual(build_terminal_argv('xterm', 'x')[:2], ['xterm', '-e'])

    def test_stop_all_also_ends_the_tracking_error_recorder(self):
        self.assertIn("'[r]ecord_tracking_error.py'", build_stop_all_command())


class DataCollectionRoutingTests(unittest.TestCase):
    @mock.patch('colmag_launcher.os.makedirs')
    @mock.patch('colmag_launcher.dataset.load_participants', return_value=[])
    @mock.patch('colmag_launcher.dataset.next_session_id', return_value='S01')
    @mock.patch('colmag_launcher.messagebox.askokcancel', return_value=True)
    def test_demo_start_skips_sensor_checks_and_uses_demo_session_root(
            self, confirm, next_session, _participants, mkdir):
        launcher = Launcher.__new__(Launcher)
        launcher._pipeline_action_ready = mock.Mock(return_value=True)
        launcher._ensure_container = mock.Mock(return_value=True)
        launcher._sensor_in_use = mock.Mock()
        launcher._checked_sensor_port = mock.Mock()
        launcher._launch_stage = mock.Mock(return_value=True)

        launcher.start_character_recording('P01', 0, 'Lab meeting', demo=True)

        launcher._sensor_in_use.assert_not_called()
        launcher._checked_sensor_port.assert_not_called()
        self.assertTrue(next_session.call_args.args[0].endswith('/data_collection/demo'))
        self.assertEqual(next_session.call_args.args[1], 'P01')
        self.assertIn('DEMO', confirm.call_args.args[0])
        self.assertIn('Lab meeting', confirm.call_args.args[1])
        self.assertIn('/data_collection/demo/characters/P01/S01', mkdir.call_args.args[0])
        stage, command = launcher._launch_stage.call_args.args
        self.assertEqual(stage, 'interface')
        self.assertIn('--input-source trackpad', command)

    @mock.patch('colmag_launcher.messagebox.showwarning')
    @mock.patch('colmag_launcher.in_container_detached')
    @mock.patch('colmag_launcher.in_container', return_value=(True, 'running'))
    def test_demo_cannot_replace_an_active_interface_stage(
            self, _probe, start_detached, showwarning):
        launcher = Launcher.__new__(Launcher)
        launcher._select_stage_log = mock.Mock()
        command = build_record_command('', 'P01', 'S01', 0, demo=True)

        self.assertFalse(launcher._launch_stage('interface', command))

        start_detached.assert_not_called()
        showwarning.assert_called_once()

    @mock.patch('colmag_launcher.messagebox.showwarning')
    def test_demo_bypasses_consent_but_sensor_recording_requires_it(self, warn):
        panel = DataPanel.__new__(DataPanel)
        panel.participants = [{'participant_id': 'P01', 'consent': False}]
        panel.source = mock.Mock()
        panel.source.get.return_value = 'touchpad'
        panel.height = mock.Mock()
        panel.height.get.return_value = '0'
        panel.experiment_name = mock.Mock()
        panel.experiment_name.get.return_value = 'Lab meeting'
        panel._save_experiment = mock.Mock(return_value=True)
        panel.parent = mock.Mock()

        panel.start_recording('P01')

        panel.parent.start_character_recording.assert_called_once_with(
            'P01', 0, 'Lab meeting', demo=True)
        warn.assert_not_called()
        panel.parent.start_character_recording.reset_mock()
        panel._save_experiment.reset_mock()
        panel.source.get.return_value = 'serial'

        panel.start_recording('P01')

        panel.parent.start_character_recording.assert_not_called()
        panel._save_experiment.assert_not_called()
        warn.assert_called_once()


class TeleoperationCollectionTests(unittest.TestCase):
    def test_host_argv_preserves_literal_names_and_separate_output(self):
        experiment = "Simon's $(touch /tmp/never) `echo nope`; study"
        args = build_teleoperation_argv('/tmp/env/python', 'P02', 'S03',
                    participant_name='Ada Lovelace', experiment_name=experiment)
        self.assertEqual(args[0], '/tmp/env/python')
        self.assertTrue(args[1].endswith('/tools/teleoperation_demo.py'))
        self.assertEqual(args[args.index('--experiment-name') + 1], experiment)
        self.assertEqual(args[args.index('--participant-name') + 1], 'Ada Lovelace')
        self.assertEqual(args[args.index('--output-dir') + 1], 'data_collection/teleoperation/P02/S03')
        self.assertEqual(args[args.index('--input-source') + 1], 'trackpad')
        self.assertEqual(args[args.index('--dwell-seconds') + 1], '2')
        self.assertNotIn('--port', args)
        self.assertNotIn('--ros', args)

    def test_board_argv_keeps_docker_port_and_magnet_count(self):
        args = build_teleoperation_argv('/tmp/env/python', 'P01', 'S01', 'serial',
                    serial_port='/host/dev/ttyUSB0', magnet_count=3)
        self.assertEqual(args[args.index('--port') + 1], '/host/dev/ttyUSB0')
        self.assertEqual(args[args.index('--baudrate') + 1], '921600')
        self.assertEqual(args[args.index('--magnet-count') + 1], '3')
        with self.assertRaisesRegex(ValueError, 'serial port'):
            build_teleoperation_argv('python', 'P01', 'S01', 'serial')

    def test_invalid_settings_and_unsafe_ids_are_rejected(self):
        settings = parse_teleoperation_settings('0', '10', '25', '2', 'Not specified')
        self.assertEqual(settings['magnet_count'], None)
        self.assertEqual(parse_teleoperation_settings('0', '10', '50', '2')['tolerance_mm'], 50)
        for values in [('x', '10', '25', '2', '1'), ('0', '0', '25', '2', '1'),
                       ('0', '10', 'nan', '2', '1'), ('0', '10', '25', 'inf', '1'),
                       ('0', '10', '120', '2', '1'), ('0', '10', '100', '2', '1'),
                       ('0', '10', '25', '2', '4')]:
            with self.subTest(values=values), self.assertRaises(ValueError):
                parse_teleoperation_settings(*values)
        for participant, session in [('../P01', 'S01'), ('P01', '../S01')]:
            with self.assertRaises(ValueError):
                build_teleoperation_argv('python', participant, session)

    def test_teleoperation_session_numbering_is_independent_of_characters(self):
        with tempfile.TemporaryDirectory() as folder:
            _os.makedirs(_os.path.join(folder, 'characters', 'P01', 'S99'))
            self.assertEqual(next_teleoperation_session(folder, 'P01'), 'S01')
            _os.makedirs(_os.path.join(folder, 'teleoperation', 'P01', 'S03'))
            _os.makedirs(_os.path.join(folder, 'teleoperation', 'P01', 'scratch'))
            self.assertEqual(next_teleoperation_session(folder, 'P01'), 'S04')

    @mock.patch.dict('colmag_launcher.os.environ', {'COLMAG_TELEOP_PYTHON': '/tmp/custom env/python'})
    def test_host_interpreter_override_is_literal(self):
        self.assertEqual(teleoperation_python(), '/tmp/custom env/python')

    @mock.patch('colmag_launcher.probe_teleoperation_runtime', return_value=(True, '3.3.7'))
    @mock.patch('colmag_launcher.subprocess.Popen')
    def test_trackpad_start_needs_no_container_and_does_not_precreate_session(self, popen, _probe):
        with tempfile.TemporaryDirectory() as folder, mock.patch('colmag_launcher.DATA_DIR', folder):
            launcher = Launcher.__new__(Launcher)
            launcher._pipeline_action_ready = mock.Mock(return_value=True)
            launcher._ensure_container = mock.Mock()
            launcher._sensor_in_use = mock.Mock()
            launcher._checked_sensor_port = mock.Mock()
            launcher._replace_log = mock.Mock()
            launcher.after = mock.Mock()
            self.assertTrue(launcher.start_teleoperation('P01', participant_name='Ada'))
            launcher._ensure_container.assert_not_called()
            launcher._sensor_in_use.assert_not_called()
            launcher._checked_sensor_port.assert_not_called()
            self.assertFalse(_os.path.exists(_os.path.join(folder, 'teleoperation', 'P01', 'S01')))
            self.assertFalse(popen.call_args.kwargs.get('shell', False))
            self.assertTrue(popen.call_args.kwargs['start_new_session'])
            launcher._teleoperation_log_file.close()

    @mock.patch('colmag_launcher.probe_teleoperation_runtime')
    def test_board_start_is_blocked_before_runtime_when_sensor_busy(self, probe):
        launcher = Launcher.__new__(Launcher)
        launcher._pipeline_action_ready = mock.Mock(return_value=True)
        launcher._ensure_container = mock.Mock(return_value=True)
        launcher._sensor_in_use = mock.Mock(return_value='interface')
        launcher._checked_sensor_port = mock.Mock()
        launcher._warn_sensor_busy = mock.Mock()
        self.assertFalse(launcher.start_teleoperation('P01', input_source='serial'))
        launcher._warn_sensor_busy.assert_called_once_with('interface')
        launcher._checked_sensor_port.assert_not_called()
        probe.assert_not_called()

    @mock.patch('colmag_launcher.in_container', return_value=(True, '321 python3 /colmag/tools/teleoperation_serial_stream.py'))
    def test_board_bridge_is_detected_and_included_in_cleanup(self, run):
        launcher = Launcher.__new__(Launcher)
        self.assertEqual(launcher._sensor_in_use(), 'teleoperation')
        self.assertIn('[t]eleoperation_serial_stream.py', run.call_args.args[0])
        self.assertIn('[t]eleoperation_serial_stream.py', build_stop_all_command())

    @mock.patch('colmag_launcher.os.killpg')
    def test_stop_targets_only_the_owned_host_process_group(self, kill):
        launcher = Launcher.__new__(Launcher)
        process = mock.Mock(pid=1234)
        process.poll.return_value = None
        launcher._teleoperation_process = process
        launcher._stop_teleoperation_worker()
        self.assertEqual(kill.call_args.args[0], 1234)
        process.wait.assert_called_once_with(timeout=4)

    @mock.patch('colmag_launcher.messagebox.showwarning')
    def test_second_start_cannot_replace_running_simulation(self, warn):
        launcher = Launcher.__new__(Launcher)
        launcher._pipeline_action_ready = mock.Mock(return_value=True)
        launcher._ensure_container = mock.Mock()
        process = mock.Mock()
        process.poll.return_value = None
        launcher._teleoperation_process = process
        self.assertFalse(launcher.start_teleoperation('P01'))
        launcher._ensure_container.assert_not_called()
        warn.assert_called_once()

    @mock.patch('colmag_launcher.messagebox.showerror')
    def test_old_watch_callback_cannot_watch_a_new_process_or_repeat_errors(self, error):
        launcher = Launcher.__new__(Launcher)
        launcher._closing = False
        launcher._stopping = False
        launcher.after = mock.Mock()
        old = mock.Mock()
        current = mock.Mock(returncode=1)
        current.poll.return_value = 1
        launcher._teleoperation_process = current
        launcher._watch_teleoperation(old)
        old.poll.assert_not_called()
        launcher.after.assert_not_called()
        error.assert_not_called()
        launcher._watch_teleoperation(current)
        launcher._watch_teleoperation(current)
        error.assert_called_once()


@unittest.skipUnless(_os.environ.get('COLMAG_TEST_LAUNCHER_GUI') == '1',
                     'set COLMAG_TEST_LAUNCHER_GUI=1 under a display for the Data panel rehearsal')
class TeleoperationPanelGuiTests(unittest.TestCase):
    def test_named_participant_start_routes_selected_settings_from_new_tab(self):
        from colmag import dataset
        import tkinter as tk
        with tempfile.TemporaryDirectory() as folder:
            dataset.save_participants(folder, [dict(dataset.new_participant('P01'), name='Ada', consent=True)])
            parent = tk.Tk()
            for name, font in [('f_title', ('Arial', 20)), ('f_h', ('Arial', 12)),
                               ('f_body', ('Arial', 11)), ('f_small', ('Arial', 9)), ('f_btn', ('Arial', 11))]:
                setattr(parent, name, font)
            parent.start_teleoperation = mock.Mock()
            parent.stop_teleoperation = mock.Mock()
            panel = DataPanel(parent, data_dir=folder)
            try:
                panel.pipeline.set('teleoperation')
                panel._pipeline_changed()
                panel.experiment_name.set('Week 1 pilot')
                panel.teleop_magnets.set('2')
                panel.start_teleoperation('P01')
                parent.update()
                self.assertTrue(panel.teleoperation_panel.winfo_ismapped())
                self.assertFalse(panel.character_panel.winfo_ismapped())
                parent.start_teleoperation.assert_called_once_with('P01',
                    input_source='trackpad', participant_name='Ada', experiment_name='Week 1 pilot',
                    seed=0, trials=10, tolerance_mm=25.0, dwell_seconds=2.0, magnet_count=2)
                panel.pipeline.set('characters')
                panel._pipeline_changed()
                parent.update()
                self.assertTrue(panel.character_panel.winfo_ismapped())
            finally:
                panel.destroy()
                parent.destroy()


if __name__ == '__main__':
    unittest.main()
