"""Metric, simulation-source and launcher checks for the virtual task pilot."""

import csv
import json
import os
import shlex
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from colmag.target_reaching import (  # noqa: E402
    PilotRun, TargetTrial, TrialSettings, default_targets, require_simulation,
)
from colmag_launcher import (  # noqa: E402
    Launcher, active_teleop_input_source, build_stop_all_command, build_virtual_task_command,
)


class TargetTrialTests(unittest.TestCase):
    def setUp(self):
        self.settings = TrialSettings(dwell_s=0.5, max_feedback_gap_s=0.25)
        self.target = dict(target_id='test', repetition=1, position_m=(0.5, 0.0, 0.4))

    def trial(self):
        return TargetTrial(self.target, self.settings, 10.0, 100.0,
                           self.settings.start_position_m)

    def test_completion_uses_actual_endpoint_dwell_and_wall_time(self):
        trial = self.trial()
        for wall, point in ((10.1, (0.45, 0, 0.4)), (10.2, (0.49, 0, 0.4)),
                            (10.4, (0.49, 0, 0.4)), (10.6, (0.49, 0, 0.4))):
            self.assertIsNone(trial.update(wall, 100 + (wall - 10) * 0.5, point))
        result = trial.update(10.8, 100.4, (0.49, 0, 0.4))
        self.assertEqual(result['status'], 'completed')
        self.assertAlmostEqual(result['completion_time_s'], 0.8)
        self.assertAlmostEqual(result['sim_duration_s'], 0.4)
        self.assertAlmostEqual(result['endpoint_error_m'], 0.01)
        self.assertAlmostEqual(result['path_length_m'], 0.04)

    def test_leaving_target_resets_the_dwell(self):
        trial = self.trial()
        for elapsed, point in ((0.1, (0.5, 0, 0.4)), (0.3, (0.5, 0, 0.4)),
                               (0.4, (0.45, 0, 0.4)), (0.5, (0.5, 0, 0.4)),
                               (0.7, (0.5, 0, 0.4)), (0.9, (0.5, 0, 0.4))):
            self.assertIsNone(trial.update(10 + elapsed, 100 + elapsed, point))
        self.assertEqual(trial.update(11.1, 101.1, (0.5, 0, 0.4))['status'], 'completed')

    def test_feedback_gap_cannot_satisfy_dwell(self):
        trial = self.trial()
        trial.update(10.1, 100.1, (0.5, 0, 0.4))
        self.assertIsNone(trial.update(11.0, 101, (0.5, 0, 0.4)))
        self.assertEqual(trial.inside_since, 11.0)

    def test_paused_clock_cannot_satisfy_dwell(self):
        trial = self.trial()
        for elapsed in (0.1, 0.3, 0.5, 0.7, 0.9):
            self.assertIsNone(trial.update(10 + elapsed, 100, (0.5, 0, 0.4)))
            self.assertIsNone(trial.inside_since)

    def test_dwell_restarts_after_a_single_paused_sample(self):
        trial = self.trial()
        trial.update(10.1, 100.1, (0.5, 0, 0.4))
        trial.update(10.3, 100.3, (0.5, 0, 0.4))
        self.assertIsNone(trial.update(10.4, 100.3, (0.5, 0, 0.4)))
        self.assertIsNone(trial.inside_since)
        for wall, sim in ((10.5, 100.4), (10.7, 100.6), (10.9, 100.8)):
            self.assertIsNone(trial.update(wall, sim, (0.5, 0, 0.4)))
        self.assertEqual(trial.update(11.1, 101.0, (0.5, 0, 0.4))['status'], 'completed')

    def test_clock_reset_is_not_a_success(self):
        trial = self.trial()
        trial.update(10.1, 100.1, (0.49, 0, 0.4))
        result = trial.update(10.2, 1.0, (0.5, 0, 0.4))
        self.assertEqual(result['status'], 'clock_reset')
        self.assertEqual(result['completion_time_s'], '')

    def test_timeout_and_cancel_retain_endpoint_without_completion_time(self):
        settings = TrialSettings(timeout_s=1, dwell_s=0.1)
        trial = TargetTrial(self.target, settings, 10, 100, settings.start_position_m)
        result = trial.update(11.1, 101.1, settings.start_position_m)
        self.assertEqual(result['status'], 'timed_out')
        self.assertEqual(result['completion_time_s'], '')
        other = self.trial()
        other.update(10.1, 100.1, (0.46, 0, 0.4))
        cancelled = other.finish('cancelled', 10.2, 100.2)
        self.assertEqual(cancelled['completion_time_s'], '')
        self.assertAlmostEqual(cancelled['endpoint_error_m'], 0.04)

    def test_timeout_wins_when_target_dwell_finishes_at_deadline(self):
        settings = TrialSettings(timeout_s=1, dwell_s=0.5)
        trial = TargetTrial(self.target, settings, 10, 100, settings.start_position_m)
        for elapsed in (0.5, 0.7, 0.9):
            self.assertIsNone(trial.update(10 + elapsed, 100 + elapsed, (0.5, 0, 0.4)))
        result = trial.update(11, 101, (0.5, 0, 0.4))
        self.assertEqual(result['status'], 'timed_out')
        self.assertEqual(result['completion_time_s'], '')
        self.assertAlmostEqual(result['endpoint_error_m'], 0)

    def test_common_start_is_required(self):
        with self.assertRaisesRegex(ValueError, 'common starting'):
            TargetTrial(self.target, self.settings, 10, 100, (0.5, 0, 0.4))

    def test_invalid_settings_or_samples_are_rejected(self):
        for values in ({'tolerance_m': float('nan')}, {'dwell_s': 0}, {'timeout_s': 0.2}):
            with self.assertRaises(ValueError):
                TrialSettings(**values)
        with self.assertRaises(ValueError):
            self.trial().update(10.1, 100.1, (float('inf'), 0, 0.4))


class PilotRunTests(unittest.TestCase):
    def test_run_keeps_cancelled_and_completed_trials_with_reproducible_units(self):
        with tempfile.TemporaryDirectory() as root:
            settings = TrialSettings(dwell_s=0.1)
            targets = default_targets(2)
            run = PilotRun(os.path.join(root, 'run'), 'run', 'P01', 'magnet_stack_2',
                           settings, targets, metadata={'magnet_count': 2})
            first = TargetTrial(targets[0], settings, 0, 10, settings.start_position_m)
            first.update(0.1, 10.1, (0.45, 0.0, 0.4))
            first.finish('cancelled', 0.2, 10.2)
            run.save_trial(first)
            second = TargetTrial(targets[0], settings, 1, 11, settings.start_position_m)
            second.update(1.1, 11.1, targets[0]['position_m'])
            second.update(1.21, 11.21, targets[0]['position_m'])
            run.save_trial(second)
            with open(run.manifest_path, encoding='utf-8') as stream:
                manifest = json.load(stream)
            self.assertEqual(manifest['position_units'], 'm')
            self.assertEqual(manifest['end_effector_frame'], 'fr3_link8')
            self.assertEqual(manifest['settings']['tolerance_m'], 0.020)
            self.assertEqual(len(manifest['targets']), 16)
            self.assertEqual(manifest['metadata']['magnet_count'], 2)
            self.assertEqual(len(manifest['trials']), 2)
            with open(run.summary_path, newline='') as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual([row['status'] for row in rows], ['cancelled', 'completed'])
            self.assertEqual(rows[0]['completion_time_s'], '')
            with open(os.path.join(run.output_dir, rows[1]['trajectory_csv']), newline='') as stream:
                trace = list(csv.DictReader(stream))
            self.assertEqual(len(trace), 2)
            self.assertAlmostEqual(float(trace[-1]['target_error_m']), 0)
            with self.assertRaises(FileExistsError):
                PilotRun(run.output_dir, 'run', 'P01', 'mouse', settings, targets)

    def test_simulation_guard_rejects_real_and_ambiguous_sources(self):
        require_simulation(True, ['/gazebo', '/robot_state_publisher'])
        for simulated, nodes in ((False, ['/gazebo']), (True, ['/franka_control']),
                                  (True, ['/gazebo', '/robot/franka_control']),
                                  (True, ['/robot_state_publisher'])):
            with self.assertRaises(RuntimeError):
                require_simulation(simulated, nodes)

    def test_dry_run_contains_no_measurements(self):
        from tools.target_reaching_pilot import main
        with tempfile.TemporaryDirectory() as root:
            output = os.path.join(root, 'plan')
            self.assertEqual(main(['--dry-run', '--output-dir', output,
                                   '--participant-id', 'P03',
                                   '--participant-name', 'Alex Example',
                                   '--experiment-name', 'Week 1: virtual task pilot']), 0)
            with open(os.path.join(output, 'manifest.json')) as stream:
                manifest = json.load(stream)
            self.assertEqual(manifest['trials'], [])
            self.assertTrue(manifest['metadata']['dry_run'])
            self.assertEqual(manifest['source'], 'plan_only')
            self.assertEqual(manifest['participant_id'], 'P03')
            self.assertEqual(manifest['metadata']['participant_name'], 'Alex Example')
            self.assertEqual(manifest['metadata']['experiment_name'], 'Week 1: virtual task pilot')


class PilotLauncherTests(unittest.TestCase):
    def test_condition_is_quoted_and_existing_controller_stays_separate(self):
        command = build_virtual_task_command('run', 'P01', 'magnet stack 2',
                                              'magnetometer', magnet_count=2,
                                              notes='same start; practice only')
        arguments = shlex.split(command)[3:]
        self.assertEqual(arguments[arguments.index('--condition') + 1], 'magnet stack 2')
        self.assertEqual(arguments[arguments.index('--input-source') + 1], 'serial')
        self.assertEqual(arguments[arguments.index('--magnet-count') + 1], '2')
        self.assertNotIn('roslaunch', command)
        self.assertIn('[t]arget_reaching_pilot.py', build_stop_all_command())
        self.assertIn('/tmp/colmag_gui_pilot.pid', build_stop_all_command())

    @mock.patch('colmag_launcher.messagebox.showerror')
    def test_pilot_start_in_real_mode_never_starts_a_process(self, showerror):
        launcher = mock.Mock()
        launcher.mode.get.return_value = 'real'
        Launcher.start_virtual_task(launcher, 'P01', 'test', None, 20, 0.5, 1)
        launcher._ensure_container.assert_not_called()
        launcher._launch_stage.assert_not_called()
        showerror.assert_called_once()

    def test_reader_parser_ignores_wrappers_and_rejects_unverified_inputs(self):
        wrapped = "100 bash -lc 'cd /colmag && python3 magnetometer_reader.py --input-source serial --ros'"
        unquoted = "102 bash -lc cd /colmag && python3 magnetometer_reader.py --input-source serial --ros"
        unrelated = '103 python3 unrelated.py --notes magnetometer_reader.py --input-source serial --ros'
        actual = '101 python3 /colmag/magnetometer_reader.py --input-source trackpad --ros'
        self.assertEqual(active_teleop_input_source('\n'.join((wrapped, unquoted, unrelated, actual))), 'trackpad')
        for output in ('', actual + '\n102 python3 magnetometer_reader.py --ros',
                       '103 python3 magnetometer_reader.py --record-data --ros',
                       '104 python3 magnetometer_reader.py --input-source unknown --ros'):
            with self.assertRaises(ValueError):
                active_teleop_input_source(output)

    @mock.patch('colmag_launcher.in_container')
    def test_pilot_records_running_reader_source_despite_selector_change(self, probe):
        launcher = mock.Mock()
        launcher.mode.get.return_value = 'sim'
        launcher._running_ros_nodes.return_value = '/gazebo'
        launcher.input_src.get.return_value = 'magnetometer'
        probe.return_value = (True, '101 python3 magnetometer_reader.py --input-source trackpad --ros')
        Launcher.start_virtual_task(launcher, 'P01', 'practice', None, 20, 0.5, 1)
        command = launcher._launch_stage.call_args[0][1]
        args = shlex.split(command)
        self.assertEqual(args[args.index('--input-source') + 1], 'trackpad')


class FlangeFeedbackTests(unittest.TestCase):
    def test_stale_tf_cannot_satisfy_wall_time_dwell_when_gazebo_is_slow(self):
        from tools.target_reaching_pilot import FeedbackUnavailable, RosFeedback
        source = RosFeedback.__new__(RosFeedback)
        source.clock_s = 100.01
        source.clock_changed_wall_s = 10
        source.tf_stamp_s = None
        source.tf_changed_wall_s = 0
        source.max_feedback_gap_s = 0.25
        source.base_frame, source.ee_frame = 'fr3_link0', 'fr3_link8'
        source.rospy = mock.Mock()
        source.tf2_ros = SimpleNamespace(TransformException=RuntimeError)
        stamp = mock.Mock()
        stamp.to_sec.return_value = 100.0
        source.buffer = mock.Mock()
        source.buffer.lookup_transform.return_value = SimpleNamespace(
            header=SimpleNamespace(stamp=stamp),
            transform=SimpleNamespace(translation=SimpleNamespace(x=0.45, y=0, z=0.4)))
        with mock.patch('tools.target_reaching_pilot.time.monotonic', return_value=10):
            self.assertEqual(source.sample()[2], (0.45, 0, 0.4))
        # Clock advances slowly and TF age in simulation seconds stays small,
        # but 0.3 seconds with no new measured transform invalidates feedback.
        source.clock_s = 100.02
        source.clock_changed_wall_s = 10.3
        with mock.patch('tools.target_reaching_pilot.time.monotonic', return_value=10.3):
            with self.assertRaisesRegex(FeedbackUnavailable, 'stopped updating'):
                source.sample()
        stamp.to_sec.return_value = 100.02
        with mock.patch('tools.target_reaching_pilot.time.monotonic', return_value=10.31):
            self.assertEqual(source.sample()[1], 100.02)


if __name__ == '__main__':
    unittest.main()
