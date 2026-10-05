"""Fresh measured dwell, reproducible workspace targets and matched trial logs."""

import csv
import json
import os
import sys
import tempfile
import unittest
from unittest import mock

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from colmag.teleoperation_task import (  # noqa: E402
    END_EFFECTOR_FRAME, END_EFFECTOR_REFERENCE, END_EFFECTOR_SITE,
    PROTOCOL_ID, TeleoperationRun, TeleoperationSettings, TeleoperationTrial,
    random_targets,
)
from colmag.target_reaching import distance  # noqa: E402


class TeleoperationTrialTests(unittest.TestCase):
    def setUp(self):
        self.settings = TeleoperationSettings()
        self.target = dict(target_id='test', repetition=1, position_m=(0.5, 0.0, 0.4))

    def trial(self, settings=None):
        settings = settings or self.settings
        return TeleoperationTrial(self.target, settings, 10, 100,
                                  settings.start_position_m)

    def sample(self, trial, elapsed, point=None, sim_elapsed=None, command=None):
        point = self.target['position_m'] if point is None else point
        return trial.update(10 + elapsed,
                            100 + (elapsed if sim_elapsed is None else sim_elapsed),
                            point, command)

    def complete(self, trial):
        for index in range(1, 10):
            result = self.sample(trial, index * 0.25)
        self.assertEqual(result['status'], 'completed')
        return trial

    def test_exactly_two_seconds_of_continuous_measured_dwell_are_required(self):
        trial = self.trial()
        for index in range(1, 9):
            self.assertIsNone(self.sample(trial, index * 0.25))
        self.assertAlmostEqual(trial.progress, 0.875)
        result = self.sample(trial, 2.25)
        self.assertEqual(result['status'], 'completed')
        self.assertAlmostEqual(result['completion_time_s'], 2.25)
        self.assertAlmostEqual(result['held_duration_s'], 2)
        self.assertAlmostEqual(result['sim_duration_s'], 2.25)
        self.assertEqual(trial.progress, 1)
        self.assertAlmostEqual(trial.rows[-1]['dwell_progress'], 1)

    def test_progress_requires_samples_and_render_time_cannot_fill_the_ring(self):
        trial = self.trial()
        self.sample(trial, 0.25)
        self.sample(trial, 0.5)
        self.assertAlmostEqual(trial.dwell_progress(10.7), 0.125)
        self.assertIsNone(trial.tick(10.8))
        self.assertEqual(trial.progress, 0)
        self.assertIsNone(trial.inside_since)
        self.sample(trial, 0.9)
        self.assertEqual(trial.progress, 0)
        self.assertAlmostEqual(trial.inside_since, 10.9)

    def test_one_sample_outside_resets_all_progress(self):
        trial = self.trial()
        for index in range(1, 5):
            self.sample(trial, index * 0.25)
        self.assertGreater(trial.progress, 0)
        self.sample(trial, 1.25, (0.5, 0.0, 0.42501))
        self.assertEqual(trial.progress, 0)
        for index in range(6, 14):
            self.assertIsNone(self.sample(trial, index * 0.25))
        self.assertEqual(self.sample(trial, 3.5)['status'], 'completed')
        self.assertAlmostEqual(trial.result['held_duration_s'], 2)

    def test_invalid_control_input_resets_dwell_even_when_measured_pose_stays_inside(self):
        trial = self.trial()
        for index in range(1, 5):
            self.sample(trial, index * .25)
        self.assertGreater(trial.progress, 0)
        sample = dict(tracking_valid=False, tracking_error='Magnet lifted', pose=[0, 0, .3, 0, 0, 1])
        self.assertIsNone(trial.update(11.25, 101.25, self.target['position_m'],
                                      input_sample=sample, input_valid=False))
        self.assertEqual(trial.progress, 0)
        self.assertIsNone(trial.inside_since)
        self.assertFalse(trial.rows[-1]['input_valid'])
        self.assertTrue(trial.rows[-1]['inside_tolerance'])
        self.assertEqual(json.loads(trial.rows[-1]['input_sample_json']), sample)
        for index in range(6, 14):
            self.assertIsNone(self.sample(trial, index * .25))
        result = self.sample(trial, 3.5)
        self.assertEqual(result['status'], 'completed')
        self.assertEqual(result['held_duration_s'], 2)

    def test_persistent_invalid_input_times_out_and_cannot_count_as_completion(self):
        trial = self.trial(TeleoperationSettings(timeout_s=3))
        for index in range(1, 12):
            result = trial.update(10 + index * .25, 100 + index * .25,
                                  self.target['position_m'], input_valid=False)
            self.assertIsNone(result)
            self.assertEqual(trial.progress, 0)
        result = trial.update(13, 103, self.target['position_m'], input_valid=False)
        self.assertEqual(result['status'], 'timed_out')
        self.assertEqual(result['completion_time_s'], '')
        self.assertEqual(result['held_duration_s'], 0)

    def test_tolerance_is_spherical_and_boundary_is_inside(self):
        trial = self.trial()
        self.sample(trial, 0.25, (0.5, 0, 0.425))
        self.assertIsNotNone(trial.inside_since)
        self.sample(trial, 0.5, (0.52, 0.02, 0.4))
        self.assertIsNone(trial.inside_since)
        self.assertGreater(trial.error_m, self.settings.tolerance_m)

    def test_paused_or_missing_simulation_feedback_cannot_complete_dwell(self):
        trial = self.trial()
        self.sample(trial, 0.25)
        self.sample(trial, 0.5)
        self.sample(trial, 0.75, sim_elapsed=0.5)
        self.assertEqual(trial.progress, 0)
        for index in range(4, 12):
            self.assertIsNone(self.sample(trial, index * 0.25))
        self.assertEqual(self.sample(trial, 3)['status'], 'completed')
        gap_trial = self.trial()
        self.sample(gap_trial, 0.25)
        self.assertIsNone(self.sample(gap_trial, 3))
        self.assertEqual(gap_trial.progress, 0)
        self.assertEqual(gap_trial.inside_since, 13)

    def test_commanded_target_is_not_measured_success(self):
        trial = self.trial()
        for index in range(1, 13):
            self.assertIsNone(self.sample(trial, index * 0.25,
                                          self.settings.start_position_m,
                                          command=self.target['position_m']))
        self.assertEqual(trial.status, 'running')
        self.assertEqual(trial.progress, 0)
        self.assertEqual(trial.actual_position, self.settings.start_position_m)
        self.assertEqual(trial.rows[-1]['commanded_x_m'], 0.5)
        self.assertEqual(trial.rows[-1]['actual_x_m'], 0.45)
        with self.assertRaisesRegex(ValueError, 'uninterrupted'):
            trial.finish('completed', 13)

    def test_timeout_wins_over_a_dwell_that_finishes_at_the_deadline(self):
        trial = self.trial(TeleoperationSettings(timeout_s=3))
        for index in range(1, 4):
            self.sample(trial, index * 0.25, self.settings.start_position_m)
        for index in range(4, 12):
            self.assertIsNone(self.sample(trial, index * 0.25))
        result = self.sample(trial, 3)
        self.assertEqual(result['status'], 'timed_out')
        self.assertEqual(result['completion_time_s'], '')
        self.assertAlmostEqual(result['endpoint_error_m'], 0)
        self.assertAlmostEqual(result['wall_duration_s'], 3)

    def test_timeout_without_samples_and_cancel_preserve_measured_endpoints(self):
        trial = self.trial(TeleoperationSettings(timeout_s=3))
        self.sample(trial, 0.25, (0.46, 0, 0.4))
        result = trial.tick(13)
        self.assertEqual(result['status'], 'timed_out')
        self.assertAlmostEqual(result['endpoint_error_m'], 0.04)
        self.assertEqual(result['completion_time_s'], '')
        cancelled = self.trial()
        self.sample(cancelled, 0.25, (0.47, 0, 0.4))
        result = cancelled.finish('cancelled', 10.5)
        self.assertAlmostEqual(result['endpoint_error_m'], 0.03)
        self.assertEqual(result['completion_time_s'], '')
        self.assertIs(cancelled.finish('cancelled', 12), result)
        self.assertIsNone(self.sample(cancelled, 3))
        self.assertEqual(len(cancelled.rows), 2)

    def test_simulation_clock_reset_is_retained_as_a_failed_trial(self):
        trial = self.trial()
        self.sample(trial, 0.25)
        result = self.sample(trial, 0.5, sim_elapsed=-50)
        self.assertEqual(result['status'], 'clock_reset')
        self.assertEqual(result['completion_time_s'], '')
        self.assertEqual(trial.rows[-1]['simulation_time_s'], 50)

    def test_common_start_can_be_enforced_or_explicitly_disabled(self):
        with self.assertRaisesRegex(ValueError, 'common starting'):
            TeleoperationTrial(self.target, self.settings, 10, 100, (0.55, 0, 0.4))
        free = TeleoperationSettings(require_common_start=False)
        trial = TeleoperationTrial(self.target, free, 10, 100, (0.55, 0, 0.4))
        self.assertEqual(trial.start_position, (0.55, 0, 0.4))

    def test_invalid_samples_do_not_advance_the_trial(self):
        trial = self.trial()
        for point, command in (((float('nan'), 0, 0.4), None),
                               ((0.5, 0, 0.4), (0, float('inf'), 0))):
            with self.assertRaises(ValueError):
                self.sample(trial, 0.25, point, command=command)
        self.assertEqual(len(trial.rows), 1)
        with self.assertRaises(ValueError):
            trial.update(10, 100.1, (0.5, 0, 0.4))


class TargetSequenceTests(unittest.TestCase):
    def test_seed_repeats_targets_across_conditions_without_global_rng_state(self):
        first = random_targets(200, 42)
        self.assertEqual(first, random_targets(200, 42))
        self.assertNotEqual(first, random_targets(200, 43))
        settings = TeleoperationSettings()
        half = settings.workspace_edge_m / 2 - settings.tolerance_m
        for target in first:
            point = target['position_m']
            self.assertGreaterEqual(distance(point, settings.start_position_m),
                                    2 * settings.tolerance_m)
            for value, centre in zip(point, settings.workspace_center_m):
                self.assertLessEqual(abs(value - centre), half)
        self.assertGreater(len({target['position_m'] for target in first}), 190)

    def test_invalid_or_impossible_sampling_settings_are_rejected(self):
        for kwargs in ({'dwell_s': 0}, {'timeout_s': 2}, {'tolerance_m': 0.12},
                       {'workspace_edge_m': float('nan')},
                       {'workspace_center_m': (0, float('inf'), 0)}):
            with self.assertRaises(ValueError):
                TeleoperationSettings(**kwargs)
        for count, seed in ((0, 1), (True, 1), (1, 'one')):
            with self.assertRaises(ValueError):
                random_targets(count, seed)
        with self.assertRaises(ValueError):
            random_targets(1, 2, min_distance_m=1)


class TeleoperationRunTests(unittest.TestCase):
    setUp = TeleoperationTrialTests.setUp
    trial = TeleoperationTrialTests.trial
    sample = TeleoperationTrialTests.sample
    complete = TeleoperationTrialTests.complete

    def run_directory(self, root, source='serial'):
        return TeleoperationRun(os.path.join(root, 'P01', 'S01'), 'P01', 'S01',
                                self.settings, [self.target], experiment_name='Pilot A',
                                participant_name='Example participant', input_source=source,
                                magnet_count=2 if source == 'serial' else 0, seed=7,
                                metadata={'condition': 'two magnets'})

    def test_trace_result_and_manifest_match_and_keep_failed_trials(self):
        with tempfile.TemporaryDirectory() as root:
            run = self.run_directory(root)
            first = self.trial()
            first.update(10.25, 100.25, self.settings.start_position_m,
                         self.target['position_m'], input_sample={'sensor_xyz': [1, 2, 3]})
            first.finish('cancelled', 10.5)
            run.save_trial(first)
            second = self.complete(self.trial())
            row = run.save_trial(second)
            with open(run.manifest_path, encoding='utf-8') as stream:
                manifest = json.load(stream)
            self.assertEqual(manifest['protocol'], PROTOCOL_ID)
            self.assertEqual(manifest['settings']['dwell_s'], 2)
            self.assertEqual(manifest['source'], 'mujoco_measured_gripper_center')
            self.assertEqual(manifest['protocol'], 'mujoco_random_target_reaching_v2')
            self.assertEqual(manifest['end_effector_frame'], END_EFFECTOR_FRAME)
            self.assertEqual(manifest['end_effector_site'], END_EFFECTOR_SITE)
            self.assertEqual(manifest['end_effector_reference'], END_EFFECTOR_REFERENCE)
            self.assertEqual(manifest['experiment_name'], 'Pilot A')
            self.assertEqual(manifest['participant_name'], 'Example participant')
            self.assertEqual(manifest['magnet_count'], 2)
            self.assertEqual(manifest['seed'], 7)
            self.assertEqual(len(manifest['trials']), 2)
            with open(run.summary_path, newline='') as stream:
                summary = list(csv.DictReader(stream))
            self.assertEqual([trial['status'] for trial in summary], ['cancelled', 'completed'])
            self.assertEqual(summary[0]['completion_time_s'], '')
            with open(os.path.join(run.output_dir, row['result_json'])) as stream:
                result = json.load(stream)
            with open(os.path.join(run.output_dir, row['trajectory_csv']), newline='') as stream:
                trace = list(csv.DictReader(stream))
            self.assertEqual(result['samples'], len(trace))
            self.assertEqual(result['protocol'], manifest['protocol'])
            self.assertEqual(result['end_effector_site'], manifest['end_effector_site'])
            self.assertEqual(result['end_effector_reference'], manifest['end_effector_reference'])
            self.assertEqual(result['completion_time_s'], row['completion_time_s'])
            self.assertAlmostEqual(float(trace[-1]['actual_x_m']), result['end_x_m'])
            self.assertAlmostEqual(float(trace[-1]['wall_elapsed_s']), result['wall_duration_s'])
            self.assertEqual(trace[0]['commanded_x_m'], '')
            with open(os.path.join(run.output_dir, summary[0]['trajectory_csv']), newline='') as stream:
                first_trace = list(csv.DictReader(stream))
            self.assertEqual(json.loads(first_trace[-1]['input_sample_json']),
                             {'sensor_xyz': [1, 2, 3]})
            with self.assertRaisesRegex(ValueError, 'already been saved'):
                run.save_trial(second)
            with self.assertRaises(FileExistsError):
                self.run_directory(root)

    def test_save_failure_does_not_publish_or_count_a_partial_trial(self):
        with tempfile.TemporaryDirectory() as root:
            run = self.run_directory(root, source='trackpad')
            trial = self.complete(self.trial())
            original_replace = os.replace

            def fail_manifest_once(source, destination):
                if destination == run.manifest_path:
                    raise OSError('simulated manifest write failure')
                return original_replace(source, destination)

            with mock.patch('colmag.teleoperation_task.os.replace', side_effect=fail_manifest_once):
                with self.assertRaisesRegex(OSError, 'simulated'):
                    run.save_trial(trial)
            self.assertEqual(run.completed, [])
            with open(run.manifest_path) as stream:
                self.assertEqual(json.load(stream)['trials'], [])
            with open(run.summary_path, newline='') as stream:
                self.assertEqual(list(csv.DictReader(stream)), [])
            self.assertEqual(sorted(os.listdir(run.output_dir)), ['manifest.json', 'summary.csv'])
            self.assertEqual(run.save_trial(trial)['trial'], 1)

    def test_running_trials_cannot_be_written_as_results(self):
        with tempfile.TemporaryDirectory() as root:
            run = self.run_directory(root)
            with self.assertRaisesRegex(ValueError, 'Finish'):
                run.save_trial(self.trial())
            with open(run.manifest_path) as stream:
                self.assertEqual(json.load(stream)['trials'], [])

    def test_new_protocol_cannot_label_flange_data_as_gripper_midpoint(self):
        with tempfile.TemporaryDirectory() as root:
            folder = os.path.join(root, 'new_session')
            with self.assertRaisesRegex(ValueError, 'fingertip midpoint'):
                TeleoperationRun(folder, 'P01', 'S01', self.settings, [self.target],
                                 end_effector_frame='fr3_link8', end_effector_site='attachment_site',
                                 end_effector_reference='flange')
            self.assertFalse(os.path.exists(folder))


if __name__ == '__main__':
    unittest.main()
