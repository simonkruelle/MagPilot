#!/usr/bin/env python3
"""Tests for the participant registry and coverage counts (colmag/dataset.py)."""

import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

import json
import os
import shutil
import tempfile
import unittest

from colmag import dataset
from colmag import collection_protocol as protocol


def write_take(data_dir, participant_id='P01', session_id='S01', label='digit_3',
               basename=None, height_mm=0, input_source='serial',
               sample_count=40, with_csv=True):
    """Write one fake take the way magnetometer_reader.py lays it out."""
    basename = basename or '{}_{}_{}_rep001'.format(participant_id, session_id, label)
    folder = os.path.join(data_dir, dataset.CHARACTERS_DIR, participant_id or 'none',
                          session_id or 'none', 'samples', label)
    os.makedirs(folder, exist_ok=True)
    take = {
        'label': label,
        'basename': basename,
        'participant_id': participant_id,
        'session_id': session_id,
        'height_mm': height_mm,
        'input_source': input_source,
        'sample_count': sample_count,
    }
    if height_mm is None:
        del take['height_mm']
    with open(os.path.join(folder, basename + '.json'), 'w', encoding='utf-8') as f:
        json.dump(take, f)
    if with_csv:
        open(os.path.join(folder, basename + '.csv'), 'w').close()


class ParticipantTests(unittest.TestCase):
    def test_missing_file_means_no_participants(self):
        with tempfile.TemporaryDirectory() as data_dir:
            self.assertEqual(dataset.load_participants(data_dir), [])

    def test_save_and_load_round_trip_sorted_by_id(self):
        with tempfile.TemporaryDirectory() as data_dir:
            people = [dataset.new_participant('P02'), dataset.new_participant('P01')]
            dataset.save_participants(data_dir, people)
            loaded = dataset.load_participants(data_dir)
            self.assertEqual([p['participant_id'] for p in loaded], ['P01', 'P02'])
            self.assertFalse(loaded[0]['consent'])

    def test_name_round_trip_and_display_include_stable_id(self):
        with tempfile.TemporaryDirectory() as data_dir:
            participant = dataset.new_participant('P01', '  Simon  ')
            dataset.save_participants(data_dir, [participant])
            loaded = dataset.load_participants(data_dir)[0]
            self.assertEqual(loaded['name'], 'Simon')
            self.assertEqual(dataset.participant_label(loaded), 'Simon (P01)')

    def test_legacy_registry_without_name_retains_schema_and_id(self):
        with tempfile.TemporaryDirectory() as data_dir:
            participant = dataset.new_participant('P01')
            del participant['name']
            dataset.save_participants(data_dir, [participant])
            loaded = dataset.load_participants(data_dir)[0]
            self.assertEqual(loaded, participant)
            self.assertEqual(dataset.participant_label(loaded), 'P01')
            self.assertEqual(dataset.participant_label(
                dataset.new_participant('P02', '  ')), 'P02')
            with open(os.path.join(data_dir, dataset.PARTICIPANTS_FILE)) as f:
                self.assertEqual(json.load(f)['schema_version'], 1)

    def test_damaged_file_raises_instead_of_resetting(self):
        with tempfile.TemporaryDirectory() as data_dir:
            path = os.path.join(data_dir, dataset.PARTICIPANTS_FILE)
            for content in ('{not json', '[]', '{"schema_version": 1, "participants": {}}',
                            '{"schema_version": 1, "participants": [{"participant_id": "Simon"}]}'):
                with open(path, 'w') as f:
                    f.write(content)
                with self.assertRaises(ValueError):
                    dataset.load_participants(data_dir)

    def test_next_id_never_reuses_an_id_seen_in_recordings(self):
        people = [dataset.new_participant('P01')]
        self.assertEqual(dataset.next_participant_id([]), 'P01')
        self.assertEqual(dataset.next_participant_id(people), 'P02')
        self.assertEqual(
            dataset.next_participant_id(people, seen_ids={'P04', 'unassigned'}), 'P05')

    def test_next_session_id_counts_existing_folders(self):
        with tempfile.TemporaryDirectory() as data_dir:
            self.assertEqual(dataset.next_session_id(data_dir, 'P01'), 'S01')
            os.makedirs(os.path.join(data_dir, dataset.CHARACTERS_DIR, 'P01', 'S01'))
            os.makedirs(os.path.join(data_dir, dataset.CHARACTERS_DIR, 'P01', 'S03'))
            self.assertEqual(dataset.next_session_id(data_dir, 'P01'), 'S04')


class CollectionProtocolTests(unittest.TestCase):
    def test_shared_protocol_is_twenty_characters_and_ten_repetitions(self):
        self.assertEqual(protocol.DIGITS, tuple('0123456789'))
        self.assertEqual(protocol.LETTERS, tuple('ABCDEFGHIJ'))
        self.assertEqual(protocol.CHARACTER_LABELS, tuple(
            ['digit_{}'.format(c) for c in '0123456789'] +
            ['letter_{}'.format(c) for c in 'ABCDEFGHIJ']))
        self.assertEqual(protocol.TARGET_REPS, 10)
        self.assertIs(dataset.DIGITS, protocol.DIGITS)
        self.assertIs(dataset.LETTERS, protocol.LETTERS)
        self.assertEqual(dataset.TARGET_REPS, protocol.TARGET_REPS)
        self.assertFalse(set(protocol.CONTROL_LABELS) & set(protocol.CHARACTER_LABELS))

    def test_metadata_is_json_serializable_and_cannot_mutate_protocol(self):
        metadata = protocol.protocol_metadata()
        self.assertEqual(json.loads(json.dumps(metadata)), metadata)
        self.assertEqual(metadata['protocol_id'], protocol.PROTOCOL_ID)
        self.assertEqual(metadata['character_labels'], list(protocol.CHARACTER_LABELS))
        self.assertEqual(metadata['control_labels'], list(protocol.CONTROL_LABELS))
        self.assertEqual(metadata['target_reps'], 10)
        metadata['character_labels'].clear()
        self.assertEqual(len(protocol.protocol_metadata()['character_labels']), 20)


class CollectionSettingsTests(unittest.TestCase):
    def test_missing_settings_and_round_trip(self):
        with tempfile.TemporaryDirectory() as data_dir:
            self.assertEqual(dataset.load_collection_settings(data_dir),
                             {'experiment_name': ''})
            self.assertEqual(dataset.save_collection_settings(data_dir, '  New pen pilot  '),
                             {'experiment_name': 'New pen pilot'})
            self.assertEqual(dataset.load_collection_settings(data_dir),
                             {'experiment_name': 'New pen pilot'})
            dataset.save_collection_settings(data_dir, 'Tuesday baseline')
            self.assertEqual(dataset.load_collection_settings(data_dir),
                             {'experiment_name': 'Tuesday baseline'})
            self.assertEqual(os.listdir(data_dir), [dataset.COLLECTION_SETTINGS_FILE])

    def test_invalid_existing_settings_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as data_dir:
            path = os.path.join(data_dir, dataset.COLLECTION_SETTINGS_FILE)
            for content in ('{not json', '[]', '{}', '{"experiment_name": null}',
                            '{"experiment_name": 42}'):
                with self.subTest(content=content):
                    with open(path, 'w') as f:
                        f.write(content)
                    with self.assertRaises(ValueError):
                        dataset.load_collection_settings(data_dir)
                    with self.assertRaises(ValueError):
                        dataset.save_collection_settings(data_dir, 'Replacement')
                    with open(path) as f:
                        self.assertEqual(f.read(), content)

    def test_non_string_name_rejected_without_changing_saved_settings(self):
        with tempfile.TemporaryDirectory() as data_dir:
            dataset.save_collection_settings(data_dir, 'Pilot')
            for experiment_name in (None, 1, {}, []):
                with self.assertRaises(ValueError):
                    dataset.save_collection_settings(data_dir, experiment_name)
            self.assertEqual(dataset.load_collection_settings(data_dir),
                             {'experiment_name': 'Pilot'})


class CoverageTests(unittest.TestCase):
    def test_character_of_only_accepts_dataset_labels(self):
        self.assertEqual(dataset.character_of('digit_7'), '7')
        self.assertEqual(dataset.character_of('letter_J'), 'J')
        self.assertIsNone(dataset.character_of('letter_K'))
        self.assertIsNone(dataset.character_of('control_blank'))

    def test_counts_takes_per_participant_character_and_height(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_take(data_dir, basename='a')
            write_take(data_dir, basename='b')
            write_take(data_dir, participant_id='P02', label='letter_A', basename='c')
            write_take(data_dir, height_mm=50, basename='d')
            samples, problems = dataset.scan_samples(data_dir)
            self.assertEqual(problems, {})
            self.assertEqual(
                dataset.count_coverage(samples, dataset.DIGITS, 0), {'P01': {'3': 2}})
            self.assertEqual(
                dataset.count_coverage(samples, dataset.LETTERS, 0), {'P02': {'A': 1}})
            self.assertEqual(
                dataset.count_coverage(samples, dataset.DIGITS, 50), {'P01': {'3': 1}})

    def test_skipped_takes_are_reported(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_take(data_dir, basename='no_csv', with_csv=False)
            write_take(data_dir, basename='empty', sample_count=0)
            write_take(data_dir, basename='trackpad', input_source='touchpad')
            write_take(data_dir, basename='blank', label='control_blank')
            write_take(data_dir, basename='k', label='letter_K')
            write_take(data_dir, basename='dup')
            write_take(data_dir, session_id='S02', basename='dup')
            bad = os.path.join(data_dir, dataset.CHARACTERS_DIR, 'P01', 'S01',
                               'samples', 'digit_3', 'broken.json')
            with open(bad, 'w') as f:
                f.write('{')
            samples, problems = dataset.scan_samples(data_dir)
            self.assertEqual(len(samples), 1)
            self.assertEqual(problems, {
                'no CSV': 1,
                'empty': 1,
                'not from the sensor board': 1,
                'other label': 2,
                'duplicate': 1,
                'unreadable': 1,
            })

    def test_takes_without_ids_or_height(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_take(data_dir, participant_id=None, session_id=None, height_mm=None)
            samples, problems = dataset.scan_samples(data_dir)
            self.assertEqual(samples[0]['participant_id'], 'unassigned')
            self.assertEqual(samples[0]['height_mm'], 0)
            self.assertEqual(problems, {'without height, shown at 0 mm': 1})

    def test_float_heights_from_the_recorder_match_the_height_tabs(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_take(data_dir, height_mm=50.0, basename='a')
            write_take(data_dir, height_mm=37.5, basename='b')
            samples, problems = dataset.scan_samples(data_dir)
            self.assertEqual(
                dataset.count_coverage(samples, dataset.DIGITS, 50), {'P01': {'3': 1}})
            self.assertEqual(list(problems.values()), [1])
            self.assertIn('outside', list(problems)[0])

    def test_progress_caps_each_character_at_the_target(self):
        with tempfile.TemporaryDirectory() as data_dir:
            for i in range(dataset.TARGET_REPS + 5):
                write_take(data_dir, basename='x{}'.format(i))
            write_take(data_dir, participant_id='P09', basename='other')
            samples, _ = dataset.scan_samples(data_dir)
            self.assertEqual(dataset.progress(samples, {'P01'}, 0), dataset.TARGET_REPS)
            self.assertEqual(dataset.progress(samples, {'P01'}, 50), 0)


class DemoDatasetTests(unittest.TestCase):
    def test_output_paths_preserve_board_paths_and_separate_demo(self):
        self.assertEqual(dataset.collection_data_dir('data_collection'), 'data_collection')
        self.assertEqual(dataset.collection_data_dir('data_collection', demo=True),
                         os.path.join('data_collection', 'demo'))
        self.assertEqual(dataset.session_output_dir('P01', 'S02'),
                         os.path.join('data_collection', 'characters', 'P01', 'S02'))
        self.assertEqual(dataset.session_output_dir('P01', 'S02', demo=True),
                         os.path.join('data_collection', 'demo', 'characters', 'P01', 'S02'))

    def test_board_and_demo_coverage_use_independent_roots(self):
        with tempfile.TemporaryDirectory() as data_dir:
            demo_dir = dataset.collection_data_dir(data_dir, demo=True)
            write_take(data_dir, basename='board')
            write_take(demo_dir, basename='demo', label='letter_A', input_source='touchpad')
            # Even a misplaced serial take inside demo cannot enter default board coverage.
            write_take(demo_dir, basename='demo_serial', label='letter_B')
            board_samples, board_problems = dataset.scan_samples(data_dir)
            demo_samples, demo_problems = dataset.scan_samples(demo_dir, input_source='touchpad')
            self.assertEqual(dataset.count_coverage(board_samples, dataset.DIGITS + dataset.LETTERS, 0),
                             {'P01': {'3': 1}})
            self.assertEqual(dataset.count_coverage(demo_samples, dataset.DIGITS + dataset.LETTERS, 0),
                             {'P01': {'A': 1}})
            self.assertEqual(board_problems, {})
            self.assertEqual(demo_problems, {'not from touchpad': 1})

    def test_source_filter_rejects_other_input_even_in_the_selected_root(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_take(data_dir, basename='board', label='digit_0', input_source='serial')
            write_take(data_dir, basename='trackpad', label='letter_A', input_source='touchpad')
            board_samples, board_problems = dataset.scan_samples(data_dir)
            demo_samples, demo_problems = dataset.scan_samples(data_dir, input_source='touchpad')
            self.assertEqual([sample['char'] for sample in board_samples], ['0'])
            self.assertEqual([sample['char'] for sample in demo_samples], ['A'])
            self.assertEqual(board_problems, {'not from the sensor board': 1})
            self.assertEqual(demo_problems, {'not from touchpad': 1})

    def test_board_and_demo_sessions_are_numbered_independently(self):
        with tempfile.TemporaryDirectory() as data_dir:
            demo_dir = dataset.collection_data_dir(data_dir, demo=True)
            write_take(data_dir, session_id='S03', basename='board')
            self.assertEqual(dataset.next_session_id(data_dir, 'P01'), 'S04')
            self.assertEqual(dataset.next_session_id(demo_dir, 'P01'), 'S01')
            write_take(demo_dir, session_id='S01', basename='demo', input_source='touchpad')
            self.assertEqual(dataset.next_session_id(data_dir, 'P01'), 'S04')
            self.assertEqual(dataset.next_session_id(demo_dir, 'P01'), 'S02')


def write_teleoperation_run(data_dir, participant_id='P01', session_id='S01',
                            experiment_name='Pilot A', source='trackpad',
                            statuses=('completed',), mode='mujoco', condition='baseline',
                            metadata=None):
    """Use the production writers so coverage fixtures match both recorder formats."""
    from colmag.teleoperation_task import (
        TeleoperationRun, TeleoperationSettings, TeleoperationTrial)
    from colmag.target_reaching import PilotRun, TargetTrial, TrialSettings

    target = dict(target_id='test', repetition=1, position_m=(0.5, 0.0, 0.4))
    if mode == 'mujoco':
        settings = TeleoperationSettings()
        folder = os.path.join(data_dir, dataset.TELEOPERATION_DIR, participant_id, session_id)
        run = TeleoperationRun(folder, participant_id, session_id, settings, [target] * 2,
                               experiment_name=experiment_name, participant_name='Alex',
                               input_source=source, metadata=metadata)
        trial_type = TeleoperationTrial
    else:
        settings = TrialSettings(dwell_s=0.5)
        folder = os.path.join(data_dir, dataset.VIRTUAL_TASK_DIR, session_id)
        run = PilotRun(folder, session_id, participant_id, condition, settings, [target] * 8,
                       metadata=dict(metadata or {}, experiment_name=experiment_name,
                                     input_source=source, participant_name='Alex'))
        trial_type = TargetTrial
    for status in statuses:
        trial = trial_type(target, settings, 10, 100, settings.start_position_m)
        if status == 'completed':
            for index in range(1, 10):
                trial.update(10 + index * 0.25, 100 + index * 0.25, target['position_m'])
        elif status != 'empty_cancelled':
            trial.update(10.25, 100.25, settings.start_position_m)
        if status != 'completed':
            trial.finish('cancelled' if status == 'empty_cancelled' else status, 10.5, 100.5)
        run.save_trial(trial)
    return run


class TeleoperationCoverageTests(unittest.TestCase):
    def change_manifest(self, run, change):
        with open(run.manifest_path) as stream:
            manifest = json.load(stream)
        change(manifest)
        with open(run.manifest_path, 'w') as stream:
            json.dump(manifest, stream)
        return manifest

    def change_result(self, run, change, number=1):
        path = os.path.join(run.output_dir, 'trial_{:03d}_result.json'.format(number))
        with open(path) as stream:
            result = json.load(stream)
        change(result)
        with open(path, 'w') as stream:
            json.dump(result, stream)

    def test_no_runs_is_empty(self):
        with tempfile.TemporaryDirectory() as data_dir:
            self.assertEqual(dataset.scan_teleoperation_runs(data_dir), ([], {}))
            self.assertEqual(dataset.count_teleoperation_coverage([]), {})

    def test_multiple_sessions_count_successes_and_keep_unsuccessful_attempts(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_teleoperation_run(data_dir, statuses=(
                'completed', 'cancelled', 'empty_cancelled', 'timed_out', 'feedback_lost',
                'clock_reset', 'simulation_stopped'))
            write_teleoperation_run(data_dir, session_id='S02', statuses=('completed', 'completed'))
            write_teleoperation_run(data_dir, participant_id='P02', statuses=())
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(problems, {})
            coverage = dataset.count_teleoperation_coverage(runs, ' Pilot A ', 'mujoco', 'trackpad')
            counts = coverage['P01']
            self.assertEqual((counts['completed'], counts['attempts'], counts['sessions'], counts['planned']),
                             (3, 9, 2, 4))
            self.assertEqual(counts['status_counts']['cancelled'], 2)
            self.assertEqual(counts['input_counts'], {'trackpad': 3})
            self.assertEqual(coverage['P02']['completed'], 0)
            self.assertEqual(coverage['P02']['planned'], 2)
            self.assertEqual(runs[0]['participant_name'], 'Alex')
            self.assertEqual(len(counts['configurations']), 1)

    def test_exact_experiment_mode_source_and_condition_filters(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_teleoperation_run(data_dir, experiment_name=' Pilot A ')
            write_teleoperation_run(data_dir, session_id='S02', source='serial')
            write_teleoperation_run(data_dir, session_id='S03', experiment_name='Pilot B')
            write_teleoperation_run(data_dir, session_id='S04', experiment_name='')
            write_teleoperation_run(data_dir, mode='gazebo', session_id='run1', condition='fast')
            write_teleoperation_run(data_dir, mode='gazebo', session_id='run2', condition='slow')
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(problems, {})
            self.assertEqual(dataset.count_teleoperation_coverage(
                runs, 'Pilot A', 'mujoco', 'trackpad')['P01']['completed'], 1)
            self.assertEqual(dataset.count_teleoperation_coverage(
                runs, 'Pilot A', 'mujoco')['P01']['input_counts'], {'trackpad': 1, 'serial': 1})
            self.assertEqual(dataset.count_teleoperation_coverage(
                runs, '', 'mujoco')['P01']['completed'], 1)
            self.assertEqual(dataset.count_teleoperation_coverage(runs, 'pilot a'), {})
            gazebo = dataset.count_teleoperation_coverage(runs, 'Pilot A', 'gazebo', condition=' fast ')
            self.assertEqual((gazebo['P01']['completed'], gazebo['P01']['planned']), (1, 8))
            self.assertEqual(dataset.count_teleoperation_coverage(runs)['P01']['completed'], 6)

    def test_original_flange_protocol_and_different_pointer_mapping_are_identified(self):
        with tempfile.TemporaryDirectory() as data_dir:
            old = write_teleoperation_run(data_dir, metadata={'pointer_mapping': 'legacy_full_scene'})
            def old_reference(data):
                data.update(protocol='mujoco_random_target_reaching_v1',
                            end_effector_frame='fr3_link8', end_effector_reference='flange')
                data.pop('end_effector_site', None)
            def old_manifest(data):
                old_reference(data)
                data['source'] = 'mujoco_measured_flange'
            self.change_manifest(old, old_manifest)
            self.change_result(old, old_reference)
            # Original v1 result JSON had no per-trial end-effector identity.
            self.change_result(old, lambda r: [r.pop(key, None) for key in (
                'end_effector_frame', 'end_effector_reference', 'end_effector_site')])
            write_teleoperation_run(data_dir, session_id='S02',
                                    metadata={'pointer_mapping': 'scene_height_plane_v1'})
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(problems, {})
            counts = dataset.count_teleoperation_coverage(runs)['P01']
            self.assertEqual(counts['completed'], 2)
            self.assertEqual(len(counts['protocols']), 2)
            self.assertEqual(len(counts['configurations']), 2)

    def test_legacy_touchpad_alias_matches_trackpad_without_mixing_board(self):
        with tempfile.TemporaryDirectory() as data_dir:
            run = write_teleoperation_run(data_dir)
            self.change_manifest(run, lambda m: m.update(input_source='touchpad'))
            self.change_manifest(run, lambda m: m['trials'][0].update(input_source='touchpad'))
            self.change_result(run, lambda r: r.update(input_source='touchpad'))
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(problems, {})
            self.assertEqual(dataset.count_teleoperation_coverage(
                runs, input_source='trackpad')['P01']['completed'], 1)
            self.assertEqual(dataset.count_teleoperation_coverage(runs, input_source='serial'), {})

    def test_different_seeds_and_target_plans_have_distinct_configurations(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_teleoperation_run(data_dir)
            different_seed = write_teleoperation_run(data_dir, session_id='S02')
            self.change_manifest(different_seed, lambda m: m.update(seed=17))
            different_plan = write_teleoperation_run(data_dir, session_id='S03')
            self.change_manifest(different_plan, lambda m: m['targets'].append(
                dict(target_id='other', repetition=1, position_m=[0.4, 0.05, 0.45])))
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(problems, {})
            self.assertEqual(len({r['configuration_id'] for r in runs}), 3)
            counts = dataset.count_teleoperation_coverage(runs)['P01']
            self.assertEqual(len(counts['configurations']), 3)
            self.assertEqual((counts['completed'], counts['planned']), (3, 7))

    def test_orphans_pending_directories_and_duplicates_never_add_samples(self):
        with tempfile.TemporaryDirectory() as data_dir:
            run = write_teleoperation_run(data_dir)
            shutil.copytree(run.output_dir, os.path.join(data_dir, 'teleoperation', 'copy'))
            shutil.copytree(run.output_dir, os.path.join(run.output_dir, '.trial.pending'))
            shutil.copyfile(os.path.join(run.output_dir, 'trial_001_result.json'),
                            os.path.join(run.output_dir, 'trial_002_result.json'))
            self.change_manifest(run, lambda m: m['trials'].append(dict(m['trials'][0])))
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0]['completed'], 1)
            self.assertEqual(problems, {'duplicate trial': 1, 'duplicate session': 1})
            self.assertEqual(dataset.count_teleoperation_coverage(runs + runs)['P01']['completed'], 1)

    def test_missing_or_inconsistent_artefacts_are_not_successes(self):
        cases = ('missing_csv', 'missing_json', 'bad_result', 'bad_csv', 'count', 'endpoint',
                 'nonfinite', 'zero_duration', 'wrong_source', 'wrong_protocol', 'escaped_path')
        for case in cases:
            with self.subTest(case=case), tempfile.TemporaryDirectory() as data_dir:
                run = write_teleoperation_run(data_dir)
                csv_path = os.path.join(run.output_dir, 'trial_001_trajectory.csv')
                result_path = os.path.join(run.output_dir, 'trial_001_result.json')
                if case == 'missing_csv':
                    os.unlink(csv_path)
                elif case == 'missing_json':
                    os.unlink(result_path)
                elif case == 'bad_result':
                    with open(result_path, 'w') as stream:
                        stream.write('[]')
                elif case == 'bad_csv':
                    with open(csv_path, 'w') as stream:
                        stream.write('just,header\n1,2\n')
                elif case == 'nonfinite':
                    with open(csv_path) as stream:
                        text = stream.read().replace('0.5,0.0,0.4', 'nan,0.0,0.4')
                    with open(csv_path, 'w') as stream:
                        stream.write(text)
                elif case == 'wrong_protocol':
                    self.change_result(run, lambda r: r.update(protocol='other'))
                else:
                    changes = {
                        'count': {'samples': 99}, 'endpoint': {'end_x_m': 0.6},
                        'zero_duration': {'completion_time_s': 0},
                        'wrong_source': {'input_source': 'serial'},
                        'escaped_path': {'trajectory_csv': '../outside.csv'},
                    }[case]
                    self.change_manifest(run, lambda m: m['trials'][0].update(changes))
                    self.change_result(run, lambda r: r.update(changes))
                runs, problems = dataset.scan_teleoperation_runs(data_dir)
                self.assertEqual(runs[0]['completed'], 0)
                self.assertEqual(runs[0]['attempts'], 0)
                self.assertEqual(problems, {'incomplete or inconsistent trial': 1})

    def test_corrupt_or_invalid_shapes_do_not_hide_other_sessions(self):
        invalid = ('{', '[]', '{"schema_version": 1}',
                   '{"schema_version": 1, "metadata": []}',
                   '{"schema_version": 1, "settings": NaN}')
        for content in invalid:
            with self.subTest(content=content), tempfile.TemporaryDirectory() as data_dir:
                bad = write_teleoperation_run(data_dir)
                write_teleoperation_run(data_dir, participant_id='P02')
                with open(bad.manifest_path, 'w') as stream:
                    stream.write(content)
                runs, problems = dataset.scan_teleoperation_runs(data_dir)
                self.assertEqual([r['participant_id'] for r in runs], ['P02'])
                self.assertEqual(problems, {'unreadable or invalid session': 1})

    def test_invalid_fields_in_otherwise_valid_manifest_are_reported(self):
        cases = [dict(metadata=[]), dict(settings=[]), dict(targets=[{}]), dict(trials={}),
                 dict(participant_id=42), dict(experiment_name=[]), dict(input_source={}),
                 dict(end_effector_reference='flange')]
        for changes in cases:
            with self.subTest(changes=changes), tempfile.TemporaryDirectory() as data_dir:
                run = write_teleoperation_run(data_dir)
                self.change_manifest(run, lambda m: m.update(changes))
                self.assertEqual(dataset.scan_teleoperation_runs(data_dir),
                                 ([], {'unreadable or invalid session': 1}))

    def test_invalid_trial_shapes_and_empty_completed_rows_are_skipped(self):
        with tempfile.TemporaryDirectory() as data_dir:
            run = write_teleoperation_run(data_dir, statuses=('empty_cancelled',))
            changes = dict(status='completed', completion_time_s=1.0, samples=0)
            self.change_manifest(run, lambda m: m['trials'].extend([None, [], 'invalid']))
            self.change_manifest(run, lambda m: m['trials'][0].update(changes))
            self.change_result(run, lambda r: r.update(changes))
            csv_path = os.path.join(run.output_dir, 'trial_001_trajectory.csv')
            with open(csv_path) as stream:
                header = stream.readline()
            with open(csv_path, 'w') as stream:
                stream.write(header)
            runs, problems = dataset.scan_teleoperation_runs(data_dir)
            self.assertEqual(runs[0]['completed'], 0)
            self.assertEqual(problems, {'incomplete or inconsistent trial': 4})

    def test_gazebo_plan_only_runs_are_excluded(self):
        with tempfile.TemporaryDirectory() as data_dir:
            write_teleoperation_run(data_dir, mode='gazebo', session_id='plan', statuses=(),
                                    metadata={'dry_run': True})
            self.assertEqual(dataset.scan_teleoperation_runs(data_dir), ([], {}))


class TrackingRunTests(unittest.TestCase):
    def test_lists_runs_newest_first(self):
        with tempfile.TemporaryDirectory() as data_dir:
            # Target 0 was redone, target 2 captured nothing.
            sessions = [{'target_index': 0, 'sample_count': 90},
                        {'target_index': 0, 'sample_count': 95},
                        {'target_index': 1, 'sample_count': 80},
                        {'target_index': 2, 'sample_count': 0}]
            for run_id, captured in (('run_20261001_1000', sessions), ('run_20261002_0900', [])):
                folder = os.path.join(data_dir, dataset.TRACKING_DIR, run_id)
                os.makedirs(folder)
                with open(os.path.join(folder, 'manifest.json'), 'w') as f:
                    json.dump({
                        'run_id': run_id,
                        'target_count': 10,
                        'settings': {'magnet': '12x12mm_stack', 'heights_mm': [10.0, 50.0],
                                     'magnet_offset_mm': 6.0},
                        'sessions': captured,
                    }, f)
            runs = dataset.list_tracking_runs(data_dir)
            self.assertEqual([r['run_id'] for r in runs],
                             ['run_20261002_0900', 'run_20261001_1000'])
            self.assertEqual(runs[1]['captured'], 2)
            self.assertEqual(runs[1]['magnet_offset_mm'], 6.0)
            self.assertEqual(runs[1]['heights_mm'], [10.0, 50.0])

    def test_no_tracking_folder(self):
        with tempfile.TemporaryDirectory() as data_dir:
            self.assertEqual(dataset.list_tracking_runs(data_dir), [])


if __name__ == '__main__':
    unittest.main()
