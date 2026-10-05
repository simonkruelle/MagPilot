#!/usr/bin/env python3
"""Tests for the participant registry and coverage counts (colmag/dataset.py)."""

import os as _os
import sys as _sys

_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
if _ROOT not in _sys.path:
    _sys.path.insert(0, _ROOT)

import json
import os
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
