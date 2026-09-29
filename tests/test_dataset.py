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

    def test_damaged_file_raises_instead_of_resetting(self):
        with tempfile.TemporaryDirectory() as data_dir:
            with open(os.path.join(data_dir, dataset.PARTICIPANTS_FILE), 'w') as f:
                f.write('{not json')
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
            self.assertEqual(problems, {'no height (counted as 0 mm)': 1})


class TrackingRunTests(unittest.TestCase):
    def test_lists_runs_newest_first(self):
        with tempfile.TemporaryDirectory() as data_dir:
            for run_id, captured in (('run_20261001_1000', 2), ('run_20261002_0900', 0)):
                folder = os.path.join(data_dir, dataset.TRACKING_DIR, run_id)
                os.makedirs(folder)
                with open(os.path.join(folder, 'manifest.json'), 'w') as f:
                    json.dump({
                        'run_id': run_id,
                        'target_count': 10,
                        'settings': {'magnet': '12x12mm_stack', 'heights_mm': [10.0, 50.0]},
                        'sessions': [{}] * captured,
                    }, f)
            runs = dataset.list_tracking_runs(data_dir)
            self.assertEqual([r['run_id'] for r in runs],
                             ['run_20261002_0900', 'run_20261001_1000'])
            self.assertEqual(runs[1]['captured'], 2)
            self.assertEqual(runs[1]['heights_mm'], [10.0, 50.0])

    def test_no_tracking_folder(self):
        with tempfile.TemporaryDirectory() as data_dir:
            self.assertEqual(dataset.list_tracking_runs(data_dir), [])


if __name__ == '__main__':
    unittest.main()
