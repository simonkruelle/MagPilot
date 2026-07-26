#!/usr/bin/env python3
"""Tests for safe, reloadable character-to-action customization."""

import json
import os
import sys
import tempfile
import unittest

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from colmag.action_mapping import (  # noqa: E402
    ACTION_IDS,
    ALL_LABELS,
    ActionMappingError,
    ReloadableActionMapping,
    action_legend_rows,
    bind_action_handlers,
    default_action_mapping,
    load_action_mapping,
    save_action_mapping,
    validate_action_mapping,
)


class ActionMappingTests(unittest.TestCase):
    def test_defaults_preserve_every_previous_robot_action(self):
        mapping = default_action_mapping()

        self.assertEqual(tuple(mapping), ALL_LABELS)
        self.assertEqual(mapping['0'], 'home')
        self.assertEqual(
            [mapping[str(digit)] for digit in range(1, 10)],
            ['cube_{}'.format(digit) for digit in range(1, 10)])
        expected = {
            'A': 'wave',
            'B': 'bow',
            'C': 'fist_pumps',
            'D': 'dab',
            'L': 'point_left',
            'R': 'point_right',
            'U': 'stretch_up',
            'X': 'home',
        }
        for label, action_id in expected.items():
            self.assertEqual(mapping[label], action_id)
        for label in set('ABCDEFGHIJKLMNOPQRSTUVWXYZ') - set(expected):
            self.assertEqual(mapping[label], 'nod_yes')

    def test_all_defaults_reference_curated_actions(self):
        self.assertTrue(
            set(default_action_mapping().values()).issubset(ACTION_IDS))

    def test_every_dropdown_action_binds_to_a_robot_handler(self):
        calls = []

        class FakeRobot:
            def __getattr__(self, method_name):
                return lambda: calls.append(method_name)

            def _go_to_digit(self, digit):
                calls.append(('cube', digit))

        handlers = bind_action_handlers(FakeRobot())

        self.assertEqual(tuple(handlers), ACTION_IDS)
        self.assertIsNone(handlers['none'])
        for action_id in ACTION_IDS:
            if action_id != 'none':
                self.assertTrue(callable(handlers[action_id]))
        handlers['wave']()
        handlers['cube_9']()
        self.assertEqual(calls, ['_wave', ('cube', 9)])

    def test_atomic_round_trip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'robot_actions.json')
            mapping = default_action_mapping()
            mapping['A'] = 'cheer'

            saved = save_action_mapping(path, mapping)

            self.assertEqual(saved, mapping)
            self.assertEqual(load_action_mapping(path), mapping)
            self.assertFalse(
                [name for name in os.listdir(directory)
                 if name.startswith('.robot_actions.')])

    def test_invalid_bindings_are_rejected(self):
        cases = (
            (lambda mapping: mapping.pop('A'), 'missing character bindings'),
            (lambda mapping: mapping.update({'?': 'wave'}),
             'unknown character bindings'),
            (lambda mapping: mapping.update({'A': 'unvalidated_move'}),
             'unknown action'),
        )
        for mutation, message in cases:
            mapping = default_action_mapping()
            mutation(mapping)
            with self.subTest(message=message):
                with self.assertRaisesRegex(ActionMappingError, message):
                    validate_action_mapping(mapping)

    def test_reload_keeps_last_valid_mapping_after_invalid_edit(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'robot_actions.json')
            first = default_action_mapping()
            first['A'] = 'cheer'
            save_action_mapping(path, first)
            store = ReloadableActionMapping(path)

            self.assertTrue(store.refresh())
            self.assertEqual(store.mapping['A'], 'cheer')

            with open(path, 'w', encoding='utf-8') as handle:
                handle.write('{"schema_version": 1, "bindings": {}}')
            self.assertFalse(store.refresh())
            self.assertTrue(store.last_error)
            self.assertEqual(store.mapping['A'], 'cheer')

            second = default_action_mapping()
            second['A'] = 'bow'
            save_action_mapping(path, second)
            self.assertTrue(store.refresh())
            self.assertIsNone(store.last_error)
            self.assertEqual(store.mapping['A'], 'bow')

    def test_missing_file_starts_with_defaults_without_error(self):
        with tempfile.TemporaryDirectory() as directory:
            store = ReloadableActionMapping(
                os.path.join(directory, 'missing.json'))

            self.assertFalse(store.refresh())
            self.assertIsNone(store.last_error)
            self.assertEqual(store.mapping, default_action_mapping())

    def test_deleted_file_keeps_the_last_valid_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'robot_actions.json')
            mapping = default_action_mapping()
            mapping['A'] = 'cheer'
            save_action_mapping(path, mapping)
            store = ReloadableActionMapping(path)
            store.refresh()

            os.unlink(path)

            self.assertFalse(store.refresh())
            self.assertIn('mapping file is missing', store.last_error)
            self.assertEqual(store.mapping['A'], 'cheer')

    def test_wrong_schema_keeps_last_valid_mapping(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, 'robot_actions.json')
            save_action_mapping(path, default_action_mapping())
            store = ReloadableActionMapping(path)
            store.refresh()
            with open(path, 'r', encoding='utf-8') as handle:
                payload = json.load(handle)
            payload['schema_version'] = 99
            with open(path, 'w', encoding='utf-8') as handle:
                json.dump(payload, handle)

            self.assertFalse(store.refresh())
            self.assertIn('unsupported schema_version', store.last_error)
            self.assertEqual(store.mapping, default_action_mapping())

    def test_default_legend_is_compact_and_mode_specific(self):
        mapping = default_action_mapping()

        letter_rows = action_legend_rows(mapping, 'letters')
        digit_rows = action_legend_rows(mapping, 'digits')

        self.assertIn(('A', 'Wave'), letter_rows)
        self.assertIn(('L', 'Point left'), letter_rows)
        self.assertTrue(any(
            action == 'Nod yes' and 'E' in labels
            for labels, action in letter_rows))
        self.assertIn(('1-8', 'Cube corners'), digit_rows)
        self.assertIn(('9', 'Cube center'), digit_rows)
        self.assertIn(('0', 'Home / reset'), digit_rows)

    def test_legend_reflects_a_custom_binding(self):
        mapping = default_action_mapping()
        mapping['A'] = 'shake_no'

        self.assertIn(
            ('A', 'Shake no'), action_legend_rows(mapping, 'letters'))


if __name__ == '__main__':
    unittest.main()
