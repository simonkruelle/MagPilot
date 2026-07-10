#!/usr/bin/env python3
"""Smoke test structured run/session data recording without sensor hardware."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import glob
import json
import os
import tempfile
from datetime import datetime, timedelta

from magnetometer_reader import MagnetometerReader


def make_row(timestamp, x, y, z):
    return [timestamp.isoformat(), *([0.0] * 48), x, y, z, 0.0, 0.0, 0.0]


def make_rows():
    start = datetime(2026, 5, 20, 12, 0, 0)
    return [
        make_row(start + timedelta(seconds=0.0), 0.000, 0.000, 0.02),
        make_row(start + timedelta(seconds=0.1), 0.005, 0.000, 0.03),
        make_row(start + timedelta(seconds=0.2), 0.010, 0.004, 0.04),
        make_row(start + timedelta(seconds=0.3), 0.014, 0.008, 0.05),
    ]


def load_manifest(output_dir):
    with open(os.path.join(output_dir, 'manifest.json'), 'r', encoding='utf-8') as f:
        return json.load(f)


def test_live_mode_does_not_create_layout():
    with tempfile.TemporaryDirectory() as tmpdir:
        output_dir = os.path.join(tmpdir, 'live')
        MagnetometerReader(
            enable_classifier=False,
            output_dir=output_dir,
            run_id='live_test',
        )
        assert not os.path.exists(output_dir)


def test_dry_run_layout():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            dry_run=True,
            output_dir=tmpdir,
            run_id='dry_test',
        )

        assert not os.path.exists(os.path.join(tmpdir, 'manifest.json'))
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)

        assert os.path.isdir(os.path.join(tmpdir, 'raw'))
        assert os.path.isdir(os.path.join(tmpdir, 'samples'))
        manifest = load_manifest(tmpdir)
        assert manifest['run_id'] == 'dry_test'
        assert manifest['dry_run'] is True
        assert manifest['raw_log']['enabled'] is False
        assert manifest['sessions'] == []


def test_recording_artifacts_and_manifest():
    with tempfile.TemporaryDirectory() as tmpdir:
        raw_path = os.path.join(tmpdir, 'raw', 'unit_run_raw.csv')
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='unit_run',
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_path=raw_path, raw_csv_enabled=True)

        reader.current_session = 'letter_A'
        reader.current_session_started_at = make_rows()[0][0]
        reader.session_data = make_rows()
        reader.stop_session()

        label_dir = os.path.join(tmpdir, 'samples', 'letter_A')
        csv_files = glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep001_*.csv'))
        png_files = glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep001_*.png'))
        json_files = glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep001_*.json'))
        assert len(csv_files) == len(png_files) == len(json_files) == 1

        manifest = load_manifest(tmpdir)
        assert manifest['raw_log']['enabled'] is True
        assert manifest['raw_log']['path'] == os.path.join('raw', 'unit_run_raw.csv')
        assert len(manifest['sessions']) == 1
        entry = manifest['sessions'][0]
        assert entry['label'] == 'letter_A'
        assert entry['repetition'] == 1
        assert entry['paths']['csv'].startswith(os.path.join('samples', 'letter_A'))
        assert entry['paths']['png'].endswith('.png')
        assert entry['paths']['json'].endswith('.json')

        reader.current_session = 'letter_A'
        reader.current_session_started_at = make_rows()[0][0]
        reader.session_data = make_rows()
        reader.stop_session()
        assert glob.glob(os.path.join(label_dir, 'unit_run_letter_A_rep002_*.csv'))


def test_control_labels_are_separate():
    with tempfile.TemporaryDirectory() as tmpdir:
        reader = MagnetometerReader(
            enable_classifier=False,
            record_data=True,
            output_dir=tmpdir,
            run_id='controls',
            writing_min_velocity=0.0,
        )
        reader.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)

        reader.handle_keypress('-')
        assert reader.current_session == 'control_blank'
        reader.session_data = make_rows()
        reader.handle_keypress('s')

        reader.handle_keypress('=')
        assert reader.current_session == 'control_still'
        reader.session_data = make_rows()
        reader.handle_keypress('s')

        assert os.path.isdir(os.path.join(tmpdir, 'samples', 'control_blank'))
        assert os.path.isdir(os.path.join(tmpdir, 'samples', 'control_still'))
        manifest = load_manifest(tmpdir)
        labels = [entry['label'] for entry in manifest['sessions']]
        assert labels == ['control_blank', 'control_still']


def main():
    test_live_mode_does_not_create_layout()
    test_dry_run_layout()
    test_recording_artifacts_and_manifest()
    test_control_labels_are_separate()
    print("Structured data recording smoke test passed")


if __name__ == "__main__":
    main()
