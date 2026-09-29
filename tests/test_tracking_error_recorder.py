#!/usr/bin/env python3
"""Offline checks for the magnet-tracking error recorder (no hardware)."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import csv
import json
import math
import os
import tempfile

import record_tracking_error as rte
from colmag.control_mapping import magnet_angles_degrees


def test_grid_is_symmetric_and_serpentine():
    targets = rte.build_targets(20.0, 40.0, (15.0,), ((0.0, 0.0),))
    assert len(targets) == 25
    assert [t['x_mm'] for t in targets[:5]] == [-40, -20, 0, 20, 40]
    assert [t['x_mm'] for t in targets[5:10]] == [40, 20, 0, -20, -40]
    assert len({t['label'] for t in targets}) == 25


def test_direction_matches_reader_angle_convention():
    for tilt, azimuth in ((0, 0), (20, 0), (30, 90), (45, -135)):
        got_tilt, got_azimuth = magnet_angles_degrees(*rte.direction_from_angles(tilt, azimuth))
        assert math.isclose(got_tilt, tilt, abs_tol=1e-6)
        if tilt:
            assert math.isclose(got_azimuth, azimuth, abs_tol=1e-6)


def test_simulated_run_writes_layout_and_small_errors():
    with tempfile.TemporaryDirectory() as tmpdir:
        targets = rte.build_targets(40.0, 40.0, (20.0,), ((0.0, 0.0), (20.0, 90.0)))
        recorder = rte.TrackingErrorRecorder(
            output_dir=tmpdir, run_id='sim test', targets=targets,
            settings={'settle_s': 0.05, 'capture_s': 0.2,
                      'sensor_bias_m': rte.MAGNET_HEIGHT_SENSOR_BIAS_M,
                      'height_min_m': rte.MAGNET_HEIGHT_MIN_M},
            source=rte.SimulatedSource(pos_noise_m=0.0005, dir_noise=0.01),
            input_source='simulated',
        )
        recorder.run(auto=True)

        with open(os.path.join(tmpdir, 'manifest.json'), encoding='utf-8') as f:
            manifest = json.load(f)
        assert manifest['task'] == 'magnet_tracking_error'
        assert manifest['run_id'] == 'sim_test'
        assert len(manifest['sessions']) == len(targets) == 18
        assert os.path.exists(os.path.join(tmpdir, manifest['raw_log']['path']))

        first = manifest['sessions'][0]
        with open(os.path.join(tmpdir, first['paths']['csv']), newline='') as f:
            header = next(csv.reader(f))
        assert header[:55] == rte.csv_header()
        assert header[55:] == rte.GT_COLUMNS

        with open(os.path.join(tmpdir, manifest['summary']), newline='') as f:
            rows = list(csv.DictReader(f))
        assert len(rows) == 18
        for row in rows:
            assert row['status'] == 'ok'
            assert float(row['err_xyz']) < 0.002
            assert float(row['err_angle_deg']) < 3.0


def test_skip_and_quit_prompts():
    with tempfile.TemporaryDirectory() as tmpdir:
        targets = rte.build_targets(40.0, 40.0, (20.0,), ((0.0, 0.0),))
        answers = iter(['s', '', 'q'])
        recorder = rte.TrackingErrorRecorder(
            output_dir=tmpdir, run_id='prompt', targets=targets,
            settings={'settle_s': 0.0, 'capture_s': 0.05,
                      'sensor_bias_m': rte.MAGNET_HEIGHT_SENSOR_BIAS_M,
                      'height_min_m': rte.MAGNET_HEIGHT_MIN_M},
            source=rte.SimulatedSource(), input_source='simulated',
            raw_csv=False, prompt=lambda _: next(answers),
        )
        recorder.run()
        with open(recorder.summary_path, newline='') as f:
            statuses = [row['status'] for row in csv.DictReader(f)]
        assert statuses == ['skipped', 'ok']


def test_dry_run_creates_manifest_only():
    with tempfile.TemporaryDirectory() as tmpdir:
        assert rte.main(['--dry-run', '--output-dir', tmpdir, '--run-id', 'dry']) == 0
        with open(os.path.join(tmpdir, 'manifest.json'), encoding='utf-8') as f:
            manifest = json.load(f)
        assert manifest['target_count'] == 75
        assert manifest['sessions'] == []
        assert os.listdir(os.path.join(tmpdir, 'samples')) == []
