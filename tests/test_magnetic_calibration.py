#!/usr/bin/env python3
"""Smoke test touchpad magnetic calibration fitting."""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import csv
import os
import tempfile

import numpy as np

from calibrate_touchpad_magnetics import load_rows, mag_columns
from colmag.magnetic_sim import (
    apply_magnetic_calibration,
    fit_affine_calibration,
    load_magnetic_calibration,
    save_magnetic_calibration,
    synthetic_dipole_values,
)
from magnetometer_reader import MagnetometerReader


def write_calibration_csv(path, projection_extent=0.05):
    scale = 2.5e-6
    offset = -0.012
    poses = [
        (-0.030, -0.030, 0.020),
        (-0.015, 0.020, 0.025),
        (0.000, 0.000, 0.030),
        (0.020, -0.010, 0.035),
        (0.035, 0.025, 0.040),
    ]
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', *mag_columns(), 'Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz'])
        for index, pose in enumerate(poses):
            base = synthetic_dipole_values(*pose, projection_extent=projection_extent, magnetic_scale=1.0)
            mag = [value * scale + offset for value in base]
            writer.writerow([
                f'2026-05-20T12:00:0{index}',
                *mag,
                *pose,
                0.0,
                0.0,
                0.0,
            ])
    return scale, offset, poses[2]


def main():
    with tempfile.TemporaryDirectory() as tmpdir:
        csv_path = os.path.join(tmpdir, 'magcal_raw.csv')
        json_path = os.path.join(tmpdir, 'touchpad_magnetic_calibration.json')
        expected_scale, expected_offset, sample_pose = write_calibration_csv(csv_path)

        rows = load_rows(csv_path)
        calibration = fit_affine_calibration(rows, projection_extent=0.05)
        calibration.update({
            'schema_version': 1,
            'model': 'dipole_grid_affine_v1',
        })
        save_magnetic_calibration(calibration, json_path)
        loaded = load_magnetic_calibration(json_path)

        assert np.allclose(loaded['channel_scale'], expected_scale, rtol=1e-6, atol=1e-12)
        assert np.allclose(loaded['channel_offset'], expected_offset, rtol=1e-6, atol=1e-12)

        base = synthetic_dipole_values(*sample_pose, projection_extent=0.05, magnetic_scale=1.0)
        calibrated = apply_magnetic_calibration(base, loaded)
        expected = [value * expected_scale + expected_offset for value in base]
        assert np.allclose(calibrated, expected)

        reader = MagnetometerReader(
            enable_classifier=False,
            input_source='touchpad',
            touchpad_magnetic_calibration=json_path,
        )
        reader_values = reader.synthetic_magnetic_values(*sample_pose)
        assert np.allclose(reader_values, expected)

    print("Touchpad magnetic calibration smoke test passed")


if __name__ == "__main__":
    main()
