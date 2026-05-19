#!/usr/bin/env python3
"""Fit touchpad synthetic magnetic fields to a real sensor CSV recording."""

import argparse
import csv
import os
from datetime import datetime

import numpy as np

from colmag.magnetic_sim import (
    MAGNETIC_CALIBRATION_SCHEMA_VERSION,
    fit_affine_calibration,
    save_magnetic_calibration,
)


def mag_columns():
    columns = []
    for sensor in range(1, 17):
        columns.extend([
            f'Sensor{sensor}_Bx',
            f'Sensor{sensor}_By',
            f'Sensor{sensor}_Bz',
        ])
    return columns


def load_rows(csv_path):
    rows = []
    required = mag_columns() + ['Pose_x', 'Pose_y', 'Pose_z']
    with open(csv_path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row")
        missing = [column for column in required if column not in reader.fieldnames]
        if missing:
            raise ValueError(
                "CSV is missing required columns: " + ', '.join(missing)
            )

        for raw_row in reader:
            try:
                mag = [float(raw_row[column]) for column in mag_columns()]
                pose = [
                    float(raw_row['Pose_x']),
                    float(raw_row['Pose_y']),
                    float(raw_row['Pose_z']),
                ]
            except (TypeError, ValueError):
                continue
            rows.append({'mag': mag, 'pose': pose})
    return rows


def main():
    parser = argparse.ArgumentParser(
        description='Fit a touchpad magnetic calibration JSON from a recorded raw CSV.'
    )
    parser.add_argument('csv_path', help='Recorded raw CSV, e.g. data/lab_2026-05-20/raw/magcal_001_raw.csv')
    parser.add_argument(
        '--output',
        default=None,
        help='Calibration JSON output path (default: next to CSV as touchpad_magnetic_calibration.json)',
    )
    parser.add_argument(
        '--projection-extent',
        type=float,
        default=0.05,
        help='Half-width of the X/Y sensor pad extent used by the app (default: 0.05)',
    )
    parser.add_argument(
        '--max-samples',
        type=int,
        default=20000,
        help='Maximum rows to use, evenly subsampled if needed (default: 20000)',
    )
    args = parser.parse_args()

    if args.projection_extent <= 0:
        parser.error("--projection-extent must be positive")
    if args.max_samples is not None and args.max_samples < 4:
        parser.error("--max-samples must be at least 4")

    rows = load_rows(args.csv_path)
    if not rows:
        raise SystemExit(f"No usable rows found in {args.csv_path}")

    fit = fit_affine_calibration(
        rows,
        projection_extent=args.projection_extent,
        max_samples=args.max_samples,
    )
    output_path = args.output
    if output_path is None:
        output_path = os.path.join(
            os.path.dirname(args.csv_path),
            'touchpad_magnetic_calibration.json',
        )

    calibration = {
        'schema_version': MAGNETIC_CALIBRATION_SCHEMA_VERSION,
        'model': 'dipole_grid_affine_v1',
        'created_at': datetime.now().isoformat(),
        'source_csv': os.path.abspath(args.csv_path),
        'notes': (
            'Use with magnetometer_reader.py --input-source touchpad '
            '--touchpad-magnetic-calibration <this file>. '
            'This calibrates channel scale/offset for pipeline testing; it is not a full physics model.'
        ),
        **fit,
    }
    save_magnetic_calibration(calibration, output_path)

    print(f"Loaded rows: {len(rows)}")
    print(f"Fit samples: {calibration['sample_count']}")
    print(f"Global RMSE: {calibration['fit']['global_rmse']:.6g}")
    print(f"Global MAE:  {calibration['fit']['global_mae']:.6g}")
    print(f"Saved calibration: {output_path}")


if __name__ == '__main__':
    main()
