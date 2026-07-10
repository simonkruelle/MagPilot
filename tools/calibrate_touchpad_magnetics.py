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


RMSE_THRESHOLDS = {
    'EXCELLENT': 1e-8,
    'GOOD':      1e-6,
    'ACCEPTABLE': 1e-4,
}


def quality_badge(rmse):
    if rmse < RMSE_THRESHOLDS['EXCELLENT']:
        return 'EXCELLENT'
    if rmse < RMSE_THRESHOLDS['GOOD']:
        return 'GOOD'
    if rmse < RMSE_THRESHOLDS['ACCEPTABLE']:
        return 'ACCEPTABLE'
    return 'POOR — check recording coverage or re-run calibration'


def print_sensor_table(fit):
    """Print per-sensor RMSE and scale/offset ranges in a compact table."""
    rmse_ch   = fit['fit']['rmse_by_channel']
    mae_ch    = fit['fit']['mae_by_channel']
    scales    = fit['channel_scale']
    offsets   = fit['channel_offset']

    print()
    print(f"  {'Sensor':>6}  {'Bx RMSE':>12}  {'By RMSE':>12}  {'Bz RMSE':>12}  "
          f"{'scale min/max':>20}  {'offset min/max':>22}")
    print("  " + "-" * 90)
    for s in range(16):
        i = s * 3
        bx_rmse, by_rmse, bz_rmse = rmse_ch[i], rmse_ch[i+1], rmse_ch[i+2]
        s_min = min(scales[i:i+3])
        s_max = max(scales[i:i+3])
        o_min = min(offsets[i:i+3])
        o_max = max(offsets[i:i+3])
        print(
            f"  {s+1:>6}  {bx_rmse:>12.3e}  {by_rmse:>12.3e}  {bz_rmse:>12.3e}  "
            f"  [{s_min:+.3e} .. {s_max:+.3e}]  [{o_min:+.3e} .. {o_max:+.3e}]"
        )
    print()


def main():
    parser = argparse.ArgumentParser(
        description='Fit a touchpad magnetic calibration JSON from a recorded raw CSV.'
    )
    parser.add_argument('csv_path', help='Recorded raw CSV, e.g. data/lab_2026-05-20/raw/magcal_001_raw.csv')
    parser.add_argument(
        '--output',
        default=None,
        help='Calibration JSON output path (default: same folder as CSV, touchpad_magnetic_calibration.json)',
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

    print("=" * 60)
    print("TOUCHPAD MAGNETIC CALIBRATION")
    print("=" * 60)

    print(f"[1/4] Loading CSV: {args.csv_path}")
    rows = load_rows(args.csv_path)
    if not rows:
        raise SystemExit(f"No usable rows found in {args.csv_path}")
    print(f"      Loaded {len(rows)} rows with valid pose + magnetic data")

    print(f"[2/4] Fitting affine calibration  (projection_extent={args.projection_extent}, max_samples={args.max_samples})")
    fit = fit_affine_calibration(
        rows,
        projection_extent=args.projection_extent,
        max_samples=args.max_samples,
    )
    print(f"      Used {fit['sample_count']} samples for fit")

    output_path = args.output
    if output_path is None:
        csv_dir = os.path.dirname(os.path.abspath(args.csv_path))
        output_path = os.path.join(csv_dir, 'touchpad_magnetic_calibration.json')

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

    print(f"[3/4] Saving calibration JSON: {output_path}")
    save_magnetic_calibration(calibration, output_path)

    # Results summary
    global_rmse = calibration['fit']['global_rmse']
    global_mae  = calibration['fit']['global_mae']
    badge = quality_badge(global_rmse)

    print()
    print("[4/4] Fit quality summary")
    print(f"  Global RMSE   : {global_rmse:.4e}  ->  {badge}")
    print(f"  Global MAE    : {global_mae:.4e}")
    print(f"  Real B range  : [{calibration['fit']['real_min']:.4e} .. {calibration['fit']['real_max']:.4e}]")
    print(f"  Synth B range : [{calibration['fit']['synthetic_min']:.4e} .. {calibration['fit']['synthetic_max']:.4e}]")

    print_sensor_table(calibration)

    print("=" * 60)
    print("Calibration saved successfully.")
    print()
    print("Next step — use the calibration with the touchpad simulator:")
    print()
    print(f"  python magnetometer_reader.py \\")
    print(f"    --input-source touchpad \\")
    print(f"    --touchpad-magnetic-calibration {output_path}")
    print()
    print("Or with data recording:")
    print()
    print(f"  python magnetometer_reader.py \\")
    print(f"    --input-source touchpad \\")
    print(f"    --touchpad-magnetic-calibration {output_path} \\")
    print(f"    --record-data --output-dir data/lab_$(date +%Y-%m-%d)/")
    print("=" * 60)


if __name__ == '__main__':
    main()
