#!/usr/bin/env python3
"""Smoke test air-writing velocity filtering without sensor hardware."""

from datetime import datetime, timedelta

from magnetometer_reader import MagnetometerReader


def make_row(timestamp, x, y, z):
    return [timestamp, *([0.0] * 48), x, y, z, 0.0, 0.0, 0.0]


def main():
    reader = MagnetometerReader(
        enable_classifier=False,
        writing_min_velocity=0.002,
        writing_max_velocity=0.1,
    )
    start = datetime(2026, 1, 1, 12, 0, 0)

    rows = [
        make_row(start + timedelta(seconds=0.0), 0.0, 0.0, 0.2),
        make_row(start + timedelta(seconds=0.1), 0.0, 0.0, 0.2),
        make_row(start + timedelta(seconds=0.2), 0.004, 0.0, 0.6),
        make_row(start + timedelta(seconds=0.3), 0.008, 0.0, 0.4),
        make_row(start + timedelta(seconds=0.4), 0.050, 0.050, 0.1),
        make_row(start + timedelta(seconds=0.5), 0.054, 0.050, 0.8),
        make_row(start + timedelta(seconds=0.6), 0.058, 0.050, 0.3),
    ]

    pose_x, pose_y, pose_z, timestamps = reader.extract_pose_trail(rows, include_timestamps=True)
    writing_mask, _, _ = reader.writing_sample_mask(pose_x, pose_y, pose_z, timestamps)
    assert writing_mask.tolist() == [False, True, True, False, False, True, True]

    image = reader.rows_to_digit_image(rows)
    assert image.min() < 1.0

    print("Air-writing velocity filter smoke test passed")


if __name__ == "__main__":
    main()
