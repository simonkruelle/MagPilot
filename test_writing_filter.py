#!/usr/bin/env python3
"""Smoke test air-writing velocity filtering without sensor hardware."""

from datetime import datetime, timedelta

from colmag.interaction import InputMode
from magnetometer_reader import MagnetometerReader


def make_row(timestamp, x, y, z):
    return [timestamp, *([0.0] * 48), x, y, z, 0.0, 0.0, 0.0]


def make_touch_row(timestamp, x, y, z, pen_down):
    return [timestamp, *([0.0] * 48), x, y, z, 1.0 if pen_down else 0.0, 0.0, 0.0]


def main():
    reader = MagnetometerReader(
        enable_classifier=False,
        writing_min_velocity=0.002,
        writing_max_velocity=0.1,
    )
    start = datetime(2026, 1, 1, 12, 0, 0)

    rows = [
        make_row(start + timedelta(seconds=0.0), 0.0, 0.035, 0.2),
        make_row(start + timedelta(seconds=0.1), 0.0, 0.035, 0.2),
        make_row(start + timedelta(seconds=0.2), 0.004, 0.035, 0.6),
        make_row(start + timedelta(seconds=0.3), 0.008, 0.035, 0.4),
        make_row(start + timedelta(seconds=0.4), 0.050, 0.050, 0.1),
        make_row(start + timedelta(seconds=0.5), 0.054, 0.050, 0.8),
        make_row(start + timedelta(seconds=0.6), 0.058, 0.050, 0.3),
    ]

    pose_x, pose_y, pose_z, timestamps = reader.extract_pose_trail(rows, include_timestamps=True)
    writing_mask, _, _ = reader.writing_sample_mask(pose_x, pose_y, pose_z, timestamps)
    assert writing_mask.tolist() == [False, True, True, False, False, True, True]

    image = reader.rows_to_digit_image(rows)
    assert image.min() < 1.0

    long_rows = [
        make_row(
            start + timedelta(seconds=index * 0.1),
            index * 0.003,
            0.0,
            0.2 + index * 0.01,
        )
        for index in range(12)
    ]
    short_reader = MagnetometerReader(
        enable_classifier=False,
        trail_length=5,
        writing_min_velocity=0.0,
    )
    pose_rows = short_reader.live_pose_rows(long_rows)
    assert len(pose_rows) == short_reader.trail_length

    pose_x, pose_y, pose_z, timestamps = short_reader.extract_pose_trail(
        pose_rows,
        include_timestamps=True,
    )
    writing_mask, _, _ = short_reader.writing_sample_mask(pose_x, pose_y, pose_z, timestamps)
    capture_mask = short_reader.classification_sample_mask(pose_rows)
    assert len(writing_mask) == len(capture_mask) == short_reader.trail_length
    assert (writing_mask & capture_mask).shape == writing_mask.shape

    short_reader.mark_classification_start(pose_rows[2][0])
    capture_mask = short_reader.classification_sample_mask(pose_rows)
    assert capture_mask.tolist() == [False, False, True, True, True]

    touch_reader = MagnetometerReader(
        enable_classifier=False,
        input_source='touchpad',
        touchpad_ink_mode='pen',
        trail_length=10,
        writing_min_velocity=0.0,
    )
    touch_rows = [
        make_touch_row(start + timedelta(seconds=0.0), 0.000, 0.035, 0.5, False),
        make_touch_row(start + timedelta(seconds=0.1), 0.005, 0.035, 0.5, True),
        make_touch_row(start + timedelta(seconds=0.2), 0.010, 0.035, 0.5, True),
        make_touch_row(start + timedelta(seconds=0.3), 0.015, 0.035, 0.5, False),
    ]
    pen_mask = touch_reader.touchpad_pen_mask(touch_rows)
    assert pen_mask.tolist() == [False, True, True, False]

    hover_image = touch_reader.rows_to_digit_image([touch_rows[0], touch_rows[-1]])
    assert hover_image.min() == 1.0
    touch_image = touch_reader.rows_to_digit_image(touch_rows)
    assert touch_image.min() < 0.25

    button = next(button for button in touch_reader.joystick.buttons if button.name == 'Letters')
    button_rows = [
        make_touch_row(start + timedelta(seconds=0.0), button.x, button.y, 0.5, True),
        make_touch_row(start + timedelta(seconds=0.1), button.x + 0.001, button.y, 0.5, True),
    ]
    button_image = touch_reader.rows_to_digit_image(button_rows)
    assert button_image.min() == 1.0

    mag_values = touch_reader.synthetic_magnetic_values(0.01, -0.01, 0.02)
    assert len(mag_values) == 48
    assert any(abs(value) > 0.0 for value in mag_values)

    velocity_touch_reader = MagnetometerReader(
        enable_classifier=False,
        input_source='touchpad',
        touchpad_ink_mode='velocity',
        writing_min_velocity=0.002,
        writing_max_velocity=0.2,
    )
    velocity_rows = [
        make_touch_row(start + timedelta(seconds=0.0), 0.000, 0.035, 0.5, False),
        make_touch_row(start + timedelta(seconds=0.1), 0.004, 0.035, 0.5, False),
        make_touch_row(start + timedelta(seconds=0.2), 0.008, 0.035, 0.5, False),
    ]
    assert velocity_touch_reader.touchpad_pen_mask(velocity_rows) is None
    velocity_image = velocity_touch_reader.rows_to_digit_image(velocity_rows)
    assert velocity_image.min() < 0.25

    persistent_reader = MagnetometerReader(
        enable_classifier=False,
        input_source='touchpad',
        touchpad_ink_mode='pen',
        trail_length=5,
        writing_min_velocity=0.0,
    )
    persistent_reader.app_controller.mode = InputMode.LETTERS
    persistent_reader.classification_started_at = start - timedelta(seconds=1)
    persistent_rows = [
        make_touch_row(
            start + timedelta(seconds=index * 0.01),
            index * 0.001,
            0.0,
            0.5,
            True,
        )
        for index in range(12)
    ]
    display_rows = persistent_reader.display_pose_rows(persistent_rows)
    assert len(display_rows) == len(persistent_rows)
    pose_x, _, _, _ = persistent_reader.extract_pose_trail(
        display_rows,
        trail_length=0,
        include_timestamps=True,
    )
    assert len(pose_x) == len(persistent_rows)

    print("Air-writing velocity filter smoke test passed")


if __name__ == "__main__":
    main()
