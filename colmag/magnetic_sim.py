"""Synthetic magnetic field helpers for touchpad/offline simulation."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


MAGNETIC_CALIBRATION_SCHEMA_VERSION = 1


def synthetic_dipole_values(
    pose_x,
    pose_y,
    pose_z,
    projection_extent=0.05,
    magnetic_scale=1.0,
):
    """Generate a simple 4x4 dipole-like magnetic field vector."""
    sensor_axis = np.linspace(
        -float(projection_extent),
        float(projection_extent),
        4,
        dtype=float,
    )
    values = []
    magnet_z = max(abs(float(pose_z)), 1e-3)
    moment_z = 1.0

    for sensor_y in sensor_axis:
        for sensor_x in sensor_axis:
            rx = float(sensor_x) - float(pose_x)
            ry = float(sensor_y) - float(pose_y)
            rz = -magnet_z
            r_sq = rx * rx + ry * ry + rz * rz + 1e-9
            r = r_sq ** 0.5
            r3 = r_sq * r
            r5 = r3 * r_sq
            dot = moment_z * rz
            bx = 3.0 * rx * dot / r5
            by = 3.0 * ry * dot / r5
            bz = 3.0 * rz * dot / r5 - moment_z / r3
            values.extend([
                bx * magnetic_scale,
                by * magnetic_scale,
                bz * magnetic_scale,
            ])

    return values


def load_magnetic_calibration(path):
    """Load a touchpad magnetic calibration JSON file."""
    with open(path, 'r', encoding='utf-8') as f:
        calibration = json.load(f)

    scales = calibration.get('channel_scale')
    offsets = calibration.get('channel_offset')
    if not isinstance(scales, list) or not isinstance(offsets, list):
        raise ValueError("Calibration must contain channel_scale and channel_offset arrays")
    if len(scales) != 48 or len(offsets) != 48:
        raise ValueError("Calibration arrays must contain 48 values each")
    return calibration


def apply_magnetic_calibration(values, calibration):
    """Apply per-channel affine calibration to synthetic magnetic values."""
    values = np.asarray(values, dtype=float)
    scales = np.asarray(calibration['channel_scale'], dtype=float)
    offsets = np.asarray(calibration['channel_offset'], dtype=float)
    calibrated = values * scales + offsets
    return calibrated.tolist()


def fit_affine_calibration(rows, projection_extent=0.05, max_samples=None):
    """Fit real magnetic channels as offset + scale * unit synthetic field."""
    if max_samples is not None and len(rows) > max_samples:
        indices = np.linspace(0, len(rows) - 1, int(max_samples), dtype=int)
        rows = [rows[index] for index in indices]

    synthetic = []
    real = []
    for row in rows:
        pose_x, pose_y, pose_z = row['pose']
        mag_values = row['mag']
        if not np.all(np.isfinite(mag_values)) or not np.all(np.isfinite(row['pose'])):
            continue
        synthetic.append(
            synthetic_dipole_values(
                pose_x,
                pose_y,
                pose_z,
                projection_extent=projection_extent,
                magnetic_scale=1.0,
            )
        )
        real.append(mag_values)

    if len(synthetic) < 4:
        raise ValueError("Need at least 4 finite calibration samples")

    synthetic = np.asarray(synthetic, dtype=float)
    real = np.asarray(real, dtype=float)
    channel_scale = []
    channel_offset = []
    predicted = np.zeros_like(real)

    for channel in range(real.shape[1]):
        design = np.column_stack([synthetic[:, channel], np.ones(len(synthetic))])
        scale, offset = np.linalg.lstsq(design, real[:, channel], rcond=None)[0]
        channel_scale.append(float(scale))
        channel_offset.append(float(offset))
        predicted[:, channel] = synthetic[:, channel] * scale + offset

    error = predicted - real
    rmse_by_channel = np.sqrt(np.mean(error * error, axis=0))
    mae_by_channel = np.mean(np.abs(error), axis=0)

    return {
        'sample_count': int(len(synthetic)),
        'projection_extent': float(projection_extent),
        'channel_scale': channel_scale,
        'channel_offset': channel_offset,
        'fit': {
            'global_rmse': float(np.sqrt(np.mean(error * error))),
            'global_mae': float(np.mean(np.abs(error))),
            'rmse_by_channel': rmse_by_channel.astype(float).tolist(),
            'mae_by_channel': mae_by_channel.astype(float).tolist(),
            'real_min': float(np.min(real)),
            'real_max': float(np.max(real)),
            'synthetic_min': float(np.min(synthetic)),
            'synthetic_max': float(np.max(synthetic)),
        },
    }


def save_magnetic_calibration(calibration, path):
    """Write calibration JSON with a stable schema."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', encoding='utf-8') as f:
        json.dump(calibration, f, indent=2)
        f.write('\n')
