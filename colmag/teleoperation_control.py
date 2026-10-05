"""Position-only input mappings for the 24 cm collection workspace."""

import math

from colmag.control_mapping import (
    MAGNET_HEIGHT_MAX_M, calibrated_magnet_height, map_magnet_height,
)
from colmag.robot_targets import DIGIT_CUBE_CENTER_M, DIGIT_CUBE_EDGE_M


def wheel_height_delta(delta=0, button=None, window_system='x11'):
    """One wheel notch changes Z by 1 mm; retain high-resolution wheel deltas."""
    if button in (4, 5):
        return 0.001 if button == 4 else -0.001
    delta = float(delta)
    if not math.isfinite(delta):
        raise ValueError('Wheel delta must be finite.')
    # Tk reports +/-120 per notch on Windows/X11 and unit deltas on macOS.
    notches = delta if window_system == 'aqua' else delta / 120
    return notches * 0.001


def pointer_position(u, v, height_m):
    """Map normalized robot-plane coordinates into the cube with bounded Z."""
    values = [float(u), float(v), float(height_m)]
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Input coordinates must be finite.')
    half = DIGIT_CUBE_EDGE_M / 2
    centre = DIGIT_CUBE_CENTER_M
    u, v = [max(-1.0, min(1.0, value)) for value in values[:2]]
    z = max(centre[2] - half, min(centre[2] + half, values[2]))
    return (centre[0] - v * half, centre[1] + u * half, z)


def magnet_position(pose, input_extent_m=0.05):
    """Reuse board axes and height; reject estimates outside the input area."""
    if len(pose) != 6 or not all(math.isfinite(float(value)) for value in pose):
        raise ValueError('A finite six-value board pose is required.')
    if not math.isfinite(input_extent_m) or input_extent_m <= 0:
        raise ValueError('Board input extent must be positive.')
    # An outlying estimate must not silently command a workspace corner. Allow
    # one micrometre for float32 values at the declared board-area boundary.
    if any(abs(float(value)) > input_extent_m + 1e-6 for value in pose[:2]):
        raise ValueError('Board estimate outside the ±{:g} mm input area.'.format(
            input_extent_m * 1000))
    height = calibrated_magnet_height(pose[2])
    if height > MAGNET_HEIGHT_MAX_M:
        raise ValueError('Magnet lifted above the 15 cm control range.')
    half = DIGIT_CUBE_EDGE_M / 2
    z = map_magnet_height(height, ee_min_m=DIGIT_CUBE_CENTER_M[2] - half,
                          ee_max_m=DIGIT_CUBE_CENTER_M[2] + half)
    return pointer_position(float(pose[0]) / input_extent_m,
                            float(pose[1]) / input_extent_m, z)
