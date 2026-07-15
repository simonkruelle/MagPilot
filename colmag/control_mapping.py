"""Shared magnet-orientation and height mappings for UI and robot control."""

import math


MAGNET_HEIGHT_MIN_M = 0.007
MAGNET_HEIGHT_MAX_M = 0.150
MAGNET_HEIGHT_GAIN = 2.0
MAGNET_HEIGHT_SENSOR_BIAS_M = 0.010

# The IK target is the flange. With the hand mounted, approximately 0.120 m of
# flange height puts the fingertips at ground level; retain 1 mm clearance.
FLANGE_GROUND_OFFSET_M = 0.120
EE_HEIGHT_MIN_M = FLANGE_GROUND_OFFSET_M + 0.001
EE_HEIGHT_MAX_M = 0.680


def magnet_angles_degrees(mx, my, mz):
    """Return dipole tilt and board-plane azimuth from a direction vector."""
    mx, my, mz = float(mx), float(my), float(mz)
    norm = math.sqrt(mx * mx + my * my + mz * mz)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError('Magnet orientation vector is invalid.')
    mx, my, mz = mx / norm, my / norm, mz / norm
    theta = math.degrees(math.acos(max(-1.0, min(1.0, mz))))
    phi = math.degrees(math.atan2(mx, my))
    return theta, phi


def fold_robot_twist_degrees(phi_deg):
    """Fold raw azimuth into a continuous -90..90 robot wrist command.

    The magnet visualizer keeps the full -180..180 azimuth. The robot wrist is
    symmetric over 180 degrees, so folding its control signal prevents the
    numeric jump when raw phi crosses from +180 to -180 degrees.
    """
    phi = (float(phi_deg) + 180.0) % 360.0 - 180.0
    if abs(phi) > 90.0:
        phi = math.copysign(180.0 - abs(phi), phi)
    return phi


def calibrated_magnet_height(
        raw_height_m,
        sensor_bias_m=MAGNET_HEIGHT_SENSOR_BIAS_M,
        minimum_m=MAGNET_HEIGHT_MIN_M):
    """Convert the sensor's biased Z estimate to physical height over sensors."""
    height = abs(float(raw_height_m)) - float(sensor_bias_m)
    return max(float(minimum_m), height)


def normalized_height_fraction(
        height_m,
        sensor_min_m=MAGNET_HEIGHT_MIN_M,
        sensor_max_m=MAGNET_HEIGHT_MAX_M,
        gain=MAGNET_HEIGHT_GAIN):
    """Map sensor height to 0..1 with decreasing sensitivity near the top."""
    sensor_min_m = float(sensor_min_m)
    sensor_max_m = float(sensor_max_m)
    gain = float(gain)
    if sensor_max_m <= sensor_min_m:
        raise ValueError('sensor_max_m must be greater than sensor_min_m')
    if gain < 0.0:
        raise ValueError('height gain must be non-negative')

    linear = (abs(float(height_m)) - sensor_min_m) / (
        sensor_max_m - sensor_min_m)
    linear = max(0.0, min(1.0, linear))
    if gain <= 1e-9:
        return linear
    return -math.expm1(-gain * linear) / -math.expm1(-gain)


def map_magnet_height(
        height_m,
        sensor_min_m=MAGNET_HEIGHT_MIN_M,
        sensor_max_m=MAGNET_HEIGHT_MAX_M,
        ee_min_m=EE_HEIGHT_MIN_M,
        ee_max_m=EE_HEIGHT_MAX_M,
        gain=MAGNET_HEIGHT_GAIN):
    """Map measured magnet height to the robot flange target height."""
    if float(ee_max_m) <= float(ee_min_m):
        raise ValueError('ee_max_m must be greater than ee_min_m')
    fraction = normalized_height_fraction(
        height_m, sensor_min_m, sensor_max_m, gain)
    return float(ee_min_m) + fraction * (float(ee_max_m) - float(ee_min_m))
