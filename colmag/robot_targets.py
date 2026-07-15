"""Cartesian targets used by the discrete robot gesture commands."""


DIGIT_CUBE_CENTER_M = (0.45, 0.0, 0.40)
DIGIT_CUBE_EDGE_M = 0.24

# Digits 1..4 circle the lower face; 5..8 repeat that order on the upper
# face. Digit 9 occupies the center of the cube.
_DIGIT_CUBE_DIRECTIONS = (
    (-1.0, 1.0, -1.0),
    (1.0, 1.0, -1.0),
    (1.0, -1.0, -1.0),
    (-1.0, -1.0, -1.0),
    (-1.0, 1.0, 1.0),
    (1.0, 1.0, 1.0),
    (1.0, -1.0, 1.0),
    (-1.0, -1.0, 1.0),
    (0.0, 0.0, 0.0),
)


def digit_cube_target(digit, center=DIGIT_CUBE_CENTER_M,
                      edge_m=DIGIT_CUBE_EDGE_M):
    """Return the Cartesian cube target for a digit from 1 through 9."""
    digit = int(digit)
    if digit < 1 or digit > 9:
        raise ValueError('digit must be between 1 and 9')
    center = tuple(float(value) for value in center)
    if len(center) != 3:
        raise ValueError('cube center must contain x, y, and z')
    edge_m = float(edge_m)
    if edge_m <= 0.0:
        raise ValueError('cube edge length must be positive')

    half = 0.5 * edge_m
    direction = _DIGIT_CUBE_DIRECTIONS[digit - 1]
    return tuple(center[axis] + half * direction[axis] for axis in range(3))
