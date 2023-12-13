"""
Classes and Functions for geometry.
"""
from enum import Enum
from numbers import Number
import numpy as np

# Define an enumeration named Color
class Direction(Enum):
    """
    Class represetning direction
    """
    X = 1
    Y = 2
    Z = 3

def rotation_matrix(direction: Direction, angle: Number) -> np.ndarray:
    """
    Return a 3D rotation matrix about the X, Y, or Z-axis. 
    Follows this link on rotation matrices:
    """
    assert isinstance(direction, Direction)
    assert isinstance(angle, Number)

    cos_angle = np.cos(angle)
    sin_angle = np.sin(angle)
    if direction == Direction.X:
        matrix = np.array([
            [1, 0, 0],
            [0, cos_angle, -sin_angle],
            [0, sin_angle, cos_angle]
        ])
    elif direction == Direction.Y:
        matrix =  np.array([
            [cos_angle, 0, sin_angle],
            [0, 1, 0],
            [-sin_angle, 0, cos_angle]
        ])
    elif direction == Direction.Z:
        matrix =  np.array([
            [cos_angle, -sin_angle, 0],
            [sin_angle, cos_angle, 0],
            [0, 0, 1]
        ])
    else:
        assert False
    return matrix
