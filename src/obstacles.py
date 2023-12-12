"""
Obstacles
"""
import numpy as np
from polygon import Sphere

obstacles = [
    Sphere(.1, np.array([[.5], [.3], [.3]]))
]

goal_cartesian_point = (0.1, .5, .3)

