"""
Classes and functions for Polygons and Edges
"""
import math
from numbers import Number
from abc import ABC, abstractmethod
import numpy as np


class Polygon(ABC):
    """
    Class for plotting, drawing, checking visibility and collision with
    polygons.
    """
    _ID = 0
    def __init__(self, hollow=False):
        """
        Save the input coordinates to the internal attribute  vertices.
        """
        self._hollow = hollow
        self._id = Polygon._ID
        Polygon._ID += 1

    @abstractmethod
    def is_collision(self, test_points: np.ndarray) -> bool:
        """
        Checks whether the any point is in collsion with a polygon (that is,
        inside for a filled in polygon, and outside for a hollow polygon). In
        the context of this homework, this function is best implemented using
        Polygon.is_visible.
        """

class Cylinder(Polygon):
    """"
    Object representing Cylinder in 3D space. 
    Segments of Cylinders will be used to represent the robot. 
    """
    def __init__(self, radius: Number, height: Number):
        super().__init__(hollow=False)
        assert isinstance(radius)
        assert isinstance(height)
        self.radius = radius
        self.height = height
        self._rotation = None
        self._translation = None
    def get_body_frame(self, rotation: np.ndarray, translation: np.ndarray) -> None:
        """
        It is assume that every cylinder will have its own body frame.
        such that the center of the bottom face of the cylinder is at the origin and the 
        cylinder extends in the z-direction for self.height meters. 
        Outside of the Cylinder class, the the rotation matrix and the translation 
        vector iwll be calculate
        """
        assert (
            isinstance(rotation, np.ndarray)
            and rotation.shape == (3,3)
            and np.allclose(rotation @ rotation.T, np.eye(3))
        )
        assert (isinstance(translation, np.ndarray) and translation.shape == (1,3))
        self._rotation = rotation
        self._translation = translation
    
    def _collides_with_point(self, point: np.ndarray) -> bool:
        """
        Boolean function to determine if point collides with cylinder.
        Point is assumed to be expressed in the body frame. 
        """
        assert (isinstance(point, np.ndarray)
            and point.shape == (1,3)
        )
        xy_distance = math.sqrt(point[0] ** 2 + point[1] ** 2)
        return xy_distance <= self.radius and 0 <= point[3] <= self.height

    def is_collision(self, test_points: np.ndarray) -> bool:
        """
        Checks whether the any point in test_points is in collsion with the cylinder
        """
        assert self._rotation is not None
        assert self._translation is not None
        assert (isinstance(test_points, np.ndarray)
            and len(test_points.shape) == 2
            and test_points.shape[1] == 3
        )
        for point in test_points.T:
            point_in_body_frame = self._rotation.T @ (point - self._translation)
            if self._collides_with_point(point_in_body_frame):
                return True
        return False


class Sphere(Polygon):
    """"
    Object representing Sphere in 3D space. Sphere should be in world boyd freom
    """
    def __init__(self, radius: Number, center: np.ndarray, hollow=False):
        super().__init__(hollow)
        self.radius = radius
        assert isinstance(center, np.ndarray) and center.shape == (1,3)
        self.center = center
    

    def is_collision(self, test_points: np.ndarray) -> bool:
        """
        Checks whether the any point in test_points is in collsion with the sphere
        """
        assert (isinstance(test_points, np.ndarray)
            and test_points.ndim == 2
            and test_points.shape[1] == 3
        )
        for point in test_points.T:
            dist_from_center = np.linalg.norm(point - self.center)
            if (self.hollow and dist_from_center > self.radius) or
                (not self.hollow and dist_from_center <= self.radius):
                return True
        return False