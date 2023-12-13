"""
Classes and functions for Polygons and Edges
"""
import math
from numbers import Number
from abc import ABC, abstractmethod
import matplotlib.pyplot as plt
import numpy as np
import geometry


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
    @abstractmethod
    def plot(self, ax, color='b') -> None:
        """
        Abstract method for plotting polygon in 3D space. 
        """

class Sphere(Polygon):
    """"
    Object representing Sphere in 3D space. Sphere should be in world boyd freom
    """
    def __init__(self, radius: Number, center: np.ndarray, hollow=False):
        super().__init__(hollow)
        self.radius = radius
        assert isinstance(center, np.ndarray) and center.shape == (3,1)
        self.center = center
    def is_collision(self, test_points: np.ndarray) -> bool:
        """
        Checks whether the any point in test_points is in collsion with the sphere
        """
        assert (isinstance(test_points, np.ndarray)
            and test_points.ndim == 2
            and test_points.shape[0] == 3
        )
        for point in test_points.T:
            dist_from_center = np.linalg.norm(point - self.center)
            within_sphere = dist_from_center <= self.radius
            filled_in = not self._hollow
            if ((filled_in and within_sphere) or
                (not filled_in and within_sphere)):
                print(point, 'in sphere')
                return True
        return False

    def plot(self, ax, color='b') -> None:
        u = np.linspace(0, 2 * np.pi, 50)
        v = np.linspace(0, np.pi, 50)
        x = self.center[0][0] + self.radius * np.outer(np.cos(u), np.sin(v))
        y = self.center[1][0] + self.radius * np.outer(np.sin(u), np.sin(v))
        z = self.center[2][0] + self.radius * np.outer(np.ones(np.size(u)), np.cos(v))

        ax.plot_surface(x, y, z, color=color, alpha=0.6)

class Cylinder(Polygon):
    """"
    Object representing Cylinder in 3D space. 
    Segments of Cylinders will be used to represent the robot. 
    """
    def __init__(self, radius: Number, height: Number):
        super().__init__(hollow=False)
        assert isinstance(radius, Number)
        assert isinstance(height, Number)
        self.radius = radius
        self.height = height
        self._rotation = None
        self._translation = None
    def plot(self, ax, color='r') -> None:
        """
        Plot Cylinder
        """
        # Create points for the cylinder surface
        num_points = 50
        phi = np.linspace(0, 2 * np.pi, num_points)
        z = np.linspace(0, self.height, num_points)
        phi, z = np.meshgrid(phi, z)

        # Parametric equations for the surface of a cylinder
        x = self.radius * np.cos(phi)
        y = self.radius * np.sin(phi)
        # Transform the points based on the orientation and position
        points = np.array([x.flatten(), y.flatten(), z.flatten()])
        transformed_points = np.dot(self._rotation, points) + self._translation

        # Reshape the points back to the original shape
        x = np.reshape(transformed_points[0, :], (num_points, num_points))
        y = np.reshape(transformed_points[1, :], (num_points, num_points))
        z = np.reshape(transformed_points[2, :], (num_points, num_points))

        # Plot the surface
        ax.plot_surface(x, y, z, color=color, alpha=0.6)

    def set_body_frame(self, rotation: np.ndarray, translation: np.ndarray) -> None:
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
        assert (isinstance(translation, np.ndarray) and translation.shape == (3,1))
        self._rotation = rotation
        self._translation = translation

    def get_body_frame(self):
        """
        It is assume that every cylinder will have its own body frame.
        such that the center of the bottom face of the cylinder is at the origin and the 
        cylinder extends in the z-direction for self.height meters. 
        Outside of the Cylinder class, the the rotation matrix and the translation 
        vector iwll be calculate
        """
        return self._rotation, self._translation
    def _collides_with_point(self, point: np.ndarray) -> bool:
        """
        Boolean function to determine if point collides with cylinder.
        Point is assumed to be expressed in the body frame. 
        """
        assert (isinstance(point, np.ndarray)
            and point.shape == (3,1)
        ), f"{point} is wrong shape"
        xy_distance = math.sqrt(point[0, 0] ** 2 + point[1, 0] ** 2)
        return xy_distance <= self.radius and 0 <= point[2, 0] <= self.height

    def is_collision(self, test_points: np.ndarray) -> bool:
        """
        Checks whether the any point in test_points is in collsion with the cylinder
        """
        assert self._rotation is not None
        assert self._translation is not None
        assert (isinstance(test_points, np.ndarray)
            and test_points.ndim == 2
            and test_points.shape[0] == 3
        )
        for point in test_points.T:
            point = point.reshape(3,1)
            point_in_body_frame = self._rotation.T @ (point - self._translation)
            if self._collides_with_point(point_in_body_frame):
                print(point, 'in cylinder')
                return True
        return False
    def collides_with_sphere(self, sphere: Sphere) -> bool:
        """
        Returns true if cylinder collides with sphere.
        """
        # Center of sphere wrt to the cylinder
        sphere_center = self._rotation.T @ (sphere.center - self._translation)
        # Closest point to cylinder's axis
        closest_point_to_axis = np.clip(
            np.array([[0],[0],[sphere_center[2, 0]]]),
            0,
            self.height
        )
        return np.linalg.norm(sphere_center - closest_point_to_axis) < sphere.radius + self.radius


################ Testing Functions

def plot_cylinder():
    """
    Plotting a cylinder 
    """
    cylinder = Cylinder(1, 5)
    cylinder.set_body_frame(
        geometry.rotation_matrix(geometry.Direction.X, np.pi / 4),
        np.array([[0], [0], [1]])
    )
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #ax.set_z
    cylinder.plot(ax)
    plt.show()

def plot_sphere():
    """
    Plotting a sphere 
    """
    sphere = Sphere(1, np.array([[1], [1], [1]]))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    #ax.set_z
    sphere.plot(ax)
    plt.show()

def test_collision():
    """
    Test 
    """
    sphere = Sphere(1, np.array([[1], [1], [1]]))
    cylinder = Cylinder(1, 5)
    cylinder.set_body_frame(
        geometry.rotation_matrix(geometry.Direction.X, np.pi / 4),
        np.array([[0], [0], [1]])
    )

    for _ in range(5):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        test_points = 10 * np.random.rand(3, 50) - 5
        cylinder.is_collision(test_points)
        sphere.is_collision(test_points)
        cylinder.plot(ax)
        sphere.plot(ax)
        ax.scatter(test_points[0, :], test_points[1, :], test_points[2, :], c='r', marker='o')
        plt.show()

def test_collision_with_sphere():
    """
    Test if sphere collides with cylinder
    """
    sphere = Sphere(1, np.array([[-10], [-10], [-10]]))
    cylinder = Cylinder(1, 5)
    cylinder.set_body_frame(
        geometry.rotation_matrix(geometry.Direction.X, np.pi / 4),
        np.array([[0], [0], [1]])
    )

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    cylinder.plot(ax)
    sphere.plot(ax)
    if cylinder.collides_with_sphere(sphere):
        print("Collision")
    else:
        print("No collision")
    plt.show()

if __name__ == "__main__":
    test_collision_with_sphere()
