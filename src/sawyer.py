import numpy as np
import rospy # intera_interface - Sawyer Python API
import intera_interface # initialize our ROS node, registering it with the Master 
from polygon import Cylinder
from geometry import get_rotation_matrix
from geometry import Direction
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class Sawyer:
    """
    Class for contorlling the Sawyer Robot
    """
    def __init__(self):
        rospy.init_node('path_planning')
        self._limb = intera_interface.Limb('right')
        self._num_joints = 7
        self._angles = self._limb.joint_angles() # print the current joint angles
        self._angles = {}
        self._angles['right_j0']=0.0
        self._angles['right_j1']=0.0
        self._angles['right_j2']=0.0
        self._angles['right_j3']=0.0
        self._angles['right_j4']=0.0
        self._angles['right_j5']=0.0
        self._angles['right_j6']=0.0
        self.move_to_joint_positions()

        # Initializing Cylinders
        self._link_lengths = [0.081, 0.1925, .4, 0.0165, .4, .1363, .13375]
        radii = [0.08, 0.08, 0.05, 0.05, 0.03, 0.05, 0.03]
        self._cylinders = [Cylinder(r, l) for r, l in zip(radii, self._link_lengths)]
        self.set_reference_frames()
    
    def set_reference_frames(self) -> None:
        """
        Set the reference frames for the cylinders.
        This should happen whenever an angle is changed
        """
        # rotations[i] = rotation matrix from reference frame i - 1 to i if i >= 1
        # rotation[0] = rotation matrix from world reference frame to reference frame 0
        rotations = [
            get_rotation_matrix(Direction.Z, self._angles['right_j0']),
            (get_rotation_matrix(Direction.Z, self._angles['right_j1'])
            @ get_rotation_matrix(Direction.X, - np.pi / 2)),
            (get_rotation_matrix(Direction.Z, self._angles['right_j2'])
            @ get_rotation_matrix(Direction.Y, np.pi / 2)),
            (get_rotation_matrix(Direction.Z, self._angles['right_j3'])
            @ get_rotation_matrix(Direction.Y,  np.pi / 2)),
            (get_rotation_matrix(Direction.Z, self._angles['right_j4'])
            @ get_rotation_matrix(Direction.Y, - np.pi / 2)),
            (get_rotation_matrix(Direction.Z, self._angles['right_j5'])
            @ get_rotation_matrix(Direction.Y, - np.pi / 2)),
            (get_rotation_matrix(Direction.Z, self._angles['right_j6'])
            @ get_rotation_matrix(Direction.Y,  np.pi / 2))
        ]

        # tranlations[i] = translation vector from reference frame i - 1 to i if i >= 1
        # translations[0] = translation vector from world reference frame to reference frame 0
        translations = (
            [np.zeros((3,1))]
            +[np.array([[0], [0], [self._link_lengths[i-1]]]) for i in range(1, self._num_joints)]
        )

        rotation_prev = rotations[0]
        translation_prev = translations[0]
        self._cylinders[0].set_body_frame(rotation_prev, translation_prev)

        for i in range(1, self._num_joints):
            rotation_new = rotation_prev @ rotations[i]
            translation_new = translation_prev + rotation_prev @ translations[i]
            self._cylinders[i].set_body_frame(rotation_new, translation_new)

            rotation_prev = rotation_new
            translation_prev = translation_new

    def plot(self, ax):
        """
        Plot the sawyer robot in 3d space
        """
        self.set_reference_frames()
        for cylinder in self._cylinders:
            cylinder.plot(ax)
    def move_to_joint_positions(self):
        """
        Move the sawyer robot to the current positions.
        """
        self._limb.move_to_joint_positions(self._angles)
    def is_collision(self, test_points: np.ndarray) -> bool:
        """
        Test if sawyer is in collision with any points in 
        test_points. 
        """
        for cylinder in self._cylinders:
            if cylinder.is_collision(test_points):
                return True
        return False


#########
# Tests
########

def plot_sawyer():
    """
    Testing the plot for sawyer
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    Sawyer().plot(ax)
    plt.show()


if __name__ == "__main__":
    plot_sawyer()
