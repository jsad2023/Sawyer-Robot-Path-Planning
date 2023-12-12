"""
Sawyer class to control robot.
"""
from numbers import Number
from typing import List
import numpy as np
import rospy # intera_interface - Sawyer Python API
import intera_interface # initialize our ROS node, registering it with the Master 
from polygon import Cylinder, Sphere
from geometry import rotation_matrix
from geometry import Direction
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from inverse_kinematics import ik_service_client

class Sawyer:
    """
    Class for contorlling the Sawyer Robot
    """
    joint_names = [
    'right_j0',
    'right_j1',
    'right_j2',
    'right_j3',
    'right_j4',
    'right_j5',
    'right_j6',
    ]
    def __init__(self):
        rospy.init_node('path_planning')
        self._limb = intera_interface.Limb('right')
        self._num_joints = 7
        self.angles = self._limb.joint_angles() # print the current joint angles
        self.angles = {}
        self.angles['right_j0']=0.0
        self.angles['right_j1']=0.0
        self.angles['right_j2']=0.0
        self.angles['right_j3']=0.0
        self.angles['right_j4']=0.0
        self.angles['right_j5']=0.0
        self.angles['right_j6']=0.0
        self.move_to_joint_positions()
        # Limits on the joint angles
        #self.limits = {}
        #self.limits['right_j0']=(-np.pi / 4, np.pi / 3)
        #self.limits['right_j1']=(-np.pi / 6, np.pi / 6)
        #self.limits['right_j2']=(-np.pi, np.pi)
        #self.limits['right_j3']=(-np.pi / 6, np.pi / 6)
        #self.limits['right_j4']=(-np.pi, np.pi)
        #self.limits['right_j5']=(-5 * np.pi / 6, 5 * np.pi / 6)
        #self.limits['right_j6']=(-np.pi, np.pi)


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
            rotation_matrix(Direction.Z, self.angles['right_j0']),
            ( rotation_matrix(Direction.X, - np.pi / 2)
            @ rotation_matrix(Direction.Z, self.angles['right_j1'])),
            (rotation_matrix(Direction.Y, np.pi / 2)
            @ rotation_matrix(Direction.Z, self.angles['right_j2'])),
            (rotation_matrix(Direction.Y,  np.pi / 2)
            @ rotation_matrix(Direction.Z, -self.angles['right_j3'])),
            ( rotation_matrix(Direction.Y, - np.pi / 2)
            @rotation_matrix(Direction.Z, self.angles['right_j4'])),
            (rotation_matrix(Direction.Y, - np.pi / 2)
            @ rotation_matrix(Direction.Z, self.angles['right_j5'])),
            ( rotation_matrix(Direction.Y,  np.pi / 2)
            @rotation_matrix(Direction.Z, self.angles['right_j6']))
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
        self._limb.move_to_joint_positions(self.angles)
    def change_joint_angle(self, joint_name: str, angle: Number, single_change=False) -> bool:
        """ 
        Change the value of a joint angle.
        Returns true if joint angle was changed to the specified number.
        """
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        if joint_name not in self.angles:
            return False

        #lowest, highest = self.limits[joint_name]
        #if angle < lowest or angle > highest:
        #    return False

        self.angles[joint_name] = angle
        if single_change:
            self.set_reference_frames()

        return True
    def change_config(self, config: dict):
        """
        Change entire configuration of Sawyer robot
        """
        old_config = self.angles
        for joint_name, angle in config.items():
            if not self.change_joint_angle(joint_name, angle):
                print(f"Configuration {config} is not possible.\n{joint_name} can't be at angle {angle}")
                self.angles = old_config
                return False
        self.set_reference_frames()
        return True
    def is_collision(self, test_points: np.ndarray, config=None) -> bool:
        """
        Test if sawyer is in collision with any points in 
        test_points at the specified configuration. 
        """
        if config is None:
            config = self.angles
        old_config = self.angles
        self.angles = config
        for cylinder in self._cylinders:
            if cylinder.is_collision(test_points):
                self.angles = old_config
                return True
        self.angles = old_config
        return False    
    def collides_with_sphere(self, obstacles: List[Sphere]):
        """
        Test if sawyer is in collision with any spheres in 
        a list of spheres test_points at the specified configuration. 
        Returns true if there is a collision
        """
        for cylinder in self._cylinders:
            for sphere in obstacles:
                if cylinder.collides_with_sphere(sphere):
                    return True
        return False
    
    def get_config(self):
        return self.angles
    
    def get_endpoint(self, config=None):
        if config is not None:
            old_config = self.angles
            self.change_config(config)
        rotation_matrix, translation = self._cylinders[-1].get_body_frame()
        v = np.array([[0], [0], [self._link_lengths[-1]]])
        print(rotation_matrix, translation)
        if config is not None:
            self.angles = old_config
        return rotation_matrix @ v + translation
    
    def print_reference_frames(self):
        print('----------------reference frames for config', self.angles)
        for cylinder in self._cylinders:
            print(cylinder.get_body_frame())


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

def rotating_sawyer():
    """
    Rotating Sawyer
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    sawyer = Sawyer()
    sawyer.plot(ax)
    sawyer.angles['right_j0'] = np.pi / 4
    sawyer.set_reference_frames()
    sawyer.plot(ax, color='b')
    plt.show()


if __name__ == "__main__":
    rotate_sawyer()
