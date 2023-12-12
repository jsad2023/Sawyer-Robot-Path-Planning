"""
Module for Rapidly-exploring Random Trees (RRT)
"""
import rospy # intera_interface - Sawyer Python API
from typing import List
from numbers import Number
import numpy as np
import matplotlib.pyplot as plt
from sawyer import Sawyer
from obstacles import obstacles, goal_cartesian_point
from inverse_kinematics import ik_service_client
sawyer = Sawyer()
class Node:
    """
    Class for nodes in RRT
    """
    def __init__(self, config: dict):
        self.config = config
        self.parent = None

def get_endpoint_from_config(node: Node) -> np.ndarray:
    old_config = sawyer.get_config()
    if not sawyer.change_config(node1.config):
        return np.fill((3,1), np.inf)
    v1 = sawyer.get_endpoint()
    sawyer.change_config(old_config)
    return v1

def cost_between_nodes(node1: Node, node2: Node) -> Number:
    """
    Function to determine the cost between nodes
    """
    angles1 = np.array(list(node1.config.values())).reshape(7,1)
    angles2 = np.array(list(node2.config.values())).reshape(7,1)
    return np.linalg.norm(angles1 - angles2)


#def generate_new_node(config1: dict, config2: dict, mult: Number) -> dict:
#    """
#    Create a new config that is the difference between
#    the two configurations.
#    """
#    new_config = {}
#    for name in sawyer.joint_names:
#        new_config[name] = mult * (config1[name] - config2[name])
#    return Node(new_config)


class RRT:
    """
    Class for RRT
    """
    def __init__(self, start: dict,  goal: dict, max_iter=10000):
        self.start = Node(start)
        self.dimensions = 7
        self.goal = Node(goal)
        self.max_iter = max_iter
        self.step_size = np.pi 
        self.threshold = 2.25
        self.nodes = [self.start]

    def generate_random_node(self) -> Node:
        """
        Generate a random configuration in free space
        """
        while True:
            #print('loop1')
            close_configuration = np.random.choice(self.nodes)
            angles = np.array(list(close_configuration.config.values()))
            angles = angles.reshape(7,1) + np.random.uniform(-np.pi / 6, np.pi / 6, (self.dimensions, 1))
            #random_array = list(np.random.uniform(low=-np.pi, high=np.pi, size=self.dimensions))
            config = dict(zip(sawyer.joint_names, list(angles.flatten())))
            #input(f"{config}")
            if not self.node_collides(Node(config)):
                #print(config, close_configuration.config)
                break
        return Node(config)
    def generate_new_node(self, q_near: Node, q_rand: Node) -> Node:
        """
        Generate new node which is q_near + (q_rand - q_near) * step_size
        """

        angles_near = np.array(list(q_near.config.values())).reshape(7,1)
        angles_rand = np.array(list(q_rand.config.values())).reshape(7,1)
        direction = angles_rand - angles_near
        distance = np.linalg.norm(direction)
        if distance > self.step_size:
            direction = self.step_size * direction / distance
        new_angles = angles_near + direction

        new_config = {}
        for name, angle in zip(sawyer.joint_names, list(new_angles.flatten())):
            new_config[name] = angle
        return Node(new_config)

    def node_collides(self, q: Node) -> bool:
        """
        Determien if node True if node is impossible to reach
        """
        old_config = sawyer.get_config()
        if not sawyer.change_config(q.config):
            return True
        collision = sawyer.collides_with_sphere(obstacles)
        sawyer.change_config(old_config)
        return collision

    def nearest_neighbor(self, target: Node):
        """
        Performs nearest neighbors
        """
        distances = [cost_between_nodes(target, node) for node in self.nodes]
        min_index = np.argmin(distances)
        return self.nodes[min_index]

    #def steer(self, from_config, to_config):
    #    direction = to_config - from_config
    #    norm_direction = direction / np.linalg.norm(direction)
    #    return from_config + self.step_size * norm_direction

    def is_goal_reached(self, q: Node):
        """
        Determien if goal is reached. 
        """
        return cost_between_nodes(q, self.goal) < self.threshold

    def extend(self, q_rand: Node) -> Node:
        """
        Extend step for RRT
        """
        q_near = self.nearest_neighbor(q_rand)
        #new_config = self.steer(nearest_node.config, random_config)
        q_new = self.generate_new_node(q_near, q_rand)
        if not self.node_collides(q_new):
            self.nodes.append(q_new)
            q_new.parent = q_near
            return q_new
        return None

    def plan(self) -> List[Node]:
        """
        Planner for RRT
        """
        for i in range(self.max_iter):
            if i % 50 == 0:
                print(f"We have {i} points in RRT")
            q_rand = self.generate_random_node()
            new_node = self.extend(q_rand)
            if new_node and self.is_goal_reached(new_node):
                print("Goal reached!")
                self.goal.parent = new_node
                self.nodes.append(self.goal)
                return self.extract_path()
        print("RRT reached the maximum number of iterations.")
        return None

    def extract_path(self):
        """
        Extract path
        """
        node_path = []
        current_node = self.goal
        while current_node is not None:
            node_path.append(current_node)
            current_node = current_node.parent
        print(f"Length of paths is {len(node_path)}")
        return np.array(node_path[::-1])

def visualize_rrt(rrt, path=None):
    nodes = np.array([node.config for node in rrt.nodes])

    plt.scatter(nodes[:, 0], nodes[:, 1], c='blue', marker='o', label='RRT Nodes')

    if path is not None:
        plt.plot(path[:, 0], path[:, 1], c='green', linewidth=2, label='RRT Path')

    plt.scatter(rrt.start.config[0], rrt.start.config[1], c='red', marker='s', label='Start')
    plt.scatter(rrt.goal_config[0], rrt.goal_config[1], c='orange', marker='s', label='Goal')

    plt.legend()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Rapidly-exploring Random Trees (RRT)')
    plt.show()

def move_sawyer_along_path(node_path: List[Node]) -> None:
    for node in node_path:
        print("Moving....")
        if not sawyer.change_config(node.config):
            print("something went wrong")
        sawyer.move_to_joint_positions()
        rospy.sleep(.75)

def example_run():
    x_end, y_end, z_end  = goal_cartesian_point 
    goal_config = ik_service_client(x_end, y_end, z_end)
    #x_end, y_end, z_end  = (0.9, 0.16, 0.4)
    start_config = {
        'right_j0':0.0,
        'right_j1':0.0,
        'right_j2':0.0,
        'right_j3':0.0,
        'right_j4':0.0,
        'right_j5':0.0,
        'right_j6':0.0
    }
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    sawyer.plot(ax)
    sawyer.change_config(goal_config)
    sawyer.plot(ax)
    for sphere in obstacles:
        sphere.plot(ax)
    plt.show()
    rrt = RRT(start_config, goal_config, max_iter=10000)
    if rrt.node_collides(Node(goal_config)):
        print("Goal is in obstacle")
        return
    sawyer.change_config(start_config)
    print('starting endpoint', sawyer.get_endpoint(config=start_config))
    print('goal endpoint', sawyer.get_endpoint(config=goal_config))
    print('starting_cost', cost_between_nodes(Node(goal_config), Node(start_config)))
    print("Starting planner....")
    move_sawyer_along_path(rrt.plan())




if __name__ == "__main__":
    example_run()
