"""
Module for Rapidly-exploring Random Trees (RRT)
"""
from numbers import Number
import numpy as np
import matplotlib.pyplot as plt
from sawyer import Sawyer
from obstacles import obstacles

sawyer = Sawyer()
class Node:
    """
    Class for nodes in RRT
    """
    def __init__(self, config: dict):
        self.config = config
        self.parent = None

def cost_between_nodes(node1: Node, node2: Node) -> Number:
    """
    Function to determine the cost between nodes
    """
    v1 = np.array(node1.config.values())
    v2 = np.array(node2.config.values())
    return np.linalg.norm(v1 - v2)

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
    def __init__(self, start: dict,  goal: dict, max_iter=1000):
        self.start = Node(start)
        self.dimensions = 7
        self.goal = Node(goal)
        self.max_iter = max_iter
        self.step_size = 0.1
        self.nodes = [self.start]

    def generate_random_node(self) -> Node:
        """
        Generate a random configuration in free space
        """
        random_array = [np.random.uniform(low, high) for low, high in sawyer.limits.values()]
        while True:
            config = dict(zip(sawyer.joint_names, random_array))
            if not self.node_collides(Node(config)):
                break
        return Node(config)
    def generate_new_node(self, q_near: Node, q_rand: Node) -> Node:
        """
        Generate new node which is q_near + (q_rand - q_near) * step_size
        """
        new_config = {}
        for name in sawyer.joint_names:
            new_config[name] = (
                self.step_size * (q_near.config[name] - q_rand.config[name])
                + q_near.config[name]
            )
        return Node(new_config)
    def node_collides(self, q: Node) -> bool:
        """
        Determien if node collides.
        """
        old_config = sawyer.get_config()
        sawyer.change_config(q.config)
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
        return cost_between_nodes(q, self.goal) < self.step_size

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

    def plan(self) -> list(Node):
        """
        Planner for RRT
        """
        for _ in range(self.max_iter):
            q_rand = self.generate_random_node()
            new_node = self.extend(q_rand)
            if new_node and self.is_goal_reached(new_node.config):
                print("Goal reached!")
                return self.extract_path(new_node)
        print("RRT reached the maximum number of iterations.")
        return None

    def extract_path(self, end_node: Node):
        """
        Extract path
        """
        node_path = []
        current_node = end_node
        while current_node is not None:
            node_path.append(current_node)
            current_node = current_node.parent
        return np.array(path[::-1])

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

# Example usage
start_config = np.array([0, 0, 0, 0, 0, 0, 0])
goal_config = np.array([1, 1, 1, 1, 1, 1, 1])
dimensions = 7

rrt = RRT(start_config, dimensions, goal_config, step_size=0.1, max_iter=1000)
path = rrt.plan()

if path is not None:
    visualize_rrt(rrt, path)
