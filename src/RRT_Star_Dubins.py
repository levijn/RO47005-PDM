import numpy as np
import random
import matplotlib.pyplot as plt

from collision_detection import Point
from dubins import Dubins
from helper_functions import get_distance, get_dubins_distance, collision_check

class Node:
    """
    Node of the RRT_Star_Dubins, the node now also contains the yaw
    """
    def __init__(self, x, y, yaw):
        """
        Attributes
        ----------

        x : float | x position of Node
        y : float | y position of Node
        yaw: float | yaw / orientation of Node
        parent: Node | the Node parent of the current Node
        dparent: int | the depth of the Node parent
        cost: float | the cost to reach the node from start Node

        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pos = [x, y, yaw]
        self.parent = None
        self.dparent = 0
        self.cost = 0.0

class RRT_Star_Dubins:
    """
    Class for the RRT* planner using dubins as connector / steering function
    """

    def __init__(self, map, gamma, n_max=500, min_dist_nodes=0.5, goal_sample_rate=50, dubins=Dubins(1, 0.25)):
        self.start = map.start
        self.goal = map.goal
        self.map = map
        self.n_max = n_max
        self.gamma = gamma
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_sample_rate = goal_sample_rate
        self.dubins = dubins
        self.best_goal_node = None
        self.min_path_cost = np.Infinity
        self.best_path = []
        self.goal_reached = False
    
    def find_nearest_node(self, new_node):
        """Returns the nearest node and the distance to that node"""
        min_distance = np.Infinity

        for node in self.nodes:
            distance = get_dubins_distance(node, new_node, self.dubins)
            if distance < min_distance:
                min_distance = distance
                parent_node = node

        return parent_node, min_distance

    def check_dist_other_nodes(self, new_node):
        for node in self.nodes:
            distance = get_distance(node, new_node)
            if distance < self.min_dist_nodes:
                return True
        return False

    def expand(self):
        """
        Main loop of the RRT_Star_Dubins algorithm, stops if max iterations has been reached
        """

        self.n += 1
        x_range = [0, self.map.size[0]]
        y_range = [0, self.map.size[1]]
        goal_sample = False
        
        if self.n % self.goal_sample_rate == 0:
            new_node = Node(self.goal.x, self.goal.y, self.goal.yaw)
            goal_sample = True
        else:
            new_node = Node(random.uniform(x_range[0],x_range[1]), random.uniform(y_range[0],y_range[1]), random.uniform(-np.pi, np.pi))

        # Check if new_node is not in collision with obstacles and has min distance to other nodes
        if self.map.is_point_in_obstacle(Point(new_node.x, new_node.y)) or (self.check_dist_other_nodes(new_node) and not goal_sample):
            return False
        
        parent, _ = self.find_nearest_node(new_node)

        # Assign the closest neighbor as parent to the new node
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1

        # Calculate node cost
        path, distance = self.dubins.dubins_path(parent.pos, new_node.pos, True)
        new_node.cost = new_node.parent.cost + distance

        # Check if connection between parent and new_node is not in collision
        in_collision = collision_check(path, self.map)

        if not in_collision:
            
            self.nodes.append(new_node)
            self.rewire(new_node)

            if goal_sample:
                final_path_cost = new_node.cost

                if final_path_cost < self.min_path_cost:
                    self.min_path_cost = final_path_cost
                    self.best_goal_node = new_node
                    self.goal_reached = True
                    print('New Best Path!')
        
        if self.n % (self.n_max // 4) == 0:
            print((self.n)/(self.n_max)*100, "% Done...")
    
    def rewire(self, new_node):
        """Rewires the tree"""

        # Find radius containing near nodes
        n = len(self.nodes)
        r = self.gamma * (np.log(n)/n)**1/2

        for node in self.nodes:

            # Rewire (if possible) the path to new_node by finding lowest cost to new_node from start node via near_node
            path_to_new_node, distance_to_new_node = self.dubins.dubins_path(node.pos, new_node.pos, True)
            
            if new_node.cost > (node.cost + distance_to_new_node) and distance_to_new_node <= r:
                in_collision = collision_check(path_to_new_node, self.map)
                
                if in_collision: # if collision skip rewire
                    continue

                new_node.parent = node
                new_node.cost = node.cost + distance_to_new_node
                new_node.dparent = node.dparent + 1

            # Rewire (if possible) the path to near node by finding lowest cost to near_node from start node via new_node
            path_to_near_node, distance_to_near_node = self.dubins.dubins_path(new_node.pos, node.pos, True)

            if (new_node.cost + distance_to_near_node) < node.cost and distance_to_near_node <= r:

                in_collision = collision_check(path_to_near_node, self.map)
                
                if in_collision: # if collision skip rewire
                    continue

                node.parent = new_node
                node.cost = new_node.cost + distance_to_near_node
                node.dparent = new_node.dparent + 1

    def run(self):
        """
        Runs the loop of the RRT algorithm, stops if max iteratations (n_max) has been reached 
        """
        while self.n < self.n_max:
            self.expand()

    def get_path(self):
        """
        Returns the best (smallest distance) path from start node to goal node

        Returns
        -------
        path : n x 3 Array
            path[:, 0] ---> x-coordinates of the path
            path[:, 1] ---> y-coordinates of the path
            path[:, 2] ---> yaw of vehicle 

        """
        node = self.best_goal_node
        x = np.Infinity
        y = np.Infinity

        while x != self.start.x and y != self.start.y:

            if node == self.best_goal_node:
                path = self.dubins.dubins_path(node.parent.pos, node.pos)
            else:
                path = np.insert(path, 0, self.dubins.dubins_path(node.parent.pos, node.pos), axis=0)

            node = node.parent
            x = node.x
            y = node.y  

        num_points = len(path)

        path = np.hstack((path, np.zeros(num_points).reshape(-1, 1)))

        path[0, 2], path[-1, 2] = self.start.yaw, self.goal.yaw

        for i in range(1, num_points-1):
            dx = path[i, 0] - path[i-1, 0]
            dy = path[i, 1] - path[i-1, 1]
            path[i, 2] = np.arctan2(dy, dx)

        return path

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.start.x,self.start.y, 'o', markersize = 20, label='start')
        ax.plot(self.goal.x,self.goal.y, 'o', markersize = 20, label='goal')
        
        # Plot the obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)
        
        # Plot all nodes
        for node in self.nodes:
            ax.plot(node.x,node.y, 'o')
            if node.parent == None:
                continue
            node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
            ax.plot(node_path[:, 0], node_path[:, 1], 'b')
        
        # Plot path from start to goal in red
        if self.goal_reached == True:
            path = self.get_path()
            ax.plot(path[:, 0], path[:, 1], 'r')
        
        ax.legend()
        plt.show()