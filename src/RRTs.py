# -*- coding: utf-8 -*-
"""
Created on Tue Dec 12 13:11:54 2023

@author: arnou
"""

import numpy as np
import random
import matplotlib.pyplot as plt
from map import Map, get_simple_map, get_random_map
from collision_detection import Point
from dubins import Dubins
from create_environment import load_environment

class Node:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pos = [x, y, yaw]
        self.parent = None
        self.dparent = 0
        self.cost = 0.0

def get_distance(node1, node2):
    """Returns the euclidean distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

def get_dubins_distance(node1, node2, dubins):
    """Returns the distance based on the dubins path between two nodes"""
    _, dubins_distance = dubins.dubins_path(node1.pos, node2.pos, True)
    return dubins_distance

def collision_check(path, map):
    """
    Check if path between two nodes is not in collision with the obstacles
    in the map. Furthermore, another check to see if the path is not outside
    the map boundaries.
    """
    in_collision = False
    outside_map = False

    for (x, y) in path:
        if in_collision or outside_map:
            break
        else:
            in_collision = map.is_point_in_obstacle(Point(x, y))
            outside_map = x < map.dimensions[0][0] or x > map.dimensions[0][1] or y < map.dimensions[1][0] or y > map.dimensions[1][1]

    return in_collision or outside_map


class RRT:
    def __init__(self, map, n_max=500, r_goal=0.5, min_dist_nodes=0.5, goal_sample_rate=50, dubins=Dubins(1, 0.25)):
        self.start = map.start
        self.goal = map.goal
        self.map = map
        self.n_max = n_max
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.goal_sample_rate = goal_sample_rate
        self.d_min = np.Infinity
        self.dubins = dubins

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
        """Checks if new_node is too close to other neighbors"""
        for node in self.nodes:
            distance = get_distance(new_node, node)
            if distance < self.min_dist_nodes:
                return True
            
        return False
    
    def run(self):
        goal_reached = False
        while goal_reached == False:
            goal_reached = self.expand()
    
    def expand(self):
        """
        Expands the tree
        returns True if goal reached or max iterations reached, else False
        """
        self.n += 1
        x_range = self.map.dimensions[0]
        y_range = self.map.dimensions[1]
        goal_sample = False
        
        if self.n % self.goal_sample_rate == 0:
            new_node = Node(self.goal.x, self.goal.y, self.goal.yaw)
            goal_sample = True
        else:
            new_node = Node(random.uniform(x_range[0],x_range[1]), random.uniform(y_range[0],y_range[1]), random.uniform(-np.pi, np.pi))
        
        # Check if new_node is not in collision with obstacles and has min distance to other nodes
        if self.map.is_point_in_obstacle(Point(new_node.x, new_node.y)) or (self.check_dist_other_nodes(new_node) and not goal_sample):
            return False
        
        # Find closest neighbor based on distance of dubins path
        parent, _ = self.find_nearest_node(new_node)

        # Assign the closest neighbor as parent to the new node
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1

        # Calculate node cost
        path, distance = self.dubins.dubins_path(parent.pos, new_node.pos, True)
        new_node.cost = new_node.parent.cost + distance

        # Check if connection between parent and new_node is not in collision
        in_collision = collision_check(path, self.map)

        # Store new node if connection between parent and new_node not in collision
        if not in_collision:
            self.nodes.append(new_node)
            d_goal = get_distance(new_node, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal = new_node
                print('Goal Reached!!!')
                self.goal_reached = True
                return True
        
        # Max iterations reached
        if self.n >= self.n_max:
            print("max iterations reached!")
            return True
        
        return False
    
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
        node = self.goal
        x = np.Infinity
        y = np.Infinity

        while x != self.start.x and y != self.start.y:

            if node == self.goal:
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
        
        # Plot path to goal in red
        if self.goal_reached == True:
            path = self.get_path()
            ax.plot(path[:, 0], path[:, 1], 'r')
        
        ax.legend()
        plt.show()
            
class RRTstar:
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
        
        # Plot  the obstacles
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

if __name__ == '__main__':
    
    #map = get_random_map(1, (20,20))
    map = get_simple_map()
    #map = load_environment("map")
    
    n_max = 300
    gamma = 1000
    r_goal = 0.5
    min_dist_nodes = 0
    goal_sample_rate = 50
    
    # rrt = RRT(map, n_max, r_goal, min_dist_nodes)
    # rrt.run()
    # rrt.plot()
    
    rrtstar = RRTstar(map, gamma, n_max=n_max, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate, dubins=Dubins(1, 0.5))
    rrtstar.run()
    rrtstar.plot()
    