# -*- coding: utf-8 -*-
"""
Created on Tue Dec 12 13:11:54 2023

@author: arnou
"""

import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from map import Map, get_simple_map, get_random_map
from collision_detection import Line, Point

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.dparent = 0
        self.cost = 0.0
    
    def line_to_node(self, node):
        """Returns a line from self to node"""
        return Line(self.x, self.y, node.x, node.y)


def get_distance(node1, node2):
    """Returns the distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

class RRT:
    def __init__(self, map, n_max=500, r_goal=0.5, min_dist_nodes=0.5, goal_sample_rate=50):
        self.start = Node(map.start.x, map.start.y)
        self.goal = Node(map.goal.x, map.goal.y)
        self.map = map
        self.n_max = n_max
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.goal_sample_rate = goal_sample_rate
        self.d_min = np.Infinity
    
    def find_nearest_node(self, new_node):
        """Returns the nearest node and the distance to that node"""
        min_distance = np.Infinity

        for node in self.nodes:
            distance = get_distance(new_node, node)
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
    
    def expand(self):
        """
        Expands the tree
        returns True if goal reached or max iterations reached, else False
        """
        self.n += 1
        x_range = [0,self.map.size[0]]
        y_range = [0,self.map.size[1]]
        
        if self.n % self.goal_sample_rate == 0:
            new_node = Node(self.goal.x, self.goal.y)
        else:
            new_node = Node(random.uniform(x_range[0],x_range[1]), random.uniform(y_range[0],y_range[1]))
        
        # Check if new_node is not in collision with obstacles and has min distance to other nodes
        if self.map.is_point_in_obstacle(Point(new_node.x, new_node.y)) or self.check_dist_other_nodes(new_node):
            return False
        
        # Find closest neighbor based on euclidean distance
        parent, euclidean_dist = RRT.find_nearest_node(self, new_node)

        # Assign the closest neighbor as parent to the new node
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1
        new_node.cost = new_node.parent.cost + euclidean_dist

        # Check if connection between new_node and parent is not in collision
        in_collision = self.map.check_collision_line(new_node.line_to_node(parent))
        
        # Store new_node if not in collision
        if not in_collision:
            self.nodes.append(new_node)
            d_goal = get_distance(new_node, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal.parent = new_node
                print('Goal Reached!!!')
                self.goal_reached = True
                return True
        
        # max iterations reached
        if self.n >= self.n_max:
            print("max iterations reached!")
            return True
        
        return False
    
    def get_path_to_goal(self):
        node = self.goal
        path = [node]
        x = np.Infinity
        y = np.Infinity

        while x != self.start.x and y != self.start.y:
            node = node.parent
            path.append(node)
            x = node.x
            y = node.y       

        return path

    def run(self):
        goal_reached = False
        while goal_reached == False:
            goal_reached = self.expand()
        

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.start.x,self.start.y, 'o', markersize = 20, label='start')
        ax.plot(self.goal.x,self.goal.y, 'o', markersize = 20, label='goal')
        
        # plot obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)
        
        # plot all nodes
        for node in self.nodes:
            ax.plot(node.x,node.y, 'o')
            if node.parent == None:
                continue
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b')
        
        # plot path to goal red
        if self.goal_reached == True:
            path = self.get_path_to_goal()
            for node in path:
                if node.parent == None:
                    continue
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r')
        
        ax.legend()
        plt.show()
         
        
if __name__ == '__main__':
    
    # map = get_random_map(1, (20,20))
    map = get_simple_map()
    
    start = map.start.list()
    goal =  map.goal.list()
    
    n_max = 500
    step = 0.1
    gamma = 100
    r_goal = 0.5
    min_dist_nodes = 0.2
    goal_sample_rate = 500
    
    rrt = RRT(map, n_max, r_goal, min_dist_nodes, goal_sample_rate)
    rrt.run()
    rrt.plot()

    