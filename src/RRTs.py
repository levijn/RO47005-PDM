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
from dubins import Dubins

class Node:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pos = [x, y, yaw]
        self.parent = None
        self.dparent = 0
        self.cost = 0.0
    
    def line_to_node(self, node):
        """Returns a line from self to node"""
        return Line(self.x, self.y, node.x, node.y)


def get_distance(node1, node2):
    """Returns the euclidean distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

def get_dubins_distance(node1, node2, dubins):
    """Returns the distance based on the dubins path between two nodes"""
    _, dubins_distance = dubins.dubins_path(node1.pos, node2.pos, True)
    return dubins_distance

class RRT:
    def __init__(self, map, n_max=500, r_goal=0.5, min_dist_nodes=0.5):
        self.start = Node(map.start.x, map.start.y, 0)
        self.goal = Node(map.goal.x, map.goal.y, np.pi/4)
        self.map = map
        self.n_max = n_max
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.goal_sample_rate = 10
        self.d_min = np.Infinity
        self.dubins = Dubins(1, 0.25)

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
            new_node = Node(self.goal.x, self.goal.y, self.goal.yaw)
        else:
            new_node = Node(random.uniform(x_range[0],x_range[1]), random.uniform(y_range[0],y_range[1]), random.uniform(-np.pi, np.pi))
        
        # Check if new_node is not in collision with obstacles and has min distance to other nodes
        if self.map.is_point_in_obstacle(Point(new_node.x, new_node.y)) or self.check_dist_other_nodes(new_node):
            return False
        
        # Find closest neighbor based on euclidean distance
        parent, euclidean_dist = RRT.find_nearest_node(self, new_node)

        # Assign the closest neighbor as parent to the new node
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1

        # Calculate node cost
        path, distance = self.dubins.dubins_path(parent.pos, new_node.pos, True)
        new_node.cost = new_node.parent.cost + distance

        # Check if connection between parent and new_node is not in collision
        in_collision = False

        for (x, y) in path:
            if in_collision:
                break
            else:
                in_collision = self.map.is_point_in_obstacle(Point(x, y))


        # Store new node if connection between parent and new_node not in collision
        if not in_collision:
            self.nodes.append(new_node)
            d_goal = get_distance(new_node, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal.parent = new_node
                print('Goal Reached!!!')
                self.goal_reached = True
                return True
        
        # Max iterations reached
        if self.n >= self.n_max:
            print("max iterations reached!")
            return True
        
        return False
    
    def get_path_to_goal(self):
        node = self.goal
        path = []
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
        return path

    def run(self):
        geal_reached = False
        while geal_reached == False:
            geal_reached = self.expand()
        

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
            node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
            ax.plot(node_path[:, 0], node_path[:, 1], 'b')
        
        # plot path to goal red
        if self.goal_reached == True:
            path = self.get_path_to_goal()
            ax.plot(path[:, 0], path[:, 1], 'r')
        
        ax.legend()
        plt.show()
            

class RRTstar:
    def __init__(self, map, gamma, n_max=500, r_goal=0.5, min_dist_nodes=0.5, goal_sample_rate=50):
        self.start = Node(map.start.x, map.start.y, 0)
        self.goal = Node(map.goal.x, map.goal.y, np.pi/4)
        self.map = map
        self.n_max = n_max
        self.gamma = gamma
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.goal_sample_rate = goal_sample_rate
        self.dubins = Dubins(1, 0.25)
        self.best_goal_node = None
        self.min_path_cost = np.Infinity
        self.best_path = []
        self.goal_reached = False
    
    
    def find_nearest_node(self, new_node):
        """Returns the nearest node and the distance to that node"""
        min_distance = np.Infinity

        for node in self.nodes:
            _, distance = self.dubins.dubins_path(node.pos, new_node.pos, True)
            if distance < min_distance:
                min_distance = distance
                parent_node = node

        return parent_node, min_distance
    
    def rewire(self, new_node):
        """Rewires the tree"""
        n = len(self.nodes)
        r = self.gamma * (np.log(n)/n)**1/2

        for node in self.nodes:
            path, distance = self.dubins.dubins_path(new_node.pos, node.pos, True)
            in_collision = False
            
            if (new_node.cost + distance) < node.cost and distance <= r:

                for (x, y) in path:
                    if in_collision:
                        break
                    else:
                        in_collision = self.map.is_point_in_obstacle(Point(x, y))
                
                if in_collision: # if collision skip rewire
                    continue

                node.parent = new_node
                node.cost = new_node.cost + distance
                node.dparent = new_node.dparent + 1

    
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
        
        parent, distance = self.find_nearest_node(new_node)

        # Assign the closest neighbor as parent to the new node
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1

        # Calculate node cost
        path, distance = self.dubins.dubins_path(parent.pos, new_node.pos, True)
        new_node.cost = new_node.parent.cost + distance

        # Check if connection between parent and new_node is not in collision
        in_collision = False

        for (x, y) in path:
            if in_collision:
                break
            else:
                in_collision = self.map.is_point_in_obstacle(Point(x, y))
        
    
        if not in_collision:
            
            self.nodes.append(new_node)
            RRTstar.rewire(self, new_node)

            if goal_sample:
                final_path_cost = new_node.cost

                if final_path_cost < self.min_path_cost:
                    self.min_path_cost = final_path_cost
                    self.best_goal_node = new_node
                    self.goal_reached = True
                    print('New Best Path!')
        
        if self.n % (self.n_max // 4) == 0:
            #print("max iterations reached!")
            print((self.n)/(self.n_max)*100, "% Done...")
    
    def get_path_to_goal(self):
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

        self.best_path = path

        return path
    
    def run(self):
        while self.n < self.n_max:
            self.expand()

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
            node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
            ax.plot(node_path[:, 0], node_path[:, 1], 'b')
        
        # plot path to goal red
        if self.goal_reached == True:
            path = self.get_path_to_goal()
            ax.plot(path[:, 0], path[:, 1], 'r')
        
        ax.legend()
        plt.show()

    def get_path(self):
        
        path = self.get_path_to_goal()
        #path = self.best_path
        num_points = len(path)

        path = np.hstack((path, np.zeros(num_points).reshape(-1, 1)))

        path[0, 2], path[-1, 2] = self.start.yaw, self.goal.yaw

        for i in range(1, num_points-1):
            dx = path[i, 0] - path[i-1, 0]
            dy = path[i, 1] - path[i-1, 1]
            path[i, 2] = np.arctan2(dy, dx)

        return path
            
        

if __name__ == '__main__':
    
    #map = get_random_map(1, (20,20))
    map = get_simple_map()
    
    start = map.start.list()
    goal =  map.goal.list()
    
    n_max = 200
    gamma = 1000
    r_goal = 0.5
    min_dist_nodes = 0.25
    goal_sample_rate = 50
    
    #rrt = RRT(map, n_max, r_goal, min_dist_nodes)
    #rrt.run()
    #rrt.plot()
    
    rrtstar = RRTstar(map, gamma, n_max=n_max, r_goal=r_goal, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate)
    rrtstar.run()
    rrtstar.plot()
    