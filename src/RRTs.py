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
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.parent = None
        self.dparent = 0
        self.cost = 0.0
    
    def line_to_node(self, node):
        """Returns a line from self to node"""
        return Line(self.x, self.y, node.x, node.y)


def distance(node1, node2):
    """Returns the distance between two nodes"""
    D = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return D

class RRT:
    def __init__(self, map, n_max=500, r_goal=0.5, min_dist_nodes=0.5):
        self.start = Node(map.start.x, map.start.y)
        self.goal = Node(map.goal.x, map.goal.y)
        self.map = map
        self.n_max = n_max
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.d_min = np.Infinity
    
    def FindNearest(self,Node):
        """Returns the nearest node and the distance to that node"""
        D = np.Infinity
        for node2 in self.nodes:
            d = distance(Node,node2)
            if d < D:
                D = d
                p = node2
        return p, D
    
    def check_dist_other_nodes(self,Nnode):
        for node in self.nodes:
            d = distance(node, Nnode)
            if d < self.min_dist_nodes:
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
        
        Nnode = Node(random.uniform(x_range[0],x_range[1]),random.uniform(y_range[0],y_range[1]))
        parent, D = RRT.FindNearest(self, Nnode)
        #TODO: optimize this by first checking if new node in collision
        
        Nnode.parent = parent
        Nnode.dparent = Nnode.parent.dparent + 1
        Nnode.cost = Nnode.parent.cost + D
        
        # create line between Nnode and parent and check collision
        in_collision = self.map.check_collision_line(Nnode.line_to_node(parent))
        
        # check if node is too close to other nodes
        if not self.check_dist_other_nodes(Nnode) and in_collision == False:
            self.nodes.append(Nnode)
            d_goal = distance(Nnode, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal.parent = Nnode
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
        #i = 0

        while x != self.start.x and y != self.start.y:
            node = node.parent
            path.append(node)
            x = node.x
            y = node.y       
        return path

    def run(self):
        geal_reached = False
        while geal_reached == False:
            geal_reached = rrt.expand()
        

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
            

class RRTstar:
    def __init__(self, map, gamma, n_max=500, r_goal=0.5, min_dist_nodes=0.5):
        self.start = Node(map.start.x, map.start.y)
        self.goal = Node(map.goal.x, map.goal.y)
        self.map = map
        self.n_max = n_max
        self.gamma = gamma
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.d_min = np.Infinity
    
    
    def FindNearest(self,Node):
        """Returns the nearest node and the distance to that node"""
        D = np.Infinity
        for node2 in self.nodes:
            d = distance(Node,node2)
            if d < D:
                D = d
                p = node2
        return p, D
    
    def rewire(self,Nnode):
        """Rewires the tree"""
        n = len(self.nodes)
        r = self.gamma * (np.log(n)/n)**1/2

        for node in self.nodes:
            d = distance(Nnode,node)
                
            if Nnode.cost > (node.cost + d) and d <= r:
                in_collision = self.map.check_collision_line(Nnode.line_to_node(node))
                if in_collision == True: # if collision skip rewire
                    continue
                Nnode.parent = node
                Nnode.cost = node.cost + d
                Nnode.dparent = node.dparent + 1
            
            if (Nnode.cost + d) < node.cost and d <= r:
                in_collision = self.map.check_collision_line(Nnode.line_to_node(node))
                if in_collision == True: # if collision skip rewire
                    continue
                node.parent = Nnode
                node.cost = Nnode.cost + d
                node.dparent = Nnode.dparent + 1

    
    def check_dist_other_nodes(self,Nnode):
        for node in self.nodes:
            d = distance(node, Nnode)
            if d < self.min_dist_nodes:
                return True
        return False
    
    def expand(self):
        self.n += 1
        x_range = [0,self.map.size[0]]
        y_range = [0,self.map.size[1]]
        
        Nnode = Node(random.uniform(x_range[0],x_range[1]),random.uniform(y_range[0],y_range[1]))
        
        if map.is_point_in_obstacle(Point(Nnode.x, Nnode.y)):
            return
        
        parent, D = RRTstar.FindNearest(self,Nnode)
        #TODO: optimize this by first checking if new node in collision
        Nnode.parent = parent
        Nnode.dparent = Nnode.parent.dparent + 1
        Nnode.cost = Nnode.parent.cost + D
        #check collision
        
        # create line between Nnode and parent and check collision
        in_collision = self.map.check_collision_line(Nnode.line_to_node(parent))
        obs = self.check_dist_other_nodes(Nnode)
        
        if obs == False and in_collision == False:
            
            self.nodes.append(Nnode)
            RRTstar.rewire(self,Nnode)
            
            d_goal = distance(Nnode, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal.parent = Nnode
                print('Goal Reached!!!')
                self.goal_reached = True
        
        if self.n > self.n_max:
            print("max iterations reached!")
    
    def get_path_to_goal(self):
        if self.goal_reached == False:
            print('goal not reached')
            return None
        node = self.goal
        path = [node]
        x = np.Infinity
        y = np.Infinity
        #i = 0

        while x != self.start.x and y != self.start.y:
            node = node.parent
            path.append(node)
            x = node.x
            y = node.y       
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
    
    n_max = 1000
    step = 0.1
    gamma = 100
    r_goal = 0.5
    min_dist_nodes = 0.2
    
    # rrt = RRT(map, n_max, step, r_goal)
    # rrt.run()
    # rrt.plot()
    
    rrtstar = RRTstar(map, gamma, n_max=n_max, r_goal=r_goal, min_dist_nodes=min_dist_nodes)
    rrtstar.run()
    rrtstar.plot()
    