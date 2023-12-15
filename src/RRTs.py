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
    """Returns the distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

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
        parent, _ = RRT.find_nearest_node(self, new_node)

        #TODO: optimize this by first checking if new node in collision
        
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1
        #new_node.cost = new_node.parent.cost + distance

        path, distance = self.dubins.dubins_path(parent.pos, new_node.pos, True)
        new_node.cost = new_node.parent.cost + distance

        # create line between new node and parent and check collision
        #in_collision = self.map.check_collision_line(new_node.line_to_node(parent))
        in_collision = False

        for (x, y) in path:
            if in_collision:
                break
            else:
                in_collision = self.map.is_point_in_obstacle(Point(x, y))
        
        # check if node is too close to other nodes
        if not self.check_dist_other_nodes(new_node) and in_collision == False:
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
        #path = [node]
        path = []
        x = np.Infinity
        y = np.Infinity
        #i = 0

        while x != self.start.x and y != self.start.y:
            #node = node.parent
            #path.append(node)
            
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
            #ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b')
            ax.plot(node_path[:, 0], node_path[:, 1], 'b')
        
        # plot path to goal red
        if self.goal_reached == True:
            path = self.get_path_to_goal()
            ax.plot(path[:, 0], path[:, 1], 'r')
            # for node in path:
            #     if node.parent == None:
            #         continue
            #     ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r')
        
        ax.legend()
        plt.show()
            

# class RRTstar:
#     def __init__(self, map, gamma, n_max=500, r_goal=0.5, min_dist_nodes=0.5):
#         self.start = Node(map.start.x, map.start.y)
#         self.goal = Node(map.goal.x, map.goal.y)
#         self.map = map
#         self.n_max = n_max
#         self.gamma = gamma
#         self.r_goal = r_goal
#         self.min_dist_nodes = min_dist_nodes
#         self.nodes = [self.start]
#         self.n = 0
#         self.goal_reached = False
#         self.d_min = np.Infinity
    
    
#     def FindNearest(self,Node):
#         """Returns the nearest node and the distance to that node"""
#         D = np.Infinity
#         for node2 in self.nodes:
#             d = distance(Node,node2)
#             if d < D:
#                 D = d
#                 p = node2
#         return p, D
    
#     def rewire(self,Nnode):
#         """Rewires the tree"""
#         n = len(self.nodes)
#         r = self.gamma * (np.log(n)/n)**1/2

#         for node in self.nodes:
#             d = distance(Nnode,node)
                
#             if Nnode.cost > (node.cost + d) and d <= r:
#                 in_collision = self.map.check_collision_line(Nnode.line_to_node(node))
#                 if in_collision == True: # if collision skip rewire
#                     continue
#                 Nnode.parent = node
#                 Nnode.cost = node.cost + d
#                 Nnode.dparent = node.dparent + 1
            
#             if (Nnode.cost + d) < node.cost and d <= r:
#                 in_collision = self.map.check_collision_line(Nnode.line_to_node(node))
#                 if in_collision == True: # if collision skip rewire
#                     continue
#                 node.parent = Nnode
#                 node.cost = Nnode.cost + d
#                 node.dparent = Nnode.dparent + 1

    
#     def check_dist_other_nodes(self,Nnode):
#         for node in self.nodes:
#             d = distance(node, Nnode)
#             if d < self.min_dist_nodes:
#                 return True
#         return False
    
#     def expand(self):
#         self.n += 1
#         x_range = [0,self.map.size[0]]
#         y_range = [0,self.map.size[1]]
        
#         Nnode = Node(random.uniform(x_range[0],x_range[1]),random.uniform(y_range[0],y_range[1]))
        
#         if map.is_point_in_obstacle(Point(Nnode.x, Nnode.y)):
#             return
        
#         parent, D = RRTstar.FindNearest(self,Nnode)
#         #TODO: optimize this by first checking if new node in collision
#         Nnode.parent = parent
#         Nnode.dparent = Nnode.parent.dparent + 1
#         Nnode.cost = Nnode.parent.cost + D
#         #check collision
        
#         # create line between Nnode and parent and check collision
#         in_collision = self.map.check_collision_line(Nnode.line_to_node(parent))
#         obs = self.check_dist_other_nodes(Nnode)
        
#         if obs == False and in_collision == False:
            
#             self.nodes.append(Nnode)
#             RRTstar.rewire(self,Nnode)
            
#             d_goal = distance(Nnode, self.goal)
#             if d_goal <= self.r_goal and d_goal < self.d_min:
#                 self.d_min = d_goal
#                 self.goal.parent = Nnode
#                 print('Goal Reached!!!')
#                 self.goal_reached = True
        
#         if self.n > self.n_max:
#             print("max iterations reached!")
    
#     def get_path_to_goal(self):
#         if self.goal_reached == False:
#             print('goal not reached')
#             return None
#         node = self.goal
#         path = [node]
#         x = np.Infinity
#         y = np.Infinity
#         #i = 0

#         while x != self.start.x and y != self.start.y:
#             node = node.parent
#             path.append(node)
#             x = node.x
#             y = node.y       
#         return path
    
#     def run(self):
#         while self.n < self.n_max:
#             self.expand()


#     def plot(self):
#         fig, ax = plt.subplots()
#         ax.plot(self.start.x,self.start.y, 'o', markersize = 20, label='start')
#         ax.plot(self.goal.x,self.goal.y, 'o', markersize = 20, label='goal')
        
#         # plot obstacles
#         for patch in self.map.get_patches():
#             ax.add_patch(patch)
        
#         # plot all nodes
#         for node in self.nodes:
#             ax.plot(node.x,node.y, 'o')
#             if node.parent == None:
#                 continue
#             ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b')
        
#         # plot path to goal red
#         if self.goal_reached == True:
#             path = self.get_path_to_goal()
#             for node in path:
#                 if node.parent == None:
#                     continue
#                 ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r')
        
#         ax.legend()
#         plt.show()
            
        

if __name__ == '__main__':
    
    # map = get_random_map(1, (20,20))
    map = get_simple_map()
    
    start = map.start.list()
    goal =  map.goal.list()
    
    n_max = 10000
    step = 0.1
    gamma = 100
    r_goal = 0.5
    min_dist_nodes = 1
    
    rrt = RRT(map, n_max, r_goal, min_dist_nodes)
    rrt.run()
    rrt.plot()
    
    # rrtstar = RRTstar(map, gamma, n_max=n_max, r_goal=r_goal, min_dist_nodes=min_dist_nodes)
    # rrtstar.run()
    # rrtstar.plot()
    