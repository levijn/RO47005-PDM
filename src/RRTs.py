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
from collision_detection import Line

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


class RRTstar:
    def __init__(self, map, n_max, step, gamma, r_goal):
        self.start = Node(map.start.x, map.start.y)
        self.goal = Node(map.goal.x, map.goal.y)
        self.map = map
        self.n_max = n_max
        self.gamma = gamma
        self.r_goal = r_goal
        self.step = step
        self.Nodes = [self.start]
        self.n = 0
        self.Goalreached = False
        self.d_min = np.Infinity
    
    def distance(self, node1, node2):
        """Returns the distance between two nodes"""
        D = np.hypot(node1.x - node2.x, node1.y - node2.y)
        return D
    
    def FindNearest(self,Node):
        """Returns the nearest node and the distance to that node"""
        D = np.Infinity
        for node2 in self.Nodes:
            d = RRTstar.distance(self,Node,node2)
            if d < D:
                D = d
                p = node2
        return p, D
    
    def rewire(self,Nnode):
        """Rewires the tree"""
        n = len(self.Nodes)
        r = self.gamma * (np.log(n)/n)**1/2

        for node in self.Nodes:
            d = RRTstar.distance(self,Nnode,node)
                
            if (Nnode.cost + d) < node.cost and d <= r:
                in_collision = self.map.check_collision_line(Nnode.line_to_node(node))
                if in_collision == True: # if collision skip rewire
                    continue
                node.parent = Nnode
                node.cost = Nnode.cost + d
                node.dparent = Nnode.dparent + 1
                
            if Nnode.cost > (node.cost + d) and d <= r:
                in_collision = self.map.check_collision_line(Nnode.line_to_node(node))
                if in_collision == True: # if collision skip rewire
                    continue
                Nnode.parent = node
                Nnode.cost = node.cost + d
                Nnode.dparent = node.dparent + 1
                
    def plot(self,node,axs):
        #for node in self.Nodes:
            #plt.clf()

        #    if node.parent == None:
        #        break
            lx = [node.x, node.parent.x]
            ly = [node.y, node.parent.y]
            line, = axs.plot(lx,ly, '--')
            dot,  = axs.plot(node.x,node.y, 'o')
            patches = [line, dot]
            return patches
    
    def check_dup(self,Nnode):
        obs = False
        for node in self.Nodes:
            d = RRTstar.distance(self, node, Nnode)
            if d < 0.4:
                obs = True
                break
        return obs
    
    def expand(self,i):
        self.n += 1
        x_range = [0,self.map.size[0]] #get from map
        y_range = [0,self.map.size[1]]
        
        Nnode = Node(random.uniform(x_range[0],x_range[1]),random.uniform(y_range[0],y_range[1]))
        parent, D = RRTstar.FindNearest(self,Nnode)
        Nnode.parent = parent
        Nnode.dparent = Nnode.parent.dparent + 1
        Nnode.cost = Nnode.parent.cost + D
        #check collision
        
        # create line between Nnode and parent and check collision
        new_line = Line(Nnode.x, Nnode.y, parent.x, parent.y)
        in_collision = self.map.check_collision_line(new_line)
        
        obs = RRTstar.check_dup(self,Nnode)
        patches = []
        
        if obs == False and in_collision == False:
            
            self.Nodes.append(Nnode)
            patches.append(RRTstar.plot(self,Nnode,ax[0]))
            RRTstar.rewire(self,Nnode)
            patches.append(RRTstar.plot(self,Nnode,ax[1]))
            
            d_goal = RRTstar.distance(self, Nnode, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal.parent = Nnode
                
                print('Goal Reached!!!')
                path = rrt.path()
                #print('len path', path)
                for node in path:
                    if node.parent == None:
                        break
                    lx = [node.x, node.parent.x]
                    ly = [node.y, node.parent.y]
                    patches.append(ax[1].plot(lx,ly,'r'))
                    

        
        #if self.n > self.n_max:
           # print(self.n, 'n reached')
            #return False
        for patch in map.get_patches():
            ax[1].add_patch(patch)
        
        for patch in map.get_patches():
            ax[0].add_patch(patch)
        
        return patches
        #else:
            #return True
        
    def path(self):
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
        

if __name__ == '__main__':
    
    # map = get_random_map(3, (40,40))
    map = get_simple_map()
    
    start = map.start.list()
    goal =  map.goal.list()
    
    n_max = 500
    step = 0.1
    gamma = 50
    r_goal = 0.4
    rrt = RRTstar(map, n_max, step, gamma, r_goal)
    a = True
    fig, ax = plt.subplots(1,2)
    ax[0].plot(start[0],start[1], 'o', markersize = 20)
    ax[0].plot(goal[0],goal[1], 'o', markersize = 20)
    ax[1].plot(start[0],start[1], 'o', markersize = 20)
    ax[1].plot(goal[0],goal[1], 'o', markersize = 20)
    ani = animation.FuncAnimation(fig, rrt.expand,frames=n_max, repeat = False, blit = False)
    plt.show()
    #while a == True:
    #    a = rrt.expand()
    #path = rrt.path()

    #for node in path:
    #    if node.parent == None:
    #        break
    #    lx = [node.x, node.parent.x]
    #    ly = [node.y, node.parent.y]
    #    plt.plot(lx,ly,'r')
    #