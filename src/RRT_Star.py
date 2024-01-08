import numpy as np
import random
import matplotlib.pyplot as plt

#from map import Map, get_simple_map, get_random_map
from collision_detection import Point

class Node:
    """
    Node of the RRT_Star
    """

    def __init__(self, x, y):
        """
        Attributes
        ----------

        x : float | x position of Node
        y : float | y position of Node
        parent: Node | the Node parent of the current Node
        dparent: int | the depth of the Node parent
        cost: float | the cost to reach the node from start Node

        """
        self.x = x
        self.y = y
        self.parent = None
        self.dparent = 0
        self.cost = 0.0
    
def get_distance(node1, node2):
    """Returns the distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

def collision_check(from_node, to_node, map, n_step=0.25):
    """
    Check if path between two nodes is not in collision with the obstacles
    in the map. 
    """
    in_collision = False
    n_max = max(np.abs(to_node.x-from_node.x), np.abs(to_node.y-from_node.y))
    n = int(n_max / 0.25)

    x_r = np.linspace(from_node.x, to_node.x, n)
    y_r = np.linspace(from_node.y, to_node.y, n)

    for i in range(n):
        if in_collision:
            break
        else:
            x, y = x_r[i], y_r[i]
            in_collision = map.is_point_in_obstacle(Point(x, y))
        
    return in_collision

class RRT_Star:
    """
    Class for the RRT* planner, similiar to RRT but has a rewire step
    """
    def __init__(self, map, gamma, n_max=500, min_dist_nodes=0.5, goal_sample_rate=50):
        self.start = Node(map.start.x, map.start.y)
        self.goal = Node(map.goal.x, map.goal.y)
        self.map = map
        self.n_max = n_max
        self.gamma = gamma
        self.min_dist_nodes = min_dist_nodes
        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.goal_sample_rate = goal_sample_rate
        self.min_path_cost = np.Infinity
    
    def find_nearest_node(self, new_node):
        """Returns the nearest node and the distance to that node"""
        min_distance = np.Infinity

        for node in self.nodes:
            distance = get_distance(new_node, node)
            if distance < min_distance:
                min_distance = distance
                parent_node = node

        return parent_node, min_distance
    
    def rewire(self, new_node):
        """Rewires the tree"""
        n = len(self.nodes)
        r = self.gamma * (np.log(n)/n)**1/2 # Rewire radius

        for node in self.nodes:
            distance = get_distance(node, new_node)
                
            if new_node.cost > (node.cost + distance) and distance <= r:

                in_collision = collision_check(node, new_node, self.map)

                if in_collision == True: # if collision skip rewire
                    continue
                new_node.parent = node
                new_node.cost = node.cost + distance
                new_node.dparent = node.dparent + 1
            
            if (new_node.cost + distance) < node.cost and distance <= r:
                in_collision = collision_check(new_node, node, self.map)

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
        """
        Main loop of the RRT_Star algorithm, stops if max iterations has been reached
        """

        self.n += 1
        x_range = [0,self.map.size[0]]
        y_range = [0,self.map.size[1]]
        goal_sample = False
        
        if self.n % self.goal_sample_rate == 0:
            new_node = Node(self.goal.x, self.goal.y)
            goal_sample = True
        else:
            new_node = Node(random.uniform(x_range[0],x_range[1]), random.uniform(y_range[0],y_range[1]))

        # # Informed
        # if self.goal_reached:
        #     in_ellipse = get_distance(new_node, self.start) + get_distance(new_node, self.goal) <= self.min_path_cost
        #     if not in_ellipse:
        #         return False
        
        # Check if new_node is not in collision with obstacles and has min distance to other nodes
        if self.map.is_point_in_obstacle(Point(new_node.x, new_node.y)) or (self.check_dist_other_nodes(new_node) and not goal_sample):
            return False
        
        # Find closest neighbor based on euclidean distance
        parent, euclidean_dist = self.find_nearest_node(new_node)

        # Assign the closest neighbor as parent to the new node
        new_node.parent = parent
        new_node.dparent = new_node.parent.dparent + 1
        new_node.cost = new_node.parent.cost + euclidean_dist

        # Check if connection between new_node and parent is not in collision
        in_collision = collision_check(parent, new_node, self.map)
        
        if not in_collision:
            
            self.nodes.append(new_node)
            self.rewire(new_node)
            
            if goal_sample:
                final_path_cost = new_node.cost

                if final_path_cost < self.min_path_cost:
                    self.min_path_cost = final_path_cost
                    self.goal = new_node
                    self.goal_reached = True
                    print('New Best Path!')
        
        # Progress
        if self.n % (self.n_max // 10) == 0:
            print((self.n)/(self.n_max)*100, "% Done...")
    
    def get_path_to_goal(self):
        """Returns the found path from start to goal"""

        if self.goal_reached == False:
            print('Goal not reached')
            return None
        
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
        """
        Runs the loop of the RRT* algorithm, stops until max iteratations (n_max) has been reached 
        """

        while self.n < self.n_max:
            self.expand()

    def plot(self):
        """
        Plots the final path from start to goal in red,
        The obstacles are plotted in black,
        The randomly sampled nodes and their connections are also plotted
        """

        fig, ax = plt.subplots()
        ax.plot(self.start.x,self.start.y, 'o', markersize = 20, label='start')
        ax.plot(self.goal.x,self.goal.y, 'o', markersize = 20, label='goal')
        
        # Plot obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)
        
        # Plot all nodes
        for node in self.nodes:
            ax.plot(node.x,node.y, 'o')
            if node.parent == None:
                continue
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b')
        
        # Plot path to goal in red
        if self.goal_reached == True:
            path = self.get_path_to_goal()
            for node in path:
                if node.parent == None:
                    continue
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r')
        
        ax.legend()
        plt.show()
            
        

# if __name__ == '__main__':
    
#     # map = get_random_map(1, (20,20))
#     map = get_simple_map()
    
#     start = map.start.list()
#     goal =  map.goal.list()
    
#     n_max = 200
#     gamma = 100
#     r_goal = 0.5
#     min_dist_nodes = 0
#     goal_sample_rate = 50
    
#     rrtstar = RRT_Star(map, gamma, n_max=n_max, r_goal=r_goal, min_dist_nodes=min_dist_nodes, goal_sample_rate=goal_sample_rate)
#     rrtstar.run()
#     rrtstar.plot()