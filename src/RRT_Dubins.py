import numpy as np
import random
import matplotlib.pyplot as plt
import copy

from matplotlib.animation import FuncAnimation
from create_environment import load_environment
from collision_detection import Point
from dubins import Dubins
from helper_functions import get_distance, get_dubins_distance, collision_check

class Node:
    """
    Node of the RRT_Dubins, the node now also contains the yaw
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

class RRT_Dubins:
    """
    Class for the RRT planner using dubins as connector / steering function
    """
    def __init__(self, map, n_max=500, r_goal=0.5, min_dist_nodes=0.5, goal_sample_rate=50, dubins=Dubins(4, 0.25), timelapse=False):
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
        self.best_path = [np.Infinity]
        self.timelapse = timelapse
        self.nodes_plots = [[] for i in range(self.n_max)]

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
        """
        Runs the loop of the RRT algorithm, stops if the goal has been reached
        or if max iteratations (n_max) has been reached 
        """

        goal_reached = False
        while goal_reached == False:
            goal_reached = self.expand()
    
    def expand(self):
        """
        Main loop of the RRT algorithm
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
            if self.timelapse: self.nodes_plots[self.n-1] = copy.deepcopy(self.nodes)
            d_goal = get_distance(new_node, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal = new_node
                print('Goal Reached!!!')
                self.goal_reached = True
                if self.plot_timelapse: self.best_path[0] = self.n
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

        ax.plot(self.start.x, self.start.y, 'o', markersize = 10, label='start')
        ax.plot(self.goal.x, self.goal.y, 'o', markersize = 10, label='goal')
        
        # Plot the obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)
        
        # Plot all nodes
        for node in self.nodes:
            ax.plot(node.x, node.y, 'o', markersize=2, color='black', zorder=1)
            if node.parent == None:
                continue
            node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
            ax.plot(node_path[:, 0], node_path[:, 1], 'b', linewidth=0.7)
        
        # Plot path to goal in red
        if self.goal_reached == True:
            path = self.get_path()
            ax.plot(path[:, 0], path[:, 1], 'r')
        
        ax.legend()
        ax.set_title(f"RRT Dubins (iterations: {self.n})")
        ax.set_aspect('equal')
        ax.set_xlim(0, self.map.size[0])
        ax.set_ylim(0, self.map.size[1])
        ax.set_xticks([])
        ax.set_yticks([])
        plt.show()

    def plot_timelapse(self, save_name="name", show_timelapse=False, interval=50):
        self.run()

        fig = plt.figure()
        ax = plt.axes()
        ax.set_aspect('equal')

        ax.plot(self.start.x, self.start.y, 'o', markersize = 10, label='start', color="mediumseagreen", zorder=2)
        ax.plot(self.goal.x, self.goal.y, 'o', markersize = 10, label='goal', color="darkorange", zorder=2)

        # Plot the obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)

        def plot_function(frame):

            # Plot all nodes
            if self.nodes_plots[frame] != []: 
                ax.clear()
                ax.plot(self.start.x,self.start.y, 'o', markersize = 10, label='start', color="mediumseagreen", zorder=2)
                ax.plot(self.goal.x,self.goal.y, 'o', markersize = 10, label='goal', color="darkorange", zorder=2)

                # Plot the obstacles
                for patch in self.map.get_patches():
                    ax.add_patch(patch)

                # Plot best path
                if frame >= self.best_path[0]-1:
                    best_path = self.get_path()
                    ax.plot(best_path[:, 0], best_path[:, 1], 'r', label="Best path", linewidth=1.5, zorder=10)

            if frame % (self.n_max // 10) == 0:
                print(f"{frame / self.n_max * 100} % plotting...")

            for i in range(len(self.nodes_plots[frame])):
                node = self.nodes_plots[frame][i]

                if node.parent == None:
                    continue

                ax.plot(node.x, node.y, 'o', markersize=2, color='black', zorder=1)
                node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
                ax.plot(node_path[:, 0], node_path[:, 1], 'b', linewidth=0.7, zorder=1)
           
            # ax.legend()
            ax.set_title(f"RRT Dubins (iterations: {frame+1})")
            ax.set_aspect('equal')
            ax.set_xlim(0, self.map.size[0])
            ax.set_ylim(0, self.map.size[1])
            ax.set_xticks([])
            ax.set_yticks([])

        # Generate animation
        animation = FuncAnimation(fig, plot_function, frames=self.n_max, fargs=(), interval=interval, blit=False, repeat=False)

        # Save animation as a GIF
        animation.save(save_name + '_timelapse.gif', writer='imagemagick')
        
        if show_timelapse: plt.show()
            
if __name__ == '__main__':

    # Apply path planner on the following map:
    map_name = "map_large"                 # map_name of .json file in \maps, e.g., map.json --> map_name = map
    map = load_environment(map_name)

    n_max = 500                  # max iterations of RRT* Dubins
    r_goal = 0.5                 # gamma affecting the rewire radius
    min_dist_nodes = 0           # minimum distance between random sampled nodes (else rejected)
    goal_sample_rate = 450       # each time after goal_sample_rate iterations the random sample is placed at the goal location
    dubins = Dubins(4, 0.25)     # Dubins -> (turn radius of car, distance between points of dubins path)
    timelapse = True             # Create a timelapse of path planning stored in ... as RRT_Star_Dubins_Timelapse.gif
    show_timelapse = False       # Show the timelapse after saving it, might be very slow.
    fps = 50                     # Fps of timelapse

    rrt_dubins = RRT_Dubins(map, n_max, r_goal, min_dist_nodes, goal_sample_rate, dubins, timelapse)

    if timelapse:
        rrt_dubins.plot_timelapse("results/RRT_Dubins", show_timelapse, int(1000 // fps))
        plt.savefig("results/RRT_Dubins.png")
        plt.show()
    else:
        rrt_dubins.run()
        rrt_dubins.plot()