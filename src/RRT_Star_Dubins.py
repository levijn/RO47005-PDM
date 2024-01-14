import numpy as np
import random
import matplotlib.pyplot as plt
import time
import copy

from create_environment import load_environment
from collision_detection import Point
from dubins import Dubins
from helper_functions import get_distance, get_dubins_distance, collision_check
from matplotlib.animation import FuncAnimation

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

    def __init__(self, map, gamma, n_max=500, min_dist_nodes=0.5, goal_sample_rate=50, dubins=Dubins(4, 0.25), timelapse=False):
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
        self.timelapse = timelapse
        self.nodes_plots = []
    
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

        if self.timelapse: self.nodes_plots.append([])
        
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
            # if self.timelapse: self.nodes_plots.append(copy.deepcopy(self.nodes))
            if self.timelapse: self.nodes_plots[-1] = copy.deepcopy(self.nodes)

            if goal_sample:
                final_path_cost = new_node.cost

                if final_path_cost < self.min_path_cost:
                    self.min_path_cost = final_path_cost
                    self.best_goal_node = new_node
                    self.goal_reached = True
                    if self.plot_timelapse: self.best_path.append([self.get_path(), self.n])
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

    def plot(self, show=True, save=False, save_path=""):
        fig, ax = plt.subplots(figsize=(10, 10))

        ax.plot(self.start.x, self.start.y, 'o', markersize = 10, label='start', color="mediumseagreen", zorder=2)
        ax.plot(self.goal.x, self.goal.y, 'o', markersize = 10, label='goal', color="darkorange", zorder=2)
        
        # Plot the obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)
        
        # Plot all nodes
        for node in self.nodes:
            ax.plot(node.x, node.y, 'o', markersize=2, color='black', zorder=1)
            if node.parent == None:
                continue
            node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
            ax.plot(node_path[:, 0], node_path[:, 1], 'b', linewidth=0.7, zorder=1)
        
        # Plot path from start to goal in red
        if self.goal_reached == True:
            path = self.get_path()
            ax.plot(path[:, 0], path[:, 1], 'r', label="Best path", zorder=10)
        
        ax.legend()
        ax.set_title(f"RRT* Dubins (iterations: {self.n})")
        ax.set_aspect('equal')
        ax.set_xlim(0, self.map.size[0])
        ax.set_ylim(0, self.map.size[1])
        ax.set_xticks([])
        ax.set_yticks([])
        
        
        if save:
            plt.savefig(save_path)
        if show:
            plt.show()
        
    def create_intermediate_plots(self, show=True, save=False, save_path="", plot_score=False, plot_time=False):
        """Create a plot stopping every fraction of the iterations assuming it has not been run yet"""
        # check if not yet run
        if self.n > 0:
            print("Already run, cannot create intermediate plots")
            return
        
        fig, axs = plt.subplots(2, 2, figsize=(10, 10))
        
        stop_points = [self.n_max // 4, self.n_max // 2, self.n_max // 4 * 3, self.n_max]
        
        score = []
        time_points = [0]

        start_time = time.time()
        for n in stop_points:
            while self.n < n:
                self.expand()
                # add current best score to list
                if plot_score:
                    score.append(self.min_path_cost)
                if plot_time:
                    time_points.append(time.time() - start_time)
            
            elapsed_time = time.time() - start_time
            ax = axs.flatten()[stop_points.index(n)]
            ax.plot(self.start.x, self.start.y, 'o', markersize = 10, label='start', color="mediumseagreen", zorder=2)
            ax.plot(self.goal.x, self.goal.y, 'o', markersize = 10, label='goal', color="darkorange", zorder=2)
            
            # Plot the obstacles
            for patch in self.map.get_patches():
                ax.add_patch(patch)
            
            # Plot all nodes
            for node in self.nodes:
                ax.plot(node.x,node.y, 'o', markersize=2, color='black', zorder=1)
                if node.parent == None:
                    continue
                node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
                ax.plot(node_path[:, 0], node_path[:, 1], 'b', linewidth=0.7, zorder=1)
            
            # Plot path from start to goal in red
            if self.goal_reached == True:
                path = self.get_path()
                ax.plot(path[:, 0], path[:, 1], 'r', zorder=10)
            
            ax.set_xlim(0, self.map.size[0])
            ax.set_ylim(0, self.map.size[1])
            ax.set_aspect('equal')
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_title(f"{n} iterations (Time: {elapsed_time:.2f}s, Cost: {score[n-1]:.2f})")
            plt.subplots_adjust(wspace=0, hspace=0.1)
        
        if save:
            plt.savefig(save_path)
        if show:
            plt.show()
        
        if plot_score:
            # remove infinities from list
            inf_list = [0 for x in score if x == np.Infinity]
            score = [x for x in score if x != np.Infinity]
            score_combined = inf_list + score
            x = np.arange(0, len(score_combined))

            plt.figure()
            plt.plot(x, score_combined, label="cost")
            plt.xlabel("Number of iterations")
            plt.ylabel("Cost")
            plt.title("Cost over number of iterations")

        if save:
            plt.savefig(save_path + "_cost.png")
        if show:
            plt.show()

        if plot_time:
            x = np.arange(0, self.n_max+1)

            plt.figure()
            plt.plot(x, time_points, label="time")
            plt.xlabel("Number of iterations")
            plt.ylabel("Computation time [s]")
            plt.title("Computation time over number of iterations")

        if save:
            plt.savefig(save_path + "_time.png")
        if show:
            plt.show()      

    def plot_timelapse(self, save_name="name", show_timelapse=False, interval=50):

        while self.n < self.n_max:
            self.expand()

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
                if len(self.best_path) == 1:
                    if self.best_path[0][1] <= frame:
                        best_path = self.best_path[0][0]
                        ax.plot(best_path[:, 0], best_path[:, 1], 'r', label="Best path", linewidth=1.5, zorder=10)
                elif len(self.best_path) >= 2:
                    for i in range(len(self.best_path)-1):
                        if self.best_path[i][1] <= frame < self.best_path[i+1][1]:
                            best_path = self.best_path[i][0]
                            ax.plot(best_path[:, 0], best_path[:, 1], 'r', label="Best path", linewidth=1.5, zorder=10)
                        elif frame >= self.best_path[-1][1]:
                            best_path = self.best_path[-1][0]
                            ax.plot(best_path[:, 0], best_path[:, 1], 'r', label="Best path", linewidth=1.5, zorder=10)

            # if frame % (self.n_max // 10) == 0:
            #     print(f"{frame / self.n_max * 100} % plotting...")

            for i in range(len(self.nodes_plots[frame])):
                node = self.nodes_plots[frame][i]

                if node.parent == None:
                    continue

                ax.plot(node.x, node.y, 'o', markersize=2, color='black', zorder=1)
                node_path = self.dubins.dubins_path(node.parent.pos, node.pos)
                ax.plot(node_path[:, 0], node_path[:, 1], 'b', linewidth=0.7, zorder=1)
           
            # ax.legend()
            ax.set_title(f"RRT* Dubins (iterations: {frame+1})")
            ax.set_aspect('equal')
            ax.set_xlim(0, self.map.size[0])
            ax.set_ylim(0, self.map.size[1])
            ax.set_xticks([])
            ax.set_yticks([])


        # Generate animation
        animation = FuncAnimation(fig, plot_function, frames=self.n_max, fargs=(), interval=interval, blit=False, repeat=False)
        
        if show_timelapse: 
            plt.show()
        else:
            # Save animation as a GIF
            animation.save(save_name + '_timelapse.gif', writer='imagemagick')
            
if __name__ == '__main__':

    # Apply path planner on the following map:
    map_name = "scenario3"                 # map_name of .json file in \maps, e.g., map.json --> map_name = map
    map = load_environment(map_name)

    n_max = 250                  # max iterations of RRT* Dubins
    gamma = 1000                 # gamma affecting the rewire radius
    min_dist_nodes = 0           # minimum distance between random sampled nodes (else rejected)
    goal_sample_rate = 50        # each time after goal_sample_rate iterations the random sample is placed at the goal location
    dubins = Dubins(4, 0.25)     # Dubins -> (turn radius of car, distance between points of dubins path)
    timelapse = True             # Create a timelapse of path planning stored in results as RRT_Star_Dubins_Timelapse.gif
    show_timelapse = True        # Show the timelapse in real time instead of saving, might be very slow
    fps = 50                     # Fps of timelapse
    results = False              # Show the plot results used in report

    rrtstar_dubins = RRT_Star_Dubins(map, gamma, n_max, min_dist_nodes, goal_sample_rate, dubins, timelapse)

    if timelapse:
        rrtstar_dubins.plot_timelapse("results/RRT_Star_Dubins", show_timelapse, int(1000 // fps))
        plt.savefig("results/RRT_Star_Dubins.png")
        if not show_timelapse: plt.show()
    elif results:
        rrtstar_dubins.create_intermediate_plots(show=True, save=True, save_path="results/scenario3", plot_score=True, plot_time=True)
    else:
        rrtstar_dubins.run()
        rrtstar_dubins.plot()