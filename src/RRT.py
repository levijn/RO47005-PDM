import numpy as np
import random
import matplotlib.pyplot as plt
import copy

from matplotlib.animation import FuncAnimation
from create_environment import load_environment
from collision_detection import Line, Point

class Node:
    """
    Node of the RRT
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
    
    def line_to_node(self, node):
        """Returns a line from current node to other node, used for collision checking"""
        return Line(self.x, self.y, node.x, node.y)


def get_distance(node1, node2):
    """Returns the euclidean distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

class RRT:
    """
    Class for the RRT planner
    """

    def __init__(self, map, n_max=500, r_goal=0.5, min_dist_nodes=0.5, goal_sample_rate=50, timelapse=False):
        self.start = map.start
        self.goal = map.goal
        self.map = map
        self.n_max = n_max
        self.r_goal = r_goal
        self.min_dist_nodes = min_dist_nodes
        self.goal_sample_rate = goal_sample_rate

        self.nodes = [self.start]
        self.n = 0
        self.goal_reached = False
        self.d_min = np.Infinity
        self.best_path = [np.Infinity]
        self.timelapse = timelapse
        self.nodes_plots = [[] for i in range(self.n_max)]
    
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
        Main loop of the RRT algorithm
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
            if self.timelapse: self.nodes_plots[self.n-1] = copy.deepcopy(self.nodes)
            d_goal = get_distance(new_node, self.goal)
            if d_goal <= self.r_goal and d_goal < self.d_min:
                self.d_min = d_goal
                self.goal.parent = new_node
                print('Goal Reached!!!')
                self.goal_reached = True
                if self.plot_timelapse: self.best_path[0] = self.n

                return True
        
        # max iterations reached
        if self.n >= self.n_max:
            print("max iterations reached!")
            return True
        
        return False
    
    def get_path_to_goal(self):
        """Returns the found path from start to goal"""
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
        Runs the loop of the RRT algorithm, stops if goal has been reached
        or if max iteratations (n_max) has been reached 
        """
        goal_reached = False

        while goal_reached == False:
            goal_reached = self.expand()
        

    def plot(self):
        """
        Plots the final path from start to goal in red,
        The obstacles are plotted in black,
        The randomly sampled nodes and their connections are also plotted
        """
        fig, ax = plt.subplots()
        ax.plot(self.start.x, self.start.y, 'o', markersize = 10, label='start', color="mediumseagreen", zorder=2)
        ax.plot(self.goal.x, self.goal.y, 'o', markersize = 10, label='goal', color="darkorange", zorder=2)
        
        # Plot obstacles
        for patch in self.map.get_patches():
            ax.add_patch(patch)
        
        # Plot all nodes
        for node in self.nodes:
            ax.plot(node.x, node.y, 'o', markersize=2, color='black', zorder=1)
            if node.parent == None:
                continue
            ax.plot([node.x, node.parent.x], [node.y, node.parent.y], linewidth=0.7, color='b')
        
        # Plot path to goal red
        if self.goal_reached == True:
            path = self.get_path_to_goal()

            for node in path:
                if node.parent == None:
                    continue
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r', zorder=10)
        
        ax.legend()
        ax.set_title(f"RRT (iterations: {self.n})")
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
                    path = self.get_path_to_goal()
                    for node in path:
                        if node.parent == None:
                            continue
                        ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'r', zorder=10)

            # if frame % (self.n_max // 10) == 0:
            #     print(f"{frame / self.n_max * 100} % plotting...")

            for i in range(len(self.nodes_plots[frame])):
                node = self.nodes_plots[frame][i]

                if node.parent == None:
                    continue

                ax.plot(node.x, node.y, 'o', markersize=2, color='black', zorder=1)
                ax.plot([node.x, node.parent.x], [node.y, node.parent.y], 'b', linewidth=0.7, zorder=1)
           
            # ax.legend()
            ax.set_title(f"RRT (iterations: {frame+1})")
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

    n_max = 250                 # max iterations of RRT
    r_goal = 0.5                # If node is within r_goal distance of the goal position --> valid
    min_dist_nodes = 0          # minimum distance between random sampled nodes (else rejected)
    goal_sample_rate = 200      # each time after goal_sample_rate iterations the random sample is placed at the goal location
    timelapse = True            # Create a timelapse of path planning stored in results as RRT_Star_Dubins_Timelapse.gif
    show_timelapse = True       # Show the timelapse in real time instead of saving, might be very slow
    fps = 50                    # Fps of timelapse

    rrt = RRT(map, n_max, r_goal, min_dist_nodes, goal_sample_rate, timelapse)

    if timelapse:
        rrt.plot_timelapse("results/RRT", show_timelapse, int(1000 // fps))
        plt.savefig("results/RRT.png")
        if not show_timelapse: plt.show()
    else:
        rrt.run()
        rrt.plot()