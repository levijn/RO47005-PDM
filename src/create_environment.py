import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
import json

from create_polygons import extract_polygons, convert_to_polygons
from map import Map, Node

class EnvironmentCreator:
    def __init__(self, root, canvas_dimensions=[500, 500], env_dimensions=[500, 500], scale=5, env_name="map"):
        # Canvas attributes
        self.root = root
        self.root.title("Environment Creator")
        self.canvas_width = canvas_dimensions[0]
        self.canvas_height = canvas_dimensions[1]

        # Environment attributes
        self.env_name = env_name
        self.env_width = env_dimensions[0]
        self.env_height = env_dimensions[1]
        self.scale = scale # The actual size of the environment is scaled down, i.e., (env_width // scale, env_height // scale)

        # Pose of start and goal node
        self.node = ""
        self.start_node = [0, 0, 0]
        self.goal_node = [0, 0, 0]
        self.node_position = False
        self.node_yaw = False

        # Polygon variables
        self.lines = []  # List to store lines displayed on canvas (0, 0) top left
        self.adjusted_lines = [] # List to store adjusted lines for polygon (0, 0) bottom left

        # Drawing parameters
        self.start_x = None
        self.start_y = None

        self.canvas = tk.Canvas(root, width=self.canvas_width, height=self.canvas_height, bg="white")
        self.canvas.pack(expand=tk.YES, fill=tk.BOTH)

        self.draw_valid_area()  # Initial draw of the valid area

        # Create buttons
        startButton = tk.Button(root, height=2, width=20, text="Place Start", command=self.place_start_node)
        goalButton = tk.Button(root, height=2, width=20, text="Place Goal", command=self.place_goal_node)
        stopButton = tk.Button(root, height=2, width=20, text="Stop and Save Environment", command=self.stop)

        startButton.pack()
        goalButton.pack()
        stopButton.pack()

        # Mouse bindings
        self.canvas.bind("<Button-1>", self.start_draw)
        self.canvas.bind("<B1-Motion>", self.draw)
        self.canvas.bind("<ButtonRelease-1>", self.end_draw)

    def draw_valid_area(self):
        # Draw valid area of where the environment can be created (bounded by green rectangle)
        self.canvas.delete("valid_area")
        self.canvas.create_rectangle(2, 2, self.env_width+1, self.env_height+1, outline="green", width=1, tags="valid_area")

    def check_draw(self, x, y):
        # Check if the cursor is inside the valid draw area
        return x >= 0 and x <= self.env_width and y >= 0 and y <= self.env_height
    
    def placing_node_position(self, x=None, y=None):
        # Place the position of the start or goal node
        if self.node_position and x is not None and y is not None:
            if self.node == "start":
                self.start_node[0], self.start_node[1] = x, y
            else:
                self.goal_node[0], self.goal_node[1] = x, y
            self.canvas.unbind("<Motion>")
            self.canvas.bind("<Motion>", self.update_node_yaw)
            self.node_position = False
        return self.node_position
    
    def placing_node_yaw(self, x=None, y=None):
        # Place the yaw of the start or goal node
        if self.node_yaw and x is not None and y is not None:
            if self.node == "start":
                self.start_node[2] = -np.arctan2(y - self.start_node[1], x - self.start_node[0])
            else:
                self.goal_node[2] = -np.arctan2(y - self.goal_node[1], x - self.goal_node[0])
            self.canvas.unbind("<Motion>")
            self.node_yaw = False
        return self.node_yaw

    def start_draw(self, event):
        if not self.check_draw(event.x, event.y):
            return
        
        if self.placing_node_position(event.x, event.y):
            return
        
        if self.placing_node_yaw(event.x, event.y):
            return

        self.start_x = event.x
        self.start_y = event.y
        closest_point = self.find_closest_point(self.start_x, self.start_y)

        if closest_point is not None:
            self.start_x, self.start_y = closest_point

    def draw(self, event):
        if not self.check_draw(event.x, event.y):
            return
        
        if self.placing_node_position():
            return
        
        if self.placing_node_yaw():
            return

        if self.start_x is not None and self.start_y is not None:
            end_x = event.x
            end_y = event.y

            # Find the closest end point among existing lines
            closest_point = self.find_closest_point(end_x, end_y)

            # If a close point is found, snap to it
            if closest_point is not None:
                end_x, end_y = closest_point

            self.canvas.delete("current_line")
            self.canvas.create_line(self.start_x, self.start_y, end_x, end_y, tags="current_line")

    def end_draw(self, event):
        if not self.check_draw(event.x, event.y):
            return
        
        if self.placing_node_position():
            return
        
        if self.placing_node_yaw():
            return
        
        if self.start_x is not None and self.start_y is not None:
            end_x = event.x
            end_y = event.y

            # Find the closest end point among existing lines
            closest_point = self.find_closest_point(end_x, end_y)

            # If a close point is found, snap to it
            if closest_point is not None:
                end_x, end_y = closest_point

            line_coords = (self.start_x, self.start_y, end_x, end_y)
            line_coords_adjusted = (self.start_x, self.env_height-self.start_y, end_x, self.env_height-end_y)

            # Store the line coordinates in the list
            self.lines.append(line_coords)
            self.adjusted_lines.append(line_coords_adjusted)

            # Draw the line on the canvas
            self.canvas.create_line(line_coords, tags="all_lines")

            #print(self.lines)

            # Reset start coordinates
            self.start_x = None
            self.start_y = None

    def find_closest_point(self, x, y):
        # Find the closest end point among existing lines
        closest_point = None
        min_distance = float("inf")

        for line in self.lines:
            start_x, start_y, end_x, end_y = line

            # Check distance to start point
            distance_start = ((x - start_x) ** 2 + (y - start_y) ** 2) ** 0.5
            if distance_start < min_distance:
                min_distance = distance_start
                closest_point = (start_x, start_y)

            # Check distance to end point
            distance_end = ((x - end_x) ** 2 + (y - end_y) ** 2) ** 0.5
            if distance_end < min_distance:
                min_distance = distance_end
                closest_point = (end_x, end_y)

        # Return the closest point if it is within a certain threshold distance
        threshold_distance = 5
        return closest_point if min_distance < threshold_distance else None
    
    def draw_arrow(self, x0, y0, x1, y1, l=5):
        # Draw arrow to indicate the yaw of the node
        tag = self.node + "_arrow"

        self.canvas.delete(tag)
        angle = np.arctan2(y1 - y0, x1 - x0)
        self.canvas.create_line(x0, y0, x0+np.cos(angle)*l, y0+np.sin(angle)*l, arrow=tk.LAST, tags=tag)
    
    def update_node_position(self, event, r=10):
        x, y = event.x, event.y
        fill_color = "green" if self.node == "start" else "red"
        tag = self.node

        self.canvas.delete(tag)
        self.canvas.delete(tag + "_arrow")

        self.canvas.create_oval(x-r, y-r, x+r, y+r, fill=fill_color, outline="", tags=tag)

    def update_node_yaw(self, event):
        if self.node == "start":
            x0, y0 = self.start_node[0], self.start_node[1] 
        else:
            x0, y0 = self.goal_node[0], self.goal_node[1]
        self.draw_arrow(x0, y0, event.x, event.y)
        self.node_yaw = True

    def place_start_node(self):
        self.canvas.bind("<Motion>", self.update_node_position)
        self.node = "start"
        self.node_position = True

    def place_goal_node(self):
        self.canvas.bind("<Motion>", self.update_node_position)
        self.node = "goal"
        self.node_position = True

    def stop(self):
        self.create_environment()

    def create_environment(self):
        self.adjusted_lines = np.array(self.adjusted_lines) // self.scale
        self.start_node[0], self.start_node[1] = self.start_node[0] // self.scale, (self.env_height-self.start_node[1]) // self.scale
        self.goal_node[0], self.goal_node[1] = self.goal_node[0] // self.scale, (self.env_height-self.goal_node[1]) // self.scale
        
        polygons = extract_polygons(self.adjusted_lines)
        polygons = [[tuple(int(i) for i in vertex) for vertex in polygon] for polygon in polygons]
        print(polygons)
        print(self.start_node, self.goal_node)
        save = {}
        save["obstacles"] = polygons
        save["start"] = self.start_node
        save["goal"] = self.goal_node
        save["size"] = (self.env_width // self.scale, self.env_height // self.scale)

        with open("maps/" +self.env_name + ".json", 'w') as f:
            json.dump(save, f)

        self.root.destroy()

def plot_map(map):
    patches = map.get_patches()
    print(map.start)
    fig, ax = plt.subplots()

    for patch in patches:
        ax.add_patch(patch)
    
    plt.xlim(0, map.size[0])
    plt.ylim(0, map.size[1])
    plt.show()

def load_environment(env_name):
    f = open("maps/" + env_name + ".json")
    save = json.load(f)

    obstacles = save["obstacles"]
    start_node = Node(save["start"][0], save["start"][1], save["start"][2])
    goal_node = Node(save["goal"][0], save["goal"][1], save["goal"][2])
    size = save["size"]

    map = Map(convert_to_polygons(obstacles), start_node, goal_node, size)
    f.close()
    return map

def main(env_name="map"):
    root = tk.Tk()
    draw_canvas = EnvironmentCreator(root, [500, 500], [500, 500], 5, env_name)
    root.mainloop()

    map = load_environment(env_name)
    
    plot_map(map)


if __name__ == "__main__":
    env_name = "map"
    main(env_name)