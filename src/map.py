import numpy as np
import matplotlib.pyplot as plt

from collision_detection import Polygon, Point, Line, Circle, is_point_in_polygon, check_polygon_circle_collision

class Node:
    def __init__(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.pos = [x, y, yaw]
        self.parent = None
        self.dparent = 0
        self.cost = 0.0

    def get_patch(self, color='r', label=None):
        """Returns a matplotlib patch object"""
        return plt.Circle((self.x, self.y), radius=1, color=color, fill=True, label=label)

class Map:
    def __init__(self, obstacles: list[Polygon], start: Node, goal: Node, size: tuple[int, int] = (40, 40), plot_start_goal: bool = False):
        self.obstacles = obstacles
        self.size = size
        self.dimensions = [[0, size[0]], [0, size[1]]]
        self.start = start
        self.goal = goal
        self.car_size = 1.2
        self.plot_start_goal = plot_start_goal
        
    def get_patches(self) -> list:
        """Returns a list of matplotlib patch objects for the map"""
        patches = []
        for obstacle in self.obstacles:
            patches.append(obstacle.get_patch(color='black'))
        
        if self.plot_start_goal:
            patches.append(self.start.get_patch(color='g'))
            patches.append(self.goal.get_patch(color='b'))
        return patches
    
    
    def is_point_in_obstacle(self, point: Point) -> bool:
        """Checks if a point is inside an obstacle"""
        for obstacle in self.obstacles:
            if check_polygon_circle_collision(obstacle, Circle(point.x, point.y, self.car_size)):
                return True
            # if is_point_in_polygon(point, obstacle):
            #     return True
        return False
    
    def check_collision_line(self, line: Line, line_stepsize: float=None) -> bool:
        """Checks if a line collides with an obstacle"""
        if line_stepsize is None:
            line_stepsize = self.car_size
        #devide line into points by interpolation
        length = line.get_length()
        x = np.linspace(line.p1.x, line.p2.x, int(length / line_stepsize))
        y = np.linspace(line.p1.y, line.p2.y, int(length / line_stepsize))
        points = [[x[i], y[i]] for i in range(len(x))]
        
        # check if any of the points is in an obstacle
        for obstacle in self.obstacles:
            if isinstance(obstacle, Polygon):
                for point in points:
                    car_circle = Circle(point[0], point[1], self.car_size)
                    if check_polygon_circle_collision(obstacle, car_circle):
                        return True
        return False


def get_simple_map():
    """Returns a simple map"""
    size = (10,10)
    obstacles = []
    
    # add outer walls
    
    # add 2 inner walls 
    obstacles.append(Polygon(np.array([[3, 3], [3, 7], [6, 6], [7, 3]])))
    obstacles.append(Polygon(np.array([[0, 4], [3, 3], [3, 4], [0, 5]])))

    start = Node(1, 1, 0)
    goal = Node(2, 8, np.pi/4)
    
    return Map(obstacles, start, goal, size=size)

def get_simple_map_large():
    """Returns a large simple map"""
    size = (50, 50)
    obstacles = []
    
    # add outer walls
    
    # add 2 inner walls 
    obstacles.append(Polygon(np.array([[10, 3], [10, 30], [25, 30], [25, 3]])))
    obstacles.append(Polygon(np.array([[0, 4], [10, 3], [10, 12], [0, 12]])))

    start = Point(1, 1)
    goal = Point(10, 40)
    
    return Map(obstacles, start, goal, size=size)


def get_random_map(num_obstacles: int, size: tuple[int, int] = (40, 40)):
    """Returns a random map"""
    obstacles = []
    for i in range(num_obstacles):
        # generate random polygon
        num_vertices = np.random.randint(3, 7)
        vertices = np.random.rand(num_vertices, 2) * size
        obstacles.append(Polygon(vertices))
    
    # generate random start and goal
    reachabable = False
    while not reachabable:
        start = Point(*np.random.rand(2) * size)
        goal = Point(*np.random.rand(2) * size)
        if not any([is_point_in_polygon(start, obstacle) for obstacle in obstacles]) and \
            not any([is_point_in_polygon(goal, obstacle) for obstacle in obstacles]):
            reachabable = True
    
    return Map(obstacles, start, goal, size=size)


def create_grid_map():
    """Creates a grid map"""
    
    def create_square_polygon(x, y, size):
        """Creates a square polygon"""
        return Polygon(np.array([[x, y], [x, y+size], [x+size, y+size], [x+size, y]]))
    
    def create_rect_polygon(x, y, size_x, size_y):
        """Creates a rectangle polygon"""
        return Polygon(np.array([[x, y], [x, y+size_y], [x+size_x, y+size_y], [x+size_x, y]]))
    
    size = 30
    
    wall_size = 0.5
    #create border walls
    obstacles = []
    obstacles.append(create_rect_polygon(0, 0, size, wall_size))
    obstacles.append(create_rect_polygon(0, 0, wall_size, size))
    obstacles.append(create_rect_polygon(0, size-wall_size, size, wall_size))
    obstacles.append(create_rect_polygon(size-wall_size, 0, wall_size, size))

    size_box = 2
    num_boxes = 3
    step_size = 8.5
    start_x = 5.5
    start_y = 5.5
    #create inner walls
    for i in range(num_boxes):
        obstacles.append(create_square_polygon(start_x + i*step_size, start_y, size_box))
        obstacles.append(create_square_polygon(start_x + i*step_size, start_y + step_size, size_box))
        obstacles.append(create_square_polygon(start_x + i*step_size, start_y + 2*step_size, size_box))

    
    # create start and goal
    start = Node(2.5, 2.5, 0)
    goal = Node(size-2.5, size-2.5, 0)
    
    return Map(obstacles, start, goal, size=(size,size))
    
def get_random_line(max_x, max_y):
    coords = np.random.randint(0, max_x, size=4)
    return Line(coords[0], coords[1], coords[2], coords[3])

if __name__ == '__main__':
    #plot simple example map
    map = get_simple_map()
    patches = map.get_patches()
    
    line_collisions = []
    lines = []
    for i in range(10):
        line = get_random_line(10, 10)
        line_collisions.append(map.check_collision_line(line))
        lines.append(line)
    
    
    fig, ax = plt.subplots()
    
    for i, line in enumerate(lines):
        if line_collisions[i]:
            ax.plot([line.p1.x, line.p2.x], [line.p1.y, line.p2.y], color='r')
        else:
            ax.plot([line.p1.x, line.p2.x], [line.p1.y, line.p2.y], color='g')
    for patch in patches:
        ax.add_patch(patch)
    
    plt.xlim(0, 10)
    plt.ylim(0, 10)
    plt.show()
    
    