from collision_detection import Polygon, Point
import numpy as np


class Map:
    def __init__(self, obstacles: list[Polygon], start: Point, goal: Point, size: tuple[int, int] = (100, 100)):
        self.obstacles = obstacles
        self.size = size
        self.start = start
        self.goal = goal
        
    
    def get_patches(self) -> list:
        """Returns a list of matplotlib patch objects for the map"""
        patches = []
        for obstacle in self.obstacles:
            patches.append(obstacle.get_patch(color='black'))
        patches.append(self.start.get_patch(color='g'))
        patches.append(self.goal.get_patch(color='b'))
        return patches


def get_simple_map():
    """Returns a simple map"""
    obstacles = []
    obstacles.append(Polygon(np.array([[0, 0], [0, 10], [10, 10], [10, 0]])))
    obstacles.append(Polygon(np.array([[20, 0], [20, 10], [30, 10], [30, 0]])))
    obstacles.append(Polygon(np.array([[0, 20], [0, 30], [10, 30], [10, 20]])))
    obstacles.append(Polygon(np.array([[20, 20], [20, 30], [30, 30], [30, 20]])))
    obstacles.append(Polygon(np.array([[40, 0], [40, 10], [50, 10], [50, 0]])))
    obstacles.append(Polygon(np.array([[20, 40], [20, 50], [30, 50], [30, 40]])))
    obstacles.append(Polygon(np.array([[20, 60], [20, 70], [30, 70], [30, 60]])))
    
    start = Point(5, 5)
    goal = Point(65, 65)
    
    return Map(obstacles, start, goal)