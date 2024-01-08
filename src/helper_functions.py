from math import atan2, sin, cos
import numpy as np
from collision_detection import Point

"""
:param angle:       (float) angle [rad]

:return angle:      (float) angle [rad]
"""

normalise_angle = lambda angle: atan2(sin(angle), cos(angle))

def get_distance(node1, node2):
    """Returns the euclidean distance between two nodes"""
    distance = np.hypot(node1.x - node2.x, node1.y - node2.y)
    return distance

def get_dubins_distance(node1, node2, dubins):
    """Returns the distance based on the dubins path between two nodes"""
    _, dubins_distance = dubins.dubins_path(node1.pos, node2.pos, True)
    return dubins_distance

def collision_check(path, map):
    """
    Check if path between two nodes is not in collision with the obstacles
    in the map. Furthermore, another check to see if the path is not outside
    the map boundaries.
    """
    in_collision = False
    outside_map = False

    for (x, y) in path:
        if in_collision or outside_map:
            break
        else:
            in_collision = map.is_point_in_obstacle(Point(x, y))
            outside_map = x < map.dimensions[0][0] or x > map.dimensions[0][1] or y < map.dimensions[1][0] or y > map.dimensions[1][1]

    return in_collision or outside_map