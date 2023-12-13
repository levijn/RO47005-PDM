import numpy as np
import matplotlib.pyplot as plt


class Point:
    """Simple object to represent a point"""
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def get_patch(self, color='r', label=None):
        """Returns a matplotlib patch object"""
        return plt.Circle((self.x, self.y), radius=0.1, color=color, fill=True, label=label)
    
    def __str__(self):
        return f'({self.x}, {self.y})'
    
    def __repr__(self):
        return f'({self.x}, {self.y})'
    

class Circle:
    """Simple object to represent a circle"""
    def __init__(self, x, y, r):
        self.x = x
        self.y = y
        self.r = r
    
    def get_patch(self, fill=True, color='r', label=None):
        """Returns a matplotlib patch object"""
        return plt.Circle((self.x, self.y), radius=self.r, color=color, fill=fill, label=label)
    
    def __str__(self):
        return f'({self.x}, {self.y}, r={self.r})'

    def __repr__(self):
        return f'({self.x}, {self.y}, r={self.r})'

class Line:
    """Simple object to represent a line"""
    def __init__(self, x1, y1, x2, y2):
        self.p1 = Point(x1, y1)
        self.p2 = Point(x2, y2)
    
    def get_length(self):
        return np.hypot(self.p1.x - self.p2.x, self.p1.y - self.p2.y)
    
    def get_patch(self, color='r', label=None):
        """Returns a matplotlib patch object"""
        return plt.Line2D((self.p1.x, self.p2.x), (self.p1.y, self.p2.y), color=color, label=label)
        

class Polygon:
    def __init__(self, vertices):
        self.vertices = vertices
        
    def get_polygon_lines(self) -> list[Line]:
        """Returns a list of lines that make up the polygon"""
        lines = []
        for i in range(len(self.vertices)):
            j = (i + 1) % len(self.vertices)
            lines.append(Line(self.vertices[i,0], self.vertices[i,1], self.vertices[j,0], self.vertices[j,1]))
        return lines
    
    def get_patch(self, fill=True, color='r', label=None):
        """Returns a matplotlib patch object"""
        return plt.Polygon(self.vertices, fill=fill, color=color, label=label)


def is_point_in_polygon(point: Point, polygon: Polygon) -> bool:
    """Checks if a point is inside a polygon"""
    for i in range(len(polygon.vertices)):
        j = (i + 1) % len(polygon.vertices)
        if (polygon.vertices[i,1] > point.y) != (polygon.vertices[j,1] > point.y):
            if point.x < (polygon.vertices[j,0] - polygon.vertices[i,0]) * (point.y - polygon.vertices[i,1]) / (polygon.vertices[j,1] - polygon.vertices[i,1]) + polygon.vertices[i,0]:
                return True
    return False


def check_point_circle_collision(point: Point, circle: Circle) -> bool:
    """Checks if a point collides with a circle"""
    dist_x = point.x - circle.x
    dist_y = point.y - circle.y
    distance = np.hypot(dist_x, dist_y)
    
    if distance <= circle.r:
        return True
    return False


def check_line_point_collision(line: Line, point: Point) -> bool:
    """Checks if a point collides with a line"""
    dist1 = np.hypot(line.p1.x - point.x, line.p1.y - point.y)
    dist2 = np.hypot(line.p2.x - point.x, line.p2.y - point.y)
    
    line_length = line.get_length()
    
    # buffer to account for floating point errors
    buffer = 0.01
    if dist1 + dist2 >= line_length - buffer and dist1 + dist2 <= line_length + buffer:
        return True
    return False


def check_line_circle_collision(line, circle) -> bool:
    """Checks if a circle collides with a line"""
    # first check if either of the line's points are inside the circle
    is_inside_p1 = check_point_circle_collision(line.p1, circle)
    is_inside_p2 = check_point_circle_collision(line.p2, circle)
    if is_inside_p1 or is_inside_p2:
        return True
    
    line_length = line.get_length()
    # dot product with line and circle  
    dot = (((circle.x - line.p1.x) * (line.p2.x - line.p1.x)) + ((circle.y - line.p1.y) * (line.p2.y - line.p1.y))) / np.power(line_length, 2)
    
    # find the closest point on the line to the circle
    closest_x = line.p1.x + (dot * (line.p2.x - line.p1.x))
    closest_y = line.p1.y + (dot * (line.p2.y - line.p1.y))
    
    # check if the closest point is on the line
    is_closest_on_line = check_line_point_collision(line, Point(closest_x, closest_y))
    if not is_closest_on_line:
        return False
    
    # check if the closest point is within the circle
    dist_to_closest_point = np.hypot(closest_x - circle.x, closest_y - circle.y)
    if dist_to_closest_point <= circle.r:
        return True
    return False


def check_polygon_circle_collision(polygon, circle) -> bool:
    """Checks if a circle collides with a polygon"""
    # first check if circle is inside polygon
    is_inside = is_point_in_polygon(Point(circle.x, circle.y), polygon)
    if is_inside:
        return True
    
    # then check if any of the polygon's lines intersect with the circle
    for i in range(len(polygon.vertices)):
        j = (i + 1) % len(polygon.vertices)
        line = Line(polygon.vertices[i,0], polygon.vertices[i,1], polygon.vertices[j,0], polygon.vertices[j,1])
        if check_line_circle_collision(line, circle):
            return True
    return False


if __name__ == '__main__':
    vertices = np.array([[0,0], [2,0], [1,1], [0,1]])
    polygon = Polygon(vertices)
    
    print(check_line_point_collision(Line(0,0,1,1), Point(0,0)))
    
    circle1 = Circle(0.5, 0.5, 0.5)
    circle2 = Circle(1.5, 0.8, 0.2)
    
    circle1_collision = check_polygon_circle_collision(polygon, circle1)
    circle2_collision = check_polygon_circle_collision(polygon, circle2)
    
    fig, ax = plt.subplots()

    plt.plot(vertices[:,0], vertices[:,1])
    if circle1_collision:

        ax.add_patch(circle1.get_patch(color='r'))
    else:
        ax.add_patch(circle1.get_patch(color='g'))
    if circle2_collision:
        circle = plt.Circle((circle2.x, circle2.y), radius=circle2.r, color='r', fill=True)
        ax.add_patch(circle)
    else:
        circle = plt.Circle((circle2.x, circle2.y), radius=circle2.r, color='g', fill=True)
        ax.add_patch(circle)
    
    plt.grid()
    plt.axis('equal')
    plt.show()
    