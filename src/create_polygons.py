import numpy as np
from collision_detection import Polygon

def extract_polygons(lines):
    vertices = build_vertices_dict(lines)
    list_of_polygons = []
    polygons = []
    set_of_polygons = set()

    # Find all possible polygons based on the vertices and their connections
    for start_vertex in vertices:
        poly_found, polygon = get_poly_path(vertices, start_vertex)
        if poly_found and frozenset(sorted(polygon)) not in set_of_polygons:
            list_of_polygons.append(list(set(polygon)))
            set_of_polygons.add(frozenset(sorted(polygon)))

    # Find all the polygons in the correct order of vertices connections
    for i in range(len(list_of_polygons)):
        polygon = list_of_polygons[i]
        polygon_ordered = []
        vertex = polygon[0]
        polygon_ordered.append(vertex)

        for j in range(len(polygon)-1):
            neighbors = vertices[vertex]
            for nbor in neighbors:
                if nbor in polygon and nbor not in polygon_ordered:
                    polygon_ordered.append(nbor)
                    vertex = nbor
                if j == 0:
                    break
        
        if len(polygon_ordered) >= 3:
            polygons.append(polygon_ordered)

    return polygons


def build_vertices_dict(lines):
    vertices = {}

    for line in lines:
        start, end = (line[0], line[1]), (line[2], line[3])

        # Add start point to the dictionary
        if start not in vertices:
            vertices[start] = []

        # Add connection to end as values for the start key
        vertices[start].append(end)

        # Add end point to the dictionary
        if end not in vertices:
            vertices[end] = []

        # Add connection to start as values for the end key
        vertices[end].append(start)

    return vertices

def get_poly_path(vertices, start_vertex, n=10):
    # List of polygon containing all the vertices as (x, y) tuples
    polygon = []
    polygon.append(start_vertex)
    start_connections = vertices[start_vertex]
    path_found = False
    left_parent, right_parent = start_vertex, start_vertex

    if len(start_connections) >= 2:
        # Find the two neighbors of the start vertex
        left_nbor = start_connections[0]
        right_nbor = start_connections[1]

        # Store the two neighbors in the list of polygon
        polygon.append(left_nbor)
        polygon.append(right_nbor)

        for _ in range(n):
            if path_found:
                break
            else:
                # Find the two neighbors of the left neighbor
                left_connections = vertices[left_nbor].copy()
                left_connections.remove(left_parent) # Remove the 'parent' (already stored in polygon) of the left neighbor

                # Find the two neighbors of the right neighbor
                right_connections = vertices[right_nbor].copy()
                right_connections.remove(right_parent) # Remove the 'parent' (already stored in polygon) of the right neighbor

                # Store the neighbors as parents
                left_parent = left_nbor
                right_parent = right_nbor

                # Polygon found if the left nbor has the right nbor as neighbor
                if left_connections[0] == right_nbor:
                    path_found = True
                    polygon.append(left_nbor)
                    polygon.append(right_nbor)
                    continue

                left_nbor = left_connections[0]
                right_nbor = right_connections[0]

                # Polygon found if the left nbor and the right nbor are the same
                if left_nbor == right_nbor:
                    polygon.append(left_nbor)
                    path_found = True
                    continue

                polygon.append(left_nbor)
                polygon.append(right_nbor)
       
 
    return path_found, polygon

def convert_to_polygons(polygons_list):
    polygons = []

    for poly in polygons_list:
        polygons.append(Polygon(np.array(poly)))

    return polygons
