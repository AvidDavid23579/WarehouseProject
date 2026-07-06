import math


# Separating axis theorem (to check if two convex polygons collide)
def edges_axes(poly):
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        edge = (x2 - x1, y2 - y1)
        normal = (-edge[1], edge[0])
        length = math.hypot(*normal)
        yield (normal[0] / length, normal[1] / length)

def project(poly, axis):
    dots = [px * axis[0] + py * axis[1] for px, py in poly]
    return min(dots), max(dots)

def sat_collision(poly_a, poly_b):
    for axis in list(edges_axes(poly_a)) + list(edges_axes(poly_b)):
        min_a, max_a = project(poly_a, axis)
        min_b, max_b = project(poly_b, axis)
        if max_a < min_b or max_b < min_a:
            return False  # separating axis found
    return True  # no separating axis on any edge normal -> colliding

