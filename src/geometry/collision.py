import math

import numpy as np

from config import X_MAX, X_MIN, Y_MAX, Y_MIN


def robot_boundary_collisions(world):
    vertices = world.robot.vertices
    outside = (vertices[:, :, 0] < X_MIN) | (vertices[:, :, 0] > X_MAX) | (vertices[:, :, 1] < Y_MIN) | (vertices[:, :, 1] > Y_MAX)
    crashed = np.any(outside, axis=1)
    new = crashed & (~world.robot.crashed)

    world.robot.crashed[new] = True
    world.robot.twist[new] = 0.0


# Yield unit normals for each edge of a convex polygon
def _edge_normals(polygon: list[tuple[float, float]]):
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        edge = (x2 - x1, y2 - y1)
        normal = (-edge[1], edge[0])
        length = math.hypot(*normal)
        yield normal[0] / length, normal[1] / length


# Project polygon vertices onto *axis* and return (min, max) scalar range
def _project(polygon: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    dots = [px * axis[0] + py * axis[1] for px, py in polygon]
    return min(dots), max(dots)


# Returns True when two convex polygons overlap
def sat_collision(polygon_a: list[tuple[float, float]], polygon_b: list[tuple[float, float]]) -> bool:

    axes = list(_edge_normals(polygon_a)) + list(_edge_normals(polygon_b))

    for axis in axes:
        min_a, max_a = _project(polygon_a, axis)
        min_b, max_b = _project(polygon_b, axis)
        if max_a < min_b or max_b < min_a:
            return False

    return True
