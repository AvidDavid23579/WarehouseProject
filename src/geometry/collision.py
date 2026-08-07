import numpy as np

from config import X_MAX, X_MIN, Y_MAX, Y_MIN
from geometry.geo_compute import edge_normals, project


def robot_boundary_collisions(world):
    vertices = world.robot.vertices
    outside = (vertices[:, :, 0] < X_MIN) | (vertices[:, :, 0] > X_MAX) | (vertices[:, :, 1] < Y_MIN) | (vertices[:, :, 1] > Y_MAX)
    crashed = np.any(outside, axis=1)
    new = crashed & (~world.robot.crashed)

    world.robot.crashed[new] = True
    world.robot.twist[new] = 0.0


# Returns True when two convex polygons overlap
def sat_collision(polygon_a: list[tuple[float, float]], polygon_b: list[tuple[float, float]]) -> bool:

    axes = list(edge_normals(polygon_a)) + list(edge_normals(polygon_b))

    for axis in axes:
        min_a, max_a = project(polygon_a, axis)
        min_b, max_b = project(polygon_b, axis)
        if max_a < min_b or max_b < min_a:
            return False

    return True
