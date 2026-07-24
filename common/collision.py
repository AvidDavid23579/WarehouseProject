"""Separating Axis Theorem (SAT) collision detection for convex polygons."""

import math


def _edge_normals(polygon: list[tuple[float, float]]):
    """Yield unit normals for each edge of a convex polygon."""
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        edge = (x2 - x1, y2 - y1)
        normal = (-edge[1], edge[0])
        length = math.hypot(*normal)
        yield normal[0] / length, normal[1] / length


def _project(polygon: list[tuple[float, float]], axis: tuple[float, float]) -> tuple[float, float]:
    """Project polygon vertices onto *axis* and return (min, max) scalar range."""
    dots = [px * axis[0] + py * axis[1] for px, py in polygon]
    return min(dots), max(dots)


def sat_collision(
    polygon_a: list[tuple[float, float]], polygon_b: list[tuple[float, float]]
) -> bool:
    """Return True when two convex polygons overlap (SAT finds no separating axis)."""
    axes = list(_edge_normals(polygon_a)) + list(_edge_normals(polygon_b))

    for axis in axes:
        min_a, max_a = _project(polygon_a, axis)
        min_b, max_b = _project(polygon_b, axis)
        if max_a < min_b or max_b < min_a:
            return False

    return True
