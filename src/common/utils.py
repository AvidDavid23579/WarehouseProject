import numpy as np

from geometry.geo_compute import segment_intersects_segment
from navigation.graph import NavigationGraph


# Clamps values to the interval [min, max]
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(value, max_val))


def compress_collinear_path(
    graph: NavigationGraph,
    path: list[int],
) -> list[int]:

    if len(path) <= 2:
        return path

    result = [path[0]]

    for i in range(1, len(path) - 1):
        a = graph.node_pose[path[i - 1]]
        b = graph.node_pose[path[i]]
        c = graph.node_pose[path[i + 1]]

        v1 = b[:2] - a[:2]
        v2 = c[:2] - b[:2]

        cross = v1[0] * v2[1] - v1[1] * v2[0]

        dot = np.dot(v1, v2)

        if abs(cross) > 1e-6 or dot <= 0:
            result.append(path[i])

    result.append(path[-1])

    return result

def segment_intersects_shelf(
    p1: np.ndarray,
    p2: np.ndarray,
    vertices: np.ndarray,
) -> bool:

    for i in range(4):
        q1 = vertices[i]
        q2 = vertices[(i + 1) % 4]

        if segment_intersects_segment(p1, p2, q1, q2):
            return True

    return False
