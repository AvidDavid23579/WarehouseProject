import numpy as np

from geometry.geo_compute import segment_intersects_segment
from navigation.graph import NavigationGraph


# Clamps values to the interval [min, max]
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(value, max_val))

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
