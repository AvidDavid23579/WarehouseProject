import numpy as np

from navigation.graph import NavigationGraph


# Clamps values to the interval [min, max]
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(value, max_val))
