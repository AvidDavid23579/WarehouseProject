import numpy as np
from numpy.typing import NDArray


# Clamps values to the interval [min, max]
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(value, max_val))
