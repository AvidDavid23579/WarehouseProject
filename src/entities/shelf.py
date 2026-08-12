from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class ShelfState:
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
    vertices: np.ndarray = field(default_factory=lambda: np.empty((0, 4, 2), dtype=np.float32))
    index: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.int32))
