from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class PalletState:
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), np.float32))
    vertices: np.ndarray = field(default_factory=lambda: np.empty((0, 4, 2), np.float32))
    index: np.ndarray = field(default_factory=lambda: np.empty(0, np.int32))
