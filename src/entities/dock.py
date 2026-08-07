from dataclasses import dataclass, field

import numpy as np


@dataclass(slots=True)
class DockState:
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), np.float32))
    vertices: np.ndarray = field(default_factory=lambda: np.empty((0, 4, 2), np.float32))
    node_pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), np.float32))
