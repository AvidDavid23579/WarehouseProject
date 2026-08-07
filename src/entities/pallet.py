from dataclasses import dataclass, field

import numpy as np

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import PALLET_LENGTH, PALLET_WIDTH


@dataclass
class PalletFrame:
    x: float
    y: float
    theta: float


@dataclass(slots=True)
class PalletState:
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), np.float32))
    vertices: np.ndarray = field(default_factory=lambda: np.empty((0, 4, 2), np.float32))
    index: np.ndarray = field(default_factory=lambda: np.empty((0, 1), np.float32))
