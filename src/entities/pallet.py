from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np


@dataclass
class PalletState:
    status: np.ndarray
    index: np.ndarray
    pose: np.ndarray
    vertices: np.ndarray
    robot_id: np.ndarray

class PalletStatus(IntEnum):
    UNDELIVERED = 0
    RESERVED = 1
    DELIVERING = 2
    DELIVERED = 3
