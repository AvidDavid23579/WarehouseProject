# Helper algorithms and stores dataclasses

from dataclasses import dataclass

import numpy as np


@dataclass
class Pose:
    x: float
    y: float
    theta: float


def wrap_angle(a):
    while a > np.pi:
        a -= 2 * np.pi
    while a < -np.pi:
        a += 2 * np.pi
    return a


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))
