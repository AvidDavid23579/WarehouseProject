from dataclasses import dataclass
from enum import IntEnum
from typing import Optional


@dataclass
class Pose:
    x: float
    y: float
    theta: Optional[float] = None

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.theta)


class NavPhase(IntEnum):
    INITIAL_TURN = 0
    DRIVE = 1
    TURN = 2
    DONE = 3
