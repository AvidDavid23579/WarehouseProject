from dataclasses import dataclass
from typing import Optional


@dataclass
class Pose:
    x: float
    y: float
    theta: Optional[float] = None

    def copy(self) -> "Pose":
        return Pose(self.x, self.y, self.theta)
