from dataclasses import dataclass

from common.types import Pose


@dataclass(slots=True)
class RobotInfo:
    id: int
    length: float
    width: float
    max_velocity: float
    max_accel: float


@dataclass(slots=True)
class RobotState:
    pose: Pose

    velocity: float
    angular_velocity: float


class Robot:
    def __init__(self, info: RobotInfo, state: RobotState) -> None:
        self.info = info
        self.state = state
