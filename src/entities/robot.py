from dataclasses import dataclass

import numpy as np

from common.types import Pose
from config import PHYSICS_DT


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

    v: float
    omega: float


@dataclass(slots=True)
class RobotFrame:
    x: float
    y: float
    theta: float


class Robot:
    def __init__(self, info: RobotInfo, state: RobotState) -> None:
        self.info = info
        self.state = state

    def step(self) -> None:
        self.state.pose.x += self.state.v * np.cos(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.y += self.state.v * np.sin(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.theta += self.state.omega * PHYSICS_DT

        print(np.degrees(self.state.pose.theta))
