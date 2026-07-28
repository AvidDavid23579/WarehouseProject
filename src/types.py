from dataclasses import dataclass


@dataclass
class WorldState:
    time: float


@dataclass
class Pose:
    x: float
    y: float
    theta: float


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
