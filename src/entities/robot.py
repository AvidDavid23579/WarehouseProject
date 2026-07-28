from dataclasses import dataclass

import numpy as np

from common.types import Pose
from common.utils import clamp, wrap_angle
from config import MAX_OMEGA, MAX_VELOCITY, PHYSICS_DT


@dataclass(slots=True)
class RobotInfo:
    id: int
    goal: Pose


@dataclass(slots=True)
class RobotState:
    pose: Pose

    v: float
    omega: float

    def __init__(self, pose: Pose):
        self.pose = pose.copy()
        self.v = 0.0
        self.omega = 0.0


@dataclass(slots=True)
class RobotFrame:
    x: float
    y: float
    theta: float


class Robot:
    def __init__(self, info: RobotInfo, start_pose: Pose) -> None:
        self.info = info
        self.start_pose = start_pose.copy()
        self.state = RobotState(start_pose)

    def step(self) -> None:
        self.drive_to_pose()

        self.state.pose.x += self.state.v * np.cos(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.y += self.state.v * np.sin(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.theta += self.state.omega * PHYSICS_DT

    def drive_to_pose(self) -> None:
        goal = self.info.goal
        pose = self.state.pose

        # Calculate distance to goal
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        dist = np.hypot(dx, dy)

        # Position reached
        if dist < 0.01:
            self.state.v = 0.0

            if goal.theta is None:
                # Keep whatever heading we arrived with.
                self.state.omega = 0.0
                return

            heading_error = wrap_angle(goal.theta - pose.theta)

            self.state.omega = clamp(7.5 * heading_error, -MAX_OMEGA, MAX_OMEGA)
            if abs(heading_error) < np.deg2rad(0.2):
                self.state.omega = 0.0
            return

        # Drive toward target position.
        target_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(target_heading - pose.theta)

        self.state.omega = clamp(4.0 * heading_error, -MAX_OMEGA, MAX_OMEGA)

        # Slow down when facing away from the goal.
        self.state.v = clamp(5.0 * dist * max(0.0, np.cos(heading_error)), 0.0, MAX_VELOCITY)
