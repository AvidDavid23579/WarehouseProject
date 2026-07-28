from dataclasses import dataclass

import numpy as np

from common.types import Pose
from common.utils import clamp, rotated_rectangle_vertices, wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, MAX_VELOCITY, PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH


@dataclass(slots=True)
class RobotInfo:
    id: int
    goals: list[Pose]


@dataclass(slots=True)
class RobotState:
    pose: Pose
    goal_index: int
    crashed: bool

    v: float
    omega: float

    def __init__(self, pose: Pose):
        self.pose = pose.copy()
        self.goal_index = 0
        self.v = 0.0
        self.omega = 0.0
        self.crashed = False


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
        if self.state.crashed:
            return

        self.state.pose.x += self.state.v * np.cos(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.y += self.state.v * np.sin(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.theta += self.state.omega * PHYSICS_DT

    @property
    def goal(self) -> Pose:
        return self.info.goals[self.state.goal_index]

    def update_goal(self) -> None:
        if self.state.crashed:
            return

        goal = self.goal
        pose = self.state.pose

        dist = np.hypot(goal.x - pose.x, goal.y - pose.y)

        if dist < 0.05:
            self.state.goal_index = (self.state.goal_index + 1) % len(self.info.goals)

    def drive_to_pose(self) -> None:
        if self.state.crashed:
            return

        goal = self.goal
        pose = self.state.pose

        # Calculate distance to goal
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        dist = np.hypot(dx, dy)

        # Position reached
        if dist < DIST_TOLERANCE:
            self.state.v = 0.0

            if goal.theta is None:
                # Keep whatever heading we arrived with.
                self.state.omega = 0.0
                return

            heading_error = wrap_angle(goal.theta - pose.theta)

            self.state.omega = clamp(10 * heading_error, -MAX_OMEGA, MAX_OMEGA)
            if abs(heading_error) < ANGLE_TOLERANCE:
                self.state.omega = 0.0
            return

        # Drive toward target position.
        target_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(target_heading - pose.theta)

        self.state.omega = clamp(7.5 * heading_error, -MAX_OMEGA, MAX_OMEGA)

        # Slow down when facing away from the goal.
        self.state.v = clamp(5.0 * dist * max(0.0, np.cos(heading_error)), 0.0, MAX_VELOCITY)

    def crash(self) -> None:
        if self.state.crashed:
            return

        self.state.crashed = True
        self.state.v = 0
        self.state.omega = 0

    def robot_vertices(self) -> list[tuple[float, float]]:
        return rotated_rectangle_vertices(self.state.pose, ROBOT_LENGTH, ROBOT_WIDTH)
