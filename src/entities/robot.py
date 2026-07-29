import math
from dataclasses import dataclass

import numpy as np

from common.potential import apply_repulsion
from common.types import Pose
from common.utils import clamp, rotated_rectangle_vertices, wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, MAX_VELOCITY, PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH


@dataclass(slots=True)
class RobotInfo:
    id: int


@dataclass(slots=True)
class NavigationState:
    current_node_id: int | None
    target_node_id: int | None
    path: list[int]
    path_index: int


@dataclass(slots=True)
class RobotState:
    pose: Pose
    v: float
    omega: float
    crashed: bool

    last_goal_dist: float
    stuck_time: float

    navigation: NavigationState

    def __init__(self, pose: Pose):
        self.pose = pose.copy()

        self.v = 0.0
        self.omega = 0.0

        self.crashed = False

        self.last_goal_dist = 0.0
        self.stuck_time = 0.0

        self.navigation = NavigationState(
            current_node_id=None,
            target_node_id=None,
            path=[],
            path_index=0,
        )


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

        self.state.pose.x += self.state.v * math.cos(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.y += self.state.v * math.sin(self.state.pose.theta) * PHYSICS_DT
        self.state.pose.theta += self.state.omega * PHYSICS_DT

    def drive_to_pose(self) -> None:
        if self.state.crashed:
            return

        goal = self.goal
        pose = self.state.pose

        # Calculate distance to goal
        dx = goal.x - pose.x
        dy = goal.y - pose.y
        dist = math.hypot(dx, dy)

        # Position reached
        if dist < DIST_TOLERANCE:
            self.state.v = 0.0

            if goal.theta is None:
                # Keep whatever heading we arrived with.
                self.state.omega = 0.0
                return

            heading_error = wrap_angle(goal.theta - pose.theta)

            if abs(heading_error) < ANGLE_TOLERANCE:
                self.state.omega = 0.0
            else:
                self.state.omega = 10.0 * heading_error
            return

        # Drive toward target position.
        target_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(target_heading - pose.theta)

        self.state.omega = 7.5 * heading_error

        # Slow down when facing away from the goal.
        self.state.v = 5.0 * dist * max(0.0, math.cos(heading_error))

        apply_repulsion(self)

        self.state.v = clamp(self.state.v, -MAX_VELOCITY, MAX_VELOCITY)
        self.state.omega = clamp(self.state.omega, -MAX_OMEGA, MAX_OMEGA)

    def crash(self) -> None:
        if self.state.crashed:
            return

        self.state.crashed = True
        self.state.v = 0
        self.state.omega = 0

    def robot_vertices(self) -> list[tuple[float, float]]:
        return rotated_rectangle_vertices(self.state.pose, ROBOT_LENGTH, ROBOT_WIDTH)
