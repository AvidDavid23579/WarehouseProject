"""Mobile robot entity: kinematics, control, and collision geometry."""

import math

import numpy as np

from common.navigation import naive_drive_to_pose
from common.potential import apply_repulsion
from common.utils import Pose, rotated_rectangle_vertices, wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, ROBOT_LENGTH, ROBOT_WIDTH


class Robot:
    """Differential-drive robot that cycles through a list of goal poses."""

    # Inflated hitbox used for proactive avoidance (larger than visual footprint).
    HITBOX_SCALE = 1.5

    def __init__(self, pose: Pose, goals: list[Pose], robot_id: int):
        self.pose = pose
        self.id = robot_id

        # Visual dimensions (also used for SAT collision checks).
        self.width = ROBOT_WIDTH
        self.length = ROBOT_LENGTH

        self.hitbox_width = ROBOT_WIDTH * self.HITBOX_SCALE
        self.hitbox_length = ROBOT_LENGTH * self.HITBOX_SCALE

        self.goals = goals
        self.goals_index = 0
        self.temp_goal: Pose | None = None

        self.v = 0.0
        self.omega = 0.0

        self.last_goal_dist = 0.0
        self.stuck_time = 0.0

    # --- State queries -------------------------------------------------------

    @property
    def goal(self) -> Pose:
        return self.goals[self.goals_index]

    @property
    def target(self) -> Pose:
        """Active navigation target: temporary avoidance goal or current waypoint."""
        return self.temp_goal if self.temp_goal is not None else self.goal

    def snapshot(self) -> dict:
        """Serializable state for the renderer."""
        return {
            "id": self.id,
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
            "length": self.length,
            "width": self.width,
        }

    def robot_vertices(self) -> list[tuple[float, float]]:
        return rotated_rectangle_vertices(self.pose, self.length, self.width)

    def hitbox_vertices(self) -> list[tuple[float, float]]:
        return rotated_rectangle_vertices(self.pose, self.hitbox_length, self.hitbox_width)

    # --- Control -------------------------------------------------------------

    def rotate(self, target_heading: float) -> bool:
        """Rotate in place toward *target_heading*. Returns True when aligned."""
        error = wrap_angle(target_heading - self.pose.theta)
        self.v = 0.0
        self.omega = max(-MAX_OMEGA, min(5 * error, MAX_OMEGA))
        return abs(error) < ANGLE_TOLERANCE

    def drive(self, robots, shelves, walls) -> None:
        """Compute velocity commands toward the active target with repulsion."""
        self.v, self.omega = naive_drive_to_pose(
            self.pose,
            self.target,
            k_p_dist=5.0,
            k_p_heading=5.0,
            k_p_final=10.0,
        )

        apply_repulsion(
            self,
            robots,
            shelves,
            walls,
            MAX_OMEGA,
        )

    def update(self, dt: float, robots, shelves, walls) -> None:
        """Integrate one physics step."""
        self.drive(robots, shelves, walls)

        self.pose.x += self.v * np.cos(self.pose.theta) * dt
        self.pose.y += self.v * np.sin(self.pose.theta) * dt
        self.pose.theta += self.omega * dt

    def stop(self) -> None:
        self.v = 0.0
        self.omega = 0.0

    def reached_goal(self) -> bool:
        """True when position and heading are within tolerance of the current goal."""
        position_error = math.hypot(self.goal.x - self.pose.x, self.goal.y - self.pose.y)
        heading_error = abs(wrap_angle(self.goal.theta - self.pose.theta))
        return position_error < DIST_TOLERANCE and heading_error < ANGLE_TOLERANCE
