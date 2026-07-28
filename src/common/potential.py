import math

import numpy as np

from common.utils import _tangent, point_to_oriented_rectangle
from config import MAX_OMEGA, ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN


def _inverse_square_repulsion(
    distance: float,
    margin: float,
    strength: float,
    max_force: float,
) -> float:
    """Magnitude of a 1/d² repulsive force that vanishes beyond *margin*."""
    distance = max(distance, 1e-3)

    if distance >= margin:
        return 0.0

    magnitude = strength * (1 / distance - 1 / margin) / distance**2
    return min(magnitude, max_force)


def boundary_repulsion(
    robot,
    margin: float = 0.3,
    strength: float = 1.0,
    max_force: float = 20.0,
) -> np.ndarray:
    """Repulsive force from the world boundaries."""
    force = np.zeros(2)

    c = abs(np.cos(robot.state.pose.theta))
    s = abs(np.sin(robot.state.pose.theta))

    half_x = 0.5 * (ROBOT_LENGTH * c + ROBOT_WIDTH * s)
    half_y = 0.5 * (ROBOT_LENGTH * s + ROBOT_WIDTH * c)

    boundaries = [
        (robot.state.pose.x - X_MIN - half_x, np.array([1.0, 0.0])),  # Left
        (X_MAX - robot.state.pose.x - half_x, np.array([-1.0, 0.0])),  # Right
        (robot.state.pose.y - Y_MIN - half_y, np.array([0.0, 1.0])),  # Bottom
        (Y_MAX - robot.state.pose.y - half_y, np.array([0.0, -1.0])),  # Top
    ]

    for clearance, normal in boundaries:
        magnitude = _inverse_square_repulsion(
            clearance,
            margin,
            strength,
            max_force,
        )
        force += magnitude * normal

    return force


def _dist_to_target(robot) -> float:
    return math.hypot(
        robot.goal.x - robot.state.pose.x,
        robot.goal.y - robot.state.pose.y,
    )


def apply_repulsion(robot, boundary_margin: float = 0.15, boundary_strength: float = 0.8) -> None:
    """Blend boundary, wall, robot, and shelf repulsion into the velocity command."""
    force = boundary_repulsion(
        robot,
        boundary_margin,
        boundary_strength,
    )

    if not np.any(force):
        return

    heading = np.array(
        [
            np.cos(robot.state.pose.theta),
            np.sin(robot.state.pose.theta),
        ]
    )

    robot.state.v += float(np.dot(force, heading))
    robot.state.omega += float(np.cross(heading, force))

    dist = _dist_to_target(robot)

    if abs(dist - robot.state.last_goal_dist) < 1e-3:
        robot.state.stuck_time += 0.01
    else:
        robot.state.stuck_time = 0.0

    robot.state.last_goal_dist = dist

    if robot.state.stuck_time > 1.0:
        robot.omega += 2.0

    robot.omega = np.clip(robot.state.omega, -MAX_OMEGA, MAX_OMEGA)
