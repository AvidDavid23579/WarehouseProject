import math

import numpy as np

from common.utils import _tangent, point_to_oriented_rectangle
from config import MAX_OMEGA, ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN

LEFT = (1.0, 0.0)
RIGHT = (-1.0, 0.0)
UP = (0.0, 1.0)
DOWN = (0.0, -1.0)


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
    tangent_gain: float = 0.25,
    max_force: float = 20.0,
) -> tuple[float, float]:
    pose = robot.state.pose

    c = math.cos(pose.theta)
    s = math.sin(pose.theta)

    heading = (c, s)

    ac = abs(c)
    ass = abs(s)

    half_x = 0.5 * (ROBOT_LENGTH * ac + ROBOT_WIDTH * ass)
    half_y = 0.5 * (ROBOT_LENGTH * ass + ROBOT_WIDTH * ac)

    fx = 0.0
    fy = 0.0

    boundaries = (
        (pose.x - X_MIN - half_x, LEFT),
        (X_MAX - pose.x - half_x, RIGHT),
        (pose.y - Y_MIN - half_y, UP),
        (Y_MAX - pose.y - half_y, DOWN),
    )

    for clearance, normal in boundaries:
        magnitude = _inverse_square_repulsion(
            clearance,
            margin,
            strength,
            max_force,
        )

        if magnitude == 0.0:
            continue

        tx, ty = _tangent(normal, heading)

        fx += magnitude * (normal[0] + tangent_gain * tx)
        fy += magnitude * (normal[1] + tangent_gain * ty)

    return fx, fy


def _dist_to_target(robot) -> float:
    return math.hypot(
        robot.goal.x - robot.state.pose.x,
        robot.goal.y - robot.state.pose.y,
    )


def apply_repulsion(robot) -> None:
    fx, fy = boundary_repulsion(robot)

    if fx == 0.0 and fy == 0.0:
        return

    theta = robot.state.pose.theta
    c = math.cos(theta)
    s = math.sin(theta)

    # dot(force, heading)
    robot.state.v += fx * c + fy * s

    # cross(heading, force)
    robot.state.omega += c * fy - s * fx

    dist = _dist_to_target(robot)

    if abs(dist - robot.state.last_goal_dist) < 1e-3:
        robot.state.stuck_time += 0.01
    else:
        robot.state.stuck_time = 0.0

    robot.state.last_goal_dist = dist

    if robot.state.stuck_time > 1.0:
        robot.state.omega += 2.0
