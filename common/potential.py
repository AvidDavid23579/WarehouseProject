"""Artificial potential-field repulsion for walls and obstacles."""

import math

import numpy as np

from config import ROBOT_LENGTH, X_MAX, X_MIN, Y_MAX, Y_MIN


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


def wall_repulsion(
    robot,
    margin: float = 0.3,
    strength: float = 1.0,
    max_force: float = 10.0,
) -> np.ndarray:
    """Sum repulsive forces pushing the robot away from each world boundary."""
    half_extent = max(robot.length, robot.width) / 2.0
    force = np.zeros(2)

    walls = [
        (robot.pose.x - X_MIN - half_extent, np.array([1.0, 0.0])),
        (X_MAX - robot.pose.x - half_extent, np.array([-1.0, 0.0])),
        (robot.pose.y - Y_MIN - half_extent, np.array([0.0, 1.0])),
        (Y_MAX - robot.pose.y - half_extent, np.array([0.0, -1.0])),
    ]

    for clearance, normal in walls:
        magnitude = _inverse_square_repulsion(
            clearance,
            margin,
            strength,
            max_force,
        )
        force += magnitude * normal

    return force


def obstacle_repulsion(
    robot,
    obstacles,
    margin: float,
    strength: float,
    max_force: float,
    scale=lambda _: 1.0,
) -> np.ndarray:
    """Repulsive force from arbitrary obstacles."""

    force = np.zeros(2)
    robot_half = max(robot.length, robot.width) / 2.0

    for obstacle in obstacles:
        obstacle_scale = scale(obstacle)

        if obstacle_scale <= 0.0:
            continue

        dx = robot.pose.x - obstacle.pose.x
        dy = robot.pose.y - obstacle.pose.y
        dist = math.hypot(dx, dy)

        obstacle_half = max(obstacle.length, obstacle.width) / 2.0
        clearance = dist - robot_half - obstacle_half

        magnitude = _inverse_square_repulsion(
            clearance,
            margin,
            strength,
            max_force,
        )

        if magnitude == 0.0:
            continue

        if dist > 1e-6:
            direction = np.array([dx, dy]) / dist
        else:
            direction = np.array([1.0, 0.0])

        force += obstacle_scale * magnitude * direction

    return force


def _dist_to_target(robot) -> float:
    return math.hypot(
        robot.target.x - robot.pose.x,
        robot.target.y - robot.pose.y,
    )


def apply_repulsion(
    robot,
    robots,
    shelves,
    max_omega: float,
    wall_margin: float = 0.2,
    wall_strength: float = 2.0,
    robot_margin: float = ROBOT_LENGTH + 0.3,
    robot_strength: float = 2.0,
    shelf_margin: float = 0.2,
    shelf_strength: float = 2.0,
    goal_falloff: float = 0.6,
) -> None:
    """Blend wall, robot, and shelf repulsion into the velocity command."""

    force = wall_repulsion(
        robot,
        wall_margin,
        wall_strength,
    )

    robot_scale = min(
        1.0,
        _dist_to_target(robot) / goal_falloff,
    )

    force += obstacle_repulsion(
        robot,
        robots,
        margin=robot_margin,
        strength=robot_strength,
        max_force=10.0,
        scale=lambda other: 0.0 if other is robot else robot_scale,
    )

    force += obstacle_repulsion(
        robot,
        shelves,
        margin=shelf_margin,
        strength=shelf_strength,
        max_force=10.0,
    )

    if not np.any(force):
        return

    heading = np.array(
        [
            np.cos(robot.pose.theta),
            np.sin(robot.pose.theta),
        ]
    )

    # Project repulsion into the robot's body frame.
    robot.v += float(np.dot(force, heading))
    robot.omega += float(np.cross(heading, force))

    # Escape local minima.
    if abs(robot.v) < 0.05 and _dist_to_target(robot) > 1e-3:
        robot.omega += 0.3

    robot.omega = np.clip(robot.omega, -max_omega, max_omega)
