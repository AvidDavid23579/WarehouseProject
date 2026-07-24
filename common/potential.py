"""Artificial potential-field repulsion for walls and other robots."""

import math

import numpy as np

from config import ROBOT_LENGTH, X_MAX, X_MIN, Y_MAX, Y_MIN


def _inverse_square_repulsion(
    distance: float, margin: float, strength: float, max_force: float
) -> float:
    """Magnitude of a 1/d² repulsive force that vanishes beyond *margin*."""
    distance = max(distance, 1e-3)
    if distance >= margin:
        return 0.0
    magnitude = strength * (1 / distance - 1 / margin) / distance**2
    return min(magnitude, max_force)


def wall_repulsion(
    robot, margin: float = 0.3, strength: float = 1.0, max_force: float = 10.0
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
        magnitude = _inverse_square_repulsion(clearance, margin, strength, max_force)
        force += magnitude * normal

    return force


def robot_robot_repulsion(
    robot,
    robots,
    margin: float = 0.5,
    strength: float = 1.0,
    max_force: float = 10.0,
    goal_falloff: float = 0.6,
) -> np.ndarray:
    """Sum repulsive forces from nearby robots, scaled down near the goal."""
    force = np.zeros(2)
    half_extent = max(robot.length, robot.width) / 2.0

    dist_to_goal = math.hypot(
        robot.target.x - robot.pose.x, robot.target.y - robot.pose.y
    )
    # Weaken inter-robot repulsion when the robot is almost at its target.
    scale = min(1.0, dist_to_goal / goal_falloff)

    for other in robots:
        if other is robot:
            continue

        dx = robot.pose.x - other.pose.x
        dy = robot.pose.y - other.pose.y
        dist = math.hypot(dx, dy)

        other_half = max(other.length, other.width) / 2.0
        clearance = dist - half_extent - other_half
        magnitude = _inverse_square_repulsion(clearance, margin, strength, max_force)

        if magnitude > 0.0:
            direction = np.array([dx, dy]) / dist if dist > 1e-6 else np.array([1.0, 0.0])
            force += magnitude * scale * direction

    return force


def _dist_to_target(robot) -> float:
    return math.hypot(robot.target.x - robot.pose.x, robot.target.y - robot.pose.y)


def apply_repulsion(
    robot,
    robots,
    max_omega: float,
    wall_margin: float = 0.2,
    wall_strength: float = 2.0,
    robot_margin: float = ROBOT_LENGTH + 0.3,
    robot_strength: float = 2.0,
    goal_falloff: float = 0.6,
) -> None:
    """Blend wall and robot repulsion into the robot's current velocity command."""
    force = wall_repulsion(robot, wall_margin, wall_strength)
    force += robot_robot_repulsion(
        robot, robots, robot_margin, robot_strength, goal_falloff=goal_falloff
    )

    if not np.any(force):
        return

    heading = np.array([np.cos(robot.pose.theta), np.sin(robot.pose.theta)])

    # Project repulsion onto forward / lateral axes of the robot body.
    robot.v += float(np.dot(force, heading))
    robot.omega += float(np.cross(heading, force))

    # Escape local minima: nudge rotation when stuck away from the goal.
    if abs(robot.v) < 0.05 and _dist_to_target(robot) > 1e-3:
        robot.omega += 0.3

    robot.omega = max(-max_omega, min(robot.omega, max_omega))
