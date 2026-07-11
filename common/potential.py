# Models the map as a potential field for robots to move and navigate around

import math

import numpy as np

from config import ROBOT_LENGTH, X_MAX, X_MIN, Y_MAX, Y_MIN


def wall_repulsion(robot, margin=0.3, strength=1.0, max_force=10.0):
    half_extent = max(robot.length, robot.width) / 2.0
    force = np.zeros(2)

    walls = [
        (robot.pose.x - X_MIN - half_extent, np.array([1.0, 0.0])),
        (X_MAX - robot.pose.x - half_extent, np.array([-1.0, 0.0])),
        (robot.pose.y - Y_MIN - half_extent, np.array([0.0, 1.0])),
        (Y_MAX - robot.pose.y - half_extent, np.array([0.0, -1.0])),
    ]

    for d, normal in walls:
        d = max(d, 1e-3)
        if d < margin:
            mag = strength * (1 / d - 1 / margin) / d**2
            mag = min(mag, max_force)
            force += mag * normal

    return force


def robot_robot_repulsion(robot, robots, margin=0.5, strength=1.0, max_force=10.0, goal_falloff=0.6):
    force = np.zeros(2)
    half_extent = max(robot.length, robot.width) / 2.0

    dist_to_goal = math.hypot(robot.target.x - robot.pose.x, robot.target.y - robot.pose.y)
    scale = min(1.0, dist_to_goal / goal_falloff)

    for other in robots:
        if other is robot:
            continue

        dx = robot.pose.x - other.pose.x
        dy = robot.pose.y - other.pose.y
        dist = math.hypot(dx, dy)

        other_half_extent = max(other.length, other.width) / 2.0
        d = dist - half_extent - other_half_extent
        d = max(d, 1e-3)

        if d < margin:
            direction = np.array([dx, dy]) / dist if dist > 1e-6 else np.array([1.0, 0.0])
            mag = strength * (1 / d - 1 / margin) / d**2
            mag = min(mag, max_force) * scale
            force += mag * direction

    return force


def apply_repulsion(
    robot,
    robots,
    max_omega,
    wall_margin=0.2,
    wall_strength=2.0,
    robot_margin=ROBOT_LENGTH + 0.3,
    robot_strength=2.0,
    goal_falloff=0.6,
):
    force = wall_repulsion(robot, wall_margin, wall_strength)
    force += robot_robot_repulsion(robot, robots, robot_margin, robot_strength, goal_falloff=goal_falloff)

    if not np.any(force):
        return

    heading = np.array([np.cos(robot.pose.theta), np.sin(robot.pose.theta)])

    robot.v += float(np.dot(force, heading))
    robot.omega += float(np.cross(heading, force))

    # Local-minimum escape: if attraction/repulsion have cancelled to a
    # near-zero forward speed but the robot isn't at its goal, break the
    # symmetry with a small consistent rotational bias.
    if abs(robot.v) < 0.05 and dist_to_target(robot) > 1e-3:
        robot.omega += 0.3

    robot.omega = max(-max_omega, min(robot.omega, max_omega))


def dist_to_target(robot):
    return math.hypot(robot.target.x - robot.pose.x, robot.target.y - robot.pose.y)
