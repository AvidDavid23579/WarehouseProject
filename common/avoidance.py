"""Discrete multi-robot avoidance strategies.

Only ``temp_goal_prio_yield`` is used in the live simulation loop. The other
functions are kept as reference implementations from earlier iterations.
"""

import math

from common.collision import sat_collision
from common.utils import Pose, wrap_angle


def stop_prio_yield(robot, robots) -> None:
    """Lower-ID robot stops when hitboxes overlap (unused)."""
    for other in robots:
        if other is robot:
            continue
        if not sat_collision(robot.hitbox_vertices(), other.hitbox_vertices()):
            continue
        if robot.id < other.id:
            robot.stop()
            return


def turn_prio_yield(robot, robots) -> None:
    """Both robots sidestep perpendicular to their mutual bearing (unused)."""
    for other in robots:
        if other is robot:
            continue
        if not sat_collision(robot.hitbox_vertices(), other.hitbox_vertices()):
            continue
        if robot.id < other.id:
            dx = other.pose.x - robot.pose.x
            dy = other.pose.y - robot.pose.y
            bearing = math.atan2(dy, dx)

            left = wrap_angle(bearing + math.pi / 2)
            right = wrap_angle(bearing - math.pi / 2)

            robot.rotate(left)
            other.rotate(right)
            return


def temp_goal_prio_yield(robot, robots, offset: float) -> None:
    """Inject a short-lived sidestep goal when a lower-priority robot detects overlap.

    Priority is determined by robot ID: the higher-ID robot keeps its path while
    the lower-ID robot steers to a temporary point beside the other robot.
    """
    avoiding = False

    for other in robots:
        if other is robot or robot.id >= other.id:
            continue
        if not sat_collision(robot.hitbox_vertices(), other.hitbox_vertices()):
            continue

        avoiding = True

        # Keep an existing temp goal until the overlap clears.
        if robot.temp_goal is not None:
            break

        dx = other.pose.x - robot.pose.x
        dy = other.pose.y - robot.pose.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            break

        # Unit vector from self toward the other robot, then 90° for sidestep.
        dx /= dist
        dy /= dist
        perp_x, perp_y = -dy, dx

        avoid_x = other.pose.x + perp_x * offset
        avoid_y = other.pose.y + perp_y * offset
        robot.temp_goal = Pose(avoid_x, avoid_y, robot.goal.theta)
        break

    if not avoiding:
        robot.temp_goal = None
