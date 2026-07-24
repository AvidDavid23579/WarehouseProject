# Old, unused avoidance logic. Kept as artifacts of development

import math

from common.collision import sat_collision
from common.utils import wrap_angle
from entities.robot import Pose


def stop_prio_yield(robot, robots):
    for other in robots:
        if other is robot:
            continue
        if not sat_collision(robot.hitbox_vertices(), other.hitbox_vertices()):
            continue
        if robot.id < other.id:
            robot.stop()
            return


def turn_prio_yield(robot, robots):
    """Both robots turn perpendicular to their bearing to sidestep each other."""
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


def temp_goal_prio_yield(robot, robots, offset):
    avoiding = False

    for other in robots:
        if other is robot:
            continue
        if robot.id >= other.id:
            continue
        if not sat_collision(robot.hitbox_vertices(), other.hitbox_vertices()):
            continue

        avoiding = True

        if robot.temp_goal is not None:
            break

        dx = other.pose.x - robot.pose.x
        dy = other.pose.y - robot.pose.y
        dist = math.hypot(dx, dy)
        if dist < 1e-6:
            break

        dx /= dist
        dy /= dist

        perp_x, perp_y = -dy, dx

        avoid_x = other.pose.x + perp_x * offset
        avoid_y = other.pose.y + perp_y * offset

        robot.temp_goal = Pose(avoid_x, avoid_y, robot.goal.theta)
        break

    if not avoiding:
        robot.temp_goal = None
