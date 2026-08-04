import math

import numpy as np

from common.utils import tangent
from config import ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN
from simulator.world import World

LEFT = (1.0, 0.0)
RIGHT = (-1.0, 0.0)
UP = (0.0, 1.0)
DOWN = (0.0, -1.0)


def inverse_square_repulsion(
    distance: np.ndarray,
    margin: float,
    strength: float,
    max_force: float,
) -> float:

    force = np.zeros_like(distance)

    mask = distance < margin

    d = distance[mask].copy()
    np.maximum(d, 1e-3, out=d)

    force[mask] = strength * (1 / d - 1 / margin) / d**2
    np.minimum(force, max_force, out=force)

    return force


def boundary_repulsion(
    world: World,
    margin: float = 0.3,
    strength: float = 1.0,
    tangent_gain: float = 0.25,
    max_force: float = 20.0,
) -> tuple[np.ndarray, np.ndarray]:

    x = world.robot.pose[:, 0]
    y = world.robot.pose[:, 1]
    theta = world.robot.pose[:, 2]

    c = np.cos(theta)
    s = np.sin(theta)

    ac = np.abs(c)
    ass = np.abs(s)

    half_x = 0.5 * (ROBOT_LENGTH * ac + ROBOT_WIDTH * ass)
    half_y = 0.5 * (ROBOT_LENGTH * ass + ROBOT_WIDTH * ac)

    left = inverse_square_repulsion(x - X_MIN - half_x, margin, strength, max_force)
    right = inverse_square_repulsion(X_MAX - x - half_x, margin, strength, max_force)
    bottom = inverse_square_repulsion(y - Y_MIN - half_y, margin, strength, max_force)
    top = inverse_square_repulsion(Y_MAX - y - half_y, margin, strength, max_force)

    fx = left - right + tangent_gain * (left * -s + right * s)
    fy = bottom - top + tangent_gain * (bottom * c - top * c)

    return fx, fy


def dist_to_target(world: World) -> np.ndarray:
    mask = world.robot.target_node_id >= 0

    dist = np.zeros(world.robot.pose.shape[0], dtype=np.float32)
    target_xy = world.graph.node_pose[world.robot.target_node_id[mask]]

    dist[mask] = np.hypot(target_xy[:, 0] - world.robot.pose[mask, 0], target_xy[:, 1] - world.robot.pose[mask, 1])

    return dist


def apply_repulsion(world: World) -> None:
    fx, fy = boundary_repulsion(world)

    # Robots experiencing any repulsion
    active = (fx != 0.0) | (fy != 0.0)

    if not np.any(active):
        return

    theta = world.robot.pose[:, 2]

    c = np.cos(theta)
    s = np.sin(theta)

    # dot(force, heading)
    world.robot.twist[active, 0] += fx[active] * c[active] + fy[active] * s[active]

    # cross(heading, force)
    world.robot.twist[active, 1] += c[active] * fy[active] - s[active] * fx[active]

    dist = dist_to_target(world)

    stuck = active & (np.abs(dist - world.robot.last_goal_dist) < 1e-3)
    world.robot.stuck_time[active & ~stuck] = 0.0

    world.robot.stuck_time[stuck] += 0.01
    world.robot.stuck_time[~stuck] = 0.0

    world.robot.last_goal_dist[:] = dist

    spinning = world.robot.stuck_time > 1.0
    world.robot.twist[spinning, 1] += 2.0
