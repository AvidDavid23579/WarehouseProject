import numpy as np
from numpy.typing import NDArray

from common.utils import obb_aabb_distance, point_to_aabb, tangent
from config import MAX_OMEGA, MAX_VELOCITY, ROBOT_LENGTH, ROBOT_WIDTH, SHELF_LENGTH, SHELF_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN
from simulator.world import World

LEFT = (1.0, 0.0)
RIGHT = (-1.0, 0.0)
UP = (0.0, 1.0)
DOWN = (0.0, -1.0)


def dist_to_target(world: World) -> np.ndarray:
    mask = world.robot.target_node_id >= 0

    dist = np.zeros(world.robot.pose.shape[0], dtype=np.float32)
    target_xy = world.graph.node_pose[world.robot.target_node_id[mask]]

    dist[mask] = np.hypot(target_xy[:, 0] - world.robot.pose[mask, 0], target_xy[:, 1] - world.robot.pose[mask, 1])

    return dist


def inverse_square_repulsion(
    distance: np.ndarray,
    margin: float,
    strength: float,
    max_force: float,
) -> NDArray[np.float32]:

    force = np.zeros_like(distance)

    mask = distance < margin

    d = distance[mask].copy()
    np.maximum(d, 1e-3, out=d)

    force[mask] = strength * (1 / d - 1 / margin) / d**2
    np.minimum(force[mask], max_force, out=force[mask])

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


def obstacle_repulsion(
    world: World,
    obstacles,
    margin: float = 0.3,
    strength: float = 1.0,
    max_force: float = 20.0,
    tangent_gain: float = 0.25,
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:

    n = world.robot.pose.shape[0]

    fx = np.zeros(n, dtype=np.float32)
    fy = np.zeros(n, dtype=np.float32)

    theta = world.robot.pose[:, 2]
    heading = np.empty((n, 2), dtype=np.float32)
    heading[:, 0] = np.cos(theta)
    heading[:, 1] = np.sin(theta)

    for obstacle in obstacles:
        clearance, normal = obb_aabb_distance(
            world.robot.pose, ROBOT_LENGTH, ROBOT_WIDTH, obstacle.pose.x, obstacle.pose.y, SHELF_WIDTH, SHELF_LENGTH
        )

        clearance -= ROBOT_LENGTH / 2.0

        magnitude = inverse_square_repulsion(
            clearance,
            margin,
            strength,
            max_force,
        )

        active = magnitude > 0.0
        if not np.any(active):
            continue

        tg = tangent(normal, heading)

        fx[active] += magnitude[active] * (normal[active, 0] + tangent_gain * tg[active, 0])

        fy[active] += magnitude[active] * (normal[active, 1] + tangent_gain * tg[active, 1])

    return fx, fy


def apply_repulsion(world: World) -> None:
    fx, fy = boundary_repulsion(world)

    ofx, ofy = obstacle_repulsion(world, world.shelves, margin=0.5, strength=0.02, max_force=50.0, tangent_gain=0.55)

    fx += ofx
    fy += ofy

    # Robots experiencing any repulsion
    active = (fx != 0.0) | (fy != 0.0)

    if not np.any(active):
        return

    theta = world.robot.pose[:, 2]

    c = np.cos(theta)
    s = np.sin(theta)

    # dot(force, heading)
    world.robot.twist[active, 0] += fx[active] * c[active] + fy[active] * s[active]
    world.robot.twist[active, 0] = np.clip(world.robot.twist[active, 0], -MAX_VELOCITY, MAX_VELOCITY)

    # cross(heading, force)
    world.robot.twist[active, 1] += c[active] * fy[active] - s[active] * fx[active]
    world.robot.twist[active, 1] = np.clip(world.robot.twist[active, 1], -MAX_OMEGA, MAX_OMEGA)

    dist = dist_to_target(world)

    stuck = active & (np.abs(dist - world.robot.last_goal_dist) < 1e-3)
    world.robot.stuck_time[active & ~stuck] = 0.0

    world.robot.stuck_time[stuck] += 0.01
    world.robot.stuck_time[~stuck] = 0.0

    world.robot.last_goal_dist[:] = dist

    spinning = world.robot.stuck_time > 1.0
    world.robot.twist[spinning, 1] += 2.0
