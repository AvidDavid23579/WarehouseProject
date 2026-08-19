import numpy as np
from numpy.typing import NDArray

from config import MAX_OMEGA, MAX_VELOCITY, ROBOT_LENGTH, ROBOT_WIDTH, SHELF_LENGTH, SHELF_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN
from geometry.geo_compute import obb_aabb_distance, tangent
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
    margin: float = 0.3,
    strength: float = 1.0,
    max_force: float = 20.0,
    tangent_gain: float = 0.25,
) -> tuple[np.ndarray, np.ndarray]:

    robot_pose = world.robot.pose
    shelf_pose = world.shelf.pose

    n = robot_pose.shape[0]

    fx = np.zeros(n, dtype=np.float32)
    fy = np.zeros(n, dtype=np.float32)

    if shelf_pose.shape[0] == 0:
        return fx, fy

    theta = robot_pose[:, 2]

    heading = np.column_stack(
        (
            np.cos(theta),
            np.sin(theta),
        )
    ).astype(np.float32)

    robot_x = robot_pose[:, 0]
    robot_y = robot_pose[:, 1]

    shelf_x = shelf_pose[:, 0]
    shelf_y = shelf_pose[:, 1]

    half_x = SHELF_WIDTH * 0.5 + ROBOT_LENGTH * 0.5 + margin
    half_y = SHELF_LENGTH * 0.5 + ROBOT_LENGTH * 0.5 + margin

    # Broad phase
    dx = robot_x[:, None] - shelf_x[None, :]
    dy = robot_y[:, None] - shelf_y[None, :]

    active = (np.abs(dx) <= half_x) & (np.abs(dy) <= half_y)

    robot_idx, shelf_idx = np.nonzero(active)

    if robot_idx.size == 0:
        return fx, fy

    # Narrow phase
    clearance, normal = obb_aabb_distance(
        robot_pose,
        ROBOT_LENGTH,
        ROBOT_WIDTH,
        shelf_pose[:, 0],
        shelf_pose[:, 1],
        SHELF_WIDTH,
        SHELF_LENGTH,
    )

    clearance = clearance[robot_idx, shelf_idx] - ROBOT_LENGTH * 0.5
    normal = normal[robot_idx, shelf_idx]

    magnitude = inverse_square_repulsion(
        clearance,
        margin,
        strength,
        max_force,
    )

    valid = magnitude > 0.0

    if not np.any(valid):
        return fx, fy

    robot_idx = robot_idx[valid]
    magnitude = magnitude[valid]
    normal = normal[valid]

    h = heading[robot_idx]

    tg = tangent(normal, h)

    force = magnitude[:, None] * (normal + tangent_gain * tg)

    np.add.at(fx, robot_idx, force[:, 0])
    np.add.at(fy, robot_idx, force[:, 1])

    return fx, fy


def robot_repulsion(
    world: World,
    margin: float = 0.01,
    strength: float = 0.001,
    max_force: float = 1.0,
    tangent_gain: float = 0.0,
) -> tuple[np.ndarray, np.ndarray]:

    pose = world.robot.pose

    n = pose.shape[0]

    fx = np.zeros(n, dtype=np.float32)
    fy = np.zeros(n, dtype=np.float32)

    if n < 2:
        return fx, fy

    x = pose[:, 0]
    y = pose[:, 1]

    radius = np.float32(0.5 * np.hypot(ROBOT_LENGTH, ROBOT_WIDTH))

    interaction_distance = 2.0 * radius + margin

    dx = x[:, None] - x[None, :]
    dy = y[:, None] - y[None, :]

    dist_sq = dx * dx + dy * dy

    # Ignore self.
    np.fill_diagonal(dist_sq, np.inf)

    active = dist_sq < interaction_distance**2

    robot_i, robot_j = np.nonzero(active)

    if robot_i.size == 0:
        return fx, fy

    dx = dx[robot_i, robot_j]
    dy = dy[robot_i, robot_j]

    distance = np.sqrt(dx * dx + dy * dy).astype(np.float32)

    # Numerical safety.
    safe_distance = np.maximum(
        distance,
        np.float32(1e-3),
    )

    magnitude = inverse_square_repulsion(
        distance - 2.0 * radius,
        margin,
        strength,
        max_force,
    )

    active = magnitude > 0.0

    if not np.any(active):
        return fx, fy

    robot_i = robot_i[active]
    robot_j = robot_j[active]

    dx = dx[active]
    dy = dy[active]

    safe_distance = safe_distance[active]
    magnitude = magnitude[active]

    # Normal points away from robot j.
    nx = dx / safe_distance
    ny = dy / safe_distance

    theta = pose[robot_i, 2]

    heading_x = np.cos(theta)
    heading_y = np.sin(theta)

    # Tangent perpendicular to robot heading.
    tg_x = -heading_y
    tg_y = heading_x

    # Choose tangent direction according to which side
    # of the robot the other robot occupies.
    side = heading_x * dy - heading_y * dx

    sign = np.where(
        side >= 0.0,
        1.0,
        -1.0,
    ).astype(np.float32)

    tg_x *= sign
    tg_y *= sign

    force_x = magnitude * (nx + tangent_gain * tg_x)

    force_y = magnitude * (ny + tangent_gain * tg_y)

    np.add.at(fx, robot_i, force_x)
    np.add.at(fy, robot_i, force_y)

    return fx, fy

def right_of_way_avoidance(
    world: World,
    id: bool = True,
    broad_margin: float = 0.5,
    pf_margin: float = 0.3,
    pf_strength: float = 0.01,
    pf_tangent_gain: float = 0.05,
    pf_max_force: float = 5.0,
    stop_distance: float = 1.5,
) -> None:
    """
    Right-of-way + potential-field multi-robot avoidance.

    For each potentially colliding robot pair:
        - Higher-ID robot stops.
        - Lower-ID robot receives potential-field repulsion.

    The right-of-way rule is only activated when robots are within
    the broad-phase interaction distance.

    Assumes robot.twist[:, 0] is linear velocity and
    robot.twist[:, 1] is angular velocity.
    """

    robot = world.robot
    pose = robot.pose
    n = pose.shape[0]

    if n < 2:
        return

    x = pose[:, 0]
    y = pose[:, 1]

    # ------------------------------------------------------------------
    # Broad phase
    # ------------------------------------------------------------------

    dx = x[:, None] - x[None, :]
    dy = y[:, None] - y[None, :]

    dist_sq = dx * dx + dy * dy
    np.fill_diagonal(dist_sq, np.inf)

    # Robot bounding-circle radius.
    radius = np.float32(
        0.5 * np.hypot(ROBOT_LENGTH, ROBOT_WIDTH)
    )

    interaction_distance = (
        2.0 * radius
        + broad_margin
        + stop_distance
    )

    active = dist_sq < interaction_distance**2

    i, j = np.nonzero(active)

    if i.size == 0:
        return

    # ------------------------------------------------------------------
    # Only consider each pair once.
    #
    # i < j means:
    #     i = lower ID
    #     j = higher ID
    # ------------------------------------------------------------------

    if id:
        pair_mask = i < j
    else:
        pair_mask = j < i

    i = i[pair_mask]
    j = j[pair_mask]

    if i.size == 0:
        return

    # ------------------------------------------------------------------
    # Relative motion
    #
    # Only invoke right-of-way if the robots are actually approaching
    # each other. Robots moving apart should not trigger avoidance.
    # ------------------------------------------------------------------

    theta = pose[:, 2]

    vx = robot.twist[:, 0] * np.cos(theta)
    vy = robot.twist[:, 0] * np.sin(theta)

    rel_x = dx[i, j]
    rel_y = dy[i, j]

    rel_vx = vx[i] - vx[j]
    rel_vy = vy[i] - vy[j]

    approaching = (
        rel_x * rel_vx +
        rel_y * rel_vy
    ) < 0.0

    i = i[approaching]
    j = j[approaching]

    if i.size == 0:
        return

    # ------------------------------------------------------------------
    # Higher-ID robot yields.
    #
    # Since i < j:
    #     i -> lower ID -> continues
    #     j -> higher ID -> stops
    # ------------------------------------------------------------------

    robot.twist[j, 0] = 0
    robot.twist[j, 1] = 0

    # ------------------------------------------------------------------
    # Potential field on lower-ID robot.
    # ------------------------------------------------------------------

    dx = x[i] - x[j]
    dy = y[i] - y[j]

    distance = np.sqrt(
        dx * dx + dy * dy
    ).astype(np.float32)

    safe_distance = np.maximum(
        distance,
        np.float32(1e-3),
    )

    clearance = distance - 2.0 * radius

    magnitude = inverse_square_repulsion(
        clearance,
        pf_margin,
        pf_strength,
        pf_max_force,
    )

    valid = magnitude > 0.0

    if not np.any(valid):
        return

    i = i[valid]

    dx = dx[valid]
    dy = dy[valid]
    safe_distance = safe_distance[valid]
    magnitude = magnitude[valid]

    # Normal pointing away from the higher-ID robot.
    nx = dx / safe_distance
    ny = dy / safe_distance

    # ------------------------------------------------------------------
    # Tangential component.
    #
    # Choose the side based on the lower-ID robot's heading so it
    # naturally bends around the yielding robot instead of simply
    # pushing directly away.
    # ------------------------------------------------------------------

    theta = pose[i, 2]

    heading_x = np.cos(theta)
    heading_y = np.sin(theta)

    tangent_x = -heading_y
    tangent_y = heading_x

    side = (
        heading_x * (-dy)
        - heading_y * (-dx)
    )

    sign = np.where(
        side >= 0.0,
        1.0,
        -1.0,
    ).astype(np.float32)

    tangent_x *= sign
    tangent_y *= sign

    force_x = magnitude * (
        nx + pf_tangent_gain * tangent_x
    )

    force_y = magnitude * (
        ny + pf_tangent_gain * tangent_y
    )

    # ------------------------------------------------------------------
    # Convert world-space force into robot velocity.
    # ------------------------------------------------------------------

    robot.twist[i, 0] += (
        force_x * heading_x
        + force_y * heading_y
    )

    robot.twist[i, 1] += (
        heading_x * force_y
        - heading_y * force_x
    )

    # Respect controller limits.
    robot.twist[:, 0] = np.clip(
        robot.twist[:, 0],
        -MAX_VELOCITY,
        MAX_VELOCITY,
    )

    robot.twist[:, 1] = np.clip(
        robot.twist[:, 1],
        -MAX_OMEGA,
        MAX_OMEGA,
    )


def apply_repulsion(
    world: World,
    obstacle_margin: float = 0.3,
    boundary_margin: float = 0.3,
    robot_margin: float = 0.1,
) -> None:
    robot = world.robot

    robot.repulsion_force.fill(0.0)

    # Static obstacles.
    obstacle_fx, obstacle_fy = 0,0

    # Warehouse boundaries.
    boundary_fx, boundary_fy = boundary_repulsion(
        world,
        margin=boundary_margin,
    )

    robot_fx, robot_fy = robot_repulsion(
        world
    )

    robot.repulsion_force[:, 0] += (
        obstacle_fx + boundary_fx + robot_fx
    )
    robot.repulsion_force[:, 1] += (
        obstacle_fy + boundary_fy + robot_fy
    )

    theta = robot.pose[:, 2]
    c = np.cos(theta)
    s = np.sin(theta)

    fx = robot.repulsion_force[:, 0]
    fy = robot.repulsion_force[:, 1]

    robot.twist[:, 0] += fx * c + fy * s
    robot.twist[:, 1] += c * fy - s * fx

    robot.twist[:, 0] = np.clip(
        robot.twist[:, 0],
        -MAX_VELOCITY,
        MAX_VELOCITY,
    )

    robot.twist[:, 1] = np.clip(
        robot.twist[:, 1],
        -MAX_OMEGA,
        MAX_OMEGA,
    )

    right_of_way_avoidance(world, broad_margin=robot_margin, pf_margin=robot_margin)
