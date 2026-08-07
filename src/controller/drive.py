import numpy as np

from common.utils import wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, MAX_VELOCITY


def drive_to_pose(world) -> None:
    world.robot.arrived.fill(False)
    pose = world.robot.pose
    twist = world.robot.twist
    crashed = world.robot.crashed
    target_node = world.robot.target_node_id
    node_pose = world.graph.node_pose
    kP_velocity = 5.0
    kP_angular = 3.5
    kP_final = 10.0

    # Default: stop every robot.
    twist.fill(0.0)

    # Active robots will receive commands below.
    active = (~crashed) & (target_node != -1)

    if not np.any(active):
        return

    active_idx = np.flatnonzero(active)  # returns the indices where the mask is True
    node_idx = target_node[active]

    p = pose[active]
    g = node_pose[node_idx]

    px = p[:, 0]
    py = p[:, 1]
    theta = p[:, 2]

    gx = g[:, 0]
    gy = g[:, 1]
    goal_theta = g[:, 2]

    dx = gx - px
    dy = gy - py
    dist = np.hypot(dx, dy)

    arrived = dist < DIST_TOLERANCE
    moving = ~arrived

    # Moving
    moving_idx = active_idx[moving]

    if moving_idx.size:
        moving_idx = active_idx[moving]

        target_heading = np.arctan2(dy[moving], dx[moving])
        heading_error = wrap_angle(target_heading - theta[moving])

        twist[moving_idx, 0] = np.clip(
            kP_velocity * dist[moving] * np.maximum(0.0, np.cos(heading_error)),
            -MAX_VELOCITY,
            MAX_VELOCITY,
        )

        twist[moving_idx, 1] = np.clip(
            kP_angular * heading_error,
            -MAX_OMEGA,
            MAX_OMEGA,
        )

    # Arrived

    arrived_idx = active_idx[arrived]

    if arrived_idx.size:
        theta_a = theta[arrived]
        goal_theta_a = goal_theta[arrived]

        has_heading = ~np.isnan(goal_theta_a)

        advance = np.ones(len(arrived_idx), dtype=bool)

        if np.any(has_heading):
            heading_error = wrap_angle(goal_theta_a[has_heading] - theta_a[has_heading])

            rotate = np.abs(heading_error) >= ANGLE_TOLERANCE
            rotate_idx = arrived_idx[has_heading][rotate]
            twist[rotate_idx, 1] = np.clip(kP_final * heading_error[rotate], -MAX_OMEGA, MAX_OMEGA)
            advance[np.flatnonzero(has_heading)[rotate]] = False

        advance_idx = arrived_idx[advance]

        if advance_idx.size:
            world.robot.arrived[advance_idx] = True


def patrol(world):
    arrived = world.robot_arrived
    target = world.robot_target_node

    target[arrived] += 1
    target %= len(world.graph.node_pose)


def random_navigation(world):
    arrived = world.robot.arrived

    n = np.count_nonzero(arrived)

    if n == 0:
        return

    world.robot.target_node_id[arrived] = np.random.randint(
        0,
        len(world.graph.node_pose),
        size=n,
    )
