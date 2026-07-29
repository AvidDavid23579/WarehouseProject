import math

import numpy as np

from common.utils import clamp, wrap_angle
from config import (
    ANGLE_TOLERANCE,
    DIST_TOLERANCE,
    MAX_OMEGA,
    MAX_VELOCITY,
)


def drive_to_pose(world) -> None:

    pose = world.robot_pose
    velocity = world.robot_velocity
    crashed = world.robot_crashed
    target_node = world.robot_target_node
    node_pose = world.graph.node_pose

    for i in range(len(pose)):
        if crashed[i]:
            continue

        node_id = target_node[i]

        if node_id == -1:
            velocity[i] = 0.0
            continue

        px, py, theta = pose[i]
        gx, gy, goal_theta = node_pose[node_id]

        dx = gx - px
        dy = gy - py
        dist = math.hypot(dx, dy)

        if dist < DIST_TOLERANCE:
            velocity[i, 0] = 0.0

            # Rotate to the node heading first (if required)
            if not np.isnan(goal_theta):
                heading_error = wrap_angle(goal_theta - theta)

                if abs(heading_error) >= ANGLE_TOLERANCE:
                    velocity[i, 1] = clamp(
                        10.0 * heading_error,
                        -MAX_OMEGA,
                        MAX_OMEGA,
                    )
                    continue

            # Heading satisfied -> advance to next node
            target_node[i] = (node_id + 1) % len(node_pose)

            velocity[i, 0] = 0.0
            velocity[i, 1] = 0.0

            continue

        target_heading = math.atan2(dy, dx)
        heading_error = wrap_angle(target_heading - theta)

        velocity[i, 1] = 7.5 * heading_error
        velocity[i, 0] = 5.0 * dist * max(0.0, math.cos(heading_error))

        # apply_repulsion(world, i)

        velocity[i, 0] = clamp(
            velocity[i, 0],
            -MAX_VELOCITY,
            MAX_VELOCITY,
        )

        velocity[i, 1] = clamp(
            velocity[i, 1],
            -MAX_OMEGA,
            MAX_OMEGA,
        )
