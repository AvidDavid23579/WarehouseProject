import numpy as np

from common.types import NavPhase
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, MAX_VELOCITY
from geometry.geo_compute import wrap_angle
from navigation.graph import NavigationGraph


def drive_to_pose(world, heading_override=None, kP_velocity=4.0, kP_angular=3.0, kP_final=10.0) -> None:
    world.robot.arrived.fill(False)
    pose = world.robot.pose
    twist = world.robot.twist
    crashed = world.robot.crashed
    target_node = world.robot.target_node_id
    node_pose = world.graph.node_pose

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

    if heading_override is not None:
        goal_theta = heading_override[active]

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


def drive_to_pose_grid(
    world,
    graph: NavigationGraph,
    kP_vel=5.0,
    kP_omega=10.0,
) -> None:
    robot = world.robot

    robot.twist.fill(0.0)
    robot.arrived.fill(False)
    robot.target_node_id[:] = -1

    pose = robot.pose
    crashed = robot.crashed

    for r in range(len(pose)):
        if crashed[r]:
            continue

        path = robot.path[r]
        i = robot.path_index[r]
        phase = robot.nav_phase[r]

        # -----------------------------------------------------
        # PATH COMPLETE
        # -----------------------------------------------------

        if i >= len(path):
            robot.nav_phase[r] = NavPhase.DONE
            continue

        target_node = path[i]
        robot.target_node_id[r] = target_node
        target_pose = graph.node_pose[target_node]

        # =====================================================
        # INITIAL TURN
        #
        # Robot starts at path[0].
        # Turn toward path[0] -> path[1] before moving.
        # =====================================================

        if phase == NavPhase.INITIAL_TURN:
            if len(path) < 2:
                # Nothing to drive toward.
                robot.nav_phase[r] = NavPhase.DONE
                robot.arrived[r] = True
                continue

            next_node = path[1]
            next_pose = graph.node_pose[next_node]

            edge_heading = np.arctan2(
                next_pose[1] - target_pose[1],
                next_pose[0] - target_pose[0],
            )

            heading_error = wrap_angle(edge_heading - pose[r, 2])

            if abs(heading_error) >= ANGLE_TOLERANCE:
                robot.twist[r, 1] = np.clip(
                    kP_omega * heading_error,
                    -MAX_OMEGA,
                    MAX_OMEGA,
                )
                continue

            # Initial heading reached.
            robot.nav_phase[r] = NavPhase.DRIVE

            # Do not drive this frame.
            continue

        # =====================================================
        # DRIVE
        # =====================================================

        if phase == NavPhase.DRIVE:
            dx = target_pose[0] - pose[r, 0]
            dy = target_pose[1] - pose[r, 1]

            dist = np.hypot(dx, dy)

            if dist >= DIST_TOLERANCE:
                target_heading = np.arctan2(dy, dx)

                heading_error = wrap_angle(target_heading - pose[r, 2])

                robot.twist[r, 0] = np.clip(
                    kP_vel * dist,
                    0.0,
                    MAX_VELOCITY,
                )

                robot.twist[r, 1] = np.clip(
                    kP_omega * heading_error,
                    -MAX_OMEGA,
                    MAX_OMEGA,
                )

                continue

            # Reached node.
            robot.twist[r, 0] = 0.0

            # Final node.
            if i == len(path) - 1:
                goal_heading = target_pose[2]

                heading_error = wrap_angle(goal_heading - pose[r, 2])

                if abs(heading_error) >= ANGLE_TOLERANCE:
                    robot.nav_phase[r] = NavPhase.TURN

                    robot.twist[r, 1] = np.clip(
                        kP_omega * heading_error,
                        -MAX_OMEGA,
                        MAX_OMEGA,
                    )

                    continue

                robot.arrived[r] = True
                robot.nav_phase[r] = NavPhase.DONE
                continue

            # Normal node reached.
            robot.nav_phase[r] = NavPhase.TURN

        # =====================================================
        # TURN
        #
        # Robot is sitting at path[i] and needs to turn toward
        # path[i] -> path[i + 1].
        # =====================================================

        if i >= len(path) - 1:
            # Goal reached
            robot.nav_phase[r] = NavPhase.DONE
            return

        if robot.nav_phase[r] == NavPhase.TURN:
            next_node = path[i + 1]
            next_pose = graph.node_pose[next_node]

            edge_heading = np.arctan2(
                next_pose[1] - target_pose[1],
                next_pose[0] - target_pose[0],
            )

            heading_error = wrap_angle(edge_heading - pose[r, 2])

            if abs(heading_error) >= ANGLE_TOLERANCE:
                robot.twist[r, 1] = np.clip(
                    kP_omega * heading_error,
                    -MAX_OMEGA,
                    MAX_OMEGA,
                )
                continue

            # Heading aligned.
            robot.path_index[r] += 1

            if robot.path_index[r] >= len(path):
                robot.nav_phase[r] = NavPhase.DONE
                robot.arrived[r] = True
            else:
                robot.nav_phase[r] = NavPhase.DRIVE
