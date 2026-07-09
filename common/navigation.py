import math

from common.utils import clamp, wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE


def naive_drive_to_pose(current, target, k_p_dist, k_p_heading, k_p_final, max_v, max_omega):
    dx = target.x - current.x
    dy = target.y - current.y

    distance = math.hypot(dx, dy)

    target_heading = math.atan2(dy, dx)
    heading_error = wrap_angle(target_heading - current.theta)
    final_heading_error = wrap_angle(target.theta - current.theta)

    if distance < DIST_TOLERANCE:
        v = 0.0

        if abs(final_heading_error) < ANGLE_TOLERANCE:
            omega = 0.0
        else:
            omega = k_p_final * final_heading_error
    else:
        v = k_p_dist * distance * max(0.0, math.cos(heading_error))
        omega = k_p_heading * heading_error

    v = clamp(v, -max_v, max_v)
    omega = clamp(omega, -max_omega, max_omega)

    return v, omega
