import math

from common.utils import clamp, wrap_angle


def naive_drive_to_pose(current, target, k_p_dist, k_p_heading, k_p_final, max_v, max_omega):
    dx = target.x - current.x
    dy = target.y - current.y

    distance = math.hypot(dx, dy)

    target_heading = math.atan2(dy, dx)
    heading_error = wrap_angle(target_heading - current.theta)
    final_heading_error = wrap_angle(target.theta - current.theta)

    v = k_p_dist * distance + math.cos(heading_error)

    dist_tol = 0.05  # 5 cm
    angle_tol = math.radians(0.05)

    if distance < dist_tol:
        v = 0.0

        if abs(final_heading_error) < angle_tol:
            omega = 0.0
        else:
            omega = math.copysign(max_omega, final_heading_error)
    else:
        v = k_p_dist * distance * max(0.0, math.cos(heading_error))
        omega = k_p_heading * heading_error

    v = clamp(v, -max_v, max_v)
    omega = clamp(omega, -max_omega, max_omega)

    return v, omega
