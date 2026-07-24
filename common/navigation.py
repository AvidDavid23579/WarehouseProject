"""Low-level motion controllers that map pose error to velocity commands."""

import math

from common.utils import clamp, wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, MAX_VELOCITY


def naive_drive_to_pose(
    current,
    target,
    k_p_dist: float,
    k_p_heading: float,
    k_p_final: float,
) -> tuple[float, float]:
    """Proportional drive toward *target* with separate final heading alignment.

    Returns (linear_velocity, angular_velocity). When close to the target
    position, linear speed drops to zero and only heading is corrected.
    """
    dx = target.x - current.x
    dy = target.y - current.y
    distance = math.hypot(dx, dy)

    target_heading = math.atan2(dy, dx)
    heading_error = wrap_angle(target_heading - current.theta)
    final_heading_error = wrap_angle(target.theta - current.theta)

    if distance < DIST_TOLERANCE:
        # Position reached — rotate in place to match goal orientation.
        v = 0.0
        omega = 0.0 if abs(final_heading_error) < ANGLE_TOLERANCE else k_p_final * final_heading_error
    else:
        # Slow down when pointed away from the target (cosine gate).
        v = k_p_dist * distance * max(0.0, math.cos(heading_error))
        omega = k_p_heading * heading_error

    v = clamp(v, -MAX_VELOCITY, MAX_VELOCITY)
    omega = clamp(omega, -MAX_OMEGA, MAX_OMEGA)
    return v, omega
