"""Shared data types and small math helpers used across the project."""

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class Pose:
    """Planar position and heading (radians, counter-clockwise from +x)."""

    x: float
    y: float
    theta: float


@dataclass(slots=True)
class Obstacle:
    pose: Pose
    length: float
    width: float
    scale: float = 1.0


def wrap_angle(angle: float) -> float:
    """Normalize an angle to the interval (-pi, pi]."""
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def clamp(value: float, min_val: float, max_val: float) -> float:
    """Clamp *value* to the closed interval [min_val, max_val]."""
    return max(min_val, min(value, max_val))


def rotated_rectangle_vertices(
    pose: Pose, length: float, width: float
) -> list[tuple[float, float]]:
    """Return world-frame corners of a rectangle centered at *pose*.

    The rectangle is defined in the body frame: length along the heading
    axis, width perpendicular to it.
    """
    half_length = length / 2.0
    half_width = width / 2.0
    local_corners = [
        (half_length, half_width),
        (half_length, -half_width),
        (-half_length, -half_width),
        (-half_length, half_width),
    ]

    cos_theta = np.cos(pose.theta)
    sin_theta = np.sin(pose.theta)

    return [
        (
            pose.x + lx * cos_theta - ly * sin_theta,
            pose.y + lx * sin_theta + ly * cos_theta,
        )
        for lx, ly in local_corners
    ]


def pose_from_segment(
    x1: float, y1: float, x2: float, y2: float
) -> tuple[Pose, float]:
    """Build center pose and segment length from two endpoints."""
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    theta = math.atan2(dy, dx) if length > 1e-9 else 0.0
    pose = Pose((x1 + x2) / 2.0, (y1 + y2) / 2.0, theta)
    return pose, length


def point_to_oriented_rectangle(
    pose: Pose,
    length: float,
    width: float,
    px: float,
    py: float,
) -> tuple[float, float, float]:
    """Shortest distance from a point to an oriented rectangle's boundary.

    Returns (distance, dir_x, dir_y) where the direction unit vector points
    from the closest surface point toward (px, py). Distance is zero when
    the query point lies inside the rectangle.
    """
    dx = px - pose.x
    dy = py - pose.y
    cos_theta = math.cos(pose.theta)
    sin_theta = math.sin(pose.theta)

    # Express the query point in the rectangle body frame.
    local_x = dx * cos_theta + dy * sin_theta
    local_y = -dx * sin_theta + dy * cos_theta

    half_length = length / 2.0
    half_width = width / 2.0

    if (
        -half_length <= local_x <= half_length
        and -half_width <= local_y <= half_width
    ):
        # Inside the rectangle — push toward the nearest face.
        face_distances = (
            (half_length - local_x, (1.0, 0.0)),
            (half_length + local_x, (-1.0, 0.0)),
            (half_width - local_y, (0.0, 1.0)),
            (half_width + local_y, (0.0, -1.0)),
        )
        distance, (local_nx, local_ny) = min(face_distances, key=lambda item: item[0])
    else:
        closest_x = clamp(local_x, -half_length, half_length)
        closest_y = clamp(local_y, -half_width, half_width)
        offset_x = local_x - closest_x
        offset_y = local_y - closest_y
        distance = math.hypot(offset_x, offset_y)

        if distance > 1e-9:
            local_nx = offset_x / distance
            local_ny = offset_y / distance
        else:
            # Corner degeneracy: use the axis with larger exterior offset.
            if abs(local_x) > half_length:
                local_nx = math.copysign(1.0, local_x)
                local_ny = 0.0
            else:
                local_nx = 0.0
                local_ny = math.copysign(1.0, local_y)

    dir_x = local_nx * cos_theta - local_ny * sin_theta
    dir_y = local_nx * sin_theta + local_ny * cos_theta
    return distance, dir_x, dir_y
