import numpy as np

from common.types import Pose


def world_to_screen(self, x, y, scene_rect, meters_width, meters_height):
    px, py, pw, ph = scene_rect

    sx = px + (x / meters_width) * pw

    # Flip y because pygame y increases downward
    sy = py + ph - (y / meters_height) * ph

    return int(sx), int(sy)


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


def rotated_rectangle_vertices(pose: Pose, length: float, width: float) -> list[tuple[float, float]]:
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
