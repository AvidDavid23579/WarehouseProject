import numpy as np


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
