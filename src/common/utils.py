import math
from collections import defaultdict

import numpy as np

from common.types import Pose
from config import ROBOT_LENGTH, ROBOT_WIDTH

# Module constants
HL = ROBOT_LENGTH * 0.5
HW = ROBOT_WIDTH * 0.5

# Local robot coordinates
LX = np.array([-HL, HL, HL, -HL], dtype=np.float32)
LY = np.array([-HW, -HW, HW, HW], dtype=np.float32)


# Wraps angles to the interval [-pi, pi]
def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


# Clamps values to the interval [min, max]
def clamp(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(value, max_val))


def rotated_rectangle_vertices(
    pose: Pose, length: float, width: float
) -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
    tuple[float, float],
]:
    hl = length * 0.5
    hw = width * 0.5

    c = math.cos(pose.theta)
    s = math.sin(pose.theta)

    x = pose.x
    y = pose.y

    return (
        (x + hl * c - hw * s, y + hl * s + hw * c),
        (x + hl * c + hw * s, y + hl * s - hw * c),
        (x - hl * c + hw * s, y - hl * s - hw * c),
        (x - hl * c - hw * s, y - hl * s + hw * c),
    )


def point_to_oriented_rectangle(
    pose: Pose,
    length: float,
    width: float,
    px: float,
    py: float,
) -> tuple[float, float, float]:

    dx = px - pose.x
    dy = py - pose.y
    cos_theta = math.cos(pose.theta)
    sin_theta = math.sin(pose.theta)

    # Express the query point in the rectangle body frame.
    local_x = dx * cos_theta + dy * sin_theta
    local_y = -dx * sin_theta + dy * cos_theta

    half_length = length / 2.0
    half_width = width / 2.0

    if -half_length <= local_x <= half_length and -half_width <= local_y <= half_width:
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


def tangent(
    normal: tuple[float, float],
    heading: tuple[float, float],
) -> tuple[float, float]:
    """Choose the tangent closest to the robot's current heading."""

    nx, ny = normal
    hx, hy = heading

    # First tangent
    t1x = -ny
    t1y = nx

    # dot(t1, heading)
    if t1x * hx + t1y * hy >= 0.0:
        return t1x, t1y

    # Opposite tangent
    return -t1x, -t1y


def update_robot_vertices(world):
    pose = world.robot.pose
    vertices = world.robot.vertices

    # Extract columns
    x = pose[:, 0]
    y = pose[:, 1]
    theta = pose[:, 2]

    # Typedef
    c = np.cos(theta)
    s = np.sin(theta)

    # Create views once
    x = x[:, None]
    y = y[:, None]
    c = c[:, None]
    s = s[:, None]

    vx = vertices[:, :, 0]
    vy = vertices[:, :, 1]

    # Write directly into destination arrays
    np.multiply(c, LX, out=vx)
    vx -= s * LY
    vx += x

    np.multiply(s, LX, out=vy)
    vy += c * LY
    vy += y


class SpatialHash:
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.cells = defaultdict(list)

    def clear(self):
        self.cells.clear()

    def _cell(self, x: float, y: float):
        return (
            int(x // self.cell_size),
            int(y // self.cell_size),
        )

    def insert(self, obj):
        self.cells[self._cell(obj.pose.x, obj.pose.y)].append(obj)

    def nearby(self, obj):
        cx, cy = self._cell(obj.pose.x, obj.pose.y)

        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                yield from self.cells.get((cx + dx, cy + dy), [])
