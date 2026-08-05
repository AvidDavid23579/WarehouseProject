import math
from collections import defaultdict

import numpy as np
from numpy.typing import NDArray

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
    points: NDArray[np.float32],  # (N,2)
) -> tuple[NDArray[np.float32], NDArray[np.float32]]:

    dx = points[:, 0] - pose.x
    dy = points[:, 1] - pose.y

    c = np.cos(pose.theta)
    s = np.sin(pose.theta)

    # Transform into rectangle frame
    local_x = dx * c + dy * s
    local_y = -dx * s + dy * c

    half_length = length * 0.5
    half_width = width * 0.5

    inside = (local_x >= -half_length) & (local_x <= half_length) & (local_y >= -half_width) & (local_y <= half_width)

    n = len(points)

    distance = np.empty(n, dtype=np.float32)

    local_normal = np.empty((n, 2), dtype=np.float32)

    # Outside points

    closest_x = np.clip(local_x, -half_length, half_length)
    closest_y = np.clip(local_y, -half_width, half_width)

    offset_x = local_x - closest_x
    offset_y = local_y - closest_y

    outside = ~inside

    distance[outside] = np.hypot(
        offset_x[outside],
        offset_y[outside],
    )

    nz = outside & (distance > 1e-9)

    local_normal[nz, 0] = offset_x[nz] / distance[nz]
    local_normal[nz, 1] = offset_y[nz] / distance[nz]

    # Degenerate corners

    deg = outside & ~nz

    x_major = np.abs(local_x) > half_length

    mask = deg & x_major
    local_normal[mask, 0] = np.sign(local_x[mask])
    local_normal[mask, 1] = 0.0

    mask = deg & ~x_major
    local_normal[mask, 0] = 0.0
    local_normal[mask, 1] = np.sign(local_y[mask])

    # Inside points

    if np.any(inside):
        face_dist = np.stack(
            (
                half_length - local_x,
                half_length + local_x,
                half_width - local_y,
                half_width + local_y,
            ),
            axis=1,
        )

        face = np.argmin(face_dist[inside], axis=1)

        distance[inside] = face_dist[inside, face]

        normals = np.array(
            (
                (1.0, 0.0),
                (-1.0, 0.0),
                (0.0, 1.0),
                (0.0, -1.0),
            ),
            dtype=np.float32,
        )

        local_normal[inside] = normals[face]

    #
    # Rotate normals back to world frame
    #

    normal = np.empty_like(local_normal)

    normal[:, 0] = local_normal[:, 0] * c - local_normal[:, 1] * s
    normal[:, 1] = local_normal[:, 0] * s + local_normal[:, 1] * c

    return distance, normal


def tangent(normal: NDArray[np.float32], heading: NDArray[np.float32]) -> NDArray[np.float32]:
    tg = np.empty_like(normal)
    tg[:, 0] = -normal[:, 1]
    tg[:, 1] = normal[:, 0]
    dot = np.sum(tg * heading, axis=1)
    tg[dot < 0] *= -1
    return tg


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
