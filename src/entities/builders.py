import math

import numpy as np

from config import (
    DOCK_APPROACH_DISTANCE,
    DOCK_LENGTH,
    DOCK_ONE_POSE,
    DOCK_SPACING,
    DOCK_WIDTH,
    NUM_DOCKS,
    NUM_PALLETS_PER_SHELF,
    PALLET_LENGTH,
    PALLET_WIDTH,
    ROBOT_DOCK_DIST,
    ROBOT_LENGTH,
    ROBOT_WIDTH,
    SHELF_COL,
    SHELF_LENGTH,
    SHELF_ROW,
    SHELF_WIDTH,
    X_MAX,
    Y_MAX,
)
from entities.dock import DockState
from entities.pallet import PalletState
from entities.robot import RobotState
from entities.shelf import ShelfState
from geometry.geo_compute import obb_vertices


# Build docks. Each dock initializes a robot at its docking pose
def build_docks(
    num_docks: int = NUM_DOCKS,
    spacing: float = DOCK_SPACING,
) -> DockState:
    pose = np.empty((num_docks, 3), dtype=np.float32)

    pose[:, 0] = DOCK_ONE_POSE.x + np.arange(num_docks, dtype=np.float32) * spacing
    pose[:, 1] = DOCK_ONE_POSE.y
    pose[:, 2] = DOCK_ONE_POSE.theta

    vertices = obb_vertices(
        pose,
        DOCK_LENGTH,
        DOCK_WIDTH,
    )

    node_pose = np.empty((num_docks, 3), dtype=np.float32)
    node_pose[:, 0] = pose[:, 0]
    node_pose[:, 1] = pose[:, 1] + ROBOT_DOCK_DIST + DOCK_APPROACH_DISTANCE
    node_pose[:, 2] = np.pi / 2

    return DockState(
        pose=pose,
        vertices=vertices,
        node_pose=node_pose,
    )


def build_robots(start_pose: np.ndarray) -> RobotState:
    return RobotState(
        pose=start_pose.copy(),
        twist=np.zeros((NUM_DOCKS, 2), dtype=np.float32),
        crashed=np.zeros(NUM_DOCKS, dtype=bool),
        vertices=obb_vertices(
            start_pose,
            ROBOT_LENGTH,
            ROBOT_WIDTH,
        ),
        target_node_id=np.full(NUM_DOCKS, -1, dtype=np.int32),
        path_index=np.zeros(NUM_DOCKS, dtype=np.int32),
        paths=[[] for _ in range(NUM_DOCKS)],
    )


# Build shelves in a vertical layout
def build_shelves_vertical(
    shelf_col: int = SHELF_COL,
    shelf_row: int = SHELF_ROW,
    margin: float = 1.0,
) -> ShelfState:
    num_shelves = shelf_col * shelf_row

    pose = np.empty((num_shelves, 3), dtype=np.float32)
    index = np.arange(num_shelves, dtype=np.int32)

    horizontal_gap = (X_MAX - 2 * margin - shelf_col * SHELF_WIDTH) / (shelf_col + 1)
    vertical_gap = (Y_MAX - 2 * margin - shelf_row * SHELF_LENGTH) / (shelf_row + 1)

    k = 0
    for i in range(shelf_col):
        for j in range(shelf_row):
            pose[k] = (
                margin + horizontal_gap + SHELF_WIDTH / 2 + i * (SHELF_WIDTH + horizontal_gap),
                margin + vertical_gap + SHELF_LENGTH / 2 + j * (SHELF_LENGTH + vertical_gap),
                math.pi / 2,
            )
            k += 1

    vertices = obb_vertices(pose, SHELF_LENGTH, SHELF_WIDTH)

    return ShelfState(pose=pose, vertices=vertices, index=index)


def build_pallets(shelves: ShelfState) -> PalletState:
    num_pallets = len(shelves.pose) * 2 * NUM_PALLETS_PER_SHELF

    pose = np.empty((num_pallets, 3), dtype=np.float32)
    index = np.arange(2 * NUM_PALLETS_PER_SHELF * SHELF_ROW * SHELF_COL, dtype=np.int32)

    k = 0
    y = SHELF_WIDTH / 2 - PALLET_WIDTH / 2
    spacing = PALLET_LENGTH

    for x_shelf, y_shelf, theta in shelves.pose:
        c = math.cos(theta)
        s = math.sin(theta)

        for i in range(NUM_PALLETS_PER_SHELF):
            x = -SHELF_LENGTH / 2 + PALLET_LENGTH / 2 + i * spacing

            for y_local in (y, -y):
                pose[k] = (
                    x_shelf + x * c - y_local * s,
                    y_shelf + x * s + y_local * c,
                    theta,
                )
                k += 1

    vertices = obb_vertices(
        pose,
        PALLET_LENGTH,
        PALLET_WIDTH,
    )

    return PalletState(pose=pose, vertices=vertices, index=index)
