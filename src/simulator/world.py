from dataclasses import dataclass

import numpy as np

from common.utils import SpatialHash, rotated_rectangle_vertices, update_rotated_rectangle_vertices
from config import CELL_SIZE, PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH
from entities.dock import DockState
from entities.pallet import PalletState
from entities.robot import Robot, RobotState
from entities.shelf import ShelfState
from navigation.graph import NavigationGraph


# Data needed to render each frame
@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: np.ndarray
    pallets: np.ndarray


# One time initialization of static objects
@dataclass(slots=True)
class WorldMap:
    shelf: ShelfState
    dock: DockState
    graph: NavigationGraph


class World:
    def __init__(self):
        self.time = 0.0

        # Dynamic objects
        self.robot = RobotState()
        self.pallet = PalletState()

        # Static objects
        self.dock = DockState()
        self.shelf = ShelfState()

        # Spatial hash for potential fields/ORCA...
        self.static_hash = SpatialHash(CELL_SIZE)

        # Warehouse graph
        self.graph = NavigationGraph()

        # Robot SoA data

        # Pallets
        self.pallet_pose = np.empty((0, 3), dtype=np.float32)

    # ----------- Entity addition functions ------------------------------------

    def add_pallet(self, pallet):
        self.pallets.append(pallet)

        pose = pallet.pose

        self.pallet_pose = np.vstack(
            (
                self.pallet_pose,
                np.array(
                    [[pose.x, pose.y, pose.theta]],
                    dtype=np.float32,
                ),
            )
        )

    # ----------- Simulation update functions ---------------------------------------

    def frame(self) -> WorldFrame:
        return WorldFrame(
            time=self.time,
            robots=self.robot.pose.copy(),
            pallets=self.pallet_pose.copy(),
        )

    def world_map(self):
        return WorldMap(
            shelf=self.shelf,
            dock=self.dock,
            graph=self.graph,
        )

    def step(self):

        self.time += PHYSICS_DT

        pose = self.robot.pose
        vel = self.robot.twist
        vertices = self.robot.vertices

        pose[:, 0] += vel[:, 0] * np.cos(pose[:, 2]) * PHYSICS_DT
        pose[:, 1] += vel[:, 0] * np.sin(pose[:, 2]) * PHYSICS_DT
        pose[:, 2] += vel[:, 1] * PHYSICS_DT

        update_rotated_rectangle_vertices(pose, vertices, ROBOT_LENGTH, ROBOT_WIDTH)
