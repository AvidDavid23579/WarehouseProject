from dataclasses import dataclass

import numpy as np

from config import PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH
from entities.dock import DockState
from entities.pallet import PalletState
from entities.robot import RobotState
from entities.shelf import ShelfState
from entities.goal import GoalZone
from geometry.geo_compute import update_obb_vertices
from navigation.graph import NavigationGraph


# Data needed to render each frame
@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: RobotState
    pallet: PalletState


# One time initialization of static objects
@dataclass(slots=True)
class WorldMap:
    shelf: ShelfState
    dock: DockState
    goal: GoalZone
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
        self.goal = GoalZone()

        # Warehouse graph
        self.graph = NavigationGraph()

        # Pallets
        self.pallet_pose = np.empty((0, 3), dtype=np.float32)

    # ----------- Simulation update functions ---------------------------------------

    def frame(self) -> WorldFrame:
        return WorldFrame(
            time=self.time,
            robots=RobotState(
                pose=self.robot.pose.copy(),
                vertices=self.robot.vertices.copy()
                ),
            pallet=PalletState(
                pose=self.pallet.pose.copy(),
                vertices=self.pallet.vertices.copy(),
                index=self.pallet.index.copy(),
            ),
        )

    def world_map(self):
        return WorldMap(
            shelf=self.shelf,
            dock=self.dock,
            goal=self.goal,
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

        update_obb_vertices(pose, vertices, ROBOT_LENGTH, ROBOT_WIDTH)
