from dataclasses import dataclass

import numpy as np

from common.utils import SpatialHash, rotated_rectangle_vertices, update_rotated_rectangle_vertices
from config import CELL_SIZE, PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH
from entities.dock import Dock
from entities.robot import Robot, RobotState
from entities.shelf import Shelf
from entities.wall import Wall
from geometry.collision import robot_boundary_collisions
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
    shelves: list[Shelf]
    docks: list[Dock]
    walls: list[Wall]
    graph: NavigationGraph


class World:
    def __init__(self):
        self.time = 0.0

        # Dynamic objects
        self.robots = []
        self.pallets = []

        # Static objects
        self.docks = []
        self.shelves = []
        self.walls = []

        self.static_hash = SpatialHash(CELL_SIZE)

        self.graph = NavigationGraph()

        # Robot SoA data
        self.robot = RobotState()

        # Pallets
        self.pallet_pose = np.empty((0, 3), dtype=np.float32)

    # ----------- Robot update functions ---------------------------------------

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

        p = robot.start_pose

        self.robot.pose = np.vstack((self.robot.pose, np.array([[p.x, p.y, p.theta]], dtype=np.float32)))
        self.robot.twist = np.vstack((self.robot.twist, np.array([[0.0, 0.0]], dtype=np.float32)))
        self.robot.crashed = np.append(self.robot.crashed, False)

        vertices = rotated_rectangle_vertices(p, ROBOT_LENGTH, ROBOT_WIDTH)

        self.robot.vertices = np.vstack((self.robot.vertices, np.array([vertices], dtype=np.float32)))

        self.robot.target_node_id = np.append(self.robot.target_node_id, -1)
        self.robot.path_index = np.append(self.robot.path_index, 0)
        self.robot.paths.append([])

    def crash_robot(self, i):

        if self.robot.crashed[i]:
            return

        self.robot.crashed[i] = True
        self.robot.twist[i] = 0.0

        self.robots[i].crash()

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

    def add_dock(self, dock):
        self.docks.append(dock)

    def add_shelf(self, shelf):
        self.shelves.append(shelf)

    def add_wall(self, wall):
        self.walls.append(wall)

    # ----------- Simulation update functions ---------------------------------------

    def frame(self) -> WorldFrame:
        return WorldFrame(
            time=self.time,
            robots=self.robot.pose.copy(),
            pallets=self.pallet_pose.copy(),
        )

    def world_map(self):
        return WorldMap(
            shelves=self.shelves,
            docks=self.docks,
            walls=self.walls,
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

        robot_boundary_collisions(self)
