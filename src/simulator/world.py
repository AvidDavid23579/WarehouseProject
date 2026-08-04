from dataclasses import dataclass

import numpy as np

from common.utils import rotated_rectangle_vertices, update_robot_vertices
from config import PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN
from entities.dock import Dock
from entities.robot import Robot
from entities.shelf import Shelf
from entities.wall import Wall
from graphs.graph import NavigationGraph


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

        self.graph = NavigationGraph()

        # Robot SoA data

        # Empty matrix of shape 3: x, y, and theta
        self.robot_pose = np.empty((0, 3), dtype=np.float32)
        # Empty matrix of shape 2: v and omega
        self.robot_velocity = np.empty((0, 2), dtype=np.float32)
        # Empty list of booleans to check crash state
        self.robot_crashed = np.empty(0, dtype=np.bool_)

        # Navigation
        self.robot_target_node = np.empty(0, dtype=np.int32)
        self.robot_path_index = np.empty(0, dtype=np.int32)
        self.robot_paths = []  # list[list[int]]

        # Robot collision geometry
        self.robot_vertices = np.empty((0, 4, 2), dtype=np.float32)  # 4 corners, 2D coordinates

        # Pallets
        self.pallet_pose = np.empty((0, 3), dtype=np.float32)

    # ----------- Robot update functions ---------------------------------------

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

        p = robot.start_pose

        self.robot_pose = np.vstack((self.robot_pose, np.array([[p.x, p.y, p.theta]], dtype=np.float32)))
        self.robot_velocity = np.vstack((self.robot_velocity, np.array([[0.0, 0.0]], dtype=np.float32)))
        self.robot_crashed = np.append(self.robot_crashed, False)

        vertices = rotated_rectangle_vertices(p, ROBOT_LENGTH, ROBOT_WIDTH)

        self.robot_vertices = np.vstack((self.robot_vertices, np.array([vertices], dtype=np.float32)))

        self.robot_target_node = np.append(self.robot_target_node, -1)
        self.robot_path_index = np.append(self.robot_path_index, 0)
        self.robot_paths.append([])

    def robot_boundary_collisions(self):
        vertices = self.robot_vertices
        outside = (vertices[:, :, 0] < X_MIN) | (vertices[:, :, 0] > X_MAX) | (vertices[:, :, 1] < Y_MIN) | (vertices[:, :, 1] > Y_MAX)
        crashed = np.any(outside, axis=1)
        new = crashed & (~self.robot_crashed)

        self.robot_crashed[new] = True
        self.robot_velocity[new] = 0.0

    def crash_robot(self, i):

        if self.robot_crashed[i]:
            return

        self.robot_crashed[i] = True
        self.robot_velocity[i] = 0.0

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
            robots=self.robot_pose.copy(),
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

        pose = self.robot_pose
        vel = self.robot_velocity

        pose[:, 0] += vel[:, 0] * np.cos(pose[:, 2]) * PHYSICS_DT
        pose[:, 1] += vel[:, 0] * np.sin(pose[:, 2]) * PHYSICS_DT
        pose[:, 2] += vel[:, 1] * PHYSICS_DT

        update_robot_vertices(self)

        self.robot_boundary_collisions()
