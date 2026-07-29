from dataclasses import dataclass

import numpy as np

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH, X_MAX, X_MIN, Y_MAX, Y_MIN
from entities.dock import Dock
from entities.pallet import Pallet
from entities.robot import Robot
from entities.shelf import Shelf
from entities.wall import Wall
from graphs.graph import NavigationGraph


@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: np.ndarray
    pallets: np.ndarray


@dataclass(slots=True)
class WorldMap:
    shelves: list[Shelf]
    docks: list[Dock]
    walls: list[Wall]
    graph: NavigationGraph


class World:
    def __init__(self):
        self.time = 0.0

        self.robots = []
        self.pallets = []

        self.docks = []
        self.shelves = []
        self.walls = []

        self.graph = NavigationGraph()

        # Robot SoA data
        self.robot_pose = np.empty((0, 3), dtype=np.float32)
        self.robot_velocity = np.empty((0, 2), dtype=np.float32)
        self.robot_crashed = np.empty((0,), dtype=np.bool_)

        # Robot collision geometry
        self.robot_vertices = np.empty(
            (0, 4, 2),
            dtype=np.float32,
        )

        # Pallets
        self.pallet_pose = np.empty((0, 3), dtype=np.float32)

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

        p = robot.start_pose

        self.robot_pose = np.vstack((self.robot_pose, np.array([[p.x, p.y, p.theta]], dtype=np.float32)))

        self.robot_velocity = np.vstack((self.robot_velocity, np.array([[0.0, 0.0]], dtype=np.float32)))

        self.robot_crashed = np.append(self.robot_crashed, False)

        vertices = rotated_rectangle_vertices(
            p,
            ROBOT_LENGTH,
            ROBOT_WIDTH,
        )

        self.robot_vertices = np.vstack((self.robot_vertices, np.array([vertices], dtype=np.float32)))

    def update_robot_vertices(self):

        pose = self.robot_pose

        x = pose[:, 0]
        y = pose[:, 1]
        theta = pose[:, 2]

        c = np.cos(theta)
        s = np.sin(theta)

        hl = ROBOT_LENGTH / 2
        hw = ROBOT_WIDTH / 2

        local = np.array(
            [
                [-hl, -hw],
                [hl, -hw],
                [hl, hw],
                [-hl, hw],
            ],
            dtype=np.float32,
        )

        # rotation for all robots
        self.robot_vertices[:, :, 0] = x[:, None] + local[:, 0][None, :] * c[:, None] - local[:, 1][None, :] * s[:, None]

        self.robot_vertices[:, :, 1] = y[:, None] + local[:, 0][None, :] * s[:, None] + local[:, 1][None, :] * c[:, None]

    def step(self):

        self.time += PHYSICS_DT

        pose = self.robot_pose
        vel = self.robot_velocity

        pose[:, 0] += vel[:, 0] * np.cos(pose[:, 2]) * PHYSICS_DT
        pose[:, 1] += vel[:, 0] * np.sin(pose[:, 2]) * PHYSICS_DT
        pose[:, 2] += vel[:, 1] * PHYSICS_DT

        self.update_robot_vertices()

        self.robot_boundary_collisions()

    def frame(self) -> WorldFrame:
        return WorldFrame(
            time=self.time,
            robots=self.robot_pose,
            pallets=self.pallet_pose,
        )

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

    def world_map(self):
        return WorldMap(
            shelves=self.shelves,
            docks=self.docks,
            walls=self.walls,
            graph=self.graph,
        )

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

    @property
    def nodes(self):
        nodes = []

        for dock in self.docks:
            nodes.append(dock.approach_node)
            nodes.append(dock.dock_node)

        return nodes
