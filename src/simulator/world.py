from dataclasses import dataclass

import numpy as np

from config import PHYSICS_DT, X_MAX, X_MIN, Y_MAX, Y_MIN
from entities.dock import Dock
from entities.pallet import Pallet, PalletFrame
from entities.robot import Robot, RobotFrame
from entities.shelf import Shelf
from entities.wall import Wall
from graphs.graph import NavigationGraph


@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: np.ndarray  # shape (N, 3): x, y, theta
    pallets: np.ndarray  # shape (M, 3): x, y, theta


@dataclass(slots=True)
class WorldMap:
    shelves: list[Shelf]
    docks: list[Dock]
    walls: list[Wall]
    graph: NavigationGraph


class World:
    def __init__(self):
        self.time: float = 0.0
        self.robots: list[Robot] = []
        self.docks: list[Dock] = []
        self.shelves: list[Shelf] = []
        self.walls: list[Wall] = []
        self.pallets: list[Pallet] = []
        self.graph: NavigationGraph = NavigationGraph()

    def step(self) -> None:
        # Advance the simulation by one physics step
        self.time += PHYSICS_DT

        # Update every robot
        for robot in self.robots:
            robot.step()

        # Handles crashes
        for robot in self.robot_boundary_collisions():
            robot.crash()

    def world_map(self) -> WorldMap:
        return WorldMap(shelves=self.shelves, docks=self.docks, walls=self.walls, graph=self.graph)

    def frame(self) -> WorldFrame:
        robots = np.empty((len(self.robots), 3), dtype=np.float32)
        for i, robot in enumerate(self.robots):
            pose = robot.state.pose
            robots[i, 0] = pose.x
            robots[i, 1] = pose.y
            robots[i, 2] = pose.theta

        pallets = np.empty((len(self.pallets), 3), dtype=np.float32)
        for i, pallet in enumerate(self.pallets):
            pose = pallet.pose
            pallets[i, 0] = pose.x
            pallets[i, 1] = pose.y
            pallets[i, 2] = pose.theta

        return WorldFrame(
            time=self.time,
            robots=robots,
            pallets=pallets,
        )

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

    def add_dock(self, dock: Dock) -> None:
        self.docks.append(dock)

    def add_shelf(self, shelf: Shelf) -> None:
        self.shelves.append(shelf)

    def add_pallet(self, pallet):
        self.pallets.append(pallet)

    def add_wall(self, wall: Wall) -> None:
        self.walls.append(wall)

    # Return robots whose footprint extends outside the world bounds
    def robot_boundary_collisions(self) -> list:
        out_of_bounds = []
        for robot in self.robots:
            if robot.state.crashed:
                continue

            for x, y in robot.state.vertices:
                if x < X_MIN or x > X_MAX or y < Y_MIN or y > Y_MAX:
                    out_of_bounds.append(robot)
                    break

        return out_of_bounds

    @property
    def nodes(self):
        nodes = []

        for dock in self.docks:
            nodes.append(dock.approach_node)
            nodes.append(dock.dock_node)

        return nodes
