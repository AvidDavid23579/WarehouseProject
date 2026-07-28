from dataclasses import dataclass

from config import PHYSICS_DT, X_MAX, X_MIN, Y_MAX, Y_MIN
from entities.dock import Dock
from entities.robot import Robot, RobotFrame
from entities.shelf import Shelf
from entities.wall import Wall


@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: list[RobotFrame]


@dataclass(slots=True)
class WorldMap:
    shelves: list[Shelf]
    docks: list[Dock]
    walls: list[Wall]


class World:
    def __init__(self):
        self.time: float = 0.0
        self.robots: list[Robot] = []
        self.docks: list[Dock] = []
        self.shelves: list[Shelf] = []
        self.walls: list[Wall] = []

    def step(self) -> None:
        # Advance the simulation by one physics step
        self.time += PHYSICS_DT

        # Update every robot
        for robot in self.robots:
            robot.update_goal()
            robot.drive_to_pose()
            robot.step()

        # Handles crashes
        for robot in self.robot_boundary_collisions():
            robot.crash()

    def world_map(self) -> WorldMap:
        return WorldMap(shelves=self.shelves, docks=self.docks, walls=self.walls)

    def frame(self) -> WorldFrame:
        # Return an immutable snapshot for rendering/playback
        return WorldFrame(
            time=self.time,
            robots=[
                RobotFrame(
                    x=robot.state.pose.x,
                    y=robot.state.pose.y,
                    theta=robot.state.pose.theta,
                )
                for robot in self.robots
            ],
        )

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

    def add_dock(self, dock: Dock) -> None:
        self.docks.append(dock)

    # Return robots whose footprint extends outside the world bounds
    def robot_boundary_collisions(self) -> list:
        out_of_bounds = []
        for robot in self.robots:
            if robot.state.crashed:
                continue

            for x, y in robot.robot_vertices():
                if x < X_MIN or x > X_MAX or y < Y_MIN or y > Y_MAX:
                    out_of_bounds.append(robot)
                    break

        return out_of_bounds
