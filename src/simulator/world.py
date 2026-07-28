from dataclasses import dataclass

from config import PHYSICS_DT, X_MAX, X_MIN, Y_MAX, Y_MIN
from entities.dock import Dock
from entities.robot import Robot, RobotFrame
from entities.shelf import Shelf


@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: list[RobotFrame]


@dataclass(slots=True)
class WorldMap:
    shelves: list[Shelf]
    docks: list[Dock]


class World:
    def __init__(self):
        self.time: float = 0.0
        self.robots: list[Robot] = []
        self.docks: list[Dock] = []
        self.shelves: list[Shelf] = []

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
        return WorldMap(shelves=self.shelves, docks=self.docks)

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
