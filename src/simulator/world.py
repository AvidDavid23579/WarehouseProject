from dataclasses import dataclass, field

from config import PHYSICS_DT
from entities.robot import Robot, RobotFrame


@dataclass(slots=True)
class WorldFrame:
    time: float
    robots: list[RobotFrame]


class World:
    def __init__(self):
        self.time: float = 0.0
        self.robots: list[Robot] = []

    def step(self) -> None:
        """Advance the simulation by one physics step."""
        self.time += PHYSICS_DT

        # Update every robot
        for robot in self.robots:
            robot.update_goal()
            robot.drive_to_pose()
            robot.step()

    def frame(self) -> WorldFrame:
        """Return an immutable snapshot for rendering/playback."""
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
