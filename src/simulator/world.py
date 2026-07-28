import copy
from dataclasses import dataclass

from config import PHYSICS_DT
from entities.robot import Robot


@dataclass
class WorldState:
    time: float
    robots: list[Robot]


class World:
    def __init__(self):
        self.state = WorldState(time=0.0, robots=[])

    def step(self) -> None:
        self.state.time += PHYSICS_DT

    def get_state(self) -> WorldState:
        return WorldState(
            time=self.state.time,
            robots=copy.deepcopy(self.state.robots),
        )

    def add_robot(self, robot: Robot):
        self.state.robots.append(robot)
