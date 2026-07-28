from types import WorldState

from config import PHYSICS_DT


class World:
    def __init__(self):
        self.state = WorldState(0.0)

    def step(self) -> None:
        self.state.time += PHYSICS_DT

    def get_state(self) -> WorldState:
        return WorldState(self.state.time)
