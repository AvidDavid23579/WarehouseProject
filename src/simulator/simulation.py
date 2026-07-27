from simulator.world import World, WorldState


class Simulator:
    def __init__(self, world: World):
        self.world = world
        self.states: list[WorldState] = []

    def bake(self, steps: int) -> None:
        self.states.clear()
        for _ in range(steps):
            self.world.step()
            self.states.append(self.world.get_state())

    def timeline(self) -> list[WorldState]:
        return self.states
