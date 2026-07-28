from simulator.world import World, WorldFrame


class Simulator:
    def __init__(self, world: World):
        self.world = world
        self.frames: list[WorldFrame] = []

    def bake(self, steps: int) -> None:
        """Run the simulation and record every frame."""

        self.frames.clear()

        # Record the initial state
        self.frames.append(self.world.frame())

        for _ in range(steps):
            self.world.step()
            self.frames.append(self.world.frame())
