from common.drive import drive_to_pose
from simulator.world import World, WorldFrame, WorldMap


class Simulator:
    def __init__(self, world: World):
        self.world = world
        self.frames: list[WorldFrame] = []

    def bake(self, steps: int) -> None:
        # Runs the simulation and record every frame

        self.frames.clear()

        # Record the initial state
        self.frames.append(self.world.frame())

        for _ in range(steps):
            drive_to_pose(world=self.world)
            self.world.step()
            self.frames.append(self.world.frame())

    @property
    def world_map(self) -> WorldMap:
        return self.world.world_map()
