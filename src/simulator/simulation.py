from config import NUM_DOCKS
from controller.drive import drive_to_pose_grid
from navigation.navigator import Navigator
from simulator.world import World, WorldFrame, WorldMap


class Simulator:
    def __init__(self, world: World):
        self.world = world
        self.graph = world.graph
        self.navigator = Navigator(self.graph)
        self.frames: list[WorldFrame] = []

    def bake(self, steps: int) -> None:
        # Runs the simulation and record every frame

        self.frames.clear()

        # Record the initial state
        self.frames.append(self.world.frame())

        for i in range(NUM_DOCKS):
            self.world.robot.path[i] = self.graph.dfs(i, 56)

            self.world.robot.path_index[i] = 0
            self.world.robot.current_node_id[i] = i

        for step in range(steps):
            self.navigator.update(self.world.robot)
            drive_to_pose_grid(world=self.world, graph=self.graph)
            self.world.step()
            self.frames.append(self.world.frame())

    @property
    def world_map(self) -> WorldMap:
        return self.world.world_map()
