from common.types import NavPhase
from config import NUM_DOCKS
from controller.drive import drive_to_pose_grid
from controller.potential import apply_repulsion
from navigation.navigator import Navigator
from simulator.world import World, WorldFrame, WorldMap
import numpy as np


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
            self.world.robot.path_index[i] = 0
            self.world.robot.current_node_id[i] = i
            path = self.graph.dijkstra(i, 7 + 2 * i)

            if path is None:
                raise RuntimeError(f"No path found from {i} to {7 + 2 * i}")
            
            self.world.robot.path[i] = path

        for step in range(steps):
            self.navigator.update(self.world.robot)
            drive_to_pose_grid(world=self.world, graph=self.graph)
            self.world.step()
            self.frames.append(self.world.frame())

            for r in range(len(self.world.robot.pose)):
                p = self.world.robot.pose[r]

                if not np.all(np.isfinite(p)):
                    raise RuntimeError(
                    f"Robot {r} pose became invalid at step {step}: {p}"
                    )

                if np.max(np.abs(p)) > 1e5:
                    raise RuntimeError(
                    f"Robot {r} pose exploded at step {step}: {p}"
                    )

    @property
    def world_map(self) -> WorldMap:
        return self.world.world_map()
