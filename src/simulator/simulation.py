from controller.drive import drive_to_pose_grid
from controller.potential import apply_repulsion
from controller.path_planning import update_carried_pallets, return_to_dock, assign_pallets, handle_pickups, handle_deliveries
from simulator.world import World, WorldFrame, WorldMap


class Simulator:
    def __init__(self, world: World):
        self.world = world
        self.graph = world.graph
        self.frames: list[WorldFrame] = []

    def bake(self, steps: int) -> None:
        # Runs the simulation and record every frame
        self.frames.clear()

        # Record the initial state
        self.frames.append(self.world.frame())

        for step in range(steps):
            assign_pallets(self.world, self.graph)
            handle_pickups(self.world, self.graph)
            drive_to_pose_grid(world=self.world, graph=self.graph)
            apply_repulsion(self.world)
            self.world.step()
            update_carried_pallets(world=self.world)
            handle_deliveries(self.world, self.graph)
            return_to_dock(self.world, self.graph)

            self.frames.append(self.world.frame())

    @property
    def world_map(self) -> WorldMap:
        return self.world.world_map()
