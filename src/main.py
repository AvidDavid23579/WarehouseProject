import cProfile
import pstats
import time

from config import PHYSICS_DT, SIMULATION_DURATION
from graphs.graph_builder import GraphBuilder
from render.playback import Playback
from simulator.builders import build_docks, build_shelves_vertical
from simulator.simulation import Simulator
from simulator.world import World


def main():
    # Start simulation timer and profiling process
    profiler = cProfile.Profile()
    profiler.enable()
    start = time.perf_counter()

    # Initialize world object
    world = World()

    # Build objects according to layouts
    shelves, pallets = build_shelves_vertical()
    docks, robots = build_docks()

    # Add objects to the world
    for shelf in shelves:
        world.add_shelf(shelf)
    for pallet in pallets:
        world.add_pallet(pallet)
    for dock in docks:
        world.add_dock(dock)
    for robot in robots:
        world.add_robot(robot)

    world.robot_target_node[0] = 1

    # Build the navigation graph
    world.graph = GraphBuilder(world).build()

    # Makes the simulation
    sim = Simulator(world)
    duration = SIMULATION_DURATION
    steps = int(duration / PHYSICS_DT)
    sim.bake(steps)

    # End simulation timer
    elapsed = time.perf_counter() - start
    print(f"Simulation time: {elapsed:.6f} s")

    # End profiling process
    profiler.disable()

    stats = pstats.Stats(profiler)

    print("\n=== By cumulative time ===")
    stats.sort_stats(pstats.SortKey.CUMULATIVE)  # Time including child calls
    stats.print_stats(20)

    print("\n=== By internal time ===")
    stats.sort_stats(pstats.SortKey.TIME)  # Self time only
    stats.print_stats(20)

    Playback(sim.world_map, sim.frames).run()


if __name__ == "__main__":
    main()
