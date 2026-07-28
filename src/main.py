import cProfile
import pstats
import time

from common.types import Pose
from config import DOCK_WIDTH, PHYSICS_DT, SIMULATION_DURATION
from entities.dock import Dock
from entities.robot import Robot, RobotInfo
from render.playback import Playback
from simulator.builders import build_docks
from simulator.simulation import Simulator
from simulator.world import World


def main():
    # Start simulation timer and profiling process
    profiler = cProfile.Profile()
    profiler.enable()
    start = time.perf_counter()

    world = World()
    docks, robots = build_docks()
    for dock, robot in zip(docks, robots):
        world.add_dock(dock)

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
