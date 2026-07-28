import cProfile
import pstats
import time

from config import PHYSICS_DT
from renderer.renderer import Renderer
from simulator.simulation import Simulator
from simulator.world import World


def main():
    # Start simulation timer and profiling process
    profiler = cProfile.Profile()
    profiler.enable()
    start = time.perf_counter()

    world = World()
    sim = Simulator(world)

    duration = 10.0
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
    stats.print_stats(10)

    print("\n=== By internal time ===")
    stats.sort_stats(pstats.SortKey.TIME)  # Self time only
    stats.print_stats(10)

    renderer = Renderer()
    renderer.playback(sim.timeline())


if __name__ == "__main__":
    main()
