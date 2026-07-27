import time

from config import PHYSICS_DT
from renderer.renderer import Renderer
from simulator.simulation import Simulator
from simulator.world import World


def main():
    # Start simulation timer
    start = time.perf_counter()

    world = World()
    sim = Simulator(world)

    duration = 10.0
    steps = int(duration / PHYSICS_DT)

    sim.bake(steps)

    # End simulation timer
    elapsed = time.perf_counter() - start
    print(f"Simulation time: {elapsed:.6f} s")

    renderer = Renderer()
    renderer.playback(sim.timeline())


if __name__ == "__main__":
    main()
