import cProfile
import pstats
import time

import numpy as np

from common.types import Pose
from config import MAX_VELOCITY, PHYSICS_DT, ROBOT_LENGTH, ROBOT_WIDTH
from entities.robot import Robot, RobotInfo, RobotState
from render.playback import Playback
from render.renderer import Renderer
from simulator.simulation import Simulator
from simulator.world import World


def main():
    # Start simulation timer and profiling process
    profiler = cProfile.Profile()
    profiler.enable()
    start = time.perf_counter()

    world = World()

    robot = Robot(
        info=RobotInfo(0, Pose(12, 5, np.pi / 2)),
        start_pose=Pose(
            x=2.0,
            y=5.0,
            theta=0.0,
        ),
    )

    world.add_robot(robot)

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

    Playback(sim.frames).run()


if __name__ == "__main__":
    main()
