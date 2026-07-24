"""Entry point for the warehouse multi-robot simulation."""

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from config import ROBOT_LENGTH, ROBOT_WIDTH, WORLD_BOUNDS
from entities.robot import Pose, Robot
from entities.shelves import Shelf
from renderer import Renderer
from simulation import Simulation
from world import World

# --- Demo scenario layout ----------------------------------------------------
NUM_SHELVES = 0
NUM_ROBOTS = 20
PHYSICS_DT = 0.01
START_DELAY = 2.0
ANIMATION_INTERVAL_MS = 15


def create_shelves(count: int) -> list[Shelf]:
    """Place shelves in a row along y = 10, spaced 4 m apart."""
    return [Shelf(shelf_id=i, pose=Pose(4 * (i + 1), 10, np.pi / 2)) for i in range(count)]


def create_robots(count: int) -> list[Robot]:
    """Spawn robots along the bottom edge, each with three waypoint goals."""
    robots = []
    for i in range(count):
        if i % 2 == 0:
            start = Pose(2 * (i + 1), ROBOT_LENGTH / 2 + 0.2, np.pi / 2)
            goals = [Pose(2 * (i + 1), ROBOT_LENGTH / 2 + 17, np.pi / 2), Pose(2 * (i + 1), ROBOT_LENGTH / 2 + 0.2, np.pi / 2)]
        else:
            start = Pose(2 * (i + 1) - 2, ROBOT_LENGTH / 2 + 17, -np.pi / 2)
            goals = [Pose(2 * (i + 1) - 2, ROBOT_LENGTH / 2 + 0.2, -np.pi / 2), Pose(2 * (i + 1) - 2, ROBOT_LENGTH / 2 + 17, -np.pi / 2)]
        robots.append(Robot(start, goals, robot_id=i))
    return robots


def build_world() -> World:
    """Assemble the demo world with shelves and robots."""
    world = World(bounds=WORLD_BOUNDS)
    for robot in create_robots(NUM_ROBOTS):
        world.add_robot(robot)
    for shelf in create_shelves(NUM_SHELVES):
        world.add_shelf(shelf)
    return world


def main() -> None:
    world = build_world()
    renderer = Renderer(bounds=WORLD_BOUNDS)

    # Seed patches before physics starts so blit has artists from frame 0.
    renderer.draw(world.snapshot())

    sim = Simulation(
        world,
        renderer,
        physics_dt=PHYSICS_DT,
        start_delay=START_DELAY,
    )

    anim = FuncAnimation(
        renderer.fig,
        sim.on_frame,
        frames=range(1_000_000),
        interval=ANIMATION_INTERVAL_MS,
        blit=True,
        cache_frame_data=False,
    )

    plt.get_current_fig_manager().window.showMaximized()
    plt.show()


if __name__ == "__main__":
    main()
