# Main file. Only runs abstracted code from other files.

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from config import ROBOT_LENGTH, ROBOT_WIDTH, WORLD_BOUNDS
from entities.robot import Pose, Robot
from entities.shelves import Shelf
from renderer import Renderer
from simulation import Simulation
from world import World

world = World(bounds=WORLD_BOUNDS)
renderer = Renderer(bounds=WORLD_BOUNDS)

num_shelves = 3
shelves = []
for i in range(num_shelves):
    shelves.append(Shelf(i, Pose(4 * i, 10, 0)))

num_robots = 12
start_poses = []
robot_goals = []
id = []

for i in range(num_robots):
    start_poses.append(Pose(i + ROBOT_WIDTH / 2 + 0.1, ROBOT_LENGTH / 2 + 0.1, np.pi / 2))

    robot_goals.append(
        [
            Pose(2 * i + 0.5, 1 * i + 2.5, np.pi),
            Pose(3 * i + 2.5, 18 + 1.5, 0.0),
            Pose(39.5 - 2 * i, i + 3.5, np.pi),
        ]
    )

    id.append(i)


robots = [Robot(start, goals, prio) for start, goals, prio in zip(start_poses, robot_goals, id)]


for robot in robots:
    world.add_robot(robot)

for shelf in shelves:
    world.add_shelf(shelf)

# Seed patches once up front so blit has artists to return from frame 0,
# during the start delay, before physics begins stepping.
renderer.draw(world.snapshot())

sim = Simulation(world, renderer, physics_dt=0.01, start_delay=2.0)

ani = FuncAnimation(
    renderer.fig,
    sim.on_frame,
    frames=range(1000000),
    interval=15,
    blit=True,
    cache_frame_data=False,
)
manager = plt.get_current_fig_manager()
manager.window.showMaximized()

plt.show()
