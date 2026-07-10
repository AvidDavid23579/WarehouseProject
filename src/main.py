import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from entities.robot import Pose, Robot
from renderer import Renderer
from simulation import Simulation
from src.config import ROBOT_LENGTH, ROBOT_WIDTH, WORLD_BOUNDS
from world import World

world = World(bounds=WORLD_BOUNDS)
renderer = Renderer(bounds=WORLD_BOUNDS)

num_robots = 12
start_poses = []
robot_goals = []
prio = []

for i in range(num_robots):
    start_poses.append(Pose(i + ROBOT_WIDTH / 2 + 0.1, ROBOT_LENGTH / 2 + 0.1, np.pi / 2))

    robot_goals.append(
        [
            Pose(2 * i + 0.5, 1 * i + 2.5, np.pi),
            Pose(3 * i + 2.5, 18 + 0.5, 0.0),
            Pose(38.5 - 2 * i, i + 2.5, np.pi),
        ]
    )

    prio.append(i)


robots = [Robot(start, goals, prio) for start, goals, prio in zip(start_poses, robot_goals, prio)]


for robot in robots:
    world.add_robot(robot)

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
