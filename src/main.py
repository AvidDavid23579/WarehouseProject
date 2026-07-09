import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from config import WORLD_BOUNDS
from entities.robot import Robot
from renderer import Renderer
from simulation import Simulation
from world import World

world = World(bounds=WORLD_BOUNDS)
renderer = Renderer(bounds=WORLD_BOUNDS)

robots = [Robot(2.5, 2, 0)]
velocities = [4, 4, 1, 1]
angular_velocities = [0.5, 0.5, 0.5, 0.5]

for robot, v, omega in zip(robots, velocities, angular_velocities):
    robot.v = v
    robot.omega = omega
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
