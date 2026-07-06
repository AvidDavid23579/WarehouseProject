import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

from renderer import Renderer
from robot import Robot
from simulation import Simulation
from world import World

world = World()
renderer = Renderer()

robots = [
    Robot(4, 2, 0),
    Robot(6, 2, 0),
    Robot(8, 2, 0),
    Robot(10, 2, 0),
]
velocities = [1, 1, 1, 1]
angular_velocities = [0.5, 0.5, 0.5, 0.5]

for robot, v, omega in zip(robots, velocities, angular_velocities):
    robot.v = v
    robot.omega = omega
    world.add_robot(robot)

# Seed patches once up front so blit has artists to return from frame 0,
# during the start delay, before physics begins stepping.
renderer.draw(world.snapshot())

sim = Simulation(world, renderer, physics_dt=0.01, start_delay=3.0)

ani = FuncAnimation(renderer.fig, sim.on_frame, interval=15, blit=True)
plt.show()
