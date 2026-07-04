from arrow import now
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt
from robot import Robot
from scene import Scene
import time

start_time = time.perf_counter()
last_time = start_time

scene = Scene()

robots = [
    Robot(4, 2, 0),
    Robot(6, 2, 0),
    Robot(8, 2, 0),
    Robot(10, 2, 0)
]

velocities = [1,1,1,1]
angular_velocities = [0.5, 0.5, 0.5, 0.5]

for robot in robots:
    robot.v = velocities.pop(0)
    robot.omega = angular_velocities.pop(0)
    scene.add_robot(robot)

simulation_running = False
start_time = time.perf_counter()
last_time = start_time

def update(frame):
    global simulation_running, last_time

    now = time.perf_counter()

    # Wait 3 seconds before starting
    if not simulation_running:
        if now - start_time < 3:
            return scene.get_artists()

        # Start simulation
        simulation_running = True
        last_time = now
        return scene.get_artists()

    # Compute elapsed time since last frame
    dt = now - last_time
    last_time = now

    # Update robots
    for robot in robots:
        robot.update(dt)

    scene.update(robots)

    return scene.get_artists()

ani = FuncAnimation(
    scene.fig,
    update, 
    interval=15,
    blit=True
)

plt.show()

