import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


class Scene:
    def __init__(self, xlim=(0, 20), ylim=(0, 10)):
        self.fig, self.ax = plt.subplots()

        self.ax.set_xlim(*xlim)
        self.ax.set_ylim(*ylim)
        self.ax.set_aspect("equal")
        self.ax.grid(True)

        # Maps Robot objects -> Rectangle patches
        self.robot_patches = {}

    def add_robot(self, robot):
        patch = Rectangle(
            (-robot.length / 2, -robot.width / 2),
            robot.length,
            robot.width,
            facecolor="royalblue",
            edgecolor="black"
        )

        self.robot_patches[robot] = patch
        self.ax.add_patch(patch)

        transform = (
                Affine2D()
                .rotate(robot.theta)
                .translate(robot.x, robot.y)
                + self.ax.transData
            )

        patch.set_transform(transform)

    def update(self, robots):
        for robot in robots:
            patch = self.robot_patches[robot]

            transform = (
                Affine2D()
                .rotate(robot.theta)
                .translate(robot.x, robot.y)
                + self.ax.transData
            )

            patch.set_transform(transform)

    def get_artists(self):
        return tuple(self.robot_patches.values())