# Only calls matplotlib to render to the viewport

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


class Renderer:
    # Creates the warehouse scene and draws robots as rectangles
    def __init__(self, bounds):
        self.fig, self.ax = plt.subplots()
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.set_aspect("equal")
        self.ax.grid(True)

        self._patches = {}  # snapshot id -> Rectangle patch

    # Draws the robots as rectangles
    def _make_patch(self, state):
        patch = Rectangle(
            (-state["length"] / 2, -state["width"] / 2),
            state["length"],
            state["width"],
            facecolor="royalblue",
            edgecolor="black",
        )
        self.ax.add_patch(patch)
        return patch

    # Enable graphical robot movement
    def _apply_transform(self, patch, state):
        patch.set_transform(
            Affine2D().rotate(state["theta"]).translate(state["x"], state["y"])
            + self.ax.transData
        )

    # Draws the motion
    def draw(self, snapshot):
        for state in snapshot:
            rid = state["id"]
            if rid not in self._patches:  # Creates a robot if it doesn't exist
                self._patches[rid] = self._make_patch(state)
            self._apply_transform(
                self._patches[rid], state
            )  # Draws the robot and the motion
        return self.get_artists()

    # Return objects that get updated
    def get_artists(self):
        return tuple(self._patches.values())
