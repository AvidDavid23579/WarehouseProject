import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


class Renderer:
    def __init__(self, bounds):
        self.fig, self.ax = plt.subplots()

        x_min, x_max, y_min, y_max = bounds
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_aspect("equal")
        self.ax.grid(True)

        self._patches = {
            "robots": {},
            "shelves": {},
        }

        self._colors = {
            "robots": "royalblue",
            "shelves": "peru",
        }

    # ==========================================================
    # Patch creation
    # ==========================================================

    def _make_patch(self, state, color):
        patch = Rectangle(
            (-state["length"] / 2, -state["width"] / 2),
            state["length"],
            state["width"],
            facecolor=color,
            edgecolor="black",
        )

        self.ax.add_patch(patch)
        return patch

    # ==========================================================
    # Common transform
    # ==========================================================

    def _apply_transform(self, patch, state):
        patch.set_transform(Affine2D().rotate(state["theta"]).translate(state["x"], state["y"]) + self.ax.transData)

    # ==========================================================
    # Generic drawing
    # ==========================================================

    def _draw_entities(self, name, entities):
        patches = self._patches[name]
        color = self._colors[name]

        for state in entities:
            eid = state["id"]

            if eid not in patches:
                patches[eid] = self._make_patch(state, color)

            self._apply_transform(patches[eid], state)

    # ==========================================================
    # Public API
    # ==========================================================

    def draw(self, snapshot):
        for name, entities in snapshot.items():
            self._draw_entities(name, entities)

        return self.get_artists()

    def get_artists(self):
        artists = []

        for patches in self._patches.values():
            artists.extend(patches.values())

        return tuple(artists)
