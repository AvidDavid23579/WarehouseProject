"""Matplotlib renderer for warehouse entities using blitted rectangle patches."""

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D


class Renderer:
    """Draws robots and shelves as rotated rectangles on a fixed grid."""

    ENTITY_COLORS = {
        "robots": "royalblue",
        "shelves": "gray",
    }

    def __init__(self, bounds: tuple[float, float, float, float]):
        self.fig, self.ax = plt.subplots()

        x_min, x_max, y_min, y_max = bounds
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_aspect("equal")
        self.ax.grid(True)

        self._patches: dict[str, dict] = {name: {} for name in self.ENTITY_COLORS}

    def _make_patch(self, state: dict, color: str) -> Rectangle:
        """Create a centered rectangle patch in body-local coordinates."""
        patch = Rectangle(
            (-state["length"] / 2, -state["width"] / 2),
            state["length"],
            state["width"],
            facecolor=color,
            edgecolor="black",
        )
        self.ax.add_patch(patch)
        return patch

    def _apply_transform(self, patch: Rectangle, state: dict) -> None:
        """Rotate around origin then translate to world position."""
        transform = Affine2D().rotate(state["theta"]).translate(state["x"], state["y"]) + self.ax.transData
        patch.set_transform(transform)

    def _draw_entities(self, entity_type: str, entities: list[dict]) -> None:
        """Create or update patches for one entity category."""
        patches = self._patches[entity_type]
        color = self.ENTITY_COLORS[entity_type]

        for state in entities:
            entity_id = state["id"]
            if entity_id not in patches:
                patches[entity_id] = self._make_patch(state, color)
            self._apply_transform(patches[entity_id], state)

    def draw(self, snapshot: dict):
        """Update all entity patches from a world snapshot."""
        for entity_type, entities in snapshot.items():
            self._draw_entities(entity_type, entities)
        return self.get_artists()

    def get_artists(self) -> tuple:
        """Return every patch for matplotlib blitting."""
        artists = []
        for patches in self._patches.values():
            artists.extend(patches.values())
        return tuple(artists)
