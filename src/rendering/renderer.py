"""Matplotlib renderer driven by playback snapshots and static world geometry."""

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.transforms import Affine2D

from rendering.overlays import RenderOverlay
from simulator.playback import Playback
from simulator.snapshot import SimulationSnapshot, StaticEntityView, StaticWorld


class Renderer:
    """Visualizes a static scene plus dynamic robot snapshots.

    Rendering is driven entirely by snapshot data — no simulator access.
    Optional :class:`RenderOverlay` subclasses can be plugged in for debug views.
    """

    ENTITY_COLORS = {
        "robots": "royalblue",
        "shelves": "gray",
        "walls": "dimgray",
    }

    def __init__(
        self,
        static_world: StaticWorld,
        *,
        overlays: list[RenderOverlay] | None = None,
        fig: plt.Figure | None = None,
        ax: plt.Axes | None = None,
    ):
        self.static_world = static_world
        self._overlays = overlays or []
        self._overlay_artists: list = []

        if fig is None or ax is None:
            fig, ax = plt.subplots()
        self.fig = fig
        self.ax = ax

        x_min, x_max, y_min, y_max = static_world.bounds
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_aspect("equal")
        self.ax.grid(True)

        self._robot_length = static_world.robot_length
        self._robot_width = static_world.robot_width
        self._patches: dict[str, dict] = {name: {} for name in self.ENTITY_COLORS}

        self._draw_static_entities(static_world.shelves, "shelves")
        self._draw_static_entities(static_world.walls, "walls")

    def _entity_state(self, entity: StaticEntityView) -> dict:
        return {
            "id": entity.id,
            "x": entity.x,
            "y": entity.y,
            "theta": entity.theta,
            "length": entity.length,
            "width": entity.width,
        }

    def _robot_state(self, robot) -> dict:
        return {
            "id": robot.id,
            "x": robot.x,
            "y": robot.y,
            "theta": robot.theta,
            "length": self._robot_length,
            "width": self._robot_width,
        }

    def _make_patch(self, state: dict, color: str) -> Rectangle:
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
        transform = Affine2D().rotate(state["theta"]).translate(state["x"], state["y"]) + self.ax.transData
        patch.set_transform(transform)

    def _draw_entities(self, entity_type: str, entities) -> None:
        patches = self._patches[entity_type]
        color = self.ENTITY_COLORS[entity_type]

        for entity in entities:
            state = self._robot_state(entity) if entity_type == "robots" else self._entity_state(entity)
            entity_id = state["id"]
            if entity_id not in patches:
                patches[entity_id] = self._make_patch(state, color)
            self._apply_transform(patches[entity_id], state)

    def _draw_static_entities(self, entities: tuple[StaticEntityView, ...], entity_type: str) -> None:
        self._draw_entities(entity_type, entities)

    def _draw_overlays(self, snapshot: SimulationSnapshot) -> None:
        for artist in self._overlay_artists:
            artist.remove()
        self._overlay_artists.clear()

        for overlay in self._overlays:
            self._overlay_artists.extend(overlay.draw(self.ax, self.static_world, snapshot))

    def render(self, snapshot: SimulationSnapshot):
        """Draw the given dynamic snapshot and any active overlays."""
        self._draw_entities("robots", snapshot.robots)
        self._draw_overlays(snapshot)
        return self.get_artists()

    def render_playback(self, playback: Playback):
        """Render the playback's current snapshot."""
        return self.render(playback.current)

    def get_artists(self) -> tuple:
        artists = []
        for patches in self._patches.values():
            artists.extend(patches.values())
        artists.extend(self._overlay_artists)
        return tuple(artists)
