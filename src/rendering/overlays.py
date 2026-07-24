"""Optional debug overlays for the renderer."""

from abc import ABC, abstractmethod

from simulator.snapshot import SimulationSnapshot, StaticWorld


class RenderOverlay(ABC):
    """Base class for optional visualization layers drawn on top of the scene."""

    @abstractmethod
    def draw(self, ax, static_world: StaticWorld, snapshot: SimulationSnapshot) -> list:
        """Update overlay artists and return them for blitting/redraw."""

    def clear(self, ax) -> None:
        """Remove overlay artists when the overlay is disabled."""
