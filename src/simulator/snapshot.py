"""Immutable snapshot types for recording and playback."""

from dataclasses import dataclass

from config import ROBOT_LENGTH, ROBOT_WIDTH


@dataclass(frozen=True, slots=True)
class RobotSnapshot:
    """Dynamic state of a single robot at one instant in time."""

    id: int
    x: float
    y: float
    theta: float
    v: float
    omega: float


@dataclass(frozen=True, slots=True)
class SimulationSnapshot:
    """Dynamic world state at a single simulator time."""

    time: float
    robots: tuple[RobotSnapshot, ...]


@dataclass(frozen=True, slots=True)
class StaticEntityView:
    """Render-only description of a static obstacle."""

    id: int
    x: float
    y: float
    theta: float
    length: float
    width: float


@dataclass(frozen=True, slots=True)
class StaticWorld:
    """Static scene geometry captured once before playback."""

    bounds: tuple[float, float, float, float]
    shelves: tuple[StaticEntityView, ...]
    walls: tuple[StaticEntityView, ...]
    robot_length: float = ROBOT_LENGTH
    robot_width: float = ROBOT_WIDTH

    @classmethod
    def from_world(cls, world) -> "StaticWorld":
        """Extract immutable static geometry from a live world."""
        return cls(
            bounds=(world.x_min, world.x_max, world.y_min, world.y_max),
            shelves=tuple(
                StaticEntityView(
                    id=shelf.id,
                    x=shelf.pose.x,
                    y=shelf.pose.y,
                    theta=shelf.pose.theta,
                    length=shelf.length,
                    width=shelf.width,
                )
                for shelf in world.shelves
            ),
            walls=tuple(
                StaticEntityView(
                    id=wall.id,
                    x=wall.pose.x,
                    y=wall.pose.y,
                    theta=wall.pose.theta,
                    length=wall.length,
                    width=wall.width,
                )
                for wall in world.walls
            ),
        )
