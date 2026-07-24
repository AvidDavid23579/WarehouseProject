"""Static wall obstacles defined by a line segment and thickness."""

from common.utils import (
    Pose,
    point_to_oriented_rectangle,
    pose_from_segment,
    rotated_rectangle_vertices,
)


class Wall:
    """Finite-width wall aligned with the segment from start to end."""

    def __init__(
        self,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        width: float,
        wall_id: int | None = None,
    ):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.width = width
        self.id = wall_id if wall_id is not None else id(self)

        self.pose, segment_length = pose_from_segment(x1, y1, x2, y2)
        # Degenerate segment: treat as a small oriented square for stable geometry.
        self.length = max(segment_length, width)

        self.vertices = rotated_rectangle_vertices(
            self.pose, self.length, self.width
        )

    def distance_and_direction(
        self, px: float, py: float
    ) -> tuple[float, float, float]:
        """Shortest distance from (px, py) to the wall surface and outward direction."""
        return point_to_oriented_rectangle(
            self.pose, self.length, self.width, px, py
        )

    def clearance_from_point(
        self, px: float, py: float, body_radius: float = 0.0
    ) -> tuple[float, float, float]:
        """Signed clearance from a circular body to the wall surface."""
        surface_distance, dir_x, dir_y = self.distance_and_direction(px, py)
        return surface_distance - body_radius, dir_x, dir_y

    def snapshot(self) -> dict:
        """Serializable state for the renderer."""
        return {
            "id": self.id,
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
            "length": self.length,
            "width": self.width,
        }

    def __repr__(self) -> str:
        return (
            f"Wall(({self.x1:.2f}, {self.y1:.2f}) -> "
            f"({self.x2:.2f}, {self.y2:.2f}), width={self.width:.2f})"
        )
