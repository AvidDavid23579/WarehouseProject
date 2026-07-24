"""Static shelf obstacles placed in the warehouse floor plan."""

from common.utils import Pose, rotated_rectangle_vertices
from config import SHELF_LENGTH, SHELF_WIDTH


class Shelf:
    """Fixed rectangular obstacle; rendered but not simulated dynamically."""

    HITBOX_PADDING = 0.5

    def __init__(self, shelf_id: int, pose: Pose):
        self.id = shelf_id
        self.pose = pose

        self.width = SHELF_WIDTH
        self.length = SHELF_LENGTH

        self.hitbox_width = SHELF_WIDTH + self.HITBOX_PADDING
        self.hitbox_length = SHELF_LENGTH + self.HITBOX_PADDING

    def snapshot(self) -> dict:
        return {
            "id": self.id,
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
            "length": self.length,
            "width": self.width,
        }

    def shelf_vertices(self) -> list[tuple[float, float]]:
        return rotated_rectangle_vertices(self.pose, self.length, self.width)
