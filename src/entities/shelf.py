from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import SHELF_LENGTH, SHELF_WIDTH


class Shelf:
    def __init__(self, pose: Pose) -> None:
        self.pose = pose
        self.vertices = rotated_rectangle_vertices(self.pose, SHELF_LENGTH, SHELF_WIDTH)
