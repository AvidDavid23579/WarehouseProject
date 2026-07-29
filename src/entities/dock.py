from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import DOCK_LENGTH, DOCK_WIDTH


class Dock:
    def __init__(self, pose: Pose) -> None:
        self.pose = pose

        self.vertices = rotated_rectangle_vertices(
            self.pose,
            DOCK_LENGTH,
            DOCK_WIDTH,
        )

        self.approach_node = None
        self.dock_node = None

        self.robot = None
