import math

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import DOCK_APPROACH_DISTANCE, DOCK_LENGTH, DOCK_WIDTH, ROBOT_DOCK_DIST


class Dock:
    def __init__(self, pose: Pose) -> None:
        self.pose = pose

        self.vertices = rotated_rectangle_vertices(
            self.pose,
            DOCK_LENGTH,
            DOCK_WIDTH,
        )

        self.node_pose = Pose(self.pose.x, self.pose.y + ROBOT_DOCK_DIST + DOCK_APPROACH_DISTANCE, math.pi / 2)
        self.robot = None

    def build_graph(self, graph):
        graph.add_node(self.node_pose)
