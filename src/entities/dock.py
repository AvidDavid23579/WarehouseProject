import math

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import DOCK_APPROACH_DISTANCE, DOCK_LENGTH, DOCK_WIDTH
from graphs.node import Node


class Dock:
    def __init__(self, pose: Pose, first_node_id: int) -> None:
        self.pose = pose

        self.vertices = rotated_rectangle_vertices(
            self.pose,
            DOCK_LENGTH,
            DOCK_WIDTH,
        )

        self.dock_node = Node(first_node_id, pose)

        approach_pose = Pose(
            x=pose.x - DOCK_APPROACH_DISTANCE * math.cos(pose.theta),
            y=pose.y - DOCK_APPROACH_DISTANCE * math.sin(pose.theta),
            theta=pose.theta,
        )

        self.approach_node = Node(first_node_id + 1, approach_pose)

        self.approach_node.connect(self.dock_node)

        self.robot = None
