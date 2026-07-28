import math

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import DOCK_APPROACH_DISTANCE, DOCK_LENGTH, DOCK_WIDTH, ROBOT_DOCK_DIST
from graphs.node import Node


class Dock:
    def __init__(self, pose: Pose, first_node_id: int) -> None:
        self.pose = pose

        self.vertices = rotated_rectangle_vertices(
            self.pose,
            DOCK_LENGTH,
            DOCK_WIDTH,
        )

        dock_node_pose = Pose(pose.x, pose.y + ROBOT_DOCK_DIST, math.pi / 2)

        self.dock_node = Node(first_node_id, dock_node_pose)

        approach_pose = Pose(
            x=pose.x,
            y=dock_node_pose.y + DOCK_APPROACH_DISTANCE,
            theta=math.pi / 2,
        )

        self.approach_node = Node(first_node_id + 1, approach_pose)

        self.approach_node.connect(self.dock_node)

        self.robot = None
