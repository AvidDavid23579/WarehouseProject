import math
from dataclasses import dataclass, field

import numpy as np

from common.types import Pose
from config import ROBOT_LENGTH, SHELF_LENGTH, SHELF_WIDTH


@dataclass(slots=True)
class ShelfState:
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), np.float32))
    vertices: np.ndarray = field(default_factory=lambda: np.empty((0, 4, 2), np.float32))
    index: np.ndarray = field(default_factory=lambda: np.empty((0, 1), np.float32))

    def navigation_nodes(self) -> list[Pose]:
        margin = ROBOT_LENGTH * 1.2

        half_length = SHELF_LENGTH / 2
        half_width = SHELF_WIDTH / 2

        return [
            Pose(
                self.pose.x - half_width - margin,
                self.pose.y - half_length - margin,
                None,
            ),
            Pose(
                self.pose.x - half_width - margin,
                self.pose.y + half_length + margin,
                None,
            ),
            Pose(
                self.pose.x + half_width + margin,
                self.pose.y + half_length + margin,
                None,
            ),
            Pose(
                self.pose.x + half_width + margin,
                self.pose.y - half_length - margin,
                None,
            ),
        ]

    def build_graph(self, graph):
        poses = self.navigation_nodes()

        graph_nodes = []

        for pose in poses:
            graph_nodes.append(graph.add_node(pose))

        for i in range(len(graph_nodes)):
            a = graph_nodes[i]
            b = graph_nodes[(i + 1) % len(graph_nodes)]

            ax, ay, _ = graph.node_pose[a]
            bx, by, _ = graph.node_pose[b]

            dist = math.hypot(
                ax - bx,
                ay - by,
            )

            graph.add_edge(a, b, dist)
            graph.add_edge(b, a, dist)
