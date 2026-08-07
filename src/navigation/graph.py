from dataclasses import dataclass

import numpy as np

from common.types import Pose


@dataclass(slots=True)
class Edge:
    start: int
    end: int
    cost: float


class NavigationGraph:
    def __init__(self):
        self.node_pose = np.empty((0, 3), dtype=np.float32)
        self.edges: list[Edge] = []

    def add_node(self, pose: Pose) -> int:
        node_id = len(self.node_pose)
        self.node_pose = np.vstack((self.node_pose, np.array([[pose.x, pose.y, pose.theta]], dtype=np.float32)))

        return node_id

    def add_nodes(self, poses: np.ndarray) -> np.ndarray:
        start = len(self.node_pose)
        self.node_pose = np.vstack((self.node_pose, poses))

        return np.arange(start, start + len(poses), dtype=np.int32)

    def add_edge(self, start: int, end: int, cost: float):
        self.edges.append(Edge(start, end, cost))
