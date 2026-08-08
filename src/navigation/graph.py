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

        self.dock_nodes = np.empty(0, dtype=np.int32)
        self.corner_nodes = np.empty(0, dtype=np.int32)

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

    def adjacency_list(self) -> list[list[tuple[int, float]]]:
        adjacency = [[] for _ in range(len(self.node_pose))]

        for edge in self.edges:
            adjacency[edge.start].append((edge.end, edge.cost))

        return adjacency

    def dfs(self, start: int, goal: int) -> list[int] | None:
        adjacency = self.adjacency_list()

        visited = set()
        parent = {start: None}

        stack = [start]

        while stack:
            current = stack.pop()

            if current in visited:
                continue

            visited.add(current)

            if current == goal:
                break

            for neighbor, _ in adjacency[current]:
                if neighbor not in visited:
                    parent[neighbor] = current
                    stack.append(neighbor)

        if goal not in parent:
            return None

        path = []
        current = goal

        while current is not None:
            path.append(current)
            current = parent[current]

        path.reverse()

        return path
