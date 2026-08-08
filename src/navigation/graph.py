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
        self.shelf_nodes = np.empty(0, dtype=np.int32)

    def add_node(self, pose: Pose) -> int:
        node_id = len(self.node_pose)
        self.node_pose = np.vstack((self.node_pose, np.array([[pose.x, pose.y, pose.theta]], dtype=np.float32)))

        return node_id

    def add_nodes(self, poses: np.ndarray) -> np.ndarray:
        start = len(self.node_pose)
        self.node_pose = np.vstack((self.node_pose, poses))

        return np.arange(start, start + len(poses), dtype=np.int32)

    def add_edge(self, start: int, end: int, cost: float):
        for edge in self.edges:
            if edge.start == start and edge.end == end:
                return

        self.edges.append(Edge(start, end, cost))

    def adjacency_list(self) -> list[list[tuple[int, float]]]:
        adjacency = [[] for _ in range(len(self.node_pose))]

        for edge in self.edges:
            adjacency[edge.start].append((edge.end, edge.cost))

        return adjacency

    def deg_node(self, node_id: int) -> int:
        degree = 0

        for edge in self.edges:
            if edge.start == node_id:
                degree += 1
            if edge.end == node_id:
                degree += 1

        return degree

    def nodes_cardinal(self, node: int):
        EPS = 1e-5

        x, y, _ = self.node_pose[node]

        left = []
        right = []
        up = []
        down = []

        for other in self.shelf_nodes:
            if other == node:
                continue

            ox, oy, _ = self.node_pose[other]

            dx = ox - x
            dy = oy - y

            # Same horizontal line
            if abs(dy) < EPS:
                if dx < -EPS:
                    left.append(other)
                elif dx > EPS:
                    right.append(other)

            # Same vertical line
            elif abs(dx) < EPS:
                if dy < -EPS:
                    down.append(other)
                elif dy > EPS:
                    up.append(other)

        # Nearest → farthest
        left.sort(key=lambda n: x - self.node_pose[n, 0])
        right.sort(key=lambda n: self.node_pose[n, 0] - x)
        down.sort(key=lambda n: y - self.node_pose[n, 1])
        up.sort(key=lambda n: self.node_pose[n, 1] - y)

        return left, right, up, down

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
