from __future__ import annotations

from common.types import Pose


class Node:
    def __init__(self, id: int, pose: Pose) -> None:
        self.id = id
        self.pose = pose
        self.neighbors: list[Node] = []

    def connect(self, other: "Node", bidirectional: bool = True) -> None:
        if other not in self.neighbors:
            self.neighbors.append(other)

        if bidirectional:
            other.connect(self, False)

    def disconnect(self, other: "Node", bidirectional: bool = True) -> None:
        if other in self.neighbors:
            self.neighbors.remove(other)

        if bidirectional:
            other.disconnect(self, False)
