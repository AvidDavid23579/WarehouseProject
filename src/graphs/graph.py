from dataclasses import dataclass

from common.types import Pose


@dataclass(slots=True)
class Node:
    id: int
    pose: Pose


@dataclass(slots=True)
class Edge:
    start: Node
    end: Node
    cost: float


class NavigationGraph:
    def __init__(self):
        self.nodes: list[Node] = []
        self.edges: list[Edge] = []

    def add_node(self, pose: Pose) -> Node:
        node = Node(id=len(self.nodes), pose=pose)
        self.nodes.append(node)
        return node

    def add_edge(self, start: Node, end: Node, cost: float):
        self.edges.append(Edge(start, end, cost))

    def get_nodes(self) -> list[Node]:
        return self.nodes
