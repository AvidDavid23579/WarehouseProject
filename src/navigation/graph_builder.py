import math

from common.types import Pose
from config import ROBOT_LENGTH, SHELF_LENGTH, SHELF_WIDTH
from navigation.graph import NavigationGraph
from simulator.world import World


class GraphBuilder:
    def __init__(self, world: World):
        self.world = world
        self.graph: NavigationGraph = NavigationGraph()

    def build(self):
        self.build_subgraphs()
        self.connect_graphs()

        return self.graph

    def build_shelf_graph(self):
        margin = ROBOT_LENGTH * 1.2

        half_length = SHELF_LENGTH / 2
        half_width = SHELF_WIDTH / 2

        for x, y, _ in self.world.shelf.pose:
            poses = [
                Pose(
                    x - half_width - margin,
                    y - half_length - margin,
                    None,
                ),
                Pose(
                    x - half_width - margin,
                    y + half_length + margin,
                    None,
                ),
                Pose(
                    x + half_width + margin,
                    y + half_length + margin,
                    None,
                ),
                Pose(
                    x + half_width + margin,
                    y - half_length - margin,
                    None,
                ),
            ]

            graph_nodes = [self.graph.add_node(pose) for pose in poses]

            for i in range(4):
                a = graph_nodes[i]
                b = graph_nodes[(i + 1) % 4]

                ax, ay, _ = self.graph.node_pose[a]
                bx, by, _ = self.graph.node_pose[b]

                dist = math.hypot(ax - bx, ay - by)

                self.graph.add_edge(a, b, dist)
                self.graph.add_edge(b, a, dist)

    def build_dock_graph(self):
        self.graph.add_nodes(self.world.dock.node_pose)

    def build_subgraphs(self):
        self.build_dock_graph()
        self.build_shelf_graph()

    def connect_graphs(self):
        pass
