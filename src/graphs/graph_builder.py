import math

from common.types import Pose
from config import DOCK_APPROACH_DISTANCE, ROBOT_DOCK_DIST
from graphs.graph import NavigationGraph
from simulator.world import World


class GraphBuilder:
    def __init__(self, world: World):
        self.world = world
        self.graph: NavigationGraph = NavigationGraph()

    def build(self) -> NavigationGraph:
        self.build_docks_graph()
        return self.graph

    def build_docks_graph(self):
        for dock in self.world.docks:
            dock_node = self.graph.add_node(Pose(dock.pose.x, dock.pose.y + ROBOT_DOCK_DIST, math.pi / 2))
            approach = self.graph.add_node(
                Pose(
                    dock.pose.x,
                    dock.pose.y + ROBOT_DOCK_DIST + DOCK_APPROACH_DISTANCE,
                    math.pi / 2,
                )
            )

            dock.approach_node = approach
            dock.dock_node = dock_node

            self.graph.add_edge(approach, dock_node, ROBOT_DOCK_DIST)
            self.graph.add_edge(dock_node, approach, ROBOT_DOCK_DIST)
