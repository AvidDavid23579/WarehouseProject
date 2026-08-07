from common.types import Pose
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

    def build_dock_graph(self):
        self.graph.add_nodes(self.world.dock.node_pose)

    def build_subgraphs(self):
        self.build_dock_graph()
        for shelf in self.world.shelves:
            shelf.build_graph(self.graph)

    def connect_graphs(self):
        pass
