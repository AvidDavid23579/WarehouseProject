import math

from common.types import Pose
from config import (
    DOCK_APPROACH_DISTANCE,
    DOCK_ONE_POSE,
    DOCK_SPACING,
    NUM_DOCKS,
    ROBOT_DOCK_DIST,
    ROBOT_LENGTH,
    SHELF_LENGTH,
    SHELF_WIDTH,
    X_MAX,
    Y_MAX,
)
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
                Pose(x - half_width - margin, y - half_length - margin, None),
                Pose(x - half_width - margin, y + half_length + margin, None),
                Pose(x + half_width + margin, y + half_length + margin, None),
                Pose(x + half_width + margin, y - half_length - margin, None),
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
        self.graph.dock_nodes = self.graph.add_nodes(self.world.dock.node_pose)

    def build_corner_graph(self):
        x = DOCK_ONE_POSE.x
        y = DOCK_ONE_POSE.y

        margin = 0.75

        poses = [
            Pose(x, y + DOCK_APPROACH_DISTANCE + ROBOT_DOCK_DIST + margin, None),
            Pose(x, Y_MAX - y - DOCK_APPROACH_DISTANCE - ROBOT_DOCK_DIST - margin, None),
            Pose(X_MAX - x, Y_MAX - y - DOCK_APPROACH_DISTANCE - ROBOT_DOCK_DIST - margin, None),
            Pose(X_MAX - x, y + DOCK_APPROACH_DISTANCE + ROBOT_DOCK_DIST + margin, None),
        ]

        self.graph.corner_nodes = [self.graph.add_node(pose) for pose in poses]

    def build_dock_lane_graph(self):
        poses = []
        for i in range(NUM_DOCKS):
            if i == 0:
                continue
            else:
                poses.append(
                    Pose(DOCK_ONE_POSE.x + i * DOCK_SPACING, DOCK_ONE_POSE.y + DOCK_APPROACH_DISTANCE + ROBOT_DOCK_DIST + 0.75, None)
                )
        graph_nodes = [self.graph.add_node(pose) for pose in poses]

    def build_shelf_lane_graph(self):
        poses = []

    def build_subgraphs(self):
        self.build_dock_graph()
        self.build_shelf_graph()
        self.build_corner_graph()
        self.build_dock_lane_graph()

    def connect_graphs(self):
        EPS = 1e-5

        nodes = self.graph.node_pose
        n = len(nodes)

        for i in range(n):
            x, y, _ = nodes[i]

            nearest = {
                "left": (None, math.inf),
                "right": (None, math.inf),
                "up": (None, math.inf),
                "down": (None, math.inf),
            }

            for j in range(n):
                if i == j:
                    continue

                if i < NUM_DOCKS and j < NUM_DOCKS:
                    continue

                x2, y2, _ = nodes[j]

                dx = x2 - x
                dy = y2 - y

                # Same horizontal line
                if abs(dy) < EPS:
                    distance = abs(dx)

                    if dx < -EPS and distance < nearest["left"][1]:
                        nearest["left"] = (j, distance)

                    elif dx > EPS and distance < nearest["right"][1]:
                        nearest["right"] = (j, distance)

                # Same vertical line
                elif abs(dx) < EPS:
                    distance = abs(dy)

                    if dy < -EPS and distance < nearest["down"][1]:
                        nearest["down"] = (j, distance)

                    elif dy > EPS and distance < nearest["up"][1]:
                        nearest["up"] = (j, distance)

            for node, distance in nearest.values():
                if node is not None:
                    self.graph.add_edge(i, node, distance)
                    self.graph.add_edge(node, i, distance)
