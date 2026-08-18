import math

from common.types import Pose
from common.utils import segment_intersects_shelf
from config import (
    DOCK_APPROACH_DISTANCE,
    DOCK_ONE_POSE,
    DOCK_SPACING,
    GOAL_ZONE_LENGTH,
    GOAL_ZONE_WIDTH,
    NUM_DOCKS,
    NUM_PALLETS_PER_SHELF,
    PALLET_LENGTH,
    PALLET_WIDTH,
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
        margin = ROBOT_LENGTH * 1.6

        half_length = SHELF_LENGTH / 2
        half_width = SHELF_WIDTH / 2

        self.graph.shelf_nodes = []

        for x, y, _ in self.world.shelf.pose:
            poses = [
                Pose(x - half_width - margin, y - half_length - margin, None),
                Pose(x - half_width - margin, y + half_length + margin, None),
                Pose(x + half_width + margin, y + half_length + margin, None),
                Pose(x + half_width + margin, y - half_length - margin, None),
            ]

            shelf_nodes = [self.graph.add_node(pose) for pose in poses]
            self.graph.shelf_nodes.extend(shelf_nodes)

            for i in range(4):
                a = self.graph.shelf_nodes[i]
                b = self.graph.shelf_nodes[(i + 1) % 4]

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
        poses = set()

        for node in self.graph.shelf_nodes:
            if self.graph.deg_node(node) == 8:
                continue

            left, right, up, down = self.graph.nodes_cardinal(node)

            for neighbor in left:
                _, y, _ = self.graph.node_pose[neighbor]
                poses.add((DOCK_ONE_POSE.x, y))

            for neighbor in right:
                _, y, _ = self.graph.node_pose[neighbor]
                poses.add((X_MAX - DOCK_ONE_POSE.x, y))

            for neighbor in up:
                x, _, _ = self.graph.node_pose[neighbor]
                poses.add((x, Y_MAX - DOCK_ONE_POSE.y - DOCK_APPROACH_DISTANCE - ROBOT_DOCK_DIST - 0.75))

            for neighbor in down:
                x, _, _ = self.graph.node_pose[neighbor]
                poses.add((x, DOCK_ONE_POSE.y + DOCK_APPROACH_DISTANCE + ROBOT_DOCK_DIST + 0.75))

        graph_nodes = [self.graph.add_node(Pose(x, y, None)) for x, y in poses]

    def build_goal_graph(self):
        margin = ROBOT_LENGTH * 0.5

        goal_poses = []
        poses = []

        for i in range(NUM_DOCKS):
            start = (X_MAX - GOAL_ZONE_LENGTH) / 2 + (GOAL_ZONE_LENGTH / (NUM_DOCKS + 1))
            goal_poses.append(Pose(start + (GOAL_ZONE_LENGTH / (NUM_DOCKS + 1)) * i, Y_MAX - GOAL_ZONE_WIDTH - margin, None))
            poses.append(
                (
                    start + (GOAL_ZONE_LENGTH / (NUM_DOCKS + 1)) * i,
                    Y_MAX - DOCK_ONE_POSE.y - DOCK_APPROACH_DISTANCE - ROBOT_DOCK_DIST - 0.75,
                )
            )
        self.graph.goal_nodes = [self.graph.add_node(pose) for pose in goal_poses]
        graph_nodes = [self.graph.add_node(Pose(x, y, None)) for x, y in poses]

    def build_pallet_graph(self):
        self.graph.pallet_nodes = []

        margin = ROBOT_LENGTH * 0.5

        half_length = SHELF_LENGTH / 2
        half_width = SHELF_WIDTH / 2

        pallet_y = half_width - PALLET_WIDTH / 2

        for shelf_idx, (x_shelf, y_shelf, theta) in enumerate(self.world.shelf.pose):
            c = math.cos(theta)
            s = math.sin(theta)

            shelf_nodes = self.graph.shelf_nodes[shelf_idx * 4 : shelf_idx * 4 + 4]

            # 0 = top-left
            # 1 = top-right
            # 2 = bottom-right
            # 3 = bottom-left
            top_left = self.graph.node_pose[shelf_nodes[0]]
            top_right = self.graph.node_pose[shelf_nodes[1]]
            bottom_right = self.graph.node_pose[shelf_nodes[2]]
            bottom_left = self.graph.node_pose[shelf_nodes[3]]

            for i in range(NUM_PALLETS_PER_SHELF):
                x_local = -half_length + PALLET_LENGTH / 2 + i * PALLET_LENGTH

                for y_local in (pallet_y, -pallet_y):
                    # Pallet centre
                    px = x_shelf + x_local * c - y_local * s
                    py = y_shelf + x_local * s + y_local * c

                    side = 1.0 if y_local > 0 else -1.0

                    # Robot access point
                    access_dist = PALLET_WIDTH / 2 + margin

                    access_x = px - side * access_dist * s
                    access_y = py + side * access_dist * c

                    pallet_node = self.graph.add_node(Pose(access_x, access_y, None))

                    self.graph.pallet_nodes.append(pallet_node)

                    # Project onto the corresponding shelf graph edge.
                    if side > 0:
                        a = top_left
                        b = top_right
                    else:
                        a = bottom_left
                        b = bottom_right

                    # Project pallet access point onto edge AB.
                    ax, ay, _ = a
                    bx, by, _ = b

                    abx = bx - ax
                    aby = by - ay

                    t = ((access_x - ax) * abx + (access_y - ay) * aby) / (abx * abx + aby * aby)

                    t = max(0.0, min(1.0, t))

                    line_x = ax + t * abx
                    line_y = ay + t * aby

                    line_node = self.graph.add_node(Pose(line_x, line_y, theta))

    def build_subgraphs(self):
        self.build_dock_graph()
        self.build_shelf_graph()
        self.build_goal_graph()
        self.build_corner_graph()
        self.build_dock_lane_graph()
        self.build_shelf_lane_graph()
        self.build_pallet_graph()

    def visible(self, node_a: int, node_b: int) -> bool:
        p1 = self.graph.node_pose[node_a, :2]
        p2 = self.graph.node_pose[node_b, :2]

        for vertices in self.world.shelf.vertices:
            if segment_intersects_shelf(p1, p2, vertices):
                return False

        return True

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

                if i in self.graph.pallet_nodes and j in self.graph.pallet_nodes:
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
                if node is None:
                    continue

                if not self.visible(i, node):
                    continue

                self.graph.add_edge(i, node, distance)
                self.graph.add_edge(node, i, distance)
