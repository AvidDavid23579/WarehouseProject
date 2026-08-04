import math

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import NUM_PALLETS_PER_SHELF, PALLET_LENGTH, PALLET_WIDTH, ROBOT_LENGTH, SHELF_LENGTH, SHELF_WIDTH


class Shelf:
    def __init__(self, pose: Pose) -> None:
        self.pose = pose
        self.vertices = rotated_rectangle_vertices(self.pose, SHELF_LENGTH, SHELF_WIDTH)

    def pallet_poses(self) -> list[Pose]:
        poses = []

        c = math.cos(self.pose.theta)
        s = math.sin(self.pose.theta)

        y = SHELF_WIDTH / 2 - PALLET_WIDTH / 2
        spacing = PALLET_LENGTH + 0.05

        for i in range(NUM_PALLETS_PER_SHELF):
            x = -SHELF_LENGTH / 2 + PALLET_LENGTH / 2 + i * spacing

            for y_local in (y, -y):
                wx = self.pose.x + x * c - y_local * s
                wy = self.pose.y + x * s + y_local * c

                poses.append(Pose(wx, wy, self.pose.theta))

        return poses

    def navigation_nodes(self) -> list[Pose]:
        margin = ROBOT_LENGTH * 1.2

        half_length = SHELF_LENGTH / 2
        half_width = SHELF_WIDTH / 2

        return [
            Pose(
                self.pose.x - half_width - margin,
                self.pose.y - half_length - margin,
                None,
            ),
            Pose(
                self.pose.x - half_width - margin,
                self.pose.y + half_length + margin,
                None,
            ),
            Pose(
                self.pose.x + half_width + margin,
                self.pose.y + half_length + margin,
                None,
            ),
            Pose(
                self.pose.x + half_width + margin,
                self.pose.y - half_length - margin,
                None,
            ),
        ]

    def build_graph(self, graph):
        poses = self.navigation_nodes()

        graph_nodes = []

        for pose in poses:
            graph_nodes.append(graph.add_node(pose))

        for i in range(len(graph_nodes)):
            a = graph_nodes[i]
            b = graph_nodes[(i + 1) % len(graph_nodes)]

            ax, ay, _ = graph.node_pose[a]
            bx, by, _ = graph.node_pose[b]

            dist = math.hypot(
                ax - bx,
                ay - by,
            )

            graph.add_edge(a, b, dist)
            graph.add_edge(b, a, dist)
