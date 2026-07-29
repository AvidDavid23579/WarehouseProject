import math

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import PALLET_LENGTH, PALLET_WIDTH, SHELF_LENGTH, SHELF_WIDTH


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

        for i in range(15):
            x = -SHELF_LENGTH / 2 + PALLET_LENGTH / 2 + i * spacing

            for y_local in (y, -y):
                wx = self.pose.x + x * c - y_local * s
                wy = self.pose.y + x * s + y_local * c

                poses.append(Pose(wx, wy, self.pose.theta))

        return poses