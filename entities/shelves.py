import numpy as np

from common.utils import Pose
from config import SHELF_LENGTH, SHELF_WIDTH


class Shelf:
    def __init__(self, id, pose: Pose):
        self.pose = pose
        self.id = id

        self.width = SHELF_WIDTH
        self.length = SHELF_LENGTH

        self.hitbox_width = SHELF_WIDTH + 0.5
        self.hitbox_length = SHELF_LENGTH + 0.5

    def snapshot(self):
        return {
            "id": self.id,
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
            "length": self.length,
            "width": self.width,
        }

    def shelf_vertices(self):
        hl = self.length / 2.0
        hw = self.width / 2.0

        local_corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]

        c, s = np.cos(self.pose.theta), np.sin(self.pose.theta)
        return [(self.pose.x + lx * c - ly * s, self.pose.y + lx * s + ly * c) for lx, ly in local_corners]
