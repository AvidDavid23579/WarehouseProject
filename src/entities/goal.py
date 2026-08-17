from config import GOAL_ZONE_LENGTH, GOAL_ZONE_WIDTH, X_MAX, Y_MAX
from geometry.geo_compute import obb_vertices
from common.types import Pose


class GoalZone:
    def __init__(self):
        self.length = GOAL_ZONE_LENGTH
        self.width = GOAL_ZONE_WIDTH
        self.vertices = obb_vertices(Pose(X_MAX / 2, Y_MAX - self.width / 2, 0.0), self.length, self.width)
