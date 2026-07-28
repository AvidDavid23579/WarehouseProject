import math

from common.utils import rotated_rectangle_vertices


class Wall:
    def __init__(self, start_x, start_y, end_x, end_y):
        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y

        self.length = math.hypot(self.end_x - self.start_x, self.end_y - self.start_y)
        self.vertices = rotated_rectangle_vertices()
