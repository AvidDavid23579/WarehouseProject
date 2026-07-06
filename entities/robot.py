import numpy as np


class Robot:
    # Differential drive robot parameters
    def __init__(self, x, y, theta, width=0.4, length=0.6):
        self.x = x # Center
        self.y = y
        self.theta = theta

        self.width = width  # Graphic only
        self.length = length  # Graphic only

        self.v = 0.0
        self.omega = 0.0

    def vertices(self):
        hl = self.length / 2.0
        hw = self.width / 2.0

        # Corners in the robot's local frame (x forward, y left)
        local_corners = [
            (hl, hw),
            (hl, -hw),
            (-hl, -hw),
            (-hl, hw),
        ]

        c, s = np.cos(self.theta), np.sin(self.theta)
        world_corners = []
        for lx, ly in local_corners:
            wx = self.x + lx * c - ly * s
            wy = self.y + lx * s + ly * c
            world_corners.append((wx, wy))

        return world_corners

    def update(self, dt):
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.omega * dt

    def stop(self):
        self.v = 0
        self.omega = 0