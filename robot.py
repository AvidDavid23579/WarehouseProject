import numpy as np

class Robot:
    # Differential drive robot parameters
    def __init__(self, x, y, theta, width=0.4, length=0.6):
        self.x = x
        self.y = y
        self.theta = theta

        self.width = width   # Graphic only
        self.length = length # Graphic only

        self.v = 0.0
        self.omega = 0.0

    def update(self, dt):
        self.x += self.v * np.cos(self.theta) * dt
        self.y += self.v * np.sin(self.theta) * dt
        self.theta += self.omega * dt



