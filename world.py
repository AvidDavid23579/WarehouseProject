class World:
    def __init__(self):
        self.robots = []

    def add_robot(self, robot):
        self.robots.append(robot)

    def step(self, dt):
        for robot in self.robots:
            robot.update(dt)

    def snapshot(self):
        return [
            {
                "id": id(robot),
                "x": robot.x,
                "y": robot.y,
                "theta": robot.theta,
                "width": robot.width,
                "length": robot.length,
            }
            for robot in self.robots
        ]