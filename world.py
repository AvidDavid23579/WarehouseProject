from collision import sat_collision


class World:
    def __init__(self, bounds):
        self.robots = []
        self.x_min, self.x_max, self.y_min, self.y_max = bounds

    def add_robot(self, robot):
        self.robots.append(robot)

    def handle_collisions(self):
        for robot_a, robot_b in self.robot_robot_collisions():
            robot_a.stop()
            robot_b.stop()

        for robot in self.robot_wall_collisions():
            robot.stop()
    
    def robot_robot_collisions(self):
        collisions = []

        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                robot_a = self.robots[i]
                robot_b = self.robots[j]

                if sat_collision(robot_a.vertices(), robot_b.vertices()):
                    collisions.append((robot_a, robot_b))
            
        return collisions
    
    def robot_wall_collisions(self):
        collisions = []

        for robot in self.robots:
            for x, y in robot.vertices():
                if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                    collisions.append(robot)
        
        return collisions

    def step(self, dt):
        self.handle_collisions()

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
