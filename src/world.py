import copy

from common.navigation import naive_drive_to_pose
from entities.collision import sat_collision
from entities.robot import Robot


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

    def robot_collides(self, robot):
        # Wall collision
        for x, y in robot.vertices():
            if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                return True

        # Robot collision
        for other in self.robots:
            if other is robot:
                continue

            if sat_collision(robot.vertices(), other.vertices()):
                return True

        return False

    def snapshot(self):
        return [
            {
                "id": id(robot),
                "x": robot.pose.x,
                "y": robot.pose.y,
                "theta": robot.pose.theta,
                "width": robot.width,
                "length": robot.length,
            }
            for robot in self.robots
        ]

    def step(self, dt):
        for robot in self.robots:
            if robot.reached_goal():
                robot.goals_index = (robot.goals_index + 1) % len(robot.goals)

        for robot in self.robots:
            old_pose = copy.copy(robot.pose)
            robot.update(dt, self.robots)

            if self.robot_collides(robot):
                robot.pose = old_pose
                robot.stop()
