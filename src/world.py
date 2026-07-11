# Handles events happening inside the simulation environment

import copy

from common.avoidance import temp_goal_prio_yield
from common.collision import sat_collision


class World:
    def __init__(self, bounds):
        self.robots = []
        self.x_min, self.x_max, self.y_min, self.y_max = bounds

    def add_robot(self, robot):
        self.robots.append(robot)

    # --- Queries -----------------------------------------------------

    def robot_robot_collisions(self):
        collisions = []
        for i in range(len(self.robots)):
            for j in range(i + 1, len(self.robots)):
                a, b = self.robots[i], self.robots[j]
                if sat_collision(a.vertices(), b.vertices()):
                    collisions.append((a, b))
        return collisions

    def robot_wall_collisions(self):
        collisions = []
        for robot in self.robots:
            for x, y in robot.vertices():
                if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                    collisions.append(robot)
        return collisions

    def robot_collides(self, robot):
        for x, y in robot.vertices():
            if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                return True

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

    # --- Simulation loop -----------------------------------------------

    def handle_collisions(self):
        for robot_a, robot_b in self.robot_robot_collisions():
            robot_a.stop()
            robot_b.stop()
        for robot in self.robot_wall_collisions():
            robot.stop()

    def step(self, dt):
        # Goal switching + discrete avoidance planning (must run before
        # update(), since drive() reads robot.target)
        for robot in self.robots:
            if robot.reached_goal():
                robot.goals_index = (robot.goals_index + 1) % len(robot.goals)
            temp_goal_prio_yield(robot, self.robots, offset=-0.6)

        # Integrate + hard-revert on actual collision (safety net; should
        # rarely fire if repulsion is tuned well — see design-notes.md)
        for robot in self.robots:
            old_pose = copy.copy(robot.pose)
            robot.update(dt, self.robots)

            if self.robot_collides(robot):
                robot.pose = old_pose
                robot.stop()
