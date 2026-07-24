"""Simulation environment: entity registry, collision queries, and physics step."""

import copy

from common.avoidance import temp_goal_prio_yield
from common.collision import sat_collision


class World:
    """Owns all robots and shelves and advances the simulation by one dt."""

    def __init__(self, bounds: tuple[float, float, float, float]):
        self.robots: list = []
        self.shelves: list = []
        self.x_min, self.x_max, self.y_min, self.y_max = bounds

    # --- Entity management ---------------------------------------------------

    def add_robot(self, robot) -> None:
        self.robots.append(robot)

    def add_shelf(self, shelf) -> None:
        self.shelves.append(shelf)

    # --- Collision queries ---------------------------------------------------

    def robot_robot_collisions(self) -> list[tuple]:
        """Return all overlapping robot pairs detected via SAT."""
        collisions = []
        for i, robot_a in enumerate(self.robots):
            for robot_b in self.robots[i + 1 :]:
                if sat_collision(robot_a.robot_vertices(), robot_b.robot_vertices()):
                    collisions.append((robot_a, robot_b))
        return collisions

    def robot_wall_collisions(self) -> list:
        """Return robots whose footprint extends outside the world bounds."""
        out_of_bounds = []
        for robot in self.robots:
            for x, y in robot.robot_vertices():
                if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                    out_of_bounds.append(robot)
                    break
        return out_of_bounds

    def robot_collides(self, robot) -> bool:
        """True if *robot* overlaps a wall or any other robot."""
        for x, y in robot.robot_vertices():
            if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                return True

        for other in self.robots:
            if other is not robot and sat_collision(
                robot.robot_vertices(), other.robot_vertices()
            ):
                return True

        return False

    def snapshot(self) -> dict:
        """Current render state for all entities."""
        return {
            "robots": [robot.snapshot() for robot in self.robots],
            "shelves": [shelf.snapshot() for shelf in self.shelves],
        }

    # --- Simulation ----------------------------------------------------------

    def handle_collisions(self) -> None:
        """Hard-stop any robots involved in a collision this frame."""
        for robot_a, robot_b in self.robot_robot_collisions():
            robot_a.stop()
            robot_b.stop()
        for robot in self.robot_wall_collisions():
            robot.stop()

    def step(self, dt: float) -> None:
        """Advance simulation by *dt* seconds."""
        # Plan goals and avoidance before integrating motion.
        for robot in self.robots:
            if robot.reached_goal():
                robot.goals_index = (robot.goals_index + 1) % len(robot.goals)
            temp_goal_prio_yield(robot, self.robots, offset=-0.6)

        # Integrate each robot and revert on hard collision (safety net).
        for robot in self.robots:
            old_pose = copy.copy(robot.pose)
            robot.update(dt, self.robots)

            if self.robot_collides(robot):
                robot.pose = old_pose
                robot.stop()
