"""Simulation environment: entity registry, collision queries, and physics step."""

import copy

from common.avoidance import temp_goal_prio_yield
from common.collision import SpatialHash, sat_collision
from simulator.snapshot import RobotSnapshot, SimulationSnapshot


class World:
    """Owns all robots, shelves, walls and advances the simulator by one dt."""

    def __init__(self, bounds: tuple[float, float, float, float]):
        self.robots: list = []
        self.shelves: list = []
        self.walls: list = []
        self.x_min, self.x_max, self.y_min, self.y_max = bounds
        self.robot_grid = SpatialHash(cell_size=2.0)

    # --- Entity management ---------------------------------------------------

    def add_robot(self, robot) -> None:
        self.robots.append(robot)

    def add_shelf(self, shelf) -> None:
        self.shelves.append(shelf)

    def add_wall(self, wall) -> None:
        self.walls.append(wall)

    # --- Collision queries ---------------------------------------------------

    def robot_robot_collisions(self) -> list[tuple]:
        """Return all overlapping robot pairs detected via SAT."""
        collisions = []
        for i, robot_a in enumerate(self.robots):
            for robot_b in self.robots[i + 1 :]:
                if sat_collision(robot_a.robot_vertices(), robot_b.robot_vertices()):
                    collisions.append((robot_a, robot_b))
        return collisions

    def robot_boundary_collisions(self) -> list:
        """Return robots whose footprint extends outside the world bounds."""
        out_of_bounds = []
        for robot in self.robots:
            for x, y in robot.robot_vertices():
                if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                    out_of_bounds.append(robot)
                    break
        return out_of_bounds

    def robot_shelf_collisions(self) -> list:
        collisions = []
        for robot in self.robots:
            for shelf in self.shelves:
                if sat_collision(robot.robot_vertices(), shelf.vertices):
                    collisions.append(robot)
        return collisions

    def robot_wall_collisions(self) -> list:
        """Return robots overlapping any placed wall."""
        collisions = []
        for robot in self.robots:
            for wall in self.walls:
                if sat_collision(robot.robot_vertices(), wall.vertices):
                    collisions.append(robot)
                    break
        return collisions

    def robot_collides(self, robot) -> bool:
        robot_vertices = robot.robot_vertices()

        for x, y in robot_vertices:
            if x < self.x_min or x > self.x_max or y < self.y_min or y > self.y_max:
                return True

        for other in self.robot_grid.nearby(robot):
            if other is robot:
                continue

            if sat_collision(robot_vertices, other.robot_vertices()):
                return True

        for shelf in self.shelves:
            if sat_collision(robot_vertices, shelf.vertices):
                return True

        for wall in self.walls:
            if sat_collision(robot_vertices, wall.vertices):
                return True

        return False

    def capture_snapshot(self, time: float) -> SimulationSnapshot:
        """Capture only dynamic state required for rendering."""
        return SimulationSnapshot(
            time=time,
            robots=tuple(
                RobotSnapshot(
                    id=robot.id,
                    x=robot.pose.x,
                    y=robot.pose.y,
                    theta=robot.pose.theta,
                    v=robot.v,
                    omega=robot.omega,
                )
                for robot in self.robots
            ),
        )

    # --- Simulation ----------------------------------------------------------

    def step(self, dt: float) -> None:
        """Advance simulator by *dt* seconds."""
        self.robot_grid.clear()

        for robot in self.robots:
            self.robot_grid.insert(robot)

        for robot in self.robots:
            if robot.reached_goal():
                robot.goals_index = (robot.goals_index + 1) % len(robot.goals)
            temp_goal_prio_yield(robot, self.robots, offset=-0.6)

        for robot in self.robots:
            old_pose = copy.copy(robot.pose)
            robot.update(dt, self.robots, self.shelves, self.walls)

            if self.robot_collides(robot):
                robot.pose = old_pose
                robot.stop()
