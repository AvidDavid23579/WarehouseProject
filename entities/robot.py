import math
from dataclasses import dataclass

import numpy as np

from common.utils import wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, MAX_VELOCITY
from entities.collision import sat_collision
from src.config import ROBOT_LENGTH, ROBOT_WIDTH


@dataclass
class Pose:
    x: float
    y: float
    theta: float


class Robot:
    # Differential drive robot parameters
    def __init__(self, pose: Pose, goals: Pose, prio):
        self.pose = pose

        self.width = ROBOT_WIDTH  # Graphic only
        self.length = ROBOT_LENGTH  # Graphic only

        self.hitbox_width = ROBOT_WIDTH * 2.5
        self.hitbox_length = ROBOT_LENGTH * 2.5

        self.goals = goals
        self.goals_index = 0

        self.prio = prio

        self.v = 0.0
        self.omega = 0.0

        # Assigns goals for the robot

    @property
    def goal(self):
        return self.goals[self.goals_index]

    # Returns the physical robot's 4 vertices
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

        c, s = np.cos(self.pose.theta), np.sin(self.pose.theta)
        world_corners = []
        for lx, ly in local_corners:
            wx = self.pose.x + lx * c - ly * s
            wy = self.pose.y + lx * s + ly * c
            world_corners.append((wx, wy))

        return world_corners

    # Returns the robot's hitbox vertices (to avoid each other)
    def hitbox_vertices(self):
        hl = self.hitbox_length / 2
        hw = self.hitbox_width / 2

        c, s = np.cos(self.pose.theta), np.sin(self.pose.theta)

        return [
            (
                self.pose.x + lx * c - ly * s,
                self.pose.y + lx * s + ly * c,
            )
            for lx, ly in [
                (hl, hw),
                (hl, -hw),
                (-hl, -hw),
                (-hl, hw),
            ]
        ]

    def rotate(self, theta):
        error = wrap_angle(theta - self.pose.theta)

        self.v = 0.0
        self.omega = 5 * error

        # Saturate
        self.omega = max(-MAX_OMEGA, min(self.omega, MAX_OMEGA))

        return abs(error) < ANGLE_TOLERANCE

    # Update the robot's pose at each timestep
    def update(self, dt):
        self.pose.x += self.v * np.cos(self.pose.theta) * dt
        self.pose.y += self.v * np.sin(self.pose.theta) * dt
        self.pose.theta += self.omega * dt

    # Stop the robot
    def stop(self):
        self.v = 0
        self.omega = 0

    # Check if goal is reached
    def reached_goal(self):
        goal = self.goal

        position_error = math.hypot(
            goal.x - self.pose.x,
            goal.y - self.pose.y,
        )

        heading_error = abs(wrap_angle(goal.theta - self.pose.theta))

        return position_error < DIST_TOLERANCE and heading_error < ANGLE_TOLERANCE

    # Simple priority yielding logic
    def stop_prio_yield(self, robots):
        for other in robots:
            if other is self:
                continue

            if not sat_collision(
                self.hitbox_vertices(),
                other.hitbox_vertices(),
            ):
                continue

            # Lower priority robot yields.
            if self.prio < other.prio:
                self.stop()
                return

    def turn_prio_yield(self, robots):
        for other in robots:
            if other is self:
                continue

            if not sat_collision(
                self.hitbox_vertices(),
                other.hitbox_vertices(),
            ):
                continue

            # Lower priority robot yields.
            if self.prio < other.prio:
                dx = other.pose.x - self.pose.x
                dy = other.pose.y - self.pose.y

                bearing = math.atan2(dy, dx)

                left = wrap_angle(bearing + math.pi / 2)
                right = wrap_angle(bearing - math.pi / 2)

                self.rotate(left)
                other.rotate(left)
                return
