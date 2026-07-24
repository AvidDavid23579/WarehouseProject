# Only contains anything concerning robot kinematics, dynamics and hitboxes

import math

import numpy as np

from common.navigation import naive_drive_to_pose
from common.potential import apply_repulsion
from common.utils import Pose, wrap_angle
from config import ANGLE_TOLERANCE, DIST_TOLERANCE, MAX_OMEGA, ROBOT_LENGTH, ROBOT_WIDTH


class Robot:
    def __init__(self, pose: Pose, goals, id):
        self.pose = pose

        self.width = ROBOT_WIDTH  # Graphic only
        self.length = ROBOT_LENGTH  # Graphic only

        self.hitbox_width = ROBOT_WIDTH * 1.5
        self.hitbox_length = ROBOT_LENGTH * 1.5

        self.goals = goals
        self.goals_index = 0
        self.temp_goal = None

        self.id = id

        self.v = 0.0
        self.omega = 0.0

    def snapshot(self):
        return {
            "id": self.id,
            "x": self.pose.x,
            "y": self.pose.y,
            "theta": self.pose.theta,
            "length": self.length,
            "width": self.width,
        }

    @property
    def goal(self):
        return self.goals[self.goals_index]

    @property
    def target(self):
        return self.temp_goal if self.temp_goal is not None else self.goal

    def robot_vertices(self):
        hl = self.length / 2.0
        hw = self.width / 2.0

        local_corners = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]

        c, s = np.cos(self.pose.theta), np.sin(self.pose.theta)
        return [(self.pose.x + lx * c - ly * s, self.pose.y + lx * s + ly * c) for lx, ly in local_corners]

    def hitbox_vertices(self):
        hl = self.hitbox_length / 2
        hw = self.hitbox_width / 2

        c, s = np.cos(self.pose.theta), np.sin(self.pose.theta)
        return [(self.pose.x + lx * c - ly * s, self.pose.y + lx * s + ly * c) for lx, ly in [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]]

    def rotate(self, theta):
        error = wrap_angle(theta - self.pose.theta)

        self.v = 0.0
        self.omega = max(-MAX_OMEGA, min(5 * error, MAX_OMEGA))

        return abs(error) < ANGLE_TOLERANCE

    def drive(self, robots):
        self.v, self.omega = naive_drive_to_pose(self.pose, self.target, k_p_dist=5.0, k_p_heading=5.0, k_p_final=10.0)
        apply_repulsion(self, robots, MAX_OMEGA)

    def update(self, dt, robots):
        self.drive(robots)

        self.pose.x += self.v * np.cos(self.pose.theta) * dt
        self.pose.y += self.v * np.sin(self.pose.theta) * dt
        self.pose.theta += self.omega * dt

    def stop(self):
        self.v = 0
        self.omega = 0

    def reached_goal(self):
        goal = self.goal
        position_error = math.hypot(goal.x - self.pose.x, goal.y - self.pose.y)
        heading_error = abs(wrap_angle(goal.theta - self.pose.theta))
        return position_error < DIST_TOLERANCE and heading_error < ANGLE_TOLERANCE
