from dataclasses import dataclass, field

import numpy as np

from common.types import Pose
from config import NUM_DOCKS


@dataclass(slots=True)
class RobotInfo:
    id: int


@dataclass(slots=True)
class RobotState:
    # Physics
    pose: np.ndarray = field(default_factory=lambda: np.empty((0, 3), dtype=np.float32))
    twist: np.ndarray = field(default_factory=lambda: np.empty((0, 2), dtype=np.float32))

    # Collision
    crashed: np.ndarray = field(default_factory=lambda: np.empty(0, dtype=np.bool_))
    vertices: np.ndarray = field(default_factory=lambda: np.empty((0, 4, 2), dtype=np.float32))

    # Controller
    arrived: np.ndarray = field(default_factory=lambda: np.ones(NUM_DOCKS, dtype=np.bool_))
    last_goal_dist: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.float32))
    stuck_time: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.float32))
    repulsion_force: np.ndarray = field(default_factory=lambda: np.zeros((NUM_DOCKS, 2), dtype=np.float32))

    # Navigation
    current_node_id: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.int32))
    target_node_id: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.int32))
    path_index: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.int32))
    nav_phase: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.int8))
    path_length: np.ndarray = field(default_factory=lambda: np.empty(NUM_DOCKS, dtype=np.int32))
    path: list[list[int]] = field(default_factory=list)


@dataclass(slots=True)
class RobotFrame:
    x: float
    y: float
    theta: float


class Robot:
    def __init__(self, info: RobotInfo, start_pose: Pose):
        self.info = info
        self.start_pose = start_pose.copy()

        self.goal = None

    def crash(self):
        pass
