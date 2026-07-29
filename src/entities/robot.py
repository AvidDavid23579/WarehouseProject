from dataclasses import dataclass

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import ROBOT_LENGTH, ROBOT_WIDTH


@dataclass(slots=True)
class RobotInfo:
    id: int


@dataclass(slots=True)
class NavigationState:
    current_node_id: int | None
    target_node_id: int | None
    path: list[int]
    path_index: int


@dataclass(slots=True)
class RobotState:
    pose: Pose
    v: float
    omega: float
    crashed: bool

    last_goal_dist: float
    stuck_time: float

    vertices: list[tuple[float, float]]

    navigation: NavigationState

    def __init__(self, pose: Pose):
        self.pose = pose.copy()

        self.v = 0.0
        self.omega = 0.0

        self.crashed = False

        self.last_goal_dist = 0.0
        self.stuck_time = 0.0

        self.vertices = rotated_rectangle_vertices(
            self.pose,
            ROBOT_LENGTH,
            ROBOT_WIDTH,
        )

        self.navigation = NavigationState(
            current_node_id=None,
            target_node_id=None,
            path=[],
            path_index=0,
        )


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
        self.navigation = NavigationState(
            current_node_id=None,
            target_node_id=None,
            path=[],
            path_index=0,
        )

    def crash(self):
        pass
