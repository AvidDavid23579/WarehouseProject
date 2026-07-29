import math

from common.types import Pose
from config import DOCK_ONE_POSE, DOCK_SPACING, NUM_DOCKS, ROBOT_DOCK_DIST, SHELF_COL, SHELF_ROW
from entities.dock import Dock
from entities.pallet import Pallet
from entities.robot import Robot, RobotInfo
from entities.shelf import Shelf


def build_docks(
    num_docks: int = NUM_DOCKS,
    spacing: float = DOCK_SPACING,
) -> tuple[list[Dock], list[Robot]]:

    docks = []
    robots = []

    for i in range(num_docks):
        dock_pose = Pose(
            x=DOCK_ONE_POSE.x + i * spacing,
            y=DOCK_ONE_POSE.y,
            theta=DOCK_ONE_POSE.theta,
        )

        dock = Dock(dock_pose)

        info = RobotInfo(id=i)

        robot_pose = Pose(
            x=DOCK_ONE_POSE.x + i * spacing,
            y=DOCK_ONE_POSE.y + ROBOT_DOCK_DIST,
            theta=math.pi / 2,
        )
        robot = Robot(info, robot_pose)
        dock.robot = robot

        docks.append(dock)
        robots.append(robot)

    return docks, robots


def build_shelves_vertical(shelves_col: int = SHELF_COL, shelves_row: int = SHELF_ROW) -> tuple[list[Shelf], list[Pallet]]:

    shelves = []
    pallets = []
