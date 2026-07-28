import math

from common.types import Pose
from config import DOCK_ONE_POSE, DOCK_SPACING, NUM_DOCKS
from entities.dock import Dock


def build_docks(
    num_docks: int = NUM_DOCKS,
    spacing: float = DOCK_SPACING,
    first_node_id: int = 0,
) -> list[Dock]:

    docks = []

    next_node_id = first_node_id

    for i in range(num_docks):
        pose = Pose(
            x=DOCK_ONE_POSE.x + i * spacing,
            y=DOCK_ONE_POSE.y,
            theta=DOCK_ONE_POSE.theta,
        )

        docks.append(Dock(pose, next_node_id))
        next_node_id += 2

        docks.append(Dock(pose, next_node_id))
        next_node_id += 2

    return docks
