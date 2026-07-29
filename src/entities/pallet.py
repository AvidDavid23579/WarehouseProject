from dataclasses import dataclass

from common.types import Pose
from common.utils import rotated_rectangle_vertices
from config import PALLET_LENGTH, PALLET_WIDTH


@dataclass
class PalletFrame:
    x: float
    y: float
    theta: float


class Pallet:
    def __init__(self, pose: Pose):
        self.pose = pose

    @property
    def vertices(self):
        return rotated_rectangle_vertices(
            self.pose,
            PALLET_LENGTH,
            PALLET_WIDTH,
        )
