from types import RobotInfo, RobotState


class Robot:
    def __init__(self, info: RobotInfo, state: RobotState) -> None:
        self.info = info
        self.state = state
