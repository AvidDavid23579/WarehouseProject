import numpy as np


class Navigator:
    def __init__(self, graph):
        self.graph = graph

    def update(self, robot) -> None:
        for r, path in enumerate(robot.path):
            if not path:
                robot.target_node_id[r] = -1
                continue

            i = robot.path_index[r]

            if i >= len(path):
                robot.target_node_id[r] = -1
                continue

            robot.target_node_id[r] = path[i]
