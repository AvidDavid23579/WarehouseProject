from collections import defaultdict

import numpy as np


class GridHash:
    def __init__(self, cell_size: float):
        self.cell_size = cell_size
        self.cells = defaultdict(list)

    def clear(self):
        self.cells.clear()

    def _cell(self, x: float, y: float):
        return (
            int(x // self.cell_size),
            int(y // self.cell_size),
        )

    def insert(self, index: int, vertices: np.ndarray):
        xmin = np.min(vertices[:, 0])
        xmax = np.max(vertices[:, 0])
        ymin = np.min(vertices[:, 1])
        ymax = np.max(vertices[:, 1])

        ix0, iy0 = self._cell(xmin, ymin)
        ix1, iy1 = self._cell(xmax, ymax)

        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                self.cells[(ix, iy)].append(index)

    def nearby(self, positions):
        result = []

        for x, y in positions[:, :2]:
            cx, cy = self._cell(x, y)

            objs = set()

            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    objs.update(self.cells.get((cx + dx, cy + dy), []))

            result.append(list(objs))

        return result
