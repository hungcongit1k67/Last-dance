from dataclasses import dataclass

import numpy as np

from src.core.constants import OBSTACLE


@dataclass
class ObstacleLayer:
    grid: np.ndarray

    def is_obstacle(self, row: int, col: int) -> bool:
        return int(self.grid[row, col]) == OBSTACLE
