from dataclasses import dataclass

import numpy as np


@dataclass
class RadiationLayer:
    grid: np.ndarray

    def dose_rate(self, row: int, col: int) -> float:
        return float(self.grid[row, col])
