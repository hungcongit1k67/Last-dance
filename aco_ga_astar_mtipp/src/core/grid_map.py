from __future__ import annotations

from dataclasses import dataclass
from math import sqrt
from typing import Iterable, List, Sequence, Tuple

import numpy as np

from src.core.constants import OBSTACLE, TARGET
from src.core.target import Target

GridPosition = Tuple[int, int]


@dataclass
class GridMap:
    """Two-layer map: obstacle/target layer + radiation layer."""

    obstacle_grid: np.ndarray
    radiation_grid: np.ndarray
    grid_size: float = 1.0
    robot_velocity: float = 1.0
    ri_max: float = 3.0
    allow_diagonal: bool = True
    prevent_corner_cutting: bool = True

    def __post_init__(self) -> None:
        self.obstacle_grid = np.asarray(self.obstacle_grid, dtype=int)
        self.radiation_grid = np.asarray(self.radiation_grid, dtype=float)
        if self.obstacle_grid.shape != self.radiation_grid.shape:
            raise ValueError(
                f"obstacle_grid shape {self.obstacle_grid.shape} does not match "
                f"radiation_grid shape {self.radiation_grid.shape}"
            )
        if self.robot_velocity <= 0:
            raise ValueError("robot_velocity must be positive")
        if self.grid_size <= 0:
            raise ValueError("grid_size must be positive")

    @property
    def shape(self) -> Tuple[int, int]:
        return self.obstacle_grid.shape

    @property
    def rows(self) -> int:
        return self.shape[0]

    @property
    def cols(self) -> int:
        return self.shape[1]

    def in_bounds(self, pos: GridPosition) -> bool:
        r, c = pos
        return 0 <= r < self.rows and 0 <= c < self.cols

    def is_obstacle(self, pos: GridPosition) -> bool:
        r, c = pos
        return int(self.obstacle_grid[r, c]) == OBSTACLE

    def is_target(self, pos: GridPosition) -> bool:
        r, c = pos
        return int(self.obstacle_grid[r, c]) == TARGET

    def is_passable(self, pos: GridPosition, avoid_high_risk: bool = True) -> bool:
        if not self.in_bounds(pos):
            return False
        if self.is_obstacle(pos):
            return False
        if avoid_high_risk and self.radiation_at(pos) >= self.ri_max:
            return False
        return True

    def radiation_at(self, pos: GridPosition) -> float:
        r, c = pos
        return float(self.radiation_grid[r, c])

    def extract_targets(self) -> List[Target]:
        targets: List[Target] = []
        count = 1
        for r in range(self.rows):
            for c in range(self.cols):
                if self.is_target((r, c)):
                    targets.append(Target(id=f"T{count}", row=r, col=c))
                    count += 1
        if not targets:
            raise ValueError("No targets with value 2 were found in obstacle_grid")
        return targets

    def neighbors(self, pos: GridPosition, avoid_high_risk: bool = True) -> Iterable[GridPosition]:
        r, c = pos
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.allow_diagonal:
            directions += [(-1, -1), (-1, 1), (1, -1), (1, 1)]

        for dr, dc in directions:
            nxt = (r + dr, c + dc)
            if not self.is_passable(nxt, avoid_high_risk=avoid_high_risk):
                continue
            if self.prevent_corner_cutting and dr != 0 and dc != 0:
                # When moving diagonally, both adjacent orthogonal cells must be free.
                if not self.is_passable((r + dr, c), avoid_high_risk=avoid_high_risk):
                    continue
                if not self.is_passable((r, c + dc), avoid_high_risk=avoid_high_risk):
                    continue
            yield nxt

    def to_world(self, pos: GridPosition) -> Tuple[float, float]:
        r, c = pos
        return c * self.grid_size, r * self.grid_size

    def path_is_valid(self, path: Sequence[GridPosition], avoid_high_risk: bool = True) -> bool:
        if not path:
            return False
        return all(self.is_passable(p, avoid_high_risk=avoid_high_risk) for p in path)
