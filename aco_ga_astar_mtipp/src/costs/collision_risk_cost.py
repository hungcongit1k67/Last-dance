from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

from src.core.constants import OBSTACLE
from src.core.grid_map import GridMap

GridPosition = Tuple[int, int]


def cell_safety(
    grid_map: GridMap,
    cell: GridPosition,
    c1: float = 0.5,
    radius: int = 5,
    max_distance: float = 7.0,
) -> float:
    """
    S(c) = C1 * (24 - N_obs(c)) / 24
         + (1 - C1) * d_min(c, O_c) / 3

    radius = 2 tương ứng vùng lân cận 5x5.
    """
    row, col = cell
    rows, cols = grid_map.obstacle_grid.shape

    obstacle_positions = []

    for r in range(row - radius, row + radius + 1):
        for c in range(col - radius, col + radius + 1):
            if r == row and c == col:
                continue

            if 0 <= r < rows and 0 <= c < cols:
                if grid_map.obstacle_grid[r, c] == OBSTACLE:
                    obstacle_positions.append((r, c))

    max_neighbors = (2 * radius + 1) ** 2 - 1
    n_obs = len(obstacle_positions)

    obstacle_density_score = (max_neighbors - n_obs) / max_neighbors

    if n_obs == 0:
        d_min = max_distance
    else:
        d_min = min(
            math.hypot(row - r, col - c)
            for r, c in obstacle_positions
        )
        d_min = min(d_min, max_distance)

    distance_score = d_min / max_distance

    safety = c1 * obstacle_density_score + (1.0 - c1) * distance_score

    return float(np.clip(safety, 0.0, 1.0))


def cell_collision_risk(
    grid_map: GridMap,
    cell: GridPosition,
    c1: float = 0.5,
    radius: int = 2,
    max_distance: float = 3.0,
) -> float:
    return 1.0 - cell_safety(
        grid_map=grid_map,
        cell=cell,
        c1=c1,
        radius=radius,
        max_distance=max_distance,
    )


def path_collision_risk(
    grid_map: GridMap,
    path: Sequence[GridPosition],
    c1: float = 0.5,
    radius: int = 2,
    max_distance: float = 3.0,
) -> float:
    """Sum of (1 - S(c)) for all cells except the last one.

    Implements formula 11: risk(P) = sum_{n=1}^{|p|-1} (1 - S(p_n)),
    where p_{|p|} = T_j (goal target) is excluded.
    """
    if not path:
        return 0.0

    # Exclude the last cell (goal target) per formula 11
    cells = path[:-1] if len(path) > 1 else path
    return float(
        sum(
            cell_collision_risk(
                grid_map=grid_map,
                cell=cell,
                c1=c1,
                radius=radius,
                max_distance=max_distance,
            )
            for cell in cells
        )
    )