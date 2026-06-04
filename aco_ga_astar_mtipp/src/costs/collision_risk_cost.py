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
    """risk(P) = Σ_n d(p_n, p_{n+1}) · [(1 - S(p_n)) + (1 - S(p_{n+1}))] / 2.

    Rủi ro va chạm có trọng số theo khoảng cách giữa hai ô liên tiếp
    (thay cho việc cộng dồn (1 - S) trên mọi ô). Khớp với FMF/FMF_new.
    """
    if not path or len(path) < 2:
        return 0.0

    # Rủi ro (1 - S) tại từng ô, tính một lần để dùng lại ở hai đoạn kề.
    risks = [
        cell_collision_risk(
            grid_map=grid_map,
            cell=cell,
            c1=c1,
            radius=radius,
            max_distance=max_distance,
        )
        for cell in path
    ]

    total = 0.0
    for n in range(len(path) - 1):
        (r0, c0), (r1, c1) = path[n], path[n + 1]
        d = math.hypot(r1 - r0, c1 - c0)
        total += d * (risks[n] + risks[n + 1]) / 2.0
    return float(total)