from __future__ import annotations

import math
from typing import Optional, Tuple

GridPosition = Tuple[int, int]


def euclidean_grid_distance(a: GridPosition, b: GridPosition, grid_size: float = 1.0) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1]) * grid_size


def heading_angle(a: GridPosition, b: GridPosition) -> float:
    # Coordinate order is row, col. Use col as x and row as y.
    return math.atan2(b[0] - a[0], b[1] - a[1])


def turn_angle_degrees(prev_pos: Optional[GridPosition], current: GridPosition, nxt: GridPosition) -> float:
    if prev_pos is None:
        return 0.0
    a1 = heading_angle(prev_pos, current)
    a2 = heading_angle(current, nxt)
    diff = abs(a2 - a1)
    diff = min(diff, 2 * math.pi - diff)
    return math.degrees(diff)


def route_edges(route: list[int]) -> list[tuple[int, int]]:
    return [(route[i], route[i + 1]) for i in range(len(route) - 1)]
