from typing import Tuple

from src.core.grid_map import GridMap

GridPosition = Tuple[int, int]


def is_high_risk(grid_map: GridMap, pos: GridPosition) -> bool:
    return grid_map.radiation_at(pos) >= grid_map.ri_max


def adjusted_radiation_cost(grid_map: GridMap, pos: GridPosition) -> float:
    """Equation-like high-risk avoidance: return inf if dose-rate is intolerable."""
    if is_high_risk(grid_map, pos):
        return float("inf")
    return grid_map.radiation_at(pos)
